using System;
using System.Globalization;

namespace YawTuner
{
    /// <summary>The fixed-cascade Yaw console protocol. All indices are zero based.</summary>
    public static class Protocol
    {
        public static readonly string[] Keys = { "PKP", "PKI", "PKD", "VKP", "VKI", "VKD" };
        public static readonly double[] Limits = { 30, 10, 2, .02, .2, .001 };
        public static readonly string[] ChannelNames = {
            "angle_deg", "target_deg", "vref_dps", "encoder_dps", "raw_dps", "torque_nm", "feedback_torque_nm",
            "PKP", "PKI", "PKD", "VKP", "VKI", "VKD", "velocity_limit_dps", "online", "active", "requested", "driver_status", "fault", "feedback_age_ms",
            "stop_pending", "zero_set", "torque_limited", "velocity_limited", "step_id", "step_phase", "first_entry_ms", "final_settle_ms", "overshoot_deg", "overshoot_pct",
            "steady_error_deg", "steady_valid", "board_pass", "t90_ms", "rise_ms", "flywheel_active", "flywheel_left_rpm", "flywheel_right_rpm", "flywheel_fault",
            "remote_fresh", "operator_armed", "rc_ch5", "rc_ch6", "rc_ch7", "rc_ch8", "can_fault_epoch", "driver_temp_c", "motor_temp_c", "feedforward_on",
            "reserved49", "reserved50", "reserved51", "reserved52", "reserved53", "feedforward_limit_nm", "feedforward_nm", "pid_nm", "velocity_integral_nm", "position_integral_dps",
            "feedforward_limited", "first_entry02_ms", "final_settle02_ms", "board_pass02", "feedback_stamp_ms", "shape_duration_ms", "shape_active", "shape_position_deg",
            "shape_velocity_dps", "shape_acceleration_dps2", "tracking_error_deg", "shape_velocity_gain", "feedforward_acceleration_gain"
        };
        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        public static bool Finite(double value) { return !double.IsNaN(value) && !double.IsInfinity(value); }

        public static bool TryFrame(string line, long hostMs, out Frame frame)
        {
            frame = null;
            if (line == null || hostMs < 0) return false;
            line = line.Trim();
            if (!line.StartsWith("gimbal:", StringComparison.Ordinal)) return false;
            string[] parts = line.Substring(7).Split(',');
            if (parts.Length != 72) return false;
            double[] values = new double[72];
            for (int i = 0; i < values.Length; i++)
                if (!double.TryParse(parts[i], NumberStyles.Float, Inv, out values[i]) || !Finite(values[i])) return false;
            // Integer fields must stay exact, especially the uint32 millisecond stamp and step id.
            int[] uintFields = { 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 31, 32, 35, 38, 39, 40, 41, 42, 43, 44, 45, 48, 59, 62, 63, 64, 65 };
            foreach (int i in uintFields)
                if (values[i] < 0 || values[i] > uint.MaxValue || values[i] != Math.Truncate(values[i])) return false;
            if (values[25] > 3 || Math.Abs(values[0]) > 1000 || Math.Abs(values[1]) > 1000) return false;
            frame = new Frame { HostMs = hostMs, Values = values };
            return true;
        }

        public static Reply ParseReply(string line)
        {
            if (string.IsNullOrWhiteSpace(line)) return null;
            line = line.Trim();
            string[] tokens = line.Split('|');
            if (tokens.Length < 2 || (tokens[0] != "Y" && tokens[0] != "C") || tokens[1].Length == 0) return null;
            Reply reply = new Reply { Raw = line, Kind = tokens[1] };
            int start = 2;
            if ((reply.Kind == "STATUS" || reply.Kind == "STATE" || reply.Kind == "BUS") && tokens.Length > 2)
            {
                reply.Fields["VERSION"] = tokens[2];
                start = 3;
            }
            else if (reply.Kind == "OK" || reply.Kind == "ERROR" || reply.Kind == "BUSY" || reply.Kind == "VIEW")
            {
                if (tokens.Length > 2) { reply.Fields["ACTION"] = tokens[2]; start = 3; }
            }
            for (int i = start; i + 1 < tokens.Length; i += 2)
            {
                if (tokens[i].Length == 0 || reply.Fields.ContainsKey(tokens[i])) return null;
                reply.Fields.Add(tokens[i], tokens[i + 1]);
            }
            // A trailing description (TELEM|ON, HELP, etc.) is retained without inventing a value.
            if ((tokens.Length - start) % 2 != 0) reply.Fields["DETAIL"] = tokens[tokens.Length - 1];
            return reply;
        }

        public static Pid ReadPid(Reply reply)
        {
            if (reply == null) throw new FormatException("缺少 PID 回读。");
            double[] values = new double[6];
            if (reply.Kind == "PID")
            {
                string[] pos = reply.Get("POS").Split(',');
                string[] vel = reply.Get("VEL").Split(',');
                if (pos.Length != 3 || vel.Length != 3) throw new FormatException("PID 回读不是双环六参数格式。");
                for (int i = 0; i < 6; i++)
                    if (!double.TryParse(i < 3 ? pos[i] : vel[i - 3], NumberStyles.Float, Inv, out values[i]))
                        throw new FormatException("PID 回读包含无效数字。");
            }
            else for (int i = 0; i < 6; i++) values[i] = Required(reply, Keys[i]);
            Pid pid = new Pid(values); ValidatePid(pid); return pid;
        }

        public static StepResult ParseResult(Reply reply)
        {
            if (reply == null || reply.Kind != "STEP") throw new FormatException("这不是 Yaw STEP 成绩。");
            double id = Required(reply, "ID");
            if (id < 0 || id > uint.MaxValue || id != Math.Truncate(id)) throw new FormatException("阶跃编号无效。");
            // Freshly booted firmware has not frozen any cascade trial yet. Its real IDLE
            // report intentionally contains the legacy common KP/KI/KD fields only.
            // Read the id for the next-step barrier without treating this as trial evidence.
            if (id == 0 && reply.Get("STATE") == "IDLE" && reply.Get("CTRL").Length == 0 && reply.Get("VALID") == "0")
            {
                return new StepResult { Id = 0, State = "IDLE", Reason = reply.Get("REASON"), Raw = reply.Raw,
                    Start = Required(reply, "START"), Target = Required(reply, "TARGET"), WindowMs = Required(reply, "WINDOW_MS"),
                    FirstMs = Required(reply, "RESP_MS"), SettleMs = Required(reply, "SETTLE_MS"), Overshoot = Required(reply, "OS_DEG"),
                    Sse = Required(reply, "SSE_DEG"), SsePeak = 0, Samples = Required(reply, "N"), GapMs = Required(reply, "GAP_MS"),
                    Fault = Required(reply, "FAULT"), Valid = false, SseValid = false, BoardPass = false, CriteriaMatch = false, DirectStep = false, Gains = null };
            }
            StepResult result = new StepResult {
                Raw = reply.Raw, Id = (uint)id, State = reply.Get("STATE"), Reason = reply.Get("REASON"),
                Start = Required(reply, "START"), Target = Required(reply, "TARGET"), WindowMs = Required(reply, "WINDOW_MS"),
                FirstMs = Required(reply, "RESP02_MS"), SettleMs = Required(reply, "SETTLE02_MS"),
                Overshoot = Required(reply, "OS_DEG"), Sse = Required(reply, "SSE_DEG"), SsePeak = Required(reply, "SSE_PEAK_DEG"),
                Samples = Required(reply, "N"), GapMs = Required(reply, "GAP_MS"), Fault = Required(reply, "FAULT"),
                Valid = reply.Get("VALID") == "1", SseValid = reply.Get("SSE_VALID") == "1",
                BoardPass = reply.Get("PASS") == "1" && reply.Get("PASS02") == "1", Gains = ReadPid(reply)
            };
            result.CriteriaMatch = reply.Get("CTRL") == "CASCADE" && reply.Get("ASSESSMENT") == "FINAL_SETTLE" &&
                Near(reply.Number("BAND02"), .2, .000001) && Near(reply.Number("TIME_LIMIT_MS"), 250, .000001) &&
                Near(reply.Number("OS_LIMIT_DEG"), .2, .000001) && Near(reply.Number("SSE_LIMIT_DEG"), .2, .000001) &&
                Near(reply.Number("RESP_MS"), result.FirstMs, .000001) && Near(reply.Number("SETTLE_MS"), result.SettleMs, .000001) &&
                reply.Get("PASS") == reply.Get("PASS02");
            result.DirectStep = reply.Get("FF_ON") == "0" && Near(reply.Number("SHAPE_MS"), 0, 0) && Near(reply.Number("SHAPE_V"), 0, 0) &&
                Math.Abs(result.Start) <= .20001 && Math.Abs(Math.Abs(result.Target) - 30) <= .00001 &&
                Math.Abs(Math.Abs(result.Target - result.Start) - 30) <= .20001;
            return result;
        }

        private static double Required(Reply reply, string key)
        {
            double value = reply.Number(key);
            if (!Finite(value)) throw new FormatException("成绩/状态缺少有效字段：" + key);
            return value;
        }
        private static bool Near(double a, double b, double tolerance) { return Finite(a) && Finite(b) && Math.Abs(a - b) <= tolerance; }

        public static bool SamePid(Pid a, Pid b)
        {
            if (a == null || b == null || a.Values == null || b.Values == null || a.Values.Length != 6 || b.Values.Length != 6) return false;
            for (int i = 0; i < 6; i++)
                if (!Near(a.Values[i], b.Values[i], i < 3 ? .0000501 : .000000501)) return false;
            return true;
        }

        public static void ValidatePid(Pid pid)
        {
            if (pid == null || pid.Values == null || pid.Values.Length != 6) throw new ArgumentException("需要双环六参数。");
            for (int i = 0; i < 6; i++)
                if (!Finite(pid.Values[i]) || pid.Values[i] < 0 || pid.Values[i] > Limits[i])
                    throw new ArgumentException(Keys[i] + " 超出板端范围 0.." + Limits[i].ToString("G", Inv));
        }

        public static string ParameterCommand(int key, double value)
        {
            if (key < 0 || key >= 6 || !Finite(value) || value < 0 || value > Limits[key]) throw new ArgumentException("参数编号或值无效。");
            return "YAW " + Keys[key] + " " + value.ToString("0.#########", Inv);
        }

        public static bool AllowedCommand(string command)
        {
            if (string.IsNullOrEmpty(command) || command != command.Trim() || command.IndexOfAny(new char[] { '\r', '\n', '\0', ';' }) >= 0) return false;
            string[] reads = { "YAW", "YAW HELP", "YAW ?", "YAW STATUS", "YAW INFO", "YAW CHECK", "YAW STEP CHECK", "YAW BUS", "YAW PID", "YAW RESULT", "YAW FF", "YAW SHAPE", "YAW SHAPEV", "YAW FKA", "YAW FMAX", "YAW VIEW", "TELEM ON", "YAW HOME", "YAW OFF", "YAW STEP 30", "YAW STEP -30" };
            foreach (string item in reads) if (command == item) return true;
            for (int i = 0; i < Keys.Length; i++)
            {
                string prefix = "YAW " + Keys[i] + " ";
                if (!command.StartsWith(prefix, StringComparison.Ordinal)) continue;
                string number = command.Substring(prefix.Length);
                double value;
                return number.Length > 0 && number == number.Trim() && number.IndexOf(' ') < 0 &&
                    double.TryParse(number, NumberStyles.AllowLeadingSign | NumberStyles.AllowDecimalPoint | NumberStyles.AllowExponent, Inv, out value) &&
                    Finite(value) && value >= 0 && value <= Limits[i];
            }
            return false;
        }

        public static string HealthReason(Frame frame)
        {
            if (frame == null || frame.Values == null || frame.Values.Length != 72) return "尚未收到 72 通道 Yaw 遥测";
            double[] v = frame.Values;
            foreach (double value in v) if (!Finite(value)) return "遥测包含无效数字";
            if (v[18] != 0) return "Yaw 故障位=" + v[18].ToString("0", Inv);
            if (v[45] != 0) return "CAN 总线故障历史未清除";
            if (v[14] != 1 || v[19] >= 80) return "反馈离线或超过 80 ms";
            if (v[17] != 1) return "驱动未处于运行状态，需人工准备";
            if (v[20] != 0) return "正在停止/使能过渡";
            if (v[21] != 1) return "尚未人工建立零点";
            if (v[15] != 1 || v[16] != 1) return "Yaw 未使能，请按固件流程人工准备";
            if (v[39] != 1 || v[40] != 1) return "遥控反馈失效或未解锁";
            if (v[41] <= 1300 || v[42] > 1300 || v[43] <= 1300 || v[43] > 1700 || v[44] <= 1300 || v[44] > 1700)
                return "遥控开关不满足考核中立条件";
            if (v[48] != 0 || v[64] != 0 || v[65] != 0 || v[70] != 0 || v[71] != 0) return "存在前馈或平滑轨迹，不能按真实阶跃验收";
            return "";
        }
    }
}
