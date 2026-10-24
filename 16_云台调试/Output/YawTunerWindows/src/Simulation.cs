using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Threading;

namespace YawTuner
{
    /// <summary>A deliberately synthetic plant, for exercising the application without hardware.</summary>
    public sealed class SimulatedTransport : ITransport
    {
        private readonly object gate = new object();
        private readonly object dispatch = new object();
        private readonly Stopwatch clock = new Stopwatch();
        private Timer timer;
        private bool open, active, homed, telemetry;
        private Pid gains = DefaultGains();
        private Pid frozen;
        private uint id;
        private int phase;
        private long stepStart, homeStart;
        private double target, angle, homeAngle, lastAngle, lastT, first = -1, bandSince = -1, overshoot, steadyIntegral, steadyPeak;
        private double maxGap;
        private int samples, steadySamples;
        private string lastResult;
        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        public event Action<string> LineReceived;
        public event Action<string> Failed;
        public bool IsOpen { get { lock (gate) return open; } }
        public bool IsDemo { get { return true; } }
        private static Pid DefaultGains() { return new Pid(new double[] { 16.63, .12, .269, .0081, .003, .000209 }); }

        public void Open()
        {
            lock (gate)
            {
                if (open) return;
                gains = DefaultGains(); frozen = gains.Copy(); id = 0; phase = 0; target = angle = lastAngle = 0;
                active = telemetry = true; homed = false; homeStart = -1; lastResult = null; open = true;
                clock.Restart(); timer = new Timer(Tick, null, 5, 5);
            }
        }

        public void Send(string command)
        {
            if (!Protocol.AllowedCommand(command)) throw new InvalidOperationException("命令未通过本地固定白名单。");
            List<string> replies = new List<string>();
            lock (gate)
            {
                if (!open) throw new InvalidOperationException("演示端未连接。");
                long now = clock.ElapsedMilliseconds;
                if (command == "YAW PID") replies.Add(PidReply());
                else if (command == "YAW STATUS" || command == "YAW INFO" || command == "YAW") replies.Add(Status());
                else if (command == "YAW VIEW") replies.Add("Y|VIEW|YAW|GROUP|gimbal|COLS|72|PID_I|7_12|BAND_DEG|0.2|TIME_LIMIT_MS|250|OS_LIMIT_DEG|0.2|SSE_LIMIT_DEG|0.2");
                else if (command == "TELEM ON") { telemetry = true; replies.Add("C|OK|TELEM|ON"); }
                else if (command == "YAW OFF")
                {
                    active = false;
                    if (phase == 1) { phase = 3; lastResult = ResultLine("ABORTED", "STOP", Math.Min(5000, now - stepStart)); replies.Add(lastResult); }
                    replies.Add("Y|OK|OFF");
                }
                else if (command == "YAW HOME")
                {
                    if (!active || phase == 1) replies.Add("Y|ERROR|HOME_REQUIRES_ARMED_ACTIVE_NEUTRAL");
                    else { homeStart = now; homeAngle = angle; target = 0; homed = false; replies.Add("Y|OK|HOME|TARGET|0"); }
                }
                else if (command == "YAW STEP CHECK") replies.Add(Check(now));
                else if (command == "YAW STEP 30" || command == "YAW STEP -30")
                {
                    if (!active || !homed || phase == 1 || Math.Abs(angle) > .2)
                        replies.Add("Y|ERROR|STEP_REQUIRES_ARMED_ZERO_STATIONARY_HEALTHY|WHY|" + (!active ? "NOT_ACTIVE" : "SEND_HOME_FIRST") + "|BLOCKS|1");
                    else
                    {
                        id = unchecked(id + 1); phase = 1; stepStart = now; target = command.EndsWith("-30", StringComparison.Ordinal) ? -30 : 30;
                        frozen = gains.Copy(); homed = false; homeStart = -1;
                        lastAngle = angle = 0; lastT = overshoot = steadyIntegral = steadyPeak = maxGap = 0;
                        first = bandSince = -1; samples = steadySamples = 0;
                        replies.Add("Y|OK|STEP_STARTED|TARGET|" + N(target) + "|WINDOW_MS|5000|HOLD|1");
                    }
                }
                else if (command == "YAW RESULT")
                    replies.Add(phase == 1 ? ResultLine("RUNNING", "NONE", now - stepStart) : (lastResult ?? IdleResult()));
                else if (command == "YAW FF") replies.Add("Y|FF|ON|0|TYPE|ACCEL|FKA|0|FMAX|0|SHAPE_MS|0|SHAPE_ACTIVE|0");
                else if (command == "YAW SHAPE") replies.Add("Y|SHAPE|ENABLED|0|MS|0|ACTIVE|0|VEL_FF|0");
                else if (command == "YAW SHAPEV") replies.Add("Y|SHAPEV|0");
                else if (command == "YAW FKA") replies.Add("Y|FKA|0");
                else if (command == "YAW FMAX") replies.Add("Y|FMAX|0");
                else if (command == "YAW BUS") replies.Add("Y|BUS|DEMO_CASCADE_V1|EPOCH|0|DROP|0");
                else if (command == "YAW CHECK") replies.Add("Y|CHECK|REMOTE|OK|BLOCKS|0|CH|1500,1500,1500,1500,1500,1000,1500,1500");
                else
                {
                    bool parameter = false;
                    for (int i = 0; i < 6; i++)
                    {
                        string prefix = "YAW " + Protocol.Keys[i] + " ";
                        if (!command.StartsWith(prefix, StringComparison.Ordinal)) continue;
                        parameter = true;
                        if (phase == 1) replies.Add("Y|ERROR|PARAMETER_OR_STATE");
                        else { gains.Values[i] = double.Parse(command.Substring(prefix.Length), Inv); replies.Add("Y|OK|PARAMETER"); }
                        break;
                    }
                    if (!parameter) replies.Add("Y|HELP|DEMO|SYNTHETIC_PLANT_ONLY");
                }
            }
            Emit(replies);
        }

        private void Tick(object unused)
        {
            try
            {
                List<string> output = new List<string>();
                lock (gate)
                {
                    if (!open) return;
                    long now = clock.ElapsedMilliseconds;
                    double velocity = 0;
                    if (phase == 1)
                    {
                        double t = Math.Min(5000, now - stepStart);
                        if (t > lastT)
                        {
                            double next = Position(t);
                            double gap = t - lastT;
                            velocity = (next - lastAngle) * 1000 / gap;
                            samples++; maxGap = Math.Max(maxGap, gap);
                            overshoot = Math.Max(overshoot, (next - target) * Math.Sign(target));
                            if (Math.Abs(target - next) <= .20001)
                            { if (bandSince < 0) bandSince = t; if (first < 0) first = t; }
                            else bandSince = -1;
                            double low = Math.Max(lastT, 4000);
                            if (t > low)
                            {
                                double left = lastAngle + (next - lastAngle) * (low - lastT) / gap;
                                steadyIntegral += (left + next) * .5 * (t - low); steadySamples++;
                                steadyPeak = Math.Max(steadyPeak, Math.Max(Math.Abs(target - left), Math.Abs(target - next)));
                            }
                            lastT = t; angle = lastAngle = next;
                        }
                        if (t >= 5000)
                        {
                            phase = 2; lastResult = ResultLine("DONE", "NONE", 5000);
                        }
                    }
                    else if (homeStart >= 0)
                    {
                        double t = now - homeStart;
                        double previous = angle;
                        angle = t < 300 ? homeAngle * Math.Pow(Math.Max(0, 1 - t / 300), 3) : 0;
                        velocity = (angle - previous) * 200;
                        if (t >= 450) homed = true;
                    }
                    if (telemetry) output.Add(Telemetry(now, velocity));
                    if (lastResult != null && phase == 2 && lastT == 5000) { output.Add(lastResult); lastT = 5001; }
                }
                Emit(output);
            }
            catch (Exception error)
            {
                Action<string> handler = Failed;
                if (handler != null) handler("演示端异常：" + error.Message);
            }
        }

        private double Position(double t)
        {
            double p = frozen.Values[0] / 16.63;
            double d = frozen.Values[5] / .000209;
            double vp = frozen.Values[3] / .0081;
            double tau = 28 / Math.Sqrt(Math.Max(.05, p));
            double amplitude = Math.Max(.03, .55 + 2.8 * (p - 1) - 2 * (d - 1) + (vp - 1));
            double bump = amplitude * Math.Exp(-Math.Pow((t - 160) / 43, 2));
            double sse = .02 * Math.Cos((frozen.Values[1] + frozen.Values[4]) * 10);
            return Math.Sign(target) * (30 * (1 - Math.Exp(-t / tau)) + bump - sse * (1 - Math.Exp(-t / 500)));
        }

        private string Check(long now)
        {
            bool ready = active && homed && phase != 1 && Math.Abs(angle) <= .2;
            string why = ready ? "READY" : (!active ? "NOT_ACTIVE" : phase == 1 ? "WAIT_5S_WINDOW" : "WAIT_STILL_HOLD");
            return "Y|STEP_CHECK|READY|" + (ready ? "1" : "0") + "|WHY|" + why + "|BLOCKS|" + (ready ? "0" : "1") +
                "|ANGLE|" + N(angle) + "|TARGET|" + N(target) + "|ENC_DPS|0|RAW_DPS|0|STILL_MS|" + (ready ? "450" : "0") +
                "|WINDOW_LEFT_MS|" + (phase == 1 ? Math.Max(0, 5000 - (now - stepStart)).ToString(Inv) : "0") + "|CONSOLE_READY|" + (ready ? "1" : "0");
        }

        private string PidReply()
        {
            return "Y|PID|POS|" + gains.Values[0].ToString("F4", Inv) + "," + gains.Values[1].ToString("F4", Inv) + "," + gains.Values[2].ToString("F4", Inv) +
                "|VEL|" + gains.Values[3].ToString("F6", Inv) + "," + gains.Values[4].ToString("F6", Inv) + "," + gains.Values[5].ToString("F6", Inv) + "|VMAX_DPS|0.0|OUTPUT_LIMITS|0";
        }
        private static string IdleResult()
        {
            return "Y|STEP|ID|0|STATE|IDLE|REASON|NONE|VALID|0|START|0.000|TARGET|0.000|WINDOW_MS|0|RESP_MS|-1|T90_MS|-1|RISE_MS|-1|SETTLE_MS|-1|OS_DEG|0.000000|OS_PCT|0.000|SSE_DEG|0.000000|SSE_VALID|0|PASS|0|FAULT|0|VMAX_DPS|0|TMAX_NM|0|N|0|GAP_MS|0|LIMIT_N|0|KP|0.0000|KI|0.0000|KD|0.0000|CMD_N|0|FIRST_NM|0";
        }
        private string Status()
        {
            StringBuilder b = new StringBuilder("Y|STATUS|DEMO_CASCADE_V1|CTRL|CASCADE|ONLINE|1|ACTIVE|" + (active ? "1" : "0") + "|REQUESTED|" + (active ? "1" : "0") +
                "|STATUS|" + (active ? "1" : "0") + "|FAULT|0|AGE_MS|0|STOP_PENDING|0|ZERO_SET|1|ANGLE|" + N(angle) + "|TARGET|" + N(target));
            for (int i = 0; i < 6; i++) b.Append("|" + Protocol.Keys[i] + "|" + gains.Values[i].ToString(i < 3 ? "F4" : "F6", Inv));
            b.Append("|VREF_MAX_DPS|0|OUTPUT_LIMITS|0|TELEM_MS|5|BAUD|921600|RX|1"); return b.ToString();
        }

        private string ResultLine(string state, string reason, double duration)
        {
            double settle = bandSince >= 0 && 5000 - bandSince >= 50 ? bandSince : -1;
            double sse = duration >= 5000 ? target - steadyIntegral / 1000 : 0;
            bool done = state == "DONE";
            bool validSse = done && steadySamples >= 10 && steadyPeak <= .20001;
            bool pass = done && settle >= 0 && settle <= 250 && overshoot <= .20001 && validSse && Math.Abs(sse) <= .20001;
            StringBuilder b = new StringBuilder("Y|STEP|ID|" + id.ToString(Inv) + "|STATE|" + state + "|REASON|" + reason + "|VALID|" + (done ? "1" : "0") +
                "|START|0.000|TARGET|" + N(target) + "|WINDOW_MS|" + N(duration) + "|RESP_MS|" + N(first) + "|SETTLE_MS|" + N(settle) +
                "|OS_DEG|" + N(overshoot) + "|OS_PCT|" + N(overshoot * 100 / 30) + "|SSE_DEG|" + N(sse) + "|SSE_VALID|" + (validSse ? "1" : "0") +
                "|PASS|" + (pass ? "1" : "0") + "|FAULT|0|N|" + samples.ToString(Inv) + "|GAP_MS|" + N(maxGap));
            Pid pid = frozen ?? gains;
            for (int i = 0; i < 6; i++) b.Append("|" + Protocol.Keys[i] + "|" + pid.Values[i].ToString(i < 3 ? "F4" : "F6", Inv));
            b.Append("|CTRL|CASCADE|BAND02|0.2|RESP02_MS|" + N(first) + "|SETTLE02_MS|" + N(settle) + "|PASS02|" + (pass ? "1" : "0") +
                "|TIME_LIMIT_MS|250|OS_LIMIT_DEG|0.2|SSE_LIMIT_DEG|0.2|SSE_PEAK_DEG|" + N(steadyPeak) + "|ASSESSMENT|FINAL_SETTLE|FF_ON|0|FKA|0|FMAX|0|SHAPE_MS|0|SHAPE_V|0");
            return b.ToString();
        }

        private string Telemetry(long now, double velocity)
        {
            double[] v = new double[72];
            v[0] = angle; v[1] = target; v[2] = v[3] = v[4] = velocity;
            v[5] = v[6] = Math.Max(-1, Math.Min(1, velocity * .001));
            for (int i = 0; i < 6; i++) v[7 + i] = gains.Values[i];
            v[14] = 1; v[15] = v[16] = v[17] = active ? 1 : 0; v[21] = 1;
            v[24] = id; v[25] = phase; v[26] = v[60] = first; v[27] = v[61] = bandSince;
            v[28] = overshoot; v[29] = overshoot * 100 / 30;
            v[30] = phase == 2 ? target - steadyIntegral / 1000 : 0; v[31] = phase == 2 && steadyPeak <= .20001 ? 1 : 0;
            v[32] = v[62] = phase == 2 && bandSince >= 0 && bandSince <= 250 && overshoot <= .20001 && v[31] == 1 ? 1 : 0;
            v[39] = v[40] = 1; v[41] = 1500; v[42] = 1000; v[43] = v[44] = 1500; v[46] = 34; v[47] = 32;
            v[63] = unchecked((uint)(phase == 1 ? stepStart + (long)Math.Min(5000, now - stepStart) : (phase == 2 && lastT == 5000 ? stepStart + 5000 : now)));
            v[66] = target; v[69] = target - angle;
            StringBuilder b = new StringBuilder("gimbal:");
            for (int i = 0; i < 72; i++) { if (i != 0) b.Append(','); b.Append(N(v[i])); }
            return b.ToString();
        }
        private static string N(double value) { return value.ToString("0.######", Inv); }

        private void Emit(List<string> lines)
        {
            lock (dispatch) foreach (string line in lines)
            {
                if (!IsOpen) return;
                Action<string> handler = LineReceived; if (handler != null) handler(line);
            }
        }
        public void Close()
        {
            Timer stop;
            lock (gate) { open = false; stop = timer; timer = null; clock.Stop(); }
            if (stop != null) stop.Dispose();
        }
        public void Dispose() { Close(); }
    }
}
