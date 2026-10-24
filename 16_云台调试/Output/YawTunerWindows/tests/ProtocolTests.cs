using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Threading;

namespace YawTuner.Tests
{
    public static class ProtocolTests
    {
        private static int checks;
        private static void Check(bool value, string name) { checks++; if (!value) throw new Exception("ProtocolTests: " + name); }
        private static void Reject(Action action, string name)
        {
            bool rejected = false; try { action(); } catch (ArgumentException) { rejected = true; } catch (FormatException) { rejected = true; }
            Check(rejected, name);
        }
        private static Pid Baseline() { return new Pid(new double[] { 16.63, .12, .269, .0081, .003, .000209 }); }

        private static Frame FrameAt(long ms, uint stamp, uint id)
        {
            double[] v = new double[72];
            v[0] = Math.Min(30, ms * .15); v[1] = 30;
            Array.Copy(Baseline().Values, 0, v, 7, 6);
            v[14] = v[15] = v[16] = v[17] = v[21] = v[39] = v[40] = 1;
            v[24] = id; v[25] = ms >= 5000 ? 2 : 1;
            v[41] = 1500; v[42] = 1000; v[43] = v[44] = 1500;
            v[46] = 35; v[47] = 32; v[63] = stamp;
            return new Frame { HostMs = ms, Values = v };
        }
        private static string Csv(Frame frame)
        {
            StringBuilder b = new StringBuilder("gimbal:");
            foreach (double value in frame.Values) { if (b.Length > 7) b.Append(','); b.Append(value.ToString("R", CultureInfo.InvariantCulture)); }
            return b.ToString();
        }
        private static string ResultText()
        {
            return "Y|STEP|ID|12|STATE|DONE|REASON|NONE|VALID|1|START|0.000|TARGET|30.000|WINDOW_MS|5000|RESP_MS|100|SETTLE_MS|250|OS_DEG|0.200000|SSE_DEG|0.020000|SSE_VALID|1|PASS|1|FAULT|0|N|1000|GAP_MS|5|PKP|16.6300|PKI|0.1200|PKD|0.2690|VKP|0.008100|VKI|0.003000|VKD|0.000209|CTRL|CASCADE|BAND02|0.2|RESP02_MS|100|SETTLE02_MS|250|PASS02|1|TIME_LIMIT_MS|250|OS_LIMIT_DEG|0.2|SSE_LIMIT_DEG|0.2|SSE_PEAK_DEG|0.05|ASSESSMENT|FINAL_SETTLE|FF_ON|0|FKA|0|FMAX|0|SHAPE_MS|0|SHAPE_V|0";
        }
        private static TrialRecord Good(bool wrap)
        {
            TrialRecord record = new TrialRecord { TrialId = "test-1", RunId = "test-run", Candidate = "基线", Mode = "HARDWARE", Direction = 1, Repeat = 1,
                Gains = Baseline(), Result = Protocol.ParseResult(Protocol.ParseReply(ResultText())), Condition = "空载 24V" };
            uint origin = wrap ? uint.MaxValue - 2500 : 1000;
            for (int t = 0; t <= 5000; t += 5) record.Frames.Add(FrameAt(t, unchecked(origin + (uint)t), 12));
            return record;
        }

        public static int Run()
        {
            checks = 0;
            Frame frame;
            Check(Protocol.ChannelNames.Length == 72 && Protocol.Keys.Length == 6, "channel/key contracts");
            Check(Protocol.TryFrame(Csv(FrameAt(0, 100, 12)), 0, out frame) && frame.Stamp == 100, "72-column telemetry");
            Check(!Protocol.TryFrame("Y|PID|POS|1,2,3|VEL|.1,.2,.3", 0, out frame), "reply not telemetry");
            Check(Protocol.ParseReply(Csv(FrameAt(0, 100, 12))) == null, "telemetry not reply");
            Check(!Protocol.TryFrame(Csv(FrameAt(0, 100, 12)) + ",0", 0, out frame), "extra CSV column rejected");
            string csv = Csv(FrameAt(0, 100, 12));
            Check(!Protocol.TryFrame(csv.Replace("gimbal:0,", "gimbal:NaN,"), 0, out frame), "NaN rejected");
            Check(!Protocol.TryFrame(csv.Replace("gimbal:0,", "gimbal:Infinity,"), 0, out frame), "infinite rejected");
            Frame oversized = FrameAt(0, 100, 12); oversized.Values[63] = 4294967296.0;
            Check(!Protocol.TryFrame(Csv(oversized), 0, out frame), "uint stamp overflow rejected");
            oversized.Values[63] = 10.5;
            Check(!Protocol.TryFrame(Csv(oversized), 0, out frame), "fractional stamp rejected");
            Reply status = Protocol.ParseReply("Y|STATUS|YAW_CASCADE_VERSION|CTRL|CASCADE|ONLINE|1|STATUS|1|FAULT|0");
            Check(status != null && status.Get("VERSION") == "YAW_CASCADE_VERSION" && status.Get("STATUS") == "1", "version token does not shadow driver status");
            Check(Protocol.ParseReply("Y|STATUS|VERSION|STATUS|1|STATUS|0") == null, "ambiguous repeated status rejected");
            Check(Protocol.ParseReply("C|OK|TELEM|ON").Get("ACTION") == "TELEM", "console prefix and action");
            Check(Protocol.ParseReply("Y|OK|STEP_STARTED|TARGET|30|WINDOW_MS|5000").Get("TARGET") == "30", "action offset");
            Pid pid = Protocol.ReadPid(Protocol.ParseReply("Y|PID|POS|16.6300,0.1200,0.2690|VEL|0.008100,0.003000,0.000209|OUTPUT_LIMITS|0"));
            Check(Protocol.SamePid(pid, Baseline()), "six parameter reply");
            Reject(delegate { Protocol.ReadPid(Protocol.ParseReply("Y|PID|POS|NaN,0,0|VEL|0,0,0")); }, "invalid PID rejected");
            Reject(delegate { Protocol.ParameterCommand(5, .002); }, "firmware PID limit");
            Pid rounded = Baseline(); rounded.Values[0] += .00004; rounded.Values[5] += .0000004;
            Check(Protocol.SamePid(rounded, Baseline()), "4/6 decimal comparison");
            rounded.Values[5] += .000001;
            Check(!Protocol.SamePid(rounded, Baseline()), "meaningful parameter mismatch");
            foreach (string forbidden in new string[] { "YAW ON", "YAW READY", "YAW CLEAR", "YAW ZERO", "YAW TARGET 30", "YAW STEP 31", "YAW STEP", "YAW PKP NaN", "YAW PKP 1\r\nYAW ON", "TELEM OFF", "YAW VEL 1 2 3" })
                Check(!Protocol.AllowedCommand(forbidden), "reject unsafe command " + forbidden);
            Check(Protocol.AllowedCommand("YAW STEP -30") && Protocol.AllowedCommand(Protocol.ParameterCommand(0, 30)) && Protocol.AllowedCommand("TELEM ON"), "fixed whitelist permits intended actions");
            Check(Protocol.HealthReason(FrameAt(0, 100, 12)) == "", "running status=1 accepted");
            Frame stale = FrameAt(0, 100, 12); stale.Values[19] = 80;
            Check(Protocol.HealthReason(stale).Length > 0, "80ms health limit");
            stale = FrameAt(0, 100, 12); stale.Values[43] = 2000;
            Check(Protocol.HealthReason(stale).Length > 0, "remote firing switch guard");

            TrialRecord good = Good(false);
            Assessment a = Evaluation.Assess(good);
            Check(a.BoardComplete && a.WaveComplete && a.Pass && a.TimePass && a.OvershootPass && a.SsePass, "exact hard-condition boundaries accepted");
            Check(Evaluation.Assess(Good(true)).Pass, "legal uint32 wrap");
            TrialRecord overshoot = Good(false); overshoot.Result.Overshoot = .20002;
            Check(!Evaluation.Assess(overshoot).OvershootPass && !Evaluation.Assess(overshoot).Pass, "overshoot beyond firmware epsilon fails");
            TrialRecord transient = Good(false); transient.Result.FirstMs = 80; transient.Result.SettleMs = -1; transient.Result.BoardPass = false;
            Check(!Evaluation.Assess(transient).TimePass && !Evaluation.Assess(transient).Pass, "early entry without final settle fails");
            TrialRecord missing = Good(false); missing.Frames.RemoveRange(300, 100);
            Check(Evaluation.Assess(missing).BoardComplete && !Evaluation.Assess(missing).WaveComplete && !Evaluation.Assess(missing).Pass, "board result cannot mask missing capture");
            TrialRecord old = Good(false); old.Result.Id = 11;
            Check(!Evaluation.Assess(old).WaveComplete && !Evaluation.Assess(old).Pass, "old id not accepted");
            TrialRecord wrongTarget = Good(false); wrongTarget.Result.Target = -30;
            Check(!Evaluation.Assess(wrongTarget).BoardComplete && !Evaluation.Assess(wrongTarget).Pass, "wrong target not accepted");
            TrialRecord wrongPid = Good(false); wrongPid.Result.Gains.Values[0] += .1;
            Check(!Evaluation.Assess(wrongPid).BoardComplete, "result PID mismatch");
            TrialRecord wrongWavePid = Good(false); wrongWavePid.Frames[500].Values[7] += .1;
            Check(!Evaluation.Assess(wrongWavePid).WaveComplete, "mid-wave PID mismatch");
            TrialRecord backwards = Good(false); backwards.Frames[500].Values[63] -= 10;
            Check(!Evaluation.Assess(backwards).WaveComplete, "feedback backwards rejected");
            TrialRecord hostGap = Good(false); for (int i = 500; i < hostGap.Frames.Count; i++) hostGap.Frames[i].HostMs += 200;
            Check(!Evaluation.Assess(hostGap).WaveComplete, "host capture gap rejected");
            TrialRecord shortWave = Good(false); shortWave.Frames.RemoveRange(0, 20);
            Check(!Evaluation.Assess(shortWave).WaveComplete, "missing first100ms capture");
            TrialRecord fault = Good(false); fault.Frames[500].Values[18] = 1;
            Check(!Evaluation.Assess(fault).Pass, "transient fault retained");
            TrialRecord unstable = Good(false); unstable.Result.SseValid = false; unstable.Result.BoardPass = false;
            Check(!Evaluation.Assess(unstable).SsePass && !Evaluation.Assess(unstable).Pass, "invalid final second fails");
            StepResult shaped = Protocol.ParseResult(Protocol.ParseReply(ResultText().Replace("SHAPE_MS|0", "SHAPE_MS|100")));
            Check(!shaped.DirectStep, "smooth trajectory not step");
            StepResult firstOnly = Protocol.ParseResult(Protocol.ParseReply(ResultText().Replace("FINAL_SETTLE", "FIRST_ENTRY")));
            Check(!firstOnly.CriteriaMatch, "wrong assessment definition");
            StepResult freshBoot = Protocol.ParseResult(Protocol.ParseReply("Y|STEP|ID|0|STATE|IDLE|REASON|NONE|VALID|0|START|0.000|TARGET|0.000|WINDOW_MS|0|RESP_MS|-1|SETTLE_MS|-1|OS_DEG|0.000000|SSE_DEG|0.000000|SSE_VALID|0|PASS|0|FAULT|0|N|0|GAP_MS|0|KP|0|KI|0|KD|0"));
            Check(freshBoot.State == "IDLE" && freshBoot.Id == 0 && !freshBoot.CriteriaMatch && !freshBoot.Valid && freshBoot.Gains == null, "real noncascade fresh boot idle supported without validation claim");
            Reject(delegate { Protocol.ParseResult(Protocol.ParseReply(ResultText().Replace("OS_DEG|0.200000", "OS_DEG|NaN"))); }, "invalid result numeric");
            Check(Evaluation.BuildPlan(Baseline(), 0, new double[] { -3, 0, 3 }, 1, false, "空载").Trials.Count == 6, "percent units and positive/negative pairs");
            Check(Evaluation.BuildPlan(Baseline(), 0, new double[] { -15, -7.5, 0, 7.5, 15 }, 2, false, "空载").Trials.Count == 20, "five point scan");
            Reject(delegate { Evaluation.BuildPlan(Baseline(), 0, new double[] { -16, 0 }, 1, false, ""); }, "scan above15percent rejected");
            Reject(delegate { Evaluation.BuildPlan(Baseline(), 0, new double[] { 0 }, 6, false, ""); }, "ordinary repetitions limit");
            Pid zero = Baseline(); zero.Values[1] = 0;
            Check(Evaluation.BuildPlan(zero, 1, new double[] { 0 }, 1, false, "").Trials.Count == 2, "zero gain fixed baseline allowed");
            Reject(delegate { Evaluation.BuildPlan(zero, 1, new double[] { 0, 3 }, 1, false, ""); }, "zero gain percent scan rejected");
            Pid low = Baseline(); low.Values[5] = .000004;
            Reject(delegate { Evaluation.BuildPlan(low, 5, new double[] { 0, 15 }, 1, false, ""); }, "quantized low gain increase25percent rejected");
            low.Values[5] = .00001;
            TrialPlan collapsed = Evaluation.BuildPlan(low, 5, new double[] { -3, 0, 3 }, 1, false, "");
            Check(collapsed.Trials.Count == 2 && collapsed.Trials[0].Candidate.Contains("分辨率"), "sub-resolution offsets explicitly labelled");
            Reject(delegate { Evaluation.BuildPlan(Baseline(), 0, null, 4, true, ""); }, "acceptance min5 per direction");
            Check(Evaluation.BuildPlan(zero, 1, null, 5, true, "空载").Trials.Count == 10, "fixed acceptance both directions");
            string summary = Evaluation.Summary(new List<TrialRecord> { good });
            Check(summary.Contains("硬条件") && summary.Contains("最差") && summary.Contains("独立验收"), "summary useful limitations");
            good.Mode = "DEMO";
            Check(Evaluation.BuildAiSummary(new List<TrialRecord> { good }).Contains("不证明实机达标"), "demo AI summary clearly separated");
            return checks;
        }

        public static string SimulationSmoke()
        {
            Stopwatch watch = Stopwatch.StartNew();
            using (ManualResetEvent done = new ManualResetEvent(false))
            using (SimulatedTransport wire = new SimulatedTransport())
            {
                TrialRecord record = new TrialRecord { Direction = 1, Mode = "DEMO", Candidate = "模拟端真实五秒检查" };
                object gate = new object();
                wire.LineReceived += delegate(string line) {
                    Frame frame;
                    if (Protocol.TryFrame(line, watch.ElapsedMilliseconds, out frame)) { lock (gate) record.Frames.Add(frame); return; }
                    Reply reply = Protocol.ParseReply(line);
                    if (reply != null && reply.Kind == "STEP")
                    {
                        StepResult result = Protocol.ParseResult(reply);
                        if (result.State == "DONE") { record.Result = result; record.Gains = result.Gains; done.Set(); }
                    }
                };
                wire.Open(); wire.Send("YAW HOME"); Thread.Sleep(550); wire.Send("YAW STEP 30");
                if (!done.WaitOne(6500)) throw new Exception("Simulation smoke: 没有得到五秒完成结果");
                wire.Close();
                Assessment a; lock (gate) a = Evaluation.Assess(record);
                if (!a.BoardComplete || !a.WaveComplete) throw new Exception("Simulation smoke: " + string.Join("；", a.Issues.ToArray()));
                return "DEMO five-second smoke: BoardComplete=" + a.BoardComplete + ", WaveComplete=" + a.WaveComplete + ", Pass=" + a.Pass +
                    ", span=" + a.WaveSpanMs.ToString(CultureInfo.InvariantCulture) + "ms, frames=" + record.Frames.Count.ToString(CultureInfo.InvariantCulture);
            }
        }
    }
}
