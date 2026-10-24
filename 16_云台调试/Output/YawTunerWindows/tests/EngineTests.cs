using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

namespace YawTuner.Tests
{
    public static class EngineTests
    {
        private static int count;
        public static int Run()
        {
            count = 0;
            HandshakeAndRamApply();
            FullBidirectionalExperiment();
            StoppingCancelsQueue();
            InjectedFailure("home_ack_timeout", "命令确认超时");
            InjectedFailure("old_id", "ID");
            InjectedFailure("rx_stall", "停滞");
            InjectedFailure("stamp_stall", "反馈停滞");
            InjectedFailure("fault", "故障");
            InjectedFailure("external_pid", "外部改变");
            InjectedFailure("malformed_frame", "无效");
            InjectedFailure("disconnect", "测试断线");
            BadVersionCannotMove();
            PassiveErrorsDoNotStop();
            ReconnectAfterTransportFailure();
            return count;
        }

        private static string Root(string label)
        {
            return Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "test-sessions",
                label + "_" + Guid.NewGuid().ToString("N").Substring(0, 8));
        }
        private static void Assert(bool condition, string message)
        { if (!condition) throw new Exception("Engine test: " + message); count++; }
        private static string FirstLineShared(string path)
        {
            using (FileStream stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
            using (StreamReader reader = new StreamReader(stream)) return reader.ReadLine();
        }
        private static void Wait(Task task, int milliseconds)
        {
            if (!task.Wait(milliseconds)) throw new Exception("Engine operation timed out in test.");
        }
        private static void Fails(Task task, int milliseconds, string contains)
        {
            try { Wait(task, milliseconds); }
            catch (AggregateException ex)
            {
                string message = ex.Flatten().ToString();
                Assert(message.IndexOf(contains, StringComparison.OrdinalIgnoreCase) >= 0, "Expected " + contains + ", got " + message);
                return;
            }
            throw new Exception("Expected an operation failure: " + contains);
        }
        private static TrialPlan Plan(Pid gains)
        {
            TrialPlan plan = new TrialPlan { Condition = "OFFLINE synthetic test", Acceptance = false };
            plan.Trials.Add(new TrialSpec { Candidate = "测试候选", Gains = gains.Copy(), Direction = 1, Repeat = 1 });
            plan.Trials.Add(new TrialSpec { Candidate = "测试候选", Gains = gains.Copy(), Direction = -1, Repeat = 1 });
            return plan;
        }

        private static void HandshakeAndRamApply()
        {
            ProbeTransport wire = new ProbeTransport("");
            using (Workbench bench = new Workbench(Root("handshake")))
            {
                Wait(bench.ConnectAsync(wire), 6000);
                string[] commands = wire.Commands();
                Assert(string.Join(";", commands) == "YAW VIEW;TELEM ON;YAW STATUS;YAW PID;YAW RESULT;YAW STEP CHECK", "Connection must only query and enable telemetry.");
                Pid original = bench.Snapshot().CurrentPid;
                Assert(original != null && Protocol.SamePid(original, bench.Snapshot().Baseline), "Baseline must be actual board readback.");
                Pid desired = original.Copy(); desired.Values[0] = Math.Round(desired.Values[0] * .99, 4);
                Wait(bench.ApplyAsync(desired), 7000);
                WorkbenchSnapshot current = bench.Snapshot();
                Assert(Protocol.SamePid(current.CurrentPid, desired), "RAM apply readback failed.");
                Assert(Protocol.SamePid(current.Baseline, original), "Apply must never overwrite baseline.");
                commands = wire.Commands();
                int firstParameter = Array.FindIndex(commands, delegate(string x) { return x.StartsWith("YAW PKP ", StringComparison.Ordinal); });
                int firstHome = Array.IndexOf(commands, "YAW HOME"), firstCheck = Array.IndexOf(commands, "YAW STEP CHECK", firstHome);
                Assert(firstHome >= 0 && firstCheck > firstHome && firstParameter > firstCheck, "Apply requires HOME and CHECK before writing.");
                Wait(bench.RestoreBaselineAsync(), 7000);
                Assert(Protocol.SamePid(bench.Snapshot().CurrentPid, original), "Explicit baseline restore failed.");
                string directory = bench.ExportSummary();
                Assert(directory.IndexOf("DEMO_模拟_非验收", StringComparison.Ordinal) >= 0, "Demo sessions need separate clearly labelled paths.");
                Assert(File.Exists(Path.Combine(directory, "raw.jsonl")) && File.Exists(Path.Combine(directory, "telemetry.csv")), "Raw and telemetry files must flush before export.");
                string session = File.ReadAllText(Path.Combine(directory, "session.json"));
                Assert(session.Contains("not_verified"), "Connecting must not label hardware as validated.");
                Assert(FirstLineShared(Path.Combine(directory, "telemetry.csv")).Split(',').Length == 74, "Telemetry CSV must retain all 72 channels and host/mode columns.");
                string exported = Root("exported"); bench.Export(exported);
                Assert(File.Exists(Path.Combine(exported, "summary.json")) && File.Exists(Path.Combine(exported, "AI_分析摘要.md")), "Export must include summary and AI input.");
                bench.Disconnect();
                string closedExport = Root("exported_closed"); bench.Export(closedExport);
                Assert(File.Exists(Path.Combine(closedExport, "telemetry.csv")) && File.Exists(Path.Combine(closedExport, "summary.json")), "Closed sessions must remain exportable.");
            }
        }

        private static void FullBidirectionalExperiment()
        {
            ProbeTransport wire = new ProbeTransport("");
            using (Workbench bench = new Workbench(Root("full")))
            {
                Wait(bench.ConnectAsync(wire), 6000);
                Pid original = bench.Snapshot().CurrentPid;
                Pid candidate = original.Copy(); candidate.Values[0] = Math.Round(candidate.Values[0] * .99, 4);
                Wait(bench.RunAsync(Plan(candidate)), 25000);
                WorkbenchSnapshot state = bench.Snapshot();
                Assert(state.Records.Count == 2 && state.Completed == 2, "Both directions must finish.");
                Assert(state.Records[0].Direction == 1 && state.Records[1].Direction == -1, "Direction order lost.");
                Assert(state.Records[0].Result.Id != state.Records[1].Result.Id, "Trials must bind different new board IDs.");
                Assert(state.Records[0].Assessment.BoardComplete && state.Records[1].Assessment.BoardComplete, "Both board windows must be complete.");
                Assert(state.Records[0].Assessment.WaveComplete && state.Records[1].Assessment.WaveComplete, "Full host waveforms must cover both windows.");
                Assert(Protocol.SamePid(state.CurrentPid, original), "Normal completion must restore run-start PID.");
                Assert(state.Records[0].Mode == "DEMO" && state.Records[1].Mode == "DEMO", "Synthetic results must remain marked DEMO.");
                Assert(!string.IsNullOrEmpty(state.Records[0].RunId) && state.Records[0].RunId == state.Records[1].RunId && !state.Records[0].Acceptance,
                    "Every plan needs its own run identity and explicit acceptance flag.");
                string[] commands = wire.Commands();
                Assert(Array.FindAll(commands, delegate(string s) { return s == "YAW HOME"; }).Length >= 3, "Each direction and normal finish require HOME.");
                Assert(Array.IndexOf(commands, "YAW OFF") < 0, "Normal completion should not silently disable the operator's session.");
                Assert(Array.FindAll(commands, delegate(string s) { return s == "YAW READY" || s == "YAW ZERO" || s == "YAW ON" || s == "YAW CLEAR"; }).Length == 0, "Forbidden preparation commands must never be sent.");
                foreach (TrialRecord record in state.Records)
                    Assert(File.Exists(Path.Combine(state.SessionDirectory, "trials", record.TrialId + ".json")), "Each trial must be durable before proceeding.");
            }
        }

        private static void StoppingCancelsQueue()
        {
            ProbeTransport wire = new ProbeTransport("");
            using (Workbench bench = new Workbench(Root("stop")))
            {
                Wait(bench.ConnectAsync(wire), 6000);
                Task run = bench.RunAsync(Plan(bench.Snapshot().CurrentPid));
                Assert(wire.StepSent.WaitOne(5000), "Step did not start before stop test.");
                Thread.Sleep(100); bench.Stop(); bench.Stop();
                Fails(run, 6000, "用户停止");
                string[] commands = wire.Commands(); int off = Array.IndexOf(commands, "YAW OFF");
                Assert(off >= 0 && Array.FindAll(commands, delegate(string x) { return x == "YAW OFF"; }).Length == 1, "Stop must send exactly one OFF.");
                for (int i = off + 1; i < commands.Length; i++) Assert(!IsMutation(commands[i]), "No queued motion/parameter command after OFF.");
                WorkbenchSnapshot state = bench.Snapshot();
                Assert(state.Records.Count == 1 && !state.Records[0].Assessment.Pass, "Cancelled partial wave must be retained as failed.");
                int n = commands.Length;
                Wait(bench.RestoreBaselineAsync(), 5000);
                commands = wire.Commands();
                bool changed = false;
                for (int i = n; i < commands.Length; i++)
                {
                    Assert(commands[i] != "YAW HOME" && commands[i] != "YAW ON" && commands[i] != "YAW READY" && commands[i] != "YAW ZERO", "Stopped baseline restore must only write RAM and query.");
                    changed |= commands[i].StartsWith("YAW PKP ", StringComparison.Ordinal);
                }
                Assert(changed && Protocol.SamePid(bench.Snapshot().CurrentPid, bench.Snapshot().Baseline), "Stopped restore must read back the baseline without motion.");
            }
        }

        private static bool IsMutation(string command)
        { return command == "YAW HOME" || command.StartsWith("YAW STEP ", StringComparison.Ordinal) && command != "YAW STEP CHECK" || command.StartsWith("YAW PK", StringComparison.Ordinal) || command.StartsWith("YAW VK", StringComparison.Ordinal); }

        private static void InjectedFailure(string mode, string expected)
        {
            ProbeTransport wire = new ProbeTransport(mode);
            using (Workbench bench = new Workbench(Root(mode)))
            {
                Wait(bench.ConnectAsync(wire), 6000);
                Task run = bench.RunAsync(Plan(bench.Snapshot().CurrentPid));
                Fails(run, 10000, expected);
                string[] commands = wire.Commands(); int off = Array.IndexOf(commands, "YAW OFF");
                Assert(off >= 0, "Failure must attempt OFF: " + mode);
                Assert(Array.FindAll(commands, delegate(string x) { return x == "YAW STEP -30"; }).Length == 0, "Failure must cancel the second direction: " + mode);
                Assert(Array.FindAll(commands, delegate(string x) { return x == "YAW OFF"; }).Length == 1, "Failure must not loop OFF: " + mode);
                for (int i = off + 1; i < commands.Length; i++) Assert(!IsMutation(commands[i]), "Failure cannot restore parameters or move after OFF: " + mode);
                if (mode == "home_ack_timeout")
                {
                    int n = commands.Length;
                    try { bench.RefreshAsync(); throw new Exception("Desynchronized connection unexpectedly reusable."); }
                    catch (InvalidOperationException) { }
                    Assert(wire.Commands().Length == n, "Lost ACK requires reconnect, not another request that might match the stale ACK.");
                }
                if (mode == "disconnect") Assert(!bench.Snapshot().Connected, "Failed transport must show disconnected state.");
            }
        }

        private static void BadVersionCannotMove()
        {
            ProbeTransport wire = new ProbeTransport("bad_version");
            using (Workbench bench = new Workbench(Root("bad_version")))
            {
                Wait(bench.ConnectAsync(wire), 6000);
                Fails(bench.RunAsync(Plan(bench.Snapshot().CurrentPid)), 3000, "口径");
                Assert(Array.FindAll(wire.Commands(), IsMutation).Length == 0, "Unknown firmware must not receive motion or PID writes.");
            }
        }

        private static void PassiveErrorsDoNotStop()
        {
            ProbeTransport wire = new ProbeTransport("malformed_result");
            using (Workbench bench = new Workbench(Root("passive_error")))
            {
                Fails(bench.ConnectAsync(wire), 6000, "字段");
                Assert(Array.IndexOf(wire.Commands(), "YAW OFF") < 0, "Malformed read-only handshake must not stop the complete gimbal.");
                Assert(!bench.Snapshot().Connected, "Failed handshake must close its session.");
            }
        }

        private static void ReconnectAfterTransportFailure()
        {
            ProbeTransport broken = new ProbeTransport("");
            using (Workbench bench = new Workbench(Root("reconnect")))
            {
                Wait(bench.ConnectAsync(broken), 6000); broken.Break();
                Assert(!bench.Snapshot().Connected, "Idle failed transport must mark disconnected.");
                Assert(Array.IndexOf(broken.Commands(), "YAW OFF") < 0, "Idle transport failure must remain passive.");
                ProbeTransport replacement = new ProbeTransport("");
                Wait(bench.ConnectAsync(replacement), 6000);
                Assert(bench.Snapshot().Connected && bench.Snapshot().Baseline != null, "Failed connection must support a clean reconnect.");
            }
        }

        // Fault injection wraps only the synthetic plant. No physical serial API is touched.
        private sealed class ProbeTransport : ITransport
        {
            private readonly SimulatedTransport inner = new SimulatedTransport();
            private readonly object gate = new object();
            private readonly List<string> sent = new List<string>();
            private readonly string mode;
            private volatile bool stepped;
            private bool failed;
            private uint frozenStamp;
            public readonly ManualResetEvent StepSent = new ManualResetEvent(false);
            public event Action<string> LineReceived;
            public event Action<string> Failed;
            public bool IsOpen { get { return inner.IsOpen; } }
            public bool IsDemo { get { return true; } }
            public ProbeTransport(string fault)
            {
                mode = fault; inner.LineReceived += Receive;
                inner.Failed += delegate(string message) { Action<string> h = Failed; if (h != null) h(message); };
            }
            public string[] Commands() { lock (gate) return sent.ToArray(); }
            public void Open() { inner.Open(); }
            public void Break() { Action<string> handler = Failed; if (handler != null) handler("测试断线"); }
            public void Send(string command)
            {
                lock (gate) sent.Add(command);
                if (command == "YAW STEP 30" || command == "YAW STEP -30") { stepped = true; StepSent.Set(); }
                inner.Send(command);
            }
            private void Receive(string line)
            {
                if (mode == "home_ack_timeout" && line.StartsWith("Y|OK|HOME", StringComparison.Ordinal)) return;
                if (mode == "bad_version" && line.StartsWith("Y|STATUS|", StringComparison.Ordinal)) line = line.Replace("DEMO_CASCADE_V1", "UNKNOWN_FIRMWARE");
                if (mode == "malformed_result" && line.StartsWith("Y|STEP|", StringComparison.Ordinal)) line = "Y|STEP|ID|0|STATE|IDLE";
                if (stepped && mode == "old_id" && line.StartsWith("Y|STEP|ID|", StringComparison.Ordinal))
                {
                    string[] tokens = line.Split('|'); tokens[3] = "0"; line = string.Join("|", tokens);
                }
                Frame frame;
                if (Protocol.TryFrame(line, 0, out frame))
                {
                    if (!stepped) frozenStamp = frame.Stamp;
                    else
                    {
                        if (mode == "malformed_frame") { Action<string> bad = LineReceived; if (bad != null) bad("gimbal:NaN,0"); return; }
                        if (mode == "rx_stall") return;
                        if (mode == "stamp_stall") frame.Values[63] = frozenStamp;
                        if (mode == "fault") frame.Values[18] = 64;
                        if (mode == "external_pid") frame.Values[7] += .5;
                        if (mode == "disconnect" && !failed)
                        {
                            failed = true;
                            Action<string> broken = Failed; if (broken != null) broken("测试断线");
                            return;
                        }
                        if (mode == "stamp_stall" || mode == "fault" || mode == "external_pid")
                        {
                            string[] fields = new string[72];
                            for (int i = 0; i < fields.Length; i++) fields[i] = frame.Values[i].ToString("R", CultureInfo.InvariantCulture);
                            line = "gimbal:" + string.Join(",", fields);
                        }
                    }
                }
                Action<string> handler = LineReceived; if (handler != null) handler(line);
            }
            public void Close() { inner.Close(); }
            public void Dispose() { inner.Dispose(); StepSent.Dispose(); }
        }
    }
}
