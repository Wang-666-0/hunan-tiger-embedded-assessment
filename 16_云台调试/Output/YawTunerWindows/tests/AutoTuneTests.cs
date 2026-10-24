using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace YawTuner.Tests
{
    public static class AutoTuneTests
    {
        private static int count;
        private static uint id;
        private static void Check(bool value, string reason)
        { if (!value) throw new Exception("AI coordinator test: " + reason); count++; }
        private static void Reject(Action action, string reason)
        {
            try { action(); }
            catch (ArgumentException) { count++; return; }
            catch (InvalidOperationException) { count++; return; }
            throw new Exception("AI coordinator accepted " + reason);
        }
        private static Pid Anchor() { return new Pid(new double[] { 10, 1, .2, .01, .002, .0002 }); }
        private static AiProposal Scan(string parameter, params double[] offsets)
        { return new AiProposal { Action = "scan", Parameter = parameter, OffsetsPercent = offsets, Reason = "offline test" }; }

        public static int Run()
        {
            count = 0;
            OptionsAndBudget();
            BoundsAndCumulativeChanges();
            SelectionAndIndependentAcceptance();
            UserStopIsOneOffAndCancels();
            GuardedWaitRejectsTransientAndStaleFeedback();
            return count;
        }

        private static void OptionsAndBudget()
        {
            AutoTuneCoordinator.ValidateOptions(new AutoTuneOptions { Condition = "offline plant" }); count++;
            Reject(delegate { AutoTuneCoordinator.ValidateOptions(new AutoTuneOptions()); }, "missing condition");
            Reject(delegate { AutoTuneCoordinator.ValidateOptions(new AutoTuneOptions { Condition = "x", MaxTrials = 11 }); }, "budget less than baseline+verification");
            Reject(delegate { AutoTuneCoordinator.ValidateOptions(new AutoTuneOptions { Condition = "x", MaxTrials = 61 }); }, "unbounded budget");
            Reject(delegate { AutoTuneCoordinator.ValidateOptions(new AutoTuneOptions { Condition = "x", MaxDecisions = 0 }); }, "zero decisions");
            Reject(delegate { AutoTuneCoordinator.ValidateOptions(new AutoTuneOptions { Condition = "x", MaxDecisions = 11 }); }, "too many decisions");
            Reject(delegate { AutoTuneCoordinator.ValidateOptions(new AutoTuneOptions { Condition = "x", MaxChangePercent = 0 }); }, "zero global window");
            Reject(delegate { AutoTuneCoordinator.ValidateOptions(new AutoTuneOptions { Condition = "x", MaxChangePercent = double.NaN }); }, "NaN global window");
            Reject(delegate { AutoTuneCoordinator.ValidateOptions(new AutoTuneOptions { Condition = "x", MaxChangePercent = 16 }); }, "large global window");
            Check(AutoTuneCoordinator.FitsBudget(0, 2, 12, true), "minimum budget can screen baseline");
            Check(AutoTuneCoordinator.FitsBudget(2, 10, 12, false), "minimum budget can independently verify");
            Check(!AutoTuneCoordinator.FitsBudget(2, 2, 12, true), "minimum budget cannot consume acceptance reserve");
            Check(AutoTuneCoordinator.FitsBudget(8, 6, 24, true), "24 budget allows a second complete three-candidate scan");
            Check(!AutoTuneCoordinator.FitsBudget(14, 2, 24, true), "all steps count, including baseline candidates in scans");
            Check(!AutoTuneCoordinator.FitsBudget(int.MaxValue, int.MaxValue, 60, false), "budget addition cannot overflow");
        }

        private static void BoundsAndCumulativeChanges()
        {
            Pid anchor = Anchor();
            TrialPlan plan = AutoTuneCoordinator.ValidateProposal(Scan("PKP", 0, -3, 3), anchor, anchor, 15, "x");
            Check(plan.Trials.Count == 6, "three candidates each receive both directions");
            Check(Protocol.SamePid(plan.Trials[0].Gains, anchor) && plan.Trials[0].Direction == 1 && plan.Trials[1].Direction == -1, "baseline is an explicit bidirectional candidate");
            plan = AutoTuneCoordinator.ValidateProposal(Scan("PKP", -3, 3), anchor, anchor, 15, "x");
            Check(plan.Trials.Count == 6, "baseline is inserted if omitted");
            plan = AutoTuneCoordinator.ValidateProposal(Scan("PKP", 0, 0, 0), anchor, anchor, 15, "x");
            Check(plan.Trials.Count == 2, "duplicate offsets do not create independent experiments");
            Reject(delegate { AutoTuneCoordinator.ValidateProposal(Scan("PKP", -3, 3, 5), anchor, anchor, 15, "x"); }, "three nonzero points plus baseline");
            Reject(delegate { AutoTuneCoordinator.ValidateProposal(Scan("PKP", 0, 16), anchor, anchor, 15, "x"); }, "single-step bound");
            Reject(delegate { AutoTuneCoordinator.ValidateProposal(Scan("BOGUS", 0, 3), anchor, anchor, 15, "x"); }, "unapproved parameter");
            Reject(delegate { AutoTuneCoordinator.ValidateProposal(Scan("PKP", 0, double.NaN), anchor, anchor, 15, "x"); }, "NaN offset");
            Pid higher = anchor.Copy(); higher.Values[0] *= 1.1;
            Reject(delegate { AutoTuneCoordinator.ValidateProposal(Scan("PKP", 0, 10), higher, anchor, 15, "x"); }, "cumulative +21 percent from original anchor");
            Pid mixed = anchor.Copy(); mixed.Values[1] *= 1.16;
            Reject(delegate { AutoTuneCoordinator.ValidateProposal(Scan("PKP", 0, 1), mixed, anchor, 15, "x"); }, "unchanged key outside original global bound");
            plan = AutoTuneCoordinator.ValidateProposal(Scan("PKP", 0, 4), higher, anchor, 15, "x");
            Check(plan.Trials.Count == 4, "later +4 percent is allowed when absolute cumulative +14.4 stays bounded");
            Reject(delegate { AutoTuneCoordinator.ValidateProposal(Scan("PKP", 0, 6), anchor, anchor, 5, "x"); }, "configured smaller global bound");
            Pid zero = anchor.Copy(); zero.Values[2] = 0;
            Reject(delegate { AutoTuneCoordinator.ValidateProposal(Scan("PKD", 0, 3), zero, zero, 15, "x"); }, "nonzero offset of zero anchor");
            TrialPlan invalid = Evaluation.BuildPlan(anchor, 0, new double[] { 0 }, 1, false, "x");
            invalid.Trials[1].Gains.Values[2] = .01;
            Reject(delegate { AutoTuneCoordinator.ValidatePlan(invalid, zero, 15); }, "zero gain must remain exactly zero");
            Pid tiny = anchor.Copy(); tiny.Values[0] = .0004;
            plan = AutoTuneCoordinator.ValidateProposal(Scan("PKP", 0, -3, 3), tiny, tiny, 15, "x");
            Check(plan.Trials.Count == 2, "quantized duplicates collapse to baseline pair");
        }

        private static TrialRecord Record(Pid gains, int direction, int repeat, bool acceptance, double settle, double overshoot)
        {
            uint boardId = ++id;
            TrialRecord record = new TrialRecord { TrialId = "trial-" + boardId, RunId = "one-independent-run", Condition = "offline plant",
                Candidate = "test", Gains = gains.Copy(), Direction = direction, Repeat = repeat, Acceptance = acceptance, Mode = "DEMO" };
            record.Result = new StepResult { Id = boardId, State = "DONE", Reason = "NONE", Valid = true, SseValid = true,
                BoardPass = settle >= 0 && settle <= 250 && overshoot <= .2, CriteriaMatch = true, DirectStep = true,
                Start = 0, Target = direction * 30, WindowMs = 5000, FirstMs = Math.Min(100, settle), SettleMs = settle,
                Overshoot = overshoot, Sse = .01, SsePeak = .02, Samples = 1001, GapMs = 5, Fault = 0, Gains = gains.Copy() };
            for (int time = 0; time <= 5000; time += 5)
            {
                double[] v = new double[72]; v[1] = direction * 30;
                for (int key = 0; key < 6; key++) v[7 + key] = gains.Values[key];
                v[14] = v[15] = v[16] = v[17] = v[21] = v[39] = v[40] = 1;
                v[41] = v[43] = v[44] = 1500; v[42] = 1000;
                v[24] = boardId; v[25] = time < 5000 ? 1 : 2; v[63] = time;
                record.Frames.Add(new Frame { HostMs = time, Values = v });
            }
            record.Assessment = Evaluation.Assess(record);
            return record;
        }
        private static List<TrialRecord> Pair(Pid gains, double settle, double overshoot)
        { return new List<TrialRecord> { Record(gains, 1, 1, false, settle, overshoot), Record(gains, -1, 1, false, settle, overshoot) }; }

        private static void SelectionAndIndependentAcceptance()
        {
            Pid baseline = Anchor(), alternative = baseline.Copy(); alternative.Values[0] *= 1.03;
            List<TrialRecord> passing = Pair(baseline, 240, .1), fastFail = Pair(alternative, 100, .3);
            Check(passing[0].Assessment.Pass, "test fixture itself has full healthy 5 second evidence");
            List<TrialRecord> values = new List<TrialRecord>(fastFail); values.AddRange(passing);
            Check(Protocol.SamePid(AutoTuneCoordinator.SelectBest(values).Gains, baseline), "all hard conditions outrank faster but overshooting candidate");
            values = new List<TrialRecord> { Record(alternative, 1, 1, false, 100, .1) }; values.AddRange(passing);
            Check(Protocol.SamePid(AutoTuneCoordinator.SelectBest(values).Gains, baseline), "one-direction candidate cannot win");
            values = Pair(alternative, 100, .1); values[1].Frames.Clear(); values.AddRange(passing);
            Check(Protocol.SamePid(AutoTuneCoordinator.SelectBest(values).Gains, baseline), "incomplete waveform cannot win");
            values = Pair(alternative, 100, .1); values[1].RunId = "another-run";
            Check(AutoTuneCoordinator.SelectBest(values) == null, "opposite directions cannot be borrowed across runs");
            values = Pair(alternative, 100, .1); values[1].Result.Id = values[0].Result.Id;
            Check(AutoTuneCoordinator.SelectBest(values) == null, "duplicate board result cannot count twice");
            values = Pair(alternative, 100, .1); values[1].TrialId = values[0].TrialId;
            Check(AutoTuneCoordinator.SelectBest(values) == null, "duplicate trial cannot count twice");
            values = Pair(alternative, 100, .1); values[1].Result.Overshoot = .3; values[1].Assessment = Evaluation.Assess(values[1]);
            Check(!AutoTuneCoordinator.SelectBest(values).Pass, "worst direction controls candidate pass");
            List<TrialRecord> acceptance = new List<TrialRecord>();
            for (int repeat = 1; repeat <= 5; repeat++)
            { acceptance.Add(Record(baseline, 1, repeat, true, 200, .1)); acceptance.Add(Record(baseline, -1, repeat, true, 200, .1)); }
            Check(AutoTuneCoordinator.IndependentPass(acceptance, baseline, "offline plant"), "one new fixed-PID run with unique positive and negative five repetitions verifies");
            Check(!AutoTuneCoordinator.IndependentPass(passing, baseline, "offline plant"), "screening pair never constitutes independent acceptance");
            acceptance[9].RunId = "old-run";
            Check(!AutoTuneCoordinator.IndependentPass(acceptance, baseline, "offline plant"), "historical acceptance cannot pad count"); acceptance[9].RunId = acceptance[0].RunId;
            acceptance[9].Repeat = 4;
            Check(!AutoTuneCoordinator.IndependentPass(acceptance, baseline, "offline plant"), "repeated repeat index cannot count twice"); acceptance[9].Repeat = 5;
            acceptance[9].Acceptance = false;
            Check(!AutoTuneCoordinator.IndependentPass(acceptance, baseline, "offline plant"), "ordinary scans cannot pad acceptance"); acceptance[9].Acceptance = true;
            acceptance[9].Result.WindowMs = 4990;
            Check(!AutoTuneCoordinator.IndependentPass(acceptance, baseline, "offline plant"), "one incomplete trial invalidates independent acceptance");
        }

        private static string DirectoryFor(string label)
        { return Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "test-sessions", label + "_" + Guid.NewGuid().ToString("N").Substring(0, 8)); }
        private static void Wait(Task task, int milliseconds)
        { if (!task.Wait(milliseconds)) throw new Exception("AI coordinator test timed out."); }

        private static void UserStopIsOneOffAndCancels()
        {
            ObservedTransport transport = new ObservedTransport();
            using (Workbench bench = new Workbench(DirectoryFor("ai_stop")))
            {
                Wait(bench.ConnectAsync(transport), 5000);
                using (FakeAdvisor advisor = new FakeAdvisor())
                using (AutoTuneCoordinator coordinator = new AutoTuneCoordinator(bench, advisor, new AutoTuneOptions { Condition = "offline plant" }))
                {
                    Task active = coordinator.RunAsync();
                    // Stop before the first HOME can finish, then request it again.
                    coordinator.Stop(); coordinator.Stop();
                    bool cancelled = false;
                    try { Wait(active, 5000); }
                    catch (AggregateException) { cancelled = true; }
                    Check(cancelled, "user stop cancels active AI task");
                    Check(transport.Count("YAW OFF") == 1, "coordinator stop sends exactly one OFF");
                    Check(transport.CountPrefix("YAW STEP 30") + transport.CountPrefix("YAW STEP -30") == 0, "stop cannot enqueue a late step");
                    Check(advisor.Calls == 0, "stop cannot request a late model decision");
                    Check(!coordinator.Snapshot().Verified && !coordinator.Snapshot().Running, "cancelled workflow cannot verify");
                }
            }
        }

        private static void GuardedWaitRejectsTransientAndStaleFeedback()
        {
            foreach (string failure in new string[] { "fault", "bad_frame", "external_pid", "external_step", "home_target", "stamp_stall", "rx_stall" })
            {
                ObservedTransport transport = new ObservedTransport();
                using (Workbench bench = new Workbench(DirectoryFor("ai_guard_" + failure)))
                using (ManualResetEvent modelStarted = new ManualResetEvent(false))
                {
                    Wait(bench.ConnectAsync(transport), 5000);
                    Wait(bench.HomeAsync(), 5000);
                    Pid expected = bench.Snapshot().CurrentPid;
                    TaskCompletionSource<AiProposal> lateResponse = new TaskCompletionSource<AiProposal>();
                    CancellationToken modelToken = CancellationToken.None;
                    Task<AiProposal> active = bench.GuardedWaitAsync(expected, delegate(CancellationToken token) {
                        modelToken = token; modelStarted.Set(); return lateResponse.Task; }, CancellationToken.None);
                    Check(modelStarted.WaitOne(2000), "guarded provider starts only inside protected operation: " + failure);
                    if (failure == "stamp_stall") transport.FreezeFeedback();
                    else if (failure == "rx_stall") transport.SuppressFeedback();
                    else transport.InjectTransient(failure);
                    bool rejected = false; string rejection = "";
                    try { Wait(active, 3000); }
                    catch (AggregateException ex) { rejected = true; rejection = ex.Flatten().ToString(); }
                    Check(rejected && modelToken.IsCancellationRequested, "unsafe wait cancels provider and task: " + failure);
                    string expectedReason = failure == "fault" ? "故障" : failure == "bad_frame" ? "无效" : failure == "external_pid" ? "六参" :
                        failure == "external_step" ? "ID" : failure == "home_target" ? "零位" : "停滞";
                    Check(rejection.Contains(expectedReason), "wait reports the injected cause: " + failure);
                    Check(transport.Count("YAW OFF") == 1, "unsafe wait issues one OFF: " + failure);
                    int afterStop = transport.CommandCount;
                    // The provider deliberately ignores its cancellation and replies late.
                    // That response has no route to a motor command.
                    lateResponse.SetResult(Scan("PKP", 0, 3));
                    Thread.Sleep(30);
                    Check(transport.CommandCount == afterStop && transport.Count("YAW STEP 30") == 0 && transport.Count("YAW STEP -30") == 0,
                        "late proposal cannot enqueue any motion after stop: " + failure);
                }
            }
        }

        // Optional sustained scenario: real wall-clock 5 second captures in both
        // directions, then a fake model stop and an independently checked restore.
        public static int RunScenario()
        {
            count = 0;
            ObservedTransport transport = new ObservedTransport();
            using (Workbench bench = new Workbench(DirectoryFor("ai_model_stop")))
            {
                Wait(bench.ConnectAsync(transport), 5000);
                Pid anchor = bench.Snapshot().CurrentPid.Copy();
                using (FakeAdvisor advisor = new FakeAdvisor())
                using (AutoTuneCoordinator coordinator = new AutoTuneCoordinator(bench, advisor, new AutoTuneOptions { Condition = "offline plant" }))
                {
                    Wait(coordinator.RunAsync(), 25000);
                    AutoTuneSnapshot outcome = coordinator.Snapshot();
                    Check(advisor.Calls == 1, "complete failing baseline reaches model once");
                    Check(outcome.TrialsUsed == 2 && outcome.Decisions == 1, "normal stop counts both complete baseline steps");
                    Check(!outcome.Verified && !outcome.Running, "model stop is normal unfinished end");
                    Check(Protocol.SamePid(anchor, bench.Snapshot().CurrentPid), "normal model stop restores actual startup anchor");
                    Check(transport.Count("YAW OFF") == 0, "normal model stop does not disable healthy prepared hardware");
                    Check(transport.Count("YAW STEP 30") == 1 && transport.Count("YAW STEP -30") == 1, "baseline is exactly one full pair");
                    List<TrialRecord> records = bench.Snapshot().Records;
                    Check(records.Count == 2 && records[0].Assessment.BoardComplete && records[0].Assessment.WaveComplete &&
                        records[1].Assessment.BoardComplete && records[1].Assessment.WaveComplete, "both complete real-time 5 second waves are retained");
                    Check(File.Exists(Path.Combine(outcome.Directory, "anchor.json")) && File.Exists(Path.Combine(outcome.Directory, "report.json")) &&
                        File.Exists(Path.Combine(outcome.Directory, "trace.jsonl")), "trace, actual anchor and normal report are saved");
                    Check(!File.ReadAllText(Path.Combine(outcome.Directory, "report.json")).Contains("ApiKey"), "report has no API credential field");
                    string report = File.ReadAllText(Path.Combine(outcome.Directory, "report.json"));
                    Check(report.Contains("\"TrialsUsed\":2") && report.Contains("\"condition\":\"offline plant\"") && report.Contains("\"results\":[{"),
                        "report contains real fields and per-trial evidence, not an empty anonymous object");
                    Check(File.ReadAllText(Path.Combine(outcome.Directory, "anchor.json")).Contains("\"Values\":[") &&
                        File.ReadAllText(Path.Combine(outcome.Directory, "trace.jsonl")).Contains("\"kind\":\"proposal\""), "startup anchor and structured decisions are readable");
                }
            }
            return count;
        }

        private sealed class FakeAdvisor : IAiAdvisor
        {
            public int Calls;
            public Task<AiProposal> ProposeAsync(AiContext context, CancellationToken token)
            { token.ThrowIfCancellationRequested(); Calls++; return Task.FromResult(new AiProposal { Action = "stop", Reason = "offline test complete" }); }
            public void Dispose() { }
        }

        private sealed class ObservedTransport : ITransport
        {
            private readonly SimulatedTransport inner = new SimulatedTransport();
            private readonly List<string> commands = new List<string>();
            private readonly object incoming = new object();
            private string latestFrame;
            private bool freeze, suppress;
            public ObservedTransport()
            { inner.LineReceived += delegate(string line) {
                    lock (incoming)
                    {
                        if (line.StartsWith("gimbal:", StringComparison.Ordinal))
                        {
                            if (suppress) return;
                            if (freeze && latestFrame != null) line = latestFrame; else latestFrame = line;
                        }
                    }
                    Emit(line);
                };
                inner.Failed += delegate(string error) { Action<string> handler = Failed; if (handler != null) handler(error); }; }
            public bool IsOpen { get { return inner.IsOpen; } }
            public bool IsDemo { get { return true; } }
            public event Action<string> LineReceived;
            public event Action<string> Failed;
            public void Open() { inner.Open(); }
            public void Send(string command) { lock (commands) commands.Add(command); inner.Send(command); }
            public int Count(string exact) { lock (commands) return commands.FindAll(delegate(string item) { return item == exact; }).Count; }
            public int CountPrefix(string prefix) { lock (commands) return commands.FindAll(delegate(string item) { return item.StartsWith(prefix, StringComparison.Ordinal); }).Count; }
            public int CommandCount { get { lock (commands) return commands.Count; } }
            public void FreezeFeedback() { lock (incoming) freeze = true; }
            public void SuppressFeedback() { lock (incoming) suppress = true; }
            public void InjectTransient(string kind)
            {
                string healthy;
                lock (incoming) { suppress = true; healthy = latestFrame; }
                // Let an already-dispatched simulator callback finish. Never
                // hold a transport lock across a receiver callback or OFF reply.
                Thread.Sleep(30);
                if (kind == "bad_frame") Emit("gimbal:1,2");
                else
                {
                    Frame frame;
                    if (!Protocol.TryFrame(healthy, 0, out frame)) throw new Exception("No healthy frame for guard test.");
                    if (kind == "fault") frame.Values[18] = 1;
                    else if (kind == "external_pid") frame.Values[7] *= 1.01;
                    else if (kind == "external_step") frame.Values[24] += 1;
                    else if (kind == "home_target") frame.Values[1] = 1;
                    StringBuilder line = new StringBuilder("gimbal:");
                    for (int i = 0; i < 72; i++) { if (i != 0) line.Append(','); line.Append(frame.Values[i].ToString("0.######", CultureInfo.InvariantCulture)); }
                    Emit(line.ToString());
                }
                // Immediately return to the original healthy snapshot; a poll
                // every 100 ms would miss this deliberately brief violation.
                Emit(healthy);
            }
            private void Emit(string line) { Action<string> handler = LineReceived; if (handler != null) handler(line); }
            public void Close() { inner.Close(); }
            public void Dispose() { inner.Dispose(); }
        }
    }
}
