using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

namespace YawTuner
{
    public sealed class Workbench : IDisposable
    {
        private sealed class Pending
        {
            public readonly ManualResetEvent Signal = new ManualResetEvent(false);
            public Func<Reply, bool> Match;
            public Reply Reply;
            public Exception Error;
            public long After;
        }
        private readonly object gate = new object();
        private readonly object sendGate = new object();
        private readonly object cleanupGate = new object();
        private readonly string dataRoot;
        private readonly Stopwatch clock = Stopwatch.StartNew();
        private readonly WorkbenchSnapshot state = new WorkbenchSnapshot();
        private ITransport transport;
        private SessionStore store;
        private Pending pending;
        private CancellationTokenSource cancellation;
        private bool disposed, stopLatch, offSent, monitorHealth, changingPid, desynchronized, guardHome;
        private bool compatibleVersion, compatibleView, compatibleCriteria;
        private long receiveSequence, lastFrameHost = -1, lastStampChangeHost = -1;
        private uint lastStamp, knownId, previousId, expectedId;
        private bool stepArmed, sawNewId;
        private Pid expectedPid;
        private TrialRecord capture;
        private StepResult lastResult;
        private string abortReason;
        private Task activeOperation;
        private string activeRunId;
        public event Action Changed;
        public event Action<string> Log;

        public Workbench(string root)
        {
            if (string.IsNullOrWhiteSpace(root)) throw new ArgumentException("需要数据目录。");
            dataRoot = Path.GetFullPath(root);
            state.Stage = "未连接"; state.Detail = "请连接串口或进入明确标记的演示模式。";
        }

        public static string[] Ports() { return SerialTransport.Ports(); }

        public WorkbenchSnapshot Snapshot()
        {
            lock (gate)
            {
                return new WorkbenchSnapshot
                {
                    Connected = state.Connected, Demo = state.Demo, Busy = state.Busy,
                    Stage = state.Stage, Detail = state.Detail, SessionDirectory = state.SessionDirectory,
                    LastError = state.LastError, Latest = Copy(state.Latest),
                    CurrentPid = state.CurrentPid == null ? null : state.CurrentPid.Copy(),
                    Baseline = state.Baseline == null ? null : state.Baseline.Copy(),
                    Status = Copy(state.Status), StepCheck = Copy(state.StepCheck),
                    Completed = state.Completed, Total = state.Total, FramesReceived = state.FramesReceived,
                    // Archived records are immutable after publication; share them with the UI.
                    // Copying thousands of 72-channel arrays every 100 ms would stall acquisition.
                    InvalidLines = state.InvalidLines, Records = new List<TrialRecord>(state.Records)
                };
            }
        }

        public Task ConnectAsync(ITransport value)
        {
            if (value == null) throw new ArgumentNullException("value");
            bool staleConnection;
            lock (gate)
            {
                ThrowDisposed();
                if (state.Connected || state.Busy) throw new InvalidOperationException("请先断开当前会话。");
                staleConnection = transport != null || store != null;
            }
            // A failed transport is no longer Connected. Clean its retained resources before
            // allowing a new connection, without issuing any implicit preparation commands.
            if (staleConnection) CleanupConnection();
            lock (gate)
            {
                ThrowDisposed();
                if (state.Connected || state.Busy || transport != null) throw new InvalidOperationException("请先断开当前会话。");
                transport = value;
                state.Busy = true; state.LastError = null; state.Stage = "连接";
                stopLatch = offSent = desynchronized = false;
                compatibleVersion = compatibleView = compatibleCriteria = false;
                abortReason = null; cancellation = new CancellationTokenSource();
                lastFrameHost = lastStampChangeHost = -1; receiveSequence = 0;
                state.Records.Clear(); state.FramesReceived = state.InvalidLines = 0;
                state.Completed = state.Total = 0; state.CurrentPid = state.Baseline = null;
                state.Latest = null; lastResult = null; expectedPid = null; stepArmed = false;
            }
            Notify();
            Task operation = Task.Factory.StartNew(delegate
            {
                try
                {
                    store = new SessionStore(dataRoot, value.IsDemo); store.Failed += OnStoreFailed;
                    lock (gate) { state.SessionDirectory = store.DirectoryPath; state.Demo = value.IsDemo; }
                    value.LineReceived += OnLine; value.Failed += OnTransportFailed;
                    value.Open();
                    lock (gate) state.Connected = true;
                    CancellationToken token = cancellation.Token;
                    Reply view = Request("YAW VIEW", delegate(Reply r) { return r.Kind == "VIEW"; }, token, 2500);
                    compatibleView = ViewMatches(view);
                    Request("TELEM ON", delegate(Reply r) { return IsOk(r, "TELEM"); }, token, 2000);
                    RefreshCore(token);
                    lock (gate)
                    {
                        state.Baseline = state.CurrentPid.Copy();
                        state.Stage = "已连接";
                        state.Detail = CompatibilityText() + "；已保存板端回读基线，未改变使能或零点。";
                    }
                    SaveSummaries();
                    Emit(state.Detail);
                }
                catch (Exception ex)
                {
                    lock (gate) { state.LastError = ex.Message; state.Stage = "连接失败"; state.Detail = ex.Message; }
                    CleanupConnection();
                    throw;
                }
                finally { lock (gate) state.Busy = false; Notify(); }
            }, CancellationToken.None, TaskCreationOptions.None, TaskScheduler.Default);
            lock (gate) activeOperation = operation;
            return operation;
        }

        public Task RefreshAsync()
        {
            return Operation("读取状态", false, delegate(CancellationToken token) { RefreshCore(token); SetStage("已连接", CompatibilityText()); });
        }

        public Task ApplyAsync(Pid gains)
        {
            Protocol.ValidatePid(gains); Pid desired = gains.Copy();
            return Operation("应用参数", true, delegate(CancellationToken token)
            {
                RequireCompatible(); EnsureTelemetry(token);
                HomeCore(token); WritePid(desired, token); CheckReady(token, 3000);
                SetStage("参数已应用", "六参已回读确认，仅在 RAM 生效；连接基线保持不变。");
            });
        }

        public Task RestoreBaselineAsync()
        {
            Pid baseline;
            lock (gate) baseline = state.Baseline == null ? null : state.Baseline.Copy();
            if (baseline == null) throw new InvalidOperationException("没有已回读的连接基线。");
            return Operation("恢复连接基线", false, delegate(CancellationToken token)
            {
                RequireCompatible();
                Reply status = Request("YAW STATUS", delegate(Reply r) { return r.Kind == "STATUS"; }, token, 2000);
                bool active = status.Get("ACTIVE") == "1";
                if (active)
                {
                    lock (gate) monitorHealth = true;
                    EnsureTelemetry(token); HomeCore(token);
                }
                else if (status.Get("ACTIVE") != "0" || status.Get("REQUESTED") != "0" || status.Get("STOP_PENDING") != "0")
                    throw new InvalidOperationException("恢复基线需要已稳定停机或健康活动态；请等待停止过渡完成。");
                WritePid(baseline, token);
                if (active) CheckReady(token, 3000);
                SetStage("基线已恢复", active ? "已归零并恢复连接时六参。" : "已在停机状态独立恢复连接时六参；请按固件流程人工重新使能。");
            });
        }

        public Task HomeAsync()
        {
            return Operation("归零", true, delegate(CancellationToken token)
            { RequireCompatible(); EnsureTelemetry(token); HomeCore(token); SetStage("归零就绪", "已回到原软件零点并通过 CONSOLE_READY 检查。"); });
        }

        // Keep the same exclusive, continuously monitored operation active while
        // a model is responding. Even a single transient unsafe RX frame cancels
        // the request; the latest UI snapshot alone could miss that frame.
        public async Task<T> GuardedWaitAsync<T>(Pid expected, Func<CancellationToken, Task<T>> action, CancellationToken externalToken)
        {
            Protocol.ValidatePid(expected);
            if (action == null) throw new ArgumentNullException("action");
            externalToken.ThrowIfCancellationRequested();
            T result = default(T);
            using (externalToken.Register(Stop))
                await Operation("等待 AI 分析", true, delegate(CancellationToken operationToken)
                {
                    using (CancellationTokenSource linked = CancellationTokenSource.CreateLinkedTokenSource(operationToken, externalToken))
                    {
                        linked.Token.ThrowIfCancellationRequested();
                        Frame beforeWait;
                        lock (gate)
                        {
                            expectedPid = expected.Copy(); beforeWait = state.Latest;
                            if (!Protocol.SamePid(state.CurrentPid, expected))
                                throw new InvalidOperationException("等待 AI 前六参已发生外部变化。");
                        }
                        // Command ACK/readback can precede the next telemetry snapshot.
                        // Wait for one fresh frame under the normal fault/PID monitor.
                        while (true)
                        {
                            bool fresh; lock (gate) fresh = state.Latest != null && !object.ReferenceEquals(state.Latest, beforeWait);
                            if (fresh) break;
                            Pause(10, linked.Token);
                        }
                        lock (gate)
                        {
                            guardHome = true;
                            string homeReason = HomeHoldReason(state.Latest);
                            if (homeReason.Length != 0) throw new InvalidOperationException(homeReason);
                        }
                        Task<T> response = action(linked.Token);
                        if (response == null) throw new InvalidOperationException("AI 接口没有返回任务。");
                        try
                        {
                            while (!response.IsCompleted) { linked.Token.ThrowIfCancellationRequested(); CheckHealth(); Pause(25, linked.Token); }
                            linked.Token.ThrowIfCancellationRequested(); CheckHealth();
                            result = response.GetAwaiter().GetResult();
                            linked.Token.ThrowIfCancellationRequested(); CheckHealth();
                        }
                        finally
                        {
                            if (!response.IsCompleted) response.ContinueWith(delegate(Task<T> late) { var ignored = late.Exception; }, CancellationToken.None, TaskContinuationOptions.OnlyOnFaulted, TaskScheduler.Default);
                        }
                    }
                }).ConfigureAwait(false);
            return result;
        }

        private static string HomeHoldReason(Frame frame)
        {
            if (Math.Abs(frame.Target) > .20001 || Math.Abs(frame.Angle) > .20001 || Math.Abs(frame.Values[3]) > 6 || Math.Abs(frame.Values[4]) > 30)
                return "等待 AI 时目标/反馈离开零位静止范围，已取消后续动作。";
            return "";
        }

        public Task RunAsync(TrialPlan plan)
        {
            if (plan == null || plan.Trials == null || plan.Trials.Count == 0) throw new ArgumentException("试验计划为空。");
            TrialPlan fixedPlan = new TrialPlan { Condition = plan.Condition ?? "", Acceptance = plan.Acceptance };
            foreach (TrialSpec spec in plan.Trials)
            {
                if (spec == null || (spec.Direction != 1 && spec.Direction != -1) || spec.Repeat < 1) throw new ArgumentException("计划方向或重复编号无效。");
                Protocol.ValidatePid(spec.Gains);
                fixedPlan.Trials.Add(new TrialSpec { Candidate = spec.Candidate, Direction = spec.Direction, Repeat = spec.Repeat, Gains = spec.Gains.Copy() });
            }
            return Operation("试验准备", true, delegate(CancellationToken token)
            {
                RequireCompatible(); EnsureTelemetry(token);
                Reply initial = Request("YAW RESULT", IsStep, token, 2000);
                StepResult result = Protocol.ParseResult(initial);
                if (result.State == "RUNNING") throw new InvalidOperationException("板端已有进行中的阶跃，请等待或人工停止后重试。");
                knownId = result.Id;
                Pid runBaseline = ReadPid(token);
                activeRunId = Guid.NewGuid().ToString("N");
                lock (gate) { state.Total = fixedPlan.Trials.Count; state.Completed = 0; }
                StoreEvent("RUN_BASELINE " + JsonUtil.Encode(runBaseline));
                foreach (TrialSpec spec in fixedPlan.Trials)
                {
                    token.ThrowIfCancellationRequested();
                    HomeCore(token);
                    WritePid(spec.Gains, token);
                    CheckReady(token, 4000);
                    RunTrial(spec, fixedPlan, token);
                    lock (gate) state.Completed++;
                    Notify();
                }
                HomeCore(token);
                SetStage("恢复运行前参数", "正常结束归零后，恢复本次运行开始时的六参。");
                WritePid(runBaseline, token); CheckReady(token, 4000);
                SaveSummaries();
                SetStage("计划完成", state.Demo ? "演示完成，模拟结果不构成实机验收。" : "计划完成并恢复运行前六参；通过与否见各项硬条件和完整性报告。");
            });
        }

        private Task Operation(string name, bool safety, Action<CancellationToken> action)
        {
            CancellationTokenSource source;
            lock (gate)
            {
                ThrowDisposed();
                if (!state.Connected || transport == null || !transport.IsOpen) throw new InvalidOperationException("请先连接。");
                if (state.Busy) throw new InvalidOperationException("当前操作尚未结束；停止会取消整个计划。");
                if (desynchronized) throw new InvalidOperationException("此前命令超时，串口应答已失去同步；请断开后重新连接。");
                state.Busy = true; state.Stage = name; state.Detail = name; state.LastError = null;
                abortReason = null; stopLatch = offSent = false;
                source = new CancellationTokenSource(); cancellation = source;
                monitorHealth = safety; changingPid = false;
                expectedPid = state.CurrentPid == null ? null : state.CurrentPid.Copy();
            }
            Notify();
            Task operation = Task.Factory.StartNew(delegate
            {
                try { action(source.Token); }
                catch (Exception ex)
                {
                    string reason;
                    lock (gate) reason = abortReason ?? (ex is OperationCanceledException ? "用户停止，后续命令已取消。" : ex.Message);
                    bool requiresStop;
                    lock (gate) requiresStop = safety || monitorHealth;
                    if (requiresStop) Abort(reason);
                    lock (gate) { state.LastError = reason; state.Stage = "已停止"; state.Detail = reason + "；恢复参数与重新使能需要独立人工操作。"; }
                    SaveInterrupted(reason);
                    try { SaveSummaries(); } catch (Exception disk) { Emit("保存失败：" + disk.Message); }
                    if (ex is OperationCanceledException && abortReason == null) throw new OperationCanceledException(reason);
                    throw new InvalidOperationException(reason, ex);
                }
                finally
                {
                    lock (gate)
                    {
                        monitorHealth = changingPid = stepArmed = guardHome = false; capture = null; state.Busy = false;
                        if (cancellation == source) cancellation = null;
                    }
                    source.Dispose();
                    Notify();
                }
            }, CancellationToken.None, TaskCreationOptions.None, TaskScheduler.Default);
            lock (gate) activeOperation = operation;
            return operation;
        }

        private void RefreshCore(CancellationToken token)
        {
            Reply status = Request("YAW STATUS", delegate(Reply r) { return r.Kind == "STATUS"; }, token, 2000);
            lock (gate)
            {
                state.Status = status;
                compatibleVersion = (status.Get("VERSION") == "GL60II_FIXED_PID_V2P" ||
                    (state.Demo && status.Get("VERSION") == "DEMO_CASCADE_V1")) && status.Get("CTRL") == "CASCADE";
            }
            ReadPid(token);
            Reply step = Request("YAW RESULT", IsStep, token, 2000);
            StepResult rstep = Protocol.ParseResult(step);
            lock (gate)
            {
                // Freshly booted firmware has an IDLE report before cascade metadata is populated.
                // Only an exact V2P STATUS + official 72-column VIEW may establish that initial state.
                compatibleCriteria = rstep.CriteriaMatch || (rstep.Id == 0 && rstep.State == "IDLE" && compatibleVersion && compatibleView);
                knownId = rstep.Id;
            }
            Request("YAW STEP CHECK", delegate(Reply r) { return r.Kind == "STEP_CHECK"; }, token, 2000);
        }

        private Pid ReadPid(CancellationToken token)
        {
            Reply reply = Request("YAW PID", delegate(Reply r) { return r.Kind == "PID"; }, token, 2000);
            Pid gains = Protocol.ReadPid(reply); Protocol.ValidatePid(gains);
            lock (gate) { state.CurrentPid = gains.Copy(); expectedPid = gains.Copy(); }
            return gains;
        }

        private void WritePid(Pid gains, CancellationToken token)
        {
            Protocol.ValidatePid(gains);
            SetStage("写入与回读六参", "按六个单参数命令写入 RAM，并核对完整回读。");
            lock (gate) changingPid = true;
            try
            {
                for (int i = 0; i < 6; i++) Request(Protocol.ParameterCommand(i, gains.Values[i]), delegate(Reply r) { return IsOk(r, "PARAMETER"); }, token, 2000);
                Pid readback = ReadPid(token);
                if (!Protocol.SamePid(gains, readback)) throw new InvalidOperationException("六参回读与请求不一致，停止实验。");
            }
            finally { lock (gate) changingPid = false; }
        }

        private void HomeCore(CancellationToken token)
        {
            SetStage("等待归零", "下发 HOME；等待零位、静止、遥控与使能门禁，不改变软件零点。");
            Request("YAW HOME", delegate(Reply r) { return IsOk(r, "HOME"); }, token, 2000);
            CheckReady(token, 15000);
        }

        private void CheckReady(CancellationToken token, int timeout)
        {
            long end = clock.ElapsedMilliseconds + timeout;
            string why = "尚未收到 STEP CHECK";
            while (clock.ElapsedMilliseconds < end)
            {
                Reply reply = Request("YAW STEP CHECK", delegate(Reply r) { return r.Kind == "STEP_CHECK"; }, token, 2000);
                if (reply.Get("CONSOLE_READY") == "1") return;
                why = reply.Get("WHY") + "；BLOCKS=" + reply.Get("BLOCKS") + "；CONSOLE_READY=" + reply.Get("CONSOLE_READY");
                SetDetail("等待试验就绪：" + why);
                Pause(150, token);
            }
            throw new TimeoutException("归零/静止/遥控就绪超时：" + why);
        }

        private void RunTrial(TrialSpec spec, TrialPlan plan, CancellationToken token)
        {
            Reply before = Request("YAW RESULT", IsStep, token, 2000);
            StepResult old = Protocol.ParseResult(before);
            if (old.Id != knownId || old.State == "RUNNING") throw new InvalidOperationException("板端阶跃 ID 在试验前发生外部变化。");
            TrialRecord record = new TrialRecord
            {
                TrialId = DateTime.Now.ToString("yyyyMMdd_HHmmss_fff", CultureInfo.InvariantCulture) + "_" + Guid.NewGuid().ToString("N").Substring(0, 6),
                Candidate = spec.Candidate, Mode = state.Demo ? "DEMO" : "HARDWARE",
                RunId = activeRunId, Acceptance = plan.Acceptance,
                StartedAt = DateTimeOffset.Now.ToString("o", CultureInfo.InvariantCulture), Condition = plan.Condition,
                Direction = spec.Direction, Repeat = spec.Repeat, Gains = spec.Gains.Copy()
            };
            lock (gate)
            {
                previousId = old.Id; expectedId = unchecked(old.Id + 1U); sawNewId = false; stepArmed = true;
                capture = record;
                if (state.Latest != null) capture.Frames.Add(Copy(state.Latest));
            }
            long startedHost = clock.ElapsedMilliseconds;
            SetStage("阶跃命令确认", spec.Candidate + " " + (spec.Direction > 0 ? "+30°" : "-30°") + "，重复 " + spec.Repeat);
            Reply ack = Request(spec.Direction > 0 ? "YAW STEP 30" : "YAW STEP -30", delegate(Reply r) { return IsOk(r, "STEP_STARTED"); }, token, 2000);
            if (Math.Abs(ack.Number("TARGET") - spec.Direction * 30.0) > 0.0001 || ack.Get("WINDOW_MS") != "5000")
                throw new InvalidOperationException("阶跃 ACK 的目标或 5 秒窗口不符。");
            StepResult result = Protocol.ParseResult(Request("YAW RESULT", IsStep, token, 2000));
            ValidateRunningResult(result, spec);
            SetStage("采集完整 5 秒", "等待本轮新 ID=" + expectedId + " 的 DONE；保留所有遥测快照和重复板端戳。");
            long deadline = startedHost + 10000;
            while (true)
            {
                token.ThrowIfCancellationRequested(); CheckHealth();
                if (result.State == "ABORTED") throw new InvalidOperationException("板端中止本轮：" + result.Reason);
                if (result.State == "DONE")
                {
                    if (clock.ElapsedMilliseconds - startedHost < 4900) throw new InvalidOperationException("过早收到 DONE，疑似陈旧成绩或时钟/协议异常。");
                    break;
                }
                if (result.State != "RUNNING") throw new InvalidOperationException("本轮阶跃状态异常：" + result.State);
                if (clock.ElapsedMilliseconds >= deadline) throw new TimeoutException("等待本轮完整 5 秒 DONE 超时，ID=" + expectedId);
                Pause(150, token);
                lock (gate)
                {
                    if (lastResult != null && lastResult.Id == expectedId && (lastResult.State == "DONE" || lastResult.State == "ABORTED")) result = lastResult;
                }
                if (result.State == "RUNNING") result = Protocol.ParseResult(Request("YAW RESULT", IsStep, token, 2000));
                ValidateRunningResult(result, spec);
            }
            // Let the next telemetry snapshot arrive after DONE, so the acquisition includes its endpoint.
            long afterDone = clock.ElapsedMilliseconds;
            while (clock.ElapsedMilliseconds - afterDone < 200)
            {
                bool havePost;
                lock (gate) havePost = state.Latest != null && state.Latest.HostMs > afterDone && state.Latest.StepId == expectedId;
                if (havePost) break;
                Pause(10, token);
            }
            lock (gate)
            {
                record.Result = result; capture = null; stepArmed = false; knownId = result.Id;
            }
            record.Assessment = Evaluation.Assess(record);
            lock (gate) state.Records.Add(record);
            store.SaveTrial(record); SaveSummaries();
            Emit("完成 " + record.TrialId + "：板端完整=" + record.Assessment.BoardComplete + "，电脑波形完整=" + record.Assessment.WaveComplete + "，硬条件通过=" + record.Assessment.Pass);
        }

        private void ValidateRunningResult(StepResult result, TrialSpec spec)
        {
            if (result.Id != expectedId) throw new InvalidOperationException("阶跃 ID 混乱：期待新 ID=" + expectedId + "，收到=" + result.Id + "；不会使用旧成绩。");
            if (!result.CriteriaMatch || Math.Abs(result.Target - spec.Direction * 30.0) > 0.001 || !Protocol.SamePid(result.Gains, spec.Gains))
                throw new InvalidOperationException("本轮成绩口径、目标或六参不一致。");
            if (result.Fault != 0) throw new InvalidOperationException("板端成绩报告故障=" + result.Fault);
        }

        private Reply Request(string command, Func<Reply, bool> match, CancellationToken token, int timeout)
        {
            Pending request = new Pending { Match = match };
            try
            {
                lock (gate)
                {
                    token.ThrowIfCancellationRequested();
                    if (pending != null) throw new InvalidOperationException("内部命令并发被拒绝。");
                    request.After = receiveSequence; pending = request;
                }
                SendChecked(command, token);
                long deadline = clock.ElapsedMilliseconds + timeout;
                while (!request.Signal.WaitOne(25))
                {
                    token.ThrowIfCancellationRequested(); CheckHealth();
                    if (clock.ElapsedMilliseconds >= deadline)
                    {
                        lock (gate) desynchronized = true;
                        throw new TimeoutException("命令确认超时：" + command + "；为避免陈旧 ACK，必须重新连接后才可继续。");
                    }
                }
                token.ThrowIfCancellationRequested();
                if (request.Error != null) throw request.Error;
                if (request.Reply.Kind == "ERROR") throw new InvalidOperationException("板端拒绝 " + command + "：" + request.Reply.Raw);
                return request.Reply;
            }
            finally
            {
                lock (gate) { if (pending == request) pending = null; }
                request.Signal.Dispose();
            }
        }

        private void SendChecked(string command, CancellationToken token)
        {
            if (!Protocol.AllowedCommand(command)) throw new InvalidOperationException("命令未通过固定白名单：" + command);
            lock (sendGate)
            {
                token.ThrowIfCancellationRequested();
                ITransport output;
                lock (gate)
                {
                    if (stopLatch) throw new OperationCanceledException("停止后禁止后续队列命令。");
                    output = transport;
                }
                if (output == null || !output.IsOpen) throw new IOException("串口已断开。");
                CheckHealth();
                if (store != null) store.WriteRaw(clock.ElapsedMilliseconds, "TX", command);
                Emit("TX  " + command);
                output.Send(command);
            }
        }

        private void OnLine(string line)
        {
            string problem = null;
            try
            {
                long now = clock.ElapsedMilliseconds;
                SessionStore target = store;
                if (target != null) target.WriteRaw(now, "RX", line);
                Frame frame;
                if (Protocol.TryFrame(line, now, out frame))
                {
                    if (target != null) target.WriteFrame(frame);
                    lock (gate)
                    {
                        bool stampReversed = lastStampChangeHost >= 0 && frame.Stamp != lastStamp && unchecked(frame.Stamp - lastStamp) >= 0x80000000U;
                        receiveSequence++; state.Latest = frame; state.FramesReceived++; lastFrameHost = now;
                        if (!state.Busy && !monitorHealth)
                        {
                            // Keep idle displays and the next operation's baseline tied to live
                            // feedback, including deliberate changes made from another console.
                            state.CurrentPid = frame.Gains; knownId = frame.StepId;
                        }
                        if (lastStampChangeHost < 0 || frame.Stamp != lastStamp) { lastStamp = frame.Stamp; lastStampChangeHost = now; }
                        if (capture != null) capture.Frames.Add(frame);
                        if (monitorHealth)
                        {
                            problem = Protocol.HealthReason(frame);
                            if (string.IsNullOrEmpty(problem) && stampReversed) problem = "板端反馈时间戳倒序，已取消后续动作。";
                            if (string.IsNullOrEmpty(problem) && guardHome) problem = HomeHoldReason(frame);
                            if (string.IsNullOrEmpty(problem) && !changingPid && expectedPid != null && !Protocol.SamePid(frame.Gains, expectedPid)) problem = "运行中六参被外部改变。";
                            if (string.IsNullOrEmpty(problem) && stepArmed)
                            {
                                if (frame.StepId == expectedId) sawNewId = true;
                                else if (frame.StepId != previousId || sawNewId) problem = "遥测阶跃 ID 混乱，停止计划。";
                            }
                            else if (string.IsNullOrEmpty(problem) && frame.StepId != knownId) problem = "试验队列期间检测到外部阶跃 ID 变化。";
                        }
                    }
                    if (!string.IsNullOrEmpty(problem)) Abort(problem);
                    return;
                }
                Reply reply = Protocol.ParseReply(line);
                if (reply != null) Emit("RX  " + line);
                if (reply == null)
                {
                    bool active;
                    long invalid;
                    lock (gate) { state.InvalidLines++; invalid = state.InvalidLines; active = monitorHealth; }
                    string trimmed = line == null ? "" : line.TrimStart();
                    bool recognized = trimmed.StartsWith("gimbal:", StringComparison.Ordinal) || trimmed.StartsWith("Y|", StringComparison.Ordinal) || trimmed.StartsWith("C|", StringComparison.Ordinal);
                    if (recognized)
                    {
                        if (active) HandleFailure("收到无效遥测或协议帧，停止本轮；原始行已保留。", false);
                        else if (invalid <= 5 || invalid % 100 == 0) Emit("收到无效遥测或协议帧，已保存原始行（只读会话）。");
                    }
                    return;
                }
                lock (gate)
                {
                    receiveSequence++;
                    if (reply.Kind == "STATUS") state.Status = reply;
                    if (reply.Kind == "STEP_CHECK") state.StepCheck = reply;
                    if (reply.Kind == "STEP")
                    {
                        StepResult result = Protocol.ParseResult(reply); lastResult = result;
                        if (monitorHealth && stepArmed && result.Id != previousId && result.Id != expectedId) problem = "成绩阶跃 ID 混乱。";
                        else if (monitorHealth && result.State == "ABORTED" && stepArmed && result.Id == expectedId) problem = "板端中止：" + result.Reason;
                    }
                    if (pending != null && receiveSequence > pending.After && (reply.Kind == "ERROR" || pending.Match(reply)))
                    {
                        pending.Reply = reply; pending.Signal.Set();
                    }
                }
                if (!string.IsNullOrEmpty(problem)) Abort(problem);
            }
            catch (Exception ex) { HandleFailure("接收、解析或记录失败：" + ex.Message, false); }
        }

        private void EnsureTelemetry(CancellationToken token)
        {
            long end = clock.ElapsedMilliseconds + 2000;
            while (true)
            {
                bool have;
                lock (gate) have = state.Latest != null;
                if (have) { CheckHealth(); return; }
                if (clock.ElapsedMilliseconds >= end) throw new TimeoutException("未收到 72 通道遥测；无法开始受监控操作。");
                Pause(25, token);
            }
        }

        private void CheckHealth()
        {
            string reason = null;
            lock (gate)
            {
                if (!monitorHealth) return;
                if (!state.Connected || transport == null || !transport.IsOpen) reason = "连接已断开。";
                else if (lastFrameHost >= 0 && clock.ElapsedMilliseconds - lastFrameHost > 350) reason = "接收停滞：超过 350 ms 没有完整遥测。";
                else if (lastStampChangeHost >= 0 && clock.ElapsedMilliseconds - lastStampChangeHost >= 150) reason = "反馈停滞：板端反馈时间戳超过 150 ms 未更新。";
                else if (state.Latest != null)
                {
                    reason = Protocol.HealthReason(state.Latest);
                    if (string.IsNullOrEmpty(reason) && guardHome) reason = HomeHoldReason(state.Latest);
                }
            }
            if (!string.IsNullOrEmpty(reason)) { Abort(reason); throw new InvalidOperationException(reason); }
        }

        private void Pause(int milliseconds, CancellationToken token)
        {
            long until = clock.ElapsedMilliseconds + milliseconds;
            while (clock.ElapsedMilliseconds < until)
            {
                token.ThrowIfCancellationRequested(); CheckHealth();
                token.WaitHandle.WaitOne((int)Math.Min(25, Math.Max(1, until - clock.ElapsedMilliseconds)));
            }
        }

        public void Stop() { Abort("用户停止：已取消后续试验，仅发送一次 YAW OFF。"); Notify(); }

        private void Abort(string reason)
        {
            CancellationTokenSource source;
            lock (gate)
            {
                if (abortReason == null) abortReason = reason;
                state.LastError = abortReason; state.Detail = abortReason; state.Stage = "停止";
                stopLatch = true; source = cancellation;
            }
            if (source != null) try { source.Cancel(); } catch (ObjectDisposedException) { }
            lock (sendGate)
            {
                ITransport output;
                lock (gate) { if (offSent) return; offSent = true; output = transport; }
                try { StoreEvent("STOP " + reason); } catch { }
                if (output != null && output.IsOpen && Protocol.AllowedCommand("YAW OFF"))
                {
                    try { if (store != null) store.WriteRaw(clock.ElapsedMilliseconds, "TX", "YAW OFF"); } catch { }
                    try { output.Send("YAW OFF"); } catch (Exception ex) { Emit("OFF 发送失败：" + ex.Message); }
                }
            }
            Emit(reason); Notify();
        }

        private void OnTransportFailed(string error)
        {
            HandleFailure(error, true);
            lock (gate) state.Connected = false;
            Notify();
        }
        private void OnStoreFailed(string error) { HandleFailure(error, true); }

        private void HandleFailure(string error, bool disconnected)
        {
            bool active;
            lock (gate) active = monitorHealth;
            if (active) { Abort(error); return; }
            // Inspection must remain passive even when an old/unknown board returns a
            // malformed report. Fail its pending request without sending a system OFF.
            lock (gate)
            {
                state.LastError = error; state.Stage = "会话错误"; state.Detail = error;
                desynchronized = true;
                if (disconnected) state.Connected = false;
                if (pending != null) { pending.Error = new IOException(error); pending.Signal.Set(); }
            }
            try { StoreEvent("PASSIVE_ERROR " + error); } catch { }
            Emit(error); Notify();
        }

        public void Disconnect()
        {
            bool shouldStop;
            lock (gate) shouldStop = state.Connected || state.Busy;
            if (shouldStop) Abort("断开连接，取消队列并请求 OFF。");
            Task operation;
            lock (gate) operation = activeOperation;
            if (operation != null && !operation.IsCompleted && Task.CurrentId != operation.Id)
                try { operation.Wait(5000); } catch (AggregateException) { }
            CleanupConnection();
            lock (gate) { state.Connected = false; state.Stage = "已断开"; state.Detail = "会话文件已关闭。"; }
            Notify();
        }

        private void CleanupConnection()
        {
            lock (cleanupGate)
            {
            ITransport old;
            lock (gate) { old = transport; state.Connected = false; }
            if (old != null)
            {
                old.LineReceived -= OnLine; old.Failed -= OnTransportFailed;
                try { old.Close(); } catch (Exception ex) { Emit("关闭串口：" + ex.Message); }
                try { old.Dispose(); } catch (Exception ex) { Emit("释放串口：" + ex.Message); }
            }
            lock (gate) transport = null;
            SessionStore oldStore = store;
            if (oldStore != null) { oldStore.Failed -= OnStoreFailed; oldStore.Dispose(); }
            store = null;
            }
        }

        public string ExportSummary()
        {
            lock (gate) { if (state.Busy) throw new InvalidOperationException("请等待当前操作完成后导出。"); }
            if (store != null) SaveSummaries();
            string directory;
            lock (gate) directory = state.SessionDirectory;
            if (string.IsNullOrEmpty(directory)) throw new InvalidOperationException("还没有会话数据。");
            return directory;
        }

        public void Export(string directory)
        {
            string source = ExportSummary();
            SessionStore current = store;
            if (current == null) SessionStore.ExportClosed(source, directory);
            else current.Export(directory);
        }

        private void SaveSummaries()
        {
            SessionStore current = store;
            if (current == null) return;
            List<TrialRecord> records;
            lock (gate) records = new List<TrialRecord>(state.Records);
            current.SaveSummaries(records); current.Flush();
        }

        private void SaveInterrupted(string reason)
        {
            TrialRecord partial;
            lock (gate)
            {
                partial = capture; capture = null; stepArmed = false;
                if (partial == null) return;
                if (lastResult != null && lastResult.Id == expectedId) partial.Result = lastResult;
                partial.Assessment = Evaluation.Assess(partial); partial.Assessment.Pass = false;
                partial.Assessment.Issues.Add("实验中断：" + reason);
                state.Records.Add(partial);
            }
            try { if (store != null) store.SaveTrial(partial); } catch (Exception ex) { Emit("中断数据保存失败：" + ex.Message); }
        }

        private void StoreEvent(string value) { if (store != null) store.WriteRaw(clock.ElapsedMilliseconds, "EVENT", value); }
        private void RequireCompatible()
        {
            lock (gate)
                if (!compatibleVersion || !compatibleView || !compatibleCriteria)
                    throw new InvalidOperationException("仅允许 V2P 串级 PID、72 通道及最终持续入带 250 ms/0.2°/5 秒口径；当前握手不匹配。");
        }
        private string CompatibilityText() { return compatibleVersion && compatibleView && compatibleCriteria ? "V2P/72 通道/最终持续入带口径已核对" : "协议或验收口径不匹配，试验被禁用"; }
        private static bool ViewMatches(Reply r)
        {
            return r.Get("COLS") == "72" && Near(r.Number("BAND_DEG"), 0.2) && Near(r.Number("TIME_LIMIT_MS"), 250) && Near(r.Number("OS_LIMIT_DEG"), 0.2) && Near(r.Number("SSE_LIMIT_DEG"), 0.2);
        }
        private static bool Near(double a, double b) { return !double.IsNaN(a) && Math.Abs(a - b) < 0.00001; }
        private static bool IsOk(Reply r, string action) { return r.Kind == "OK" && r.Get("ACTION") == action; }
        private static bool IsStep(Reply r) { return r.Kind == "STEP"; }
        private void SetStage(string stage, string detail) { lock (gate) { state.Stage = stage; state.Detail = detail; } StoreEvent("STAGE " + stage + " " + detail); Emit(detail); Notify(); }
        private void SetDetail(string detail) { lock (gate) state.Detail = detail; Notify(); }
        private void ThrowDisposed() { if (disposed) throw new ObjectDisposedException("Workbench"); }
        private void Notify() { Action handler = Changed; if (handler != null) try { handler(); } catch { } }
        private void Emit(string message) { Action<string> handler = Log; if (handler != null) try { handler(message); } catch { } }

        private static Frame Copy(Frame f) { return f == null ? null : new Frame { HostMs = f.HostMs, Values = (double[])f.Values.Clone() }; }
        private static Reply Copy(Reply r) { return r == null ? null : new Reply { Raw = r.Raw, Kind = r.Kind, Fields = new Dictionary<string, string>(r.Fields) }; }
        public void Dispose() { if (disposed) return; Disconnect(); disposed = true; }
    }
}
