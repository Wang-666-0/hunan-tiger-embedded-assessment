using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace YawTuner
{
    // The model proposes experiments. This local coordinator owns every motion,
    // bound, budget, comparison and independent acceptance decision.
    public sealed class AutoTuneCoordinator : IDisposable
    {
        internal sealed class Candidate
        {
            public Pid Gains;
            public readonly List<TrialRecord> Records = new List<TrialRecord>();
            public bool Pass;
            public double WorstTime, WorstOvershoot, WorstSse;
            public int FailedConditions;
        }

        private readonly object gate = new object();
        private readonly Workbench bench;
        private readonly IAiAdvisor advisor;
        private readonly AutoTuneOptions options;
        private readonly AutoTuneSnapshot state = new AutoTuneSnapshot();
        private readonly List<TrialRecord> records = new List<TrialRecord>();
        private CancellationTokenSource cancellation;
        private bool disposed, offRequested, used;
        private string session;
        private bool demo, simulatedSuccess;
        private static readonly Encoding Utf8 = new UTF8Encoding(false);
        public event Action<string> Log;

        public AutoTuneCoordinator(Workbench workbench, IAiAdvisor aiAdvisor, AutoTuneOptions settings)
        {
            if (workbench == null) throw new ArgumentNullException("workbench");
            if (aiAdvisor == null) throw new ArgumentNullException("aiAdvisor");
            if (settings == null) throw new ArgumentNullException("settings");
            ValidateOptions(settings);
            bench = workbench; advisor = aiAdvisor;
            options = new AutoTuneOptions { Condition = settings.Condition.Trim(), MaxTrials = settings.MaxTrials,
                MaxDecisions = settings.MaxDecisions, MaxChangePercent = settings.MaxChangePercent };
        }

        public AutoTuneSnapshot Snapshot()
        {
            lock (gate) return new AutoTuneSnapshot { Running = state.Running, Verified = state.Verified,
                Stage = state.Stage, Detail = state.Detail, Directory = state.Directory, TrialsUsed = state.TrialsUsed,
                Decisions = state.Decisions, Anchor = Copy(state.Anchor), Best = Copy(state.Best) };
        }

        public Task RunAsync()
        {
            lock (gate)
            {
                if (disposed) throw new ObjectDisposedException("AutoTuneCoordinator");
                if (used) throw new InvalidOperationException("每个 AI 运行器只能执行一次；请为新一轮创建运行器。");
                if (offRequested) throw new OperationCanceledException("运行器已停止。");
                used = true; state.Running = true; cancellation = new CancellationTokenSource();
            }
            return RunCoreAsync(cancellation.Token);
        }

        private async Task RunCoreAsync(CancellationToken token)
        {
            try
            {
                WorkbenchSnapshot connected = bench.Snapshot();
                if (!connected.Connected || connected.Busy) throw new InvalidOperationException("AI 调参需要已连接且空闲的会话。");
                session = connected.SessionDirectory; demo = connected.Demo;
                if (string.IsNullOrWhiteSpace(session)) throw new InvalidOperationException("没有可追溯的会话目录。");
                await BeginOperation(delegate { return bench.RefreshAsync(); }, token).ConfigureAwait(false);
                await WaitForFirstFrameAsync(token).ConfigureAwait(false);
                token.ThrowIfCancellationRequested();
                WorkbenchSnapshot fresh = CheckBoundary(null, token);
                Protocol.ValidatePid(fresh.CurrentPid);
                lock (gate)
                {
                    state.Anchor = fresh.CurrentPid.Copy(); state.Best = fresh.CurrentPid.Copy();
                    state.Directory = Path.Combine(session, "ai_runs", Guid.NewGuid().ToString("N"));
                }
                Directory.CreateDirectory(Snapshot().Directory);
                File.WriteAllText(Path.Combine(Snapshot().Directory, "anchor.json"), JsonUtil.Encode(new {
                    condition = options.Condition, demo = demo, anchor = fresh.CurrentPid, options = options,
                    note = "真实启动回读；仅 RAM 改参；无 API 密钥" }), Utf8);
                SetStage("测量启动基线", "固定真实启动六参，执行正负各一次完整 5 秒阶跃。");
                TrialPlan baseline = Evaluation.BuildPlan(fresh.CurrentPid, 0, new double[] { 0 }, 1, false, options.Condition);
                List<TrialRecord> baselineRecords = await ExecuteAsync(baseline, false, token).ConfigureAwait(false);
                Candidate incumbent = SelectBest(baselineRecords);
                if (incumbent == null) throw new InvalidOperationException("启动基线没有完整、唯一的正负双向记录。");
                RememberBest(incumbent);

                while (true)
                {
                    token.ThrowIfCancellationRequested();
                    CheckBoundary(incumbent.Gains, token);
                    if (incumbent.Pass)
                    {
                        if (!FitsBudget(Snapshot().TrialsUsed, 10, options.MaxTrials, false))
                        { await FinishNormalAsync("候选已通过筛选，但试验预算不足以独立验收。", token).ConfigureAwait(false); return; }
                        SetStage("独立重复验收", "固定候选六参，另起一个批次，正负方向各 5 次；全部完整且硬条件通过才确认。");
                        TrialPlan acceptance = Evaluation.BuildPlan(incumbent.Gains, 0, new double[] { 0 }, 5, true, options.Condition);
                        List<TrialRecord> verification = await ExecuteAsync(acceptance, false, token).ConfigureAwait(false);
                        if (IndependentPass(verification, incumbent.Gains, options.Condition))
                        {
                            // RunAsync restores its entry PID. Explicitly apply the accepted set
                            // after the independent evidence has been checked locally.
                            CheckBoundary(null, token);
                            await BeginOperation(delegate { return bench.ApplyAsync(incumbent.Gains); }, token).ConfigureAwait(false);
                            CheckBoundary(incumbent.Gains, token);
                            lock (gate) { state.Verified = !demo; simulatedSuccess = demo; state.Best = incumbent.Gains.Copy(); }
                            SetStage(demo ? "演示流程通过" : "独立验收通过", demo ? "模拟正负各 5 次全部完整通过；仅验证软件流程，不能证明实机达标。候选保留在 RAM。" :
                                "本次同一独立批次正负各 5 次全部完整通过；候选保留在 RAM，仅适用于记录工况。");
                            SaveReport(null); return;
                        }
                        // Failed acceptance is evidence, never an opportunity to reuse the
                        // earlier passing screening pair as proof of a repeated acceptance.
                        incumbent = SelectBest(verification);
                        if (incumbent == null) throw new InvalidOperationException("独立验收记录不完整，停止后续优化。");
                        RememberBest(incumbent);
                        Trace("acceptance_failed", Summaries(verification));
                    }

                    AutoTuneSnapshot progress = Snapshot();
                    if (progress.Decisions >= options.MaxDecisions)
                    { await FinishNormalAsync("AI 建议轮数已用完；本轮尚未独立验收达标。", token).ConfigureAwait(false); return; }
                    if (!FitsBudget(progress.TrialsUsed, 2, options.MaxTrials, true))
                    { await FinishNormalAsync("剩余预算不足以扫描并保留 10 次独立验收；本轮结束。", token).ConfigureAwait(false); return; }

                    await BeginOperation(delegate { return bench.HomeAsync(); }, token).ConfigureAwait(false);
                    CheckBoundary(incumbent.Gains, token);
                    SetStage("等待 AI 分析", "云台已回零；本地持续检查反馈、保护、六参与阶跃编号，等待下一组单参数候选。");
                    AiContext context = new AiContext { Anchor = Snapshot().Anchor, Incumbent = incumbent.Gains.Copy(),
                        Condition = options.Condition, Demo = demo, Decision = progress.Decisions + 1,
                        TrialsUsed = progress.TrialsUsed, MaxTrials = options.MaxTrials,
                        MaxChangePercent = options.MaxChangePercent, Records = new List<TrialRecord>(records) };
                    lock (gate) state.Decisions++;
                    Trace("request", new { context.Decision, context.TrialsUsed, context.MaxTrials, context.Anchor, context.Incumbent,
                        context.Condition, context.Demo, context.MaxChangePercent, results = Summaries(context.Records) });
                    AiProposal proposal = await WaitForProposalAsync(context, incumbent.Gains, token).ConfigureAwait(false);
                    token.ThrowIfCancellationRequested(); CheckBoundary(incumbent.Gains, token);
                    if (proposal == null) throw new InvalidOperationException("AI 返回空建议。");
                    Trace("proposal", new { proposal.Action, proposal.Parameter, proposal.OffsetsPercent, proposal.Reason });
                    if (proposal.Action == "stop")
                    { await FinishNormalAsync("AI 建议结束：" + (proposal.Reason ?? "未提供理由"), token).ConfigureAwait(false); return; }
                    if (proposal.Action == "verify")
                    {
                        // Automatic acceptance runs before any model request whenever local
                        // evidence passes. A verify request cannot bypass a failed hard gate.
                        if (!incumbent.Pass) throw new InvalidOperationException("AI 要求验收，但当前候选尚未完整双向通过硬条件。");
                        continue;
                    }
                    TrialPlan plan = ValidateProposal(proposal, incumbent.Gains, Snapshot().Anchor, options.MaxChangePercent, options.Condition);
                    if (!FitsBudget(Snapshot().TrialsUsed, plan.Trials.Count, options.MaxTrials, true))
                    { await FinishNormalAsync("AI 候选组超出剩余试验预算；为独立验收保留 10 次，本轮结束。", token).ConfigureAwait(false); return; }
                    SetStage("自动执行候选扫描", "所有候选已一次性核对启动六参附近的全局范围；按正负成对执行并保存完整数据。");
                    List<TrialRecord> scanned = await ExecuteAsync(plan, true, token).ConfigureAwait(false);
                    Candidate next = SelectBest(scanned);
                    if (next == null) throw new InvalidOperationException("扫描没有可比较的完整双向候选。");
                    RememberBest(next);
                    Trace("selection", new { next.Gains, next.Pass, next.WorstTime, next.WorstOvershoot, next.WorstSse,
                        reason = "完整双向分组；全部硬条件优先，再比较违反条件数与各项最差值" });
                    CheckBoundary(incumbent.Gains, token);
                    await BeginOperation(delegate { return bench.ApplyAsync(next.Gains); }, token).ConfigureAwait(false);
                    CheckBoundary(next.Gains, token);
                    incumbent = next;
                }
            }
            catch (Exception ex)
            {
                Stop();
                lock (gate) { state.Verified = false; state.Stage = "已停止"; state.Detail = ex is OperationCanceledException ?
                    "用户停止或任务已取消；已请求 OFF，后续动作取消。恢复参数与重新使能需要独立操作。" :
                    ex.Message + "；已请求 OFF，后续动作取消。恢复参数与重新使能需要独立操作。"; }
                Emit(Snapshot().Detail);
                try { SaveReport(Snapshot().Detail); } catch { }
                throw;
            }
            finally { lock (gate) state.Running = false; }
        }

        private async Task<List<TrialRecord>> ExecuteAsync(TrialPlan plan, bool reserveAcceptance, CancellationToken token)
        {
            token.ThrowIfCancellationRequested(); CheckBoundary(null, token);
            ValidatePlan(plan, Snapshot().Anchor, options.MaxChangePercent);
            int before = bench.Snapshot().Records.Count;
            lock (gate)
            {
                if (!FitsBudget(state.TrialsUsed, plan.Trials.Count, options.MaxTrials, reserveAcceptance))
                    throw new InvalidOperationException("试验计划超出全部阶跃预算。");
                // Reserve the whole fixed plan before any motion. An interrupted plan
                // conservatively consumes its reservation; it can never overrun budget.
                state.TrialsUsed += plan.Trials.Count;
            }
            Trace("plan", new { plan.Acceptance, plan.Condition, count = plan.Trials.Count, trials = plan.Trials });
            try { await BeginOperation(delegate { return bench.RunAsync(plan); }, token).ConfigureAwait(false); }
            finally
            {
                // Preserve already finished and interrupted trials even when a
                // later member of this fixed plan fails or is cancelled.
                List<TrialRecord> published = bench.Snapshot().Records;
                if (published.Count > before) records.AddRange(published.GetRange(before, published.Count - before));
            }
            token.ThrowIfCancellationRequested(); CheckBoundary(null, token);
            WorkbenchSnapshot after = bench.Snapshot();
            if (after.Records.Count - before != plan.Trials.Count) throw new InvalidOperationException("本次试验新增记录数量与计划不符。");
            List<TrialRecord> result = after.Records.GetRange(before, plan.Trials.Count);
            foreach (TrialRecord record in result)
                if (record == null || !Evaluation.Assess(record).BoardComplete || !Evaluation.Assess(record).WaveComplete)
                    throw new InvalidOperationException("板端成绩或电脑波形不完整；停止调参，不能据不完整采集增加增益。");
            Trace("results", Summaries(result));
            SaveReport(null);
            return result;
        }

        private async Task<AiProposal> WaitForProposalAsync(AiContext context, Pid expected, CancellationToken token)
        {
            return await BeginGuardedProposal(context, expected, token).ConfigureAwait(false);
        }

        private Task<AiProposal> BeginGuardedProposal(AiContext context, Pid expected, CancellationToken token)
        {
            lock (gate)
            {
                token.ThrowIfCancellationRequested();
                return bench.GuardedWaitAsync(expected, delegate(CancellationToken guardedToken) {
                    return advisor.ProposeAsync(context, guardedToken); }, token);
            }
        }

        private WorkbenchSnapshot CheckBoundary(Pid expected, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            WorkbenchSnapshot current = bench.Snapshot();
            if (!current.Connected || current.Busy) throw new InvalidOperationException("AI 流程期间连接中断或出现外部并发操作。");
            if (current.SessionDirectory != session) throw new InvalidOperationException("AI 流程期间连接会话改变。");
            if (!string.IsNullOrEmpty(current.LastError)) throw new InvalidOperationException("会话报告错误：" + current.LastError);
            string health = Protocol.HealthReason(current.Latest);
            if (health.Length != 0) throw new InvalidOperationException(health);
            // The latest frame may precede a just-completed write/readback by a
            // few milliseconds. CurrentPid is the confirmed readback; the next
            // operation monitors every newly arriving frame against expectedPid.
            if (expected != null && !Protocol.SamePid(current.CurrentPid, expected))
                throw new InvalidOperationException("AI 流程期间六参被外部改变。");
            return current;
        }

        private async Task WaitForFirstFrameAsync(CancellationToken token)
        {
            // The read-only handshake can finish before the first 5 ms telemetry
            // packet. Wait for evidence before judging readiness or moving.
            Stopwatch timeout = Stopwatch.StartNew();
            while (true)
            {
                token.ThrowIfCancellationRequested();
                WorkbenchSnapshot current = bench.Snapshot();
                if (!current.Connected || current.SessionDirectory != session || !string.IsNullOrEmpty(current.LastError))
                    throw new InvalidOperationException("等待首帧时连接或会话状态异常。");
                if (current.Latest != null) return;
                if (timeout.ElapsedMilliseconds >= 2000) throw new TimeoutException("等待首个完整 72 通道遥测超时。");
                await Task.Delay(25, token).ConfigureAwait(false);
            }
        }

        private async Task FinishNormalAsync(string detail, CancellationToken token)
        {
            SetStage("结束并恢复启动参数", detail + " 正在健康归零并恢复本次 AI 启动六参。");
            CheckBoundary(null, token);
            await BeginOperation(delegate { return bench.ApplyAsync(Snapshot().Anchor); }, token).ConfigureAwait(false);
            CheckBoundary(Snapshot().Anchor, token);
            SetStage("本轮结束 · 未达标", detail + " 已归零并恢复本次 AI 启动六参；最佳候选和全部记录保存在报告中。");
            SaveReport(null);
        }

        private void RememberBest(Candidate candidate)
        { lock (gate) state.Best = candidate.Gains.Copy(); }

        private Task BeginOperation(Func<Task> operation, CancellationToken token)
        {
            // Serialize only operation admission against Stop(). Never hold this
            // lock while awaiting a motor operation or a model request.
            lock (gate) { token.ThrowIfCancellationRequested(); return operation(); }
        }

        public void Stop()
        {
            bool send;
            lock (gate)
            {
                send = !offRequested; offRequested = true;
                if (cancellation != null) try { cancellation.Cancel(); } catch (ObjectDisposedException) { }
            }
            if (send) bench.Stop();
        }

        internal static void ValidateOptions(AutoTuneOptions value)
        {
            if (value == null || string.IsNullOrWhiteSpace(value.Condition)) throw new ArgumentException("请先记录供电、负载、安装等试验工况。");
            if (value.MaxTrials < 12 || value.MaxTrials > 60) throw new ArgumentException("总阶跃预算需要 12..60 次，包含基线与独立验收。");
            if (value.MaxDecisions < 1 || value.MaxDecisions > 10) throw new ArgumentException("AI 建议轮数需要 1..10。");
            if (!Protocol.Finite(value.MaxChangePercent) || value.MaxChangePercent <= 0 || value.MaxChangePercent > 15)
                throw new ArgumentException("全局变化范围需要大于 0 且不超过启动六参的 ±15%。");
        }

        internal static bool FitsBudget(int usedTrials, int proposedTrials, int maximum, bool reserveAcceptance)
        { return usedTrials >= 0 && proposedTrials > 0 && maximum >= 12 && maximum <= 60 &&
                (long)usedTrials + proposedTrials + (reserveAcceptance ? 10 : 0) <= maximum; }

        internal static TrialPlan ValidateProposal(AiProposal proposal, Pid current, Pid anchor, double limit, string condition)
        {
            if (proposal == null || proposal.Action != "scan") throw new InvalidOperationException("AI 动作只能为 scan、verify 或 stop。");
            int key = Array.IndexOf(Protocol.Keys, proposal.Parameter);
            if (key < 0) throw new InvalidOperationException("AI 只能选择六个白名单增益中的一项。");
            if (proposal.OffsetsPercent == null || proposal.OffsetsPercent.Length == 0 || proposal.OffsetsPercent.Length > 3)
                throw new InvalidOperationException("AI 扫描只接受最多三个候选偏移，包含基线。");
            List<double> offsets = new List<double>(); offsets.Add(0);
            foreach (double value in proposal.OffsetsPercent)
            {
                if (!Protocol.Finite(value) || Math.Abs(value) > 15) throw new InvalidOperationException("AI 偏移不在单轮 ±15% 范围内。");
                if (!offsets.Contains(value)) offsets.Add(value);
            }
            if (offsets.Count > 3) throw new InvalidOperationException("AI 候选加上基线后超过三组。");
            TrialPlan plan = Evaluation.BuildPlan(current, key, offsets.ToArray(), 1, false, condition);
            ValidatePlan(plan, anchor, limit);
            return plan;
        }

        internal static void ValidatePlan(TrialPlan plan, Pid anchor, double limit)
        {
            Protocol.ValidatePid(anchor);
            if (!Protocol.Finite(limit) || limit <= 0 || limit > 15) throw new ArgumentException("全局偏移限制无效。");
            if (plan == null || plan.Trials == null || plan.Trials.Count == 0) throw new ArgumentException("计划为空。");
            foreach (TrialSpec trial in plan.Trials)
            {
                Protocol.ValidatePid(trial.Gains);
                for (int i = 0; i < 6; i++)
                {
                    double original = anchor.Values[i], desired = trial.Gains.Values[i];
                    if (original == 0)
                    { if (desired != 0) throw new InvalidOperationException("启动值为零的增益不能自动产生非零候选。"); }
                    else if (Math.Abs((desired / original - 1) * 100) > limit + 1e-9)
                        throw new InvalidOperationException(Protocol.Keys[i] + " 候选超出本次 AI 启动参数的全局 ±" + limit.ToString("0.###", CultureInfo.InvariantCulture) + "%，禁止逐轮累积漂移。");
                }
            }
        }

        internal static Candidate SelectBest(List<TrialRecord> source)
        {
            List<Candidate> candidates = new List<Candidate>();
            if (source == null) return null;
            foreach (TrialRecord record in source)
            {
                if (record == null || record.Gains == null) continue;
                Candidate candidate = null;
                foreach (Candidate item in candidates) if (Protocol.SamePid(item.Gains, record.Gains)) { candidate = item; break; }
                if (candidate == null) { candidate = new Candidate { Gains = record.Gains.Copy() }; candidates.Add(candidate); }
                candidate.Records.Add(record);
            }
            candidates.RemoveAll(delegate(Candidate item) { return !EvaluateCandidate(item); });
            candidates.Sort(CompareCandidates);
            return candidates.Count == 0 ? null : candidates[0];
        }

        private static bool EvaluateCandidate(Candidate item)
        {
            bool positive = false, negative = false, time = true, overshoot = true, sse = true;
            string runId = null, condition = null;
            HashSet<string> trialIds = new HashSet<string>(); HashSet<uint> boardIds = new HashSet<uint>();
            item.Pass = true;
            foreach (TrialRecord record in item.Records)
            {
                Assessment a = Evaluation.Assess(record);
                if (record.Result == null || !a.BoardComplete || !a.WaveComplete || string.IsNullOrEmpty(record.RunId) ||
                    string.IsNullOrEmpty(record.TrialId) || !trialIds.Add(record.TrialId) || !boardIds.Add(record.Result.Id)) return false;
                if (runId == null) { runId = record.RunId; condition = record.Condition; }
                if (record.RunId != runId || record.Condition != condition) return false;
                positive |= record.Direction == 1; negative |= record.Direction == -1;
                if (record.Direction != 1 && record.Direction != -1) return false;
                item.Pass &= a.Pass; time &= a.TimePass; overshoot &= a.OvershootPass; sse &= a.SsePass;
                item.WorstTime = Math.Max(item.WorstTime, record.Result.SettleMs < 0 ? double.PositiveInfinity : record.Result.SettleMs);
                item.WorstOvershoot = Math.Max(item.WorstOvershoot, record.Result.Overshoot);
                item.WorstSse = Math.Max(item.WorstSse, Math.Max(Math.Abs(record.Result.Sse), record.Result.SsePeak));
            }
            item.FailedConditions = (time ? 0 : 1) + (overshoot ? 0 : 1) + (sse ? 0 : 1);
            return positive && negative;
        }

        private static int CompareCandidates(Candidate a, Candidate b)
        {
            int compare = b.Pass.CompareTo(a.Pass); if (compare != 0) return compare;
            compare = a.FailedConditions.CompareTo(b.FailedConditions); if (compare != 0) return compare;
            double worstA = Math.Max(a.WorstTime / 250, Math.Max(a.WorstOvershoot / .2, a.WorstSse / .2));
            double worstB = Math.Max(b.WorstTime / 250, Math.Max(b.WorstOvershoot / .2, b.WorstSse / .2));
            compare = worstA.CompareTo(worstB); if (compare != 0) return compare;
            compare = a.WorstTime.CompareTo(b.WorstTime); if (compare != 0) return compare;
            compare = a.WorstOvershoot.CompareTo(b.WorstOvershoot); if (compare != 0) return compare;
            return a.WorstSse.CompareTo(b.WorstSse);
        }

        internal static bool IndependentPass(List<TrialRecord> values, Pid gains, string condition)
        {
            if (values == null || values.Count != 10 || string.IsNullOrWhiteSpace(condition)) return false;
            Candidate candidate = SelectBest(values);
            if (candidate == null || candidate.Records.Count != 10 || !candidate.Pass || !Protocol.SamePid(candidate.Gains, gains)) return false;
            int positive = 0, negative = 0; HashSet<string> repeats = new HashSet<string>();
            foreach (TrialRecord record in values)
            {
                if (!record.Acceptance || record.Condition != condition || record.Repeat < 1 || record.Repeat > 5 ||
                    !repeats.Add(record.Direction.ToString(CultureInfo.InvariantCulture) + ":" + record.Repeat.ToString(CultureInfo.InvariantCulture))) return false;
                if (record.Direction == 1) positive++; else if (record.Direction == -1) negative++;
            }
            return positive == 5 && negative == 5;
        }

        private static object[] Summaries(List<TrialRecord> values)
        {
            List<object> summaries = new List<object>();
            foreach (TrialRecord value in values) summaries.Add(new { value.TrialId, value.RunId, value.Mode, value.Acceptance,
                value.Condition, value.Direction, value.Repeat, value.Gains, value.Result, value.Assessment });
            return summaries.ToArray();
        }

        private void SetStage(string stage, string detail)
        { lock (gate) { state.Stage = stage; state.Detail = detail; } Trace("stage", new { stage, detail }); Emit(detail); }
        private void Trace(string kind, object content)
        {
            string directory = Snapshot().Directory;
            if (directory == null) return;
            File.AppendAllText(Path.Combine(directory, "trace.jsonl"), JsonUtil.Encode(new { at = DateTimeOffset.Now.ToString("o"), kind = kind, data = content }) + Environment.NewLine, Utf8);
        }
        private void SaveReport(string error)
        {
            AutoTuneSnapshot current = Snapshot();
            if (current.Directory == null) return;
            File.WriteAllText(Path.Combine(current.Directory, "report.json"), JsonUtil.Encode(new {
                at = DateTimeOffset.Now.ToString("o"), condition = options.Condition, demo = demo,
                simulatedSuccess = simulatedSuccess, current.Verified, current.Stage, current.Detail,
                current.Anchor, current.Best, current.TrialsUsed, current.Decisions, error = error,
                note = "全部阶跃均计入预算；中断计划保守消耗整组预留；独立验收只使用本次单批次10条记录；原始波形位于所属会话目录。",
                results = Summaries(records) }), Utf8);
        }
        private static Pid Copy(Pid value) { return value == null ? null : value.Copy(); }
        private void Emit(string text) { Action<string> handler = Log; if (handler != null) try { handler(text); } catch { } }
        public void Dispose()
        {
            bool running;
            lock (gate) { if (disposed) return; disposed = true; running = state.Running; }
            if (running) Stop();
            // A provider can still be unwinding an HTTP request. Cancel first;
            // provider disposal must tolerate that normal cancellation path.
            advisor.Dispose();
        }
    }
}
