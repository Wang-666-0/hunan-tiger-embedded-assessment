using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace YawTuner
{
    public static class Evaluation
    {
        private const double Eps = .00001;
        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        public static Assessment Assess(TrialRecord trial)
        {
            Assessment a = new Assessment();
            if (trial == null || trial.Result == null) { a.Issues.Add("未收到本轮板端完整成绩"); return a; }
            StepResult r = trial.Result;
            bool numbers = Protocol.Finite(r.Start) && Protocol.Finite(r.Target) && Protocol.Finite(r.WindowMs) &&
                Protocol.Finite(r.SettleMs) && Protocol.Finite(r.FirstMs) && Protocol.Finite(r.Overshoot) && Protocol.Finite(r.Sse) &&
                Protocol.Finite(r.SsePeak) && Protocol.Finite(r.Samples) && Protocol.Finite(r.GapMs) && Protocol.Finite(r.Fault);
            bool identity = (trial.Direction == 1 || trial.Direction == -1) &&
                Math.Abs(r.Target - 30 * trial.Direction) <= Eps && Protocol.SamePid(trial.Gains, r.Gains);
            bool metricConsistent = numbers && r.FirstMs >= -1 && r.FirstMs <= 5000 && r.FirstMs == Math.Truncate(r.FirstMs) &&
                r.SettleMs >= -1 && r.SettleMs <= 5000 && r.SettleMs == Math.Truncate(r.SettleMs) &&
                (r.SettleMs < 0 || (r.FirstMs >= 0 && r.FirstMs <= r.SettleMs)) && r.GapMs == Math.Truncate(r.GapMs) &&
                Math.Abs(r.Sse) <= r.SsePeak + Eps;
            bool complete = numbers && r.State == "DONE" && r.Reason == "NONE" && r.Valid && r.WindowMs == 5000 &&
                r.Samples >= 51 && r.Samples == Math.Truncate(r.Samples) && r.GapMs > 0 && r.GapMs < 100 &&
                r.Fault == 0 && r.CriteriaMatch && r.DirectStep && identity && metricConsistent && r.Overshoot >= 0 && r.SsePeak >= 0;
            a.BoardComplete = complete;
            if (!numbers) a.Issues.Add("板端成绩含缺失/无效数字");
            if (!metricConsistent) a.Issues.Add("板端时间/误差统计彼此不一致");
            if (r.State != "DONE" || r.Reason != "NONE" || !r.Valid) a.Issues.Add("板端试验未正常完成：" + r.State + "/" + r.Reason);
            if (r.WindowMs != 5000 || r.Samples < 51 || r.GapMs <= 0 || r.GapMs >= 100) a.Issues.Add("板端 5 秒测量窗口或反馈连续性不完整");
            if (r.Fault != 0) a.Issues.Add("板端成绩含故障");
            if (!r.CriteriaMatch) a.Issues.Add("板端考核口径不符：必须 FINAL_SETTLE、±0.2°、250 ms、超调/稳态各 0.2°");
            if (!r.DirectStep) a.Issues.Add("不是从近零开始的真实 ±30° 阶跃，或使用了前馈/平滑轨迹");
            if (!identity) a.Issues.Add("板端目标方向或六参数与本轮不一致");

            a.TimePass = numbers && r.SettleMs >= 0 && r.SettleMs <= 250;
            a.OvershootPass = numbers && r.Overshoot >= 0 && r.Overshoot <= .2 + Eps;
            a.SsePass = numbers && r.SseValid && Math.Abs(r.Sse) <= .2 + Eps && r.SsePeak >= 0 && r.SsePeak <= .2 + Eps;
            if (!a.TimePass) a.Issues.Add("最终持续入带时间未达 ≤250 ms（首次入带不作为通过依据）");
            if (!a.OvershootPass) a.Issues.Add("超调未达 ≤0.2°");
            if (!a.SsePass) a.Issues.Add("末 1 秒稳态无效、离带或误差超过 0.2°");

            EvaluateWave(trial, a);
            bool hardPass = a.TimePass && a.OvershootPass && a.SsePass;
            if (r.BoardPass != hardPass) a.Issues.Add("板端 PASS 与三项硬条件不一致");
            a.Pass = a.BoardComplete && a.WaveComplete && hardPass && r.BoardPass;
            return a;
        }

        private static void EvaluateWave(TrialRecord trial, Assessment a)
        {
            StepResult r = trial.Result;
            bool identity = true, healthy = true, timeOrder = true, running = false, done = false;
            bool haveStamp = false;
            uint lastStamp = 0;
            long firstHost = 0, lastHost = 0;
            double span = 0;
            int selected = 0;
            foreach (Frame f in trial.Frames)
            {
                if (f == null || f.Values == null || f.Values.Length != 72) { identity = false; continue; }
                if (f.StepId != r.Id) continue; // pre-command HOME frames belong to the preceding step.
                if (f.Phase != 1 && f.Phase != 2) continue;
                selected++;
                running |= f.Phase == 1; done |= f.Phase == 2;
                if (Math.Abs(f.Target - r.Target) > Eps || !Protocol.SamePid(f.Gains, trial.Gains)) identity = false;
                if (Protocol.HealthReason(f).Length != 0) healthy = false;
                if (!haveStamp)
                {
                    firstHost = lastHost = f.HostMs; lastStamp = f.Stamp; haveStamp = true; a.UniqueFeedbackFrames = 1;
                }
                else
                {
                    long hostGap = f.HostMs - lastHost;
                    if (hostGap < 0) timeOrder = false;
                    if (hostGap > a.MaxHostGapMs) a.MaxHostGapMs = hostGap;
                    lastHost = f.HostMs;
                    uint feedbackGap = unchecked(f.Stamp - lastStamp);
                    if (feedbackGap >= 0x80000000U) { timeOrder = false; continue; }
                    if (feedbackGap > 0)
                    {
                        if (feedbackGap > a.MaxFeedbackGapMs) a.MaxFeedbackGapMs = feedbackGap;
                        span += feedbackGap; lastStamp = f.Stamp; a.UniqueFeedbackFrames++;
                    }
                }
                // Holding the target after DONE must not pad a capture that missed the start.
                if (f.Phase == 2) break;
            }
            a.WaveSpanMs = span;
            // The firmware does not send its t0. This only certifies observed host coverage of about
            // 5 s and continuity of the captured *new feedback stamps*, never every CAN sample.
            a.WaveComplete = haveStamp && running && done && identity && healthy && timeOrder && selected >= 51 &&
                a.UniqueFeedbackFrames >= 51 && span >= 4980 && span <= 5500 && lastHost - firstHost >= 4980 &&
                a.MaxFeedbackGapMs < 100 && a.MaxHostGapMs <= 100;
            if (!haveStamp || !running || !done) a.Issues.Add("本机未捕获同一阶跃 ID 的运行及完成波形（可能为旧结果）");
            if (haveStamp && (span < 4980 || span > 5500 || lastHost - firstHost < 4980)) a.Issues.Add("本机波形未覆盖约 5 秒，不能用板端成绩代替完整采集");
            if (!timeOrder) a.Issues.Add("主机时间或反馈时间戳倒序（合法 uint32 回绕已处理）");
            if (a.MaxFeedbackGapMs >= 100 || a.MaxHostGapMs > 100) a.Issues.Add("本机采集有 ≥100 ms 的新反馈缺口或 >100 ms 主机缺口");
            if (!identity) a.Issues.Add("波形参数/目标与本轮记录不一致");
            if (!healthy) a.Issues.Add("波形出现故障、失能、保护条件改变或非阶跃控制");
        }

        public static TrialPlan BuildPlan(Pid baseline, int key, double[] offsets, int repeats, bool acceptance, string condition)
        {
            Protocol.ValidatePid(baseline);
            TrialPlan plan = new TrialPlan { Acceptance = acceptance, Condition = (condition ?? "").Trim() };
            if (acceptance)
            {
                if (repeats < 5) throw new ArgumentException("独立验收必须固定基线，正负方向各至少 5 次。");
                AddTrials(plan, "固定基线验收", baseline, repeats);
                return plan;
            }
            if (key < 0 || key >= 6) throw new ArgumentException("局部扫描仅支持选择一个 PID 参数。");
            if (repeats < 1 || repeats > 5) throw new ArgumentException("局部扫描每方向重复次数为 1..5。");
            if (offsets == null || offsets.Length == 0 || offsets.Length > 5) throw new ArgumentException("局部扫描需要 1..5 个候选偏移。");
            List<double> factors = new List<double>();
            factors.Add(0);
            foreach (double offset in offsets)
            {
                if (!Protocol.Finite(offset) || Math.Abs(offset) > 15 + 1e-12) throw new ArgumentException("首版仅允许单项基线附近 ±15% 的局部扫描（偏移单位为百分数）。");
                if (offset != 0 && baseline.Values[key] == 0) throw new ArgumentException(Protocol.Keys[key] + " 为 0，百分比扫描无法产生有效候选；请先人工确定可运行的非零基线，或执行 0% 固定基线试验。");
                if (!factors.Contains(offset)) factors.Add(offset);
            }
            if (factors.Count > 5) throw new ArgumentException("包含基线后候选不得超过 5 个。");
            List<double> used = new List<double>();
            foreach (double offset in factors)
            {
                Pid pid = baseline.Copy();
                // Only compare candidates that the actual 4/6-decimal firmware reports can distinguish.
                pid.Values[key] = offset == 0 ? baseline.Values[key] : Math.Round(baseline.Values[key] * (1 + offset / 100.0), key < 3 ? 4 : 6, MidpointRounding.AwayFromZero);
                Protocol.ValidatePid(pid);
                double actualPercent = baseline.Values[key] == 0 ? 0 : (pid.Values[key] / baseline.Values[key] - 1) * 100;
                if (Math.Abs(actualPercent) > 15 + 1e-9)
                    throw new ArgumentException(Protocol.Keys[key] + " 在板端回读精度下量化后的实际变化为 " + actualPercent.ToString("0.###", Inv) + "% ，超过 ±15%；请减小偏移或改选参数。");
                if (used.Contains(pid.Values[key])) continue;
                used.Add(pid.Values[key]);
                string name = offset == 0 ? "基线" : Protocol.Keys[key] + " " + (actualPercent > 0 ? "+" : "") + actualPercent.ToString("0.###", Inv) + "%";
                AddTrials(plan, name, offset == 0 ? baseline : pid, repeats);
            }
            if (used.Count == 1 && factors.Count > 1)
                foreach (TrialSpec trial in plan.Trials) trial.Candidate = "基线（扫描偏移低于板端回读分辨率，未生成新候选）";
            return plan;
        }

        private static void AddTrials(TrialPlan plan, string name, Pid pid, int repeats)
        {
            for (int repeat = 1; repeat <= repeats; repeat++)
                foreach (int direction in new int[] { 1, -1 })
                    plan.Trials.Add(new TrialSpec { Candidate = name, Direction = direction, Repeat = repeat, Gains = pid.Copy() });
        }

        private sealed class Group
        {
            public string Name;
            public string Condition;
            public string RunId;
            public bool Acceptance;
            public bool Demo;
            public Pid Gains;
            public List<TrialRecord> Records = new List<TrialRecord>();
            public int Pass, Positive, Negative, Complete;
            public double WorstTime, WorstOs, WorstSse;
            public bool Repeated;
            public HashSet<string> TrialIds = new HashSet<string>();
            public HashSet<uint> StepIds = new HashSet<uint>();
            public HashSet<string> Repeats = new HashSet<string>();
        }

        private static List<Group> Groups(List<TrialRecord> records)
        {
            List<Group> groups = new List<Group>();
            foreach (TrialRecord record in records)
            {
                if (record == null) continue;
                Group group = null;
                bool demo = IsDemo(record);
                foreach (Group item in groups)
                    if (item.Name == record.Candidate && item.Condition == (record.Condition ?? "") && item.RunId == (record.RunId ?? "") && item.Acceptance == record.Acceptance && item.Demo == demo && Protocol.SamePid(item.Gains, record.Gains)) { group = item; break; }
                if (group == null) { group = new Group { Name = record.Candidate ?? "未命名", Condition = record.Condition ?? "", RunId = record.RunId ?? "", Acceptance = record.Acceptance, Demo = demo, Gains = record.Gains }; groups.Add(group); }
                group.Records.Add(record);
                if (string.IsNullOrEmpty(record.TrialId) || !group.TrialIds.Add(record.TrialId)) group.Repeated = true;
                if (record.Result != null && !group.StepIds.Add(record.Result.Id)) group.Repeated = true;
                if (!group.Repeats.Add(record.Direction.ToString(Inv) + ":" + record.Repeat.ToString(Inv))) group.Repeated = true;
                Assessment a = Assess(record);
                if (a.Pass) group.Pass++;
                if (record.Direction == 1) group.Positive++; else if (record.Direction == -1) group.Negative++;
                if (a.BoardComplete && a.WaveComplete) group.Complete++;
                if (record.Result != null)
                {
                    group.WorstTime = Math.Max(group.WorstTime, record.Result.SettleMs < 0 ? double.PositiveInfinity : record.Result.SettleMs);
                    group.WorstOs = Math.Max(group.WorstOs, record.Result.Overshoot);
                    group.WorstSse = Math.Max(group.WorstSse, Math.Max(Math.Abs(record.Result.Sse), record.Result.SsePeak));
                }
                else group.WorstTime = group.WorstOs = group.WorstSse = double.PositiveInfinity;
            }
            groups.Sort(delegate(Group left, Group right) {
                if (left.Demo != right.Demo) return left.Demo ? 1 : -1;
                bool lp = left.Pass == left.Records.Count && left.Complete == left.Records.Count;
                bool rp = right.Pass == right.Records.Count && right.Complete == right.Records.Count;
                int compare = rp.CompareTo(lp);
                if (compare != 0) return compare;
                compare = right.Complete.CompareTo(left.Complete); if (compare != 0) return compare;
                compare = left.WorstTime.CompareTo(right.WorstTime); if (compare != 0) return compare;
                compare = left.WorstOs.CompareTo(right.WorstOs); if (compare != 0) return compare;
                return left.WorstSse.CompareTo(right.WorstSse);
            });
            return groups;
        }

        private static bool IsDemo(TrialRecord record)
        {
            string mode = record.Mode ?? "";
            return mode.IndexOf("DEMO", StringComparison.OrdinalIgnoreCase) >= 0 || mode.IndexOf("SIM", StringComparison.OrdinalIgnoreCase) >= 0 || mode.Contains("演示");
        }
        private static string Number(double value) { return Protocol.Finite(value) ? value.ToString("0.######", Inv) : "未达到/未知"; }

        public static string Summary(List<TrialRecord> records)
        {
            StringBuilder b = new StringBuilder();
            b.AppendLine("Yaw 试验对比（本地单参数局部扫描；不是贝叶斯优化）");
            b.AppendLine("硬条件逐项判定：真实 ±30°，最终持续进入 ±0.2° ≤250 ms；超调 ≤0.2°；末 1 秒稳态有效且 |误差|/峰值 ≤0.2°；板端完整 5 秒与本机完整波形均必需。");
            if (records == null || records.Count == 0) { b.AppendLine("尚无试验记录。"); return b.ToString(); }
            int rank = 0;
            foreach (Group group in Groups(records))
            {
                rank++;
                b.AppendFormat(Inv, "{0}. [{1}] {2}：通过 {3}/{4}，完整 {5}/{4}；正/负 {6}/{7}；最差稳定 {8} ms，超调 {9}°，稳态/峰值 {10}°。\r\n",
                    rank, group.Demo ? "演示数据" : "实机数据", group.Name, group.Pass, group.Records.Count, group.Complete, group.Positive, group.Negative,
                    Number(group.WorstTime), Number(group.WorstOs), Number(group.WorstSse));
                b.Append("   六参：");
                if (group.Gains != null) for (int i = 0; i < 6; i++) b.Append(Protocol.Keys[i] + "=" + Number(group.Gains.Values[i]) + " ");
                b.AppendLine();
                b.AppendLine("   工况：" + (group.Condition.Length == 0 ? "未记录" : group.Condition) + "；运行批次：" + (group.RunId.Length == 0 ? "未记录" : group.RunId));
                bool acceptance = group.Positive >= 5 && group.Negative >= 5 && group.Acceptance && !group.Repeated && group.RunId.Length > 0 && group.Condition.Length > 0;
                b.AppendLine(group.Demo ? "   演示曲线只验证软件流程，不证明实机达标。" :
                    (acceptance && group.Pass == group.Records.Count ? "   本组固定参数独立验收记录通过；仅适用于记录中的工况。" : "   尚不能认定独立验收通过；需在同一独立批次/工况固定参数后正负各至少 5 次，保留全部波形。"));
                if (group.Complete != group.Records.Count) b.AppendLine("   下一步：先处理采集缺口、反馈/保护或成绩匹配问题，再比较 PID，暂不据此增大增益。");
                else if (group.WorstOs > .2 + Eps) b.AppendLine("   下一步：超调偏高，可在当前可运行基线附近小步对照阻尼项 KD；串级耦合与速度噪声仍需实测确认，不保证增大 KD 一定改善。");
                else if (group.Pass == group.Records.Count && !acceptance) b.AppendLine("   下一步：固定本组参数，另起独立验收批次，正负各至少 5 次。");
                else if (group.WorstTime > 250) b.AppendLine("   下一步：先对照速度/力矩与采样时间，确认带宽瓶颈，再小步扫描单一参数。");
                if (group.Repeated) b.AppendLine("   发现重复/缺失的轮次编号、板端 ID 或重复序号，不能把重复记录累计为独立验收次数。");
            }
            b.AppendLine("排名先比较全部硬条件与数据完整性，再看最差成绩；不以综合分数掩盖任何不合格项。不同载荷/温度/供电/安装条件应分开比较。");
            b.AppendLine("本机波形完整性表示约 5 秒采集覆盖和新反馈戳连续；真实 t0 及最终稳定时间以板端为准，不证明捕获每一帧 CAN 反馈。失败后先检查采集/保护，再比较参数；此版本不自动更改双环结构、使能或保护。");
            return b.ToString();
        }

        public static string BuildAiSummary(List<TrialRecord> records)
        {
            StringBuilder b = new StringBuilder();
            b.AppendLine("提供给模型的离线 Yaw 调参摘要（不含 API 密钥；没有执行权限）");
            b.AppendLine("请先检查数据完整性，分别判断稳定时间、超调、末秒稳态三个硬条件，再解释原因并建议下一步。只能建议相对当前可运行基线单参数 ±15% 内的少量候选；不得要求绕过遥控保护、自动使能、清故障或以轨迹平滑替代阶跃。任何建议须人工复核后由本地范围校验。演示数据不得推断实机效果。");
            b.AppendLine(Summary(records));
            if (records != null) foreach (TrialRecord r in records)
            {
                if (r == null) continue;
                Assessment a = Assess(r);
                b.AppendFormat(Inv, "轮次={0}; 候选={1}; 模式={2}; 工况={3}; 方向={4}; 重复={5}; 开始={6}; 板端完整={7}; 波形完整={8}; 通过={9}; 波形覆盖={10}ms; 最大新反馈缺口={11}ms; 最大主机缺口={12}ms\r\n",
                    r.TrialId, r.Candidate, r.Mode, r.Condition, r.Direction, r.Repeat, r.StartedAt, a.BoardComplete, a.WaveComplete, a.Pass,
                    Number(a.WaveSpanMs), Number(a.MaxFeedbackGapMs), Number(a.MaxHostGapMs));
                b.AppendLine("诊断：" + string.Join("；", a.Issues.ToArray()));
                if (r.Result != null) b.AppendLine("板端原始成绩：" + r.Result.Raw);
            }
            return b.ToString();
        }
    }
}
