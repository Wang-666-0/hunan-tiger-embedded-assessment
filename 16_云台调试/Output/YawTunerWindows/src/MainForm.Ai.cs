using System;
using System.Collections.Concurrent;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace YawTuner
{
    public sealed partial class MainForm
    {
        readonly TextBox aiBaseUrl = new TextBox(), aiModel = new TextBox(), aiKey = new TextBox();
        readonly ComboBox aiStyle = new ComboBox(), aiSource = new ComboBox();
        readonly CheckBox aiRemember = new CheckBox(), aiStrict = new CheckBox();
        readonly NumericUpDown aiTimeout = new NumericUpDown(), aiBudget = new NumericUpDown(), aiDecisions = new NumericUpDown(), aiChange = new NumericUpDown();
        readonly Button aiSave = new Button(), aiTest = new Button(), aiStart = new Button();
        readonly TextBox aiEvents = new TextBox();
        readonly Label aiState = new Label(), aiProgress = new Label();
        readonly ConcurrentQueue<string> aiMessages = new ConcurrentQueue<string>();
        AutoTuneCoordinator autoTune;
        AutoTuneSnapshot lastAutoTune = new AutoTuneSnapshot();
        CancellationTokenSource aiProbeCancellation;
        bool loadingAiSettings, shuttingDown;
        readonly bool aiPreview;
        TabControl mainTabs;
        TabPage aiTab;

        string AiSettingsPath { get { return Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "private", "ai-settings.json"); } }

        TabPage BuildAiTab()
        {
            TabPage page = new TabPage("AI自动调参") { BackColor = Color.White, Padding = new Padding(12) };
            int aiWidth = Math.Max(650, ClientSize.Width - 400);
            Panel top = new Panel { Dock = DockStyle.Top, Height = 337, Width = aiWidth };
            Label intro = LabelText("自动流程：双向基线 → AI 建议 → 本地检查范围 → 阶跃比较 → 固定参数验收", 10, Ink);
            intro.SetBounds(0, 0, aiWidth, 26); intro.Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right; top.Controls.Add(intro);
            Label disclosure = LabelText("模型调用会向填写的接口发送六参、工况和成绩摘要，可能产生接口费用。实时控制与保护始终在本地。", 9, Muted);
            disclosure.SetBounds(0, 27, aiWidth, 26); disclosure.Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right; top.Controls.Add(disclosure);

            TableLayoutPanel settings = new TableLayoutPanel { Left = 0, Top = 60, Width = aiWidth, Height = 139, ColumnCount = 4, RowCount = 4, Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right };
            settings.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 83)); settings.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 56)); settings.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 90)); settings.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 44));
            for (int i = 0; i < 4; i++) settings.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));
            AddAiSetting(settings, "API 地址", aiBaseUrl, 0, 0); settings.SetColumnSpan(aiBaseUrl, 3);
            AddAiSetting(settings, "API 类型", aiStyle, 0, 1); AddAiSetting(settings, "模型 ID", aiModel, 2, 1);
            aiStyle.DropDownStyle = ComboBoxStyle.DropDownList; aiStyle.Items.AddRange(new string[] { "Responses API", "Chat Completions 兼容接口" });
            AddAiSetting(settings, "API Key", aiKey, 0, 2); settings.SetColumnSpan(aiKey, 3); aiKey.UseSystemPasswordChar = true;
            aiRemember.Text = "记住密钥（Windows 当前用户加密）"; aiRemember.AutoSize = true; aiRemember.Dock = DockStyle.Fill; settings.Controls.Add(aiRemember, 1, 3);
            aiStrict.Text = "严格 JSON Schema（接口需支持）"; aiStrict.AutoSize = true; aiStrict.Checked = true; aiStrict.Dock = DockStyle.Fill; settings.Controls.Add(aiStrict, 3, 3); top.Controls.Add(settings);

            FlowLayoutPanel limits = new FlowLayoutPanel { Left = 0, Top = 204, Width = aiWidth, Height = 39, Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right, WrapContents = false, AutoScroll = true };
            AiNumber(limits, "阶跃总预算", aiBudget, 12, 60, 24, 68);
            AiNumber(limits, "模型建议次数", aiDecisions, 1, 10, 4, 58);
            AiNumber(limits, "全程变化上限 %", aiChange, 1, 15, 15, 58);
            AiNumber(limits, "接口超时 s", aiTimeout, 15, 120, 60, 58);
            top.Controls.Add(limits);

            FlowLayoutPanel actions = new FlowLayoutPanel { Left = 0, Top = 247, Width = aiWidth, Height = 42, Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right, WrapContents = false };
            aiSource.DropDownStyle = ComboBoxStyle.DropDownList; aiSource.Items.AddRange(new string[] { "实时模型接口", "离线脚本演示（非实时 AI）" }); aiSource.SelectedIndex = 0; aiSource.Width = 235; aiSource.Margin = new Padding(0, 5, 8, 0); actions.Controls.Add(aiSource);
            ConfigureButton(aiSave, "保存设置", false, SaveAiSettings); aiSave.Width = 95;
            ConfigureButton(aiTest, "测试接口", false, async delegate { await TestAiAsync(); }); aiTest.Width = 95;
            ConfigureButton(aiStart, "开始 AI 自动调参", true, async delegate { await StartAiAsync(); }); aiStart.Width = 165;
            actions.Controls.AddRange(new Control[] { aiSave, aiTest, aiStart }); top.Controls.Add(actions);
            aiState.SetBounds(0, 291, aiWidth, 24); aiState.ForeColor = Accent; aiState.Font = new Font(Font.FontFamily, 15, FontStyle.Bold, GraphicsUnit.Pixel); aiState.Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right;
            aiProgress.SetBounds(0, 315, aiWidth, 22); aiProgress.ForeColor = Muted; aiProgress.Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right; top.Controls.AddRange(new Control[] { aiState, aiProgress });
            aiEvents.Multiline = true; aiEvents.ReadOnly = true; aiEvents.ScrollBars = ScrollBars.Vertical; aiEvents.Dock = DockStyle.Fill; aiEvents.BorderStyle = BorderStyle.FixedSingle; aiEvents.BackColor = Color.FromArgb(247, 250, 252);
            page.Controls.Add(aiEvents); page.Controls.Add(top);
            aiBaseUrl.TextChanged += delegate { if (!loadingAiSettings) { aiKey.Clear(); aiRemember.Checked = false; } };
            LoadAiSettings();
            return page;
        }

        void AddAiSetting(TableLayoutPanel table, string name, Control field, int column, int row)
        {
            Label label = LabelText(name, 9, Muted); label.Dock = DockStyle.Fill; table.Controls.Add(label, column, row);
            field.Dock = DockStyle.Fill; field.Margin = new Padding(0, 4, 8, 4); table.Controls.Add(field, column + 1, row);
        }
        void AiNumber(FlowLayoutPanel parent, string name, NumericUpDown number, int min, int max, int value, int width)
        {
            Label label = LabelText(name, 9, Muted); label.AutoSize = true; label.Padding = new Padding(0, 8, 0, 0); label.Margin = new Padding(0, 0, 5, 0); parent.Controls.Add(label);
            number.Minimum = min; number.Maximum = max; number.Value = value; number.Width = width; number.Margin = new Padding(0, 5, 17, 0); parent.Controls.Add(number);
        }
        AiSettings ReadAiSettings()
        {
            return new AiSettings { BaseUrl = aiBaseUrl.Text.Trim(), Model = aiModel.Text.Trim(), ApiKey = aiKey.Text.Trim(), ApiStyle = aiStyle.SelectedIndex == 1 ? "chat_completions" : "responses", RememberKey = aiRemember.Checked, StrictSchema = aiStrict.Checked, TimeoutSeconds = (int)aiTimeout.Value };
        }
        void LoadAiSettings()
        {
            try
            {
                AiSettings s = AiSettingsStore.Load(AiSettingsPath); loadingAiSettings = true;
                aiBaseUrl.Text = s.BaseUrl; aiStyle.SelectedIndex = s.ApiStyle == "chat_completions" ? 1 : 0; aiModel.Text = s.Model; aiKey.Text = s.ApiKey;
                aiRemember.Checked = s.RememberKey; aiStrict.Checked = s.StrictSchema; aiTimeout.Value = Math.Max(15, Math.Min(120, s.TimeoutSeconds));
            }
            catch (Exception e) { aiMessages.Enqueue("读取 AI 设置失败：" + e.Message); aiBaseUrl.Text = "https://api.openai.com/v1"; aiStyle.SelectedIndex = 0; }
            finally { loadingAiSettings = false; }
        }
        void SaveAiSettings()
        {
            try { AiSettingsStore.Save(AiSettingsPath, ReadAiSettings()); aiMessages.Enqueue("设置已保存；密钥" + (aiRemember.Checked ? "已按 Windows 当前用户加密保存。" : "未写入磁盘，仅保留在当前窗口。")); }
            catch (Exception e) { if (!shuttingDown) MessageBox.Show(this, e.Message, "保存 AI 设置失败"); }
        }
        async Task TestAiAsync()
        {
            await Perform(async delegate {
                aiProbeCancellation = new CancellationTokenSource();
                try
                {
                    using (IAiAdvisor advisor = aiSource.SelectedIndex == 1 ? (IAiAdvisor)new ScriptedAiAdvisor() : new HttpAiAdvisor(ReadAiSettings()))
                    {
                        aiMessages.Enqueue("正在测试接口，仅发送合成示例，不执行设备动作。");
                        AiContext context = new AiContext { ProbeOnly = true, Demo = true, Condition = "synthetic connection probe; no hardware", Anchor = new Pid(new double[] { 1, .1, .1, .005, .01, .0001 }), Incumbent = new Pid(new double[] { 1, .1, .1, .005, .01, .0001 }), MaxTrials = 24, MaxChangePercent = 15 };
                        AiProposal proposal = await advisor.ProposeAsync(context, aiProbeCancellation.Token);
                        aiMessages.Enqueue("接口测试通过：收到有效结构化建议。" + proposal.Reason);
                    }
                }
                finally { if (aiProbeCancellation != null) { aiProbeCancellation.Dispose(); aiProbeCancellation = null; } }
            });
        }
        async Task StartAiAsync()
        {
            await Perform(async delegate {
                WorkbenchSnapshot s = bench.Snapshot();
                if (!s.Connected) throw new InvalidOperationException("请先连接设备，或进入演示模式。");
                if (string.IsNullOrWhiteSpace(condition.Text)) throw new InvalidOperationException("请先在左侧填写本轮工况（供电、负载等）。");
                if (aiSource.SelectedIndex == 1 && !s.Demo) throw new InvalidOperationException("离线脚本仅可用于演示模式，不能驱动实机。");
                string health = Protocol.HealthReason(s.Latest); if (health.Length != 0) throw new InvalidOperationException(health);
                IAiAdvisor advisor = aiSource.SelectedIndex == 1 ? (IAiAdvisor)new ScriptedAiAdvisor() : new HttpAiAdvisor(ReadAiSettings());
                AutoTuneCoordinator loop = null;
                try
                {
                    loop = new AutoTuneCoordinator(bench, advisor, new AutoTuneOptions { Condition = condition.Text.Trim(), MaxTrials = (int)aiBudget.Value, MaxDecisions = (int)aiDecisions.Value, MaxChangePercent = (double)aiChange.Value });
                    autoTune = loop; loop.Log += delegate(string message) { aiMessages.Enqueue(message); logs.Enqueue("AI  " + message); };
                    aiMessages.Enqueue(s.Demo ? "开始演示自动调参；全部运动与成绩为模拟数据。" : "开始实机 AI 自动调参；将执行真实正负 30° 阶跃。请保持遥控与现场条件。");
                    await loop.RunAsync();
                    lastAutoTune = loop.Snapshot(); FillEditors(bench.Snapshot().CurrentPid);
                    aiMessages.Enqueue(lastAutoTune.Verified ? "独立验收通过，候选参数保留在 RAM，未写 Flash。" : "本轮已结束，未证明达标。请查看运行报告。");
                }
                finally
                {
                    if (loop != null) { lastAutoTune = loop.Snapshot(); loop.Dispose(); } else advisor.Dispose();
                    autoTune = null;
                }
            });
        }
        void StopAutomation()
        {
            CancellationTokenSource probe = aiProbeCancellation; if (probe != null) try { probe.Cancel(); } catch (ObjectDisposedException) { }
            AutoTuneCoordinator loop = autoTune;
            if (loop != null) loop.Stop(); else bench.Stop();
        }
        void UpdateAiUi(WorkbenchSnapshot workbenchState)
        {
            if (aiStart.Parent == null) return;
            AutoTuneSnapshot a = autoTune == null ? lastAutoTune : autoTune.Snapshot();
            bool editable = !localBusy && !workbenchState.Busy && !a.Running;
            foreach (Control control in new Control[] { aiBaseUrl, aiModel, aiKey, aiStyle, aiSource, aiRemember, aiStrict, aiTimeout, aiBudget, aiDecisions, aiChange, aiSave, aiTest }) control.Enabled = editable;
            aiStart.Enabled = editable && workbenchState.Connected;
            aiState.Text = a.Stage + (a.Verified ? " · 独立验收通过" : "");
            aiProgress.Text = "阶跃预算 " + a.TrialsUsed + " / " + aiBudget.Value + " · 模型建议 " + a.Decisions + " / " + aiDecisions.Value + " · " + a.Detail;
            string message; int drained = 0; while (drained++ < 100 && aiMessages.TryDequeue(out message)) aiEvents.AppendText(DateTime.Now.ToString("HH:mm:ss", CultureInfo.InvariantCulture) + "  " + message + "\r\n");
            if (aiEvents.TextLength > 120000) aiEvents.Text = aiEvents.Text.Substring(aiEvents.TextLength - 80000);
        }
    }
}
