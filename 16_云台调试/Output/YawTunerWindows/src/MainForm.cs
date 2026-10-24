using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace YawTuner
{
    public sealed partial class MainForm : Form
    {
        static readonly Color Ink = Color.FromArgb(27, 43, 62), Muted = Color.FromArgb(96, 113, 133), Accent = Color.FromArgb(0, 115, 123), Border = Color.FromArgb(219, 227, 234);
        readonly Workbench bench;
        readonly System.Windows.Forms.Timer timer = new System.Windows.Forms.Timer();
        readonly ConcurrentQueue<string> logs = new ConcurrentQueue<string>();
        readonly ComboBox ports = new ComboBox(), gainKey = new ComboBox(), plotMode = new ComboBox();
        readonly TextBox offsets = new TextBox(), condition = new TextBox();
        readonly TextBox[] editors = new TextBox[6];
        readonly Label[] readbacks = new Label[6];
        readonly NumericUpDown repeats = new NumericUpDown();
        readonly Label state = new Label(), readiness = new Label(), connection = new Label(), detail = new Label(), metrics = new Label(), directory = new Label();
        readonly Button connect = new Button(), demo = new Button(), read = new Button(), apply = new Button(), restore = new Button(), baseline = new Button(), scan = new Button(), accept = new Button(), home = new Button(), selectedApply = new Button(), export = new Button();
        readonly DataGridView records = new DataGridView(), channels = new DataGridView();
        readonly TextBox eventLog = new TextBox(), analysis = new TextBox(), resultDetails = new TextBox();
        readonly PlotControl plot = new PlotControl();
        readonly bool startDemo;
        readonly string previewPath;
        int recordCount = -1, channelTick, renderTicks;
        string pidSignature = "";
        string lastSession = "";
        bool editing, localBusy, allowClose;
        WorkbenchSnapshot snapshot;
        readonly List<Frame> live = new List<Frame>();
        readonly List<Button> actionButtons = new List<Button>();

        public MainForm(bool autoDemo, string preview)
            : this(autoDemo, preview, false) { }

        public MainForm(bool autoDemo, string preview, bool previewAiWorkflow)
        {
            startDemo = autoDemo; previewPath = preview; aiPreview = previewAiWorkflow;
            Text = "Yaw 调参工作台 · Windows AI 0.2.0"; MinimumSize = new Size(1140, 800); Size = new Size(1480, 980); StartPosition = FormStartPosition.CenterScreen;
            AutoScaleMode = AutoScaleMode.None;
            Font = new Font("Microsoft YaHei UI", 13F, FontStyle.Regular, GraphicsUnit.Pixel); BackColor = Color.FromArgb(242, 246, 249); ForeColor = Ink;
            bench = new Workbench(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "sessions"));
            bench.Log += delegate(string line) { logs.Enqueue(line); };
            BuildUi(); RefreshPorts();
            timer.Interval = 100; timer.Tick += Tick; timer.Start();
            Shown += async delegate { if (startDemo) await Perform(async delegate { await bench.ConnectAsync(new SimulatedTransport()); }); };
            FormClosing += async delegate(object sender, FormClosingEventArgs e) {
                if (allowClose) return;
                e.Cancel = true; timer.Stop(); Enabled = false; shuttingDown = true;
                StopAutomation();
                Text = "Yaw 调参工作台 · 正在停止并保存会话…";
                try { await Task.Run(delegate { bench.Dispose(); }); }
                catch (Exception error) { logs.Enqueue("关闭会话：" + error.Message); }
                finally { allowClose = true; Close(); }
            };
        }

        Label LabelText(string text, float size, Color color)
        { return new Label { Text = text, Font = new Font(Font.FontFamily, size * 4F / 3F, FontStyle.Regular, GraphicsUnit.Pixel), ForeColor = color, AutoSize = false, TextAlign = ContentAlignment.MiddleLeft }; }
        Button MakeButton(string text, bool primary, Action action)
        {
            Button b = new Button { Text = text, FlatStyle = FlatStyle.Flat, Height = 34, BackColor = primary ? Accent : Color.White, ForeColor = primary ? Color.White : Ink, Cursor = Cursors.Hand, Margin = new Padding(3) };
            b.FlatAppearance.BorderColor = primary ? Accent : Border; b.Click += delegate { action(); }; return b;
        }
        void ConfigureButton(Button b, string text, bool primary, Action action)
        {
            b.Text = text; b.FlatStyle = FlatStyle.Flat; b.BackColor = primary ? Accent : Color.White; b.ForeColor = primary ? Color.White : Ink; b.Cursor = Cursors.Hand;
            b.FlatAppearance.BorderColor = primary ? Accent : Border; b.Height = 33; b.Click += delegate { action(); };
        }
        Panel Card(string title, int y, int height, Panel parent)
        {
            Panel box = new Panel { Left = 0, Top = y, Width = 304, Height = height, BackColor = Color.White, BorderStyle = BorderStyle.FixedSingle };
            Label heading = LabelText(title, 10, Ink); heading.SetBounds(12, 5, 280, 27); box.Controls.Add(heading); parent.Controls.Add(box); return box;
        }
        void BuildUi()
        {
            Panel header = new Panel { Dock = DockStyle.Top, Height = 76, Width = ClientSize.Width, BackColor = Ink };
            Label title = LabelText("YAW / 调参工作台", 20, Color.White); title.SetBounds(22, 9, 480, 34);
            Label subtitle = LabelText("GL60Ⅱ · 串级 PID     30° 阶跃 / ±0.2° / 250 ms / 5 s", 9, Color.FromArgb(178, 200, 216)); subtitle.SetBounds(25, 43, 800, 23);
            SetLabel(connection, "未连接 · 参数以板端回读为准", 9, Color.White); connection.Anchor = AnchorStyles.Top | AnchorStyles.Right; connection.SetBounds(ClientSize.Width - 380, 22, 355, 32); header.Controls.AddRange(new Control[] { title, subtitle, connection });
            Panel toolbar = new Panel { Dock = DockStyle.Top, Height = 56, Width = ClientSize.Width, BackColor = Color.White, Padding = new Padding(15, 10, 15, 10) };
            Label portTitle = LabelText("串口", 9, Muted); portTitle.SetBounds(20, 12, 40, 30); ports.SetBounds(63, 14, 100, 30); ports.DropDownStyle = ComboBoxStyle.DropDownList;
            Button refreshPorts = MakeButton("刷新", false, RefreshPorts); refreshPorts.SetBounds(171, 11, 60, 33);
            ConfigureButton(connect, "连接设备", true, async delegate { await Perform(async delegate { if (bench.Snapshot().Connected) await Task.Run(delegate { bench.Disconnect(); }); else { if (ports.SelectedItem == null) throw new InvalidOperationException("未找到串口，请检查 USB 转串口并刷新。"); string selectedPort = ports.SelectedItem.ToString(); await Task.Run(delegate { return bench.ConnectAsync(new SerialTransport(selectedPort, 921600)); }); } }); }); connect.SetBounds(240, 11, 100, 33);
            ConfigureButton(demo, "演示模式", false, async delegate { await Perform(async delegate { if (bench.Snapshot().Connected) await Task.Run(delegate { bench.Disconnect(); }); await Task.Run(delegate { return bench.ConnectAsync(new SimulatedTransport()); }); }); }); demo.SetBounds(350, 11, 100, 33);
            Label serial = LabelText("921600 · 8N1   /   连接前释放 VOFA 串口", 9, Muted); serial.SetBounds(465, 12, 380, 31);
            Button stop = MakeButton("停止 / OFF", false, StopAutomation); stop.BackColor = Color.FromArgb(178, 49, 49); stop.ForeColor = Color.White; stop.FlatAppearance.BorderColor = stop.BackColor; stop.Anchor = AnchorStyles.Top | AnchorStyles.Right; stop.SetBounds(ClientSize.Width - 160, 11, 140, 34);
            toolbar.Controls.AddRange(new Control[] { portTitle, ports, refreshPorts, connect, demo, serial, stop });

            Panel workspace = new Panel { Dock = DockStyle.Fill, Padding = new Padding(16, 12, 16, 10) };
            Panel left = new Panel { Dock = DockStyle.Left, Width = 330, AutoScroll = true };
            Panel right = new Panel { Dock = DockStyle.Fill, Padding = new Padding(10, 0, 0, 0) };
            Panel p = Card("01  当前生效参数 / RAM", 0, 288, left);
            Label l = LabelText("增益             板端回读           待写入", 8, Muted); l.SetBounds(12, 35, 280, 24); p.Controls.Add(l);
            for (int i = 0; i < 6; i++)
            {
                Label k = LabelText(Protocol.Keys[i], 9, Ink); k.SetBounds(14, 63 + i * 29, 55, 26); p.Controls.Add(k);
                readbacks[i] = LabelText("—", 9, Muted); readbacks[i].SetBounds(77, 63 + i * 29, 95, 26); p.Controls.Add(readbacks[i]);
                editors[i] = new TextBox { Text = "", BorderStyle = BorderStyle.FixedSingle }; editors[i].SetBounds(176, 65 + i * 29, 110, 25); editors[i].TextChanged += delegate { if (!editing) pidSignature = "dirty"; }; p.Controls.Add(editors[i]);
            }
            ConfigureButton(read, "重新读取", false, async delegate { await Perform(async delegate { await bench.RefreshAsync(); FillEditors(bench.Snapshot().CurrentPid); }); }); read.SetBounds(12, 244, 84, 32);
            ConfigureButton(apply, "回零并写入", true, async delegate { await Perform(async delegate { await bench.ApplyAsync(ReadEditors()); FillEditors(bench.Snapshot().CurrentPid); }); }); apply.SetBounds(102, 244, 112, 32);
            ConfigureButton(home, "回零", false, async delegate { await Perform(async delegate { await bench.HomeAsync(); }); }); home.SetBounds(220, 244, 68, 32); p.Controls.AddRange(new Control[] { read, apply, home });

            p = Card("02  有界扫描 / 重复验收", 298, 326, left);
            l = LabelText("变化增益", 9, Muted); l.SetBounds(12, 39, 80, 26); p.Controls.Add(l); gainKey.Items.AddRange(Protocol.Keys); gainKey.SelectedIndex = 2; gainKey.DropDownStyle = ComboBoxStyle.DropDownList; gainKey.SetBounds(109, 40, 177, 26); p.Controls.Add(gainKey);
            l = LabelText("相对偏移 %", 9, Muted); l.SetBounds(12, 75, 90, 26); p.Controls.Add(l); offsets.Text = "0, -3, 3"; offsets.SetBounds(109, 76, 177, 26); p.Controls.Add(offsets);
            l = LabelText("每方向次数", 9, Muted); l.SetBounds(12, 111, 90, 26); p.Controls.Add(l); repeats.Minimum = 1; repeats.Maximum = 5; repeats.Value = 1; repeats.SetBounds(109, 112, 177, 26); p.Controls.Add(repeats);
            l = LabelText("工况 / 负载 / 电压", 9, Muted); l.SetBounds(12, 147, 265, 24); p.Controls.Add(l); condition.Text = ""; condition.SetBounds(12, 175, 274, 26); p.Controls.Add(condition);
            ConfigureButton(baseline, "基线正负阶跃 · 2 轮", true, async delegate { await RunPlan(false, true); }); baseline.SetBounds(12, 216, 274, 33);
            ConfigureButton(scan, "开始候选扫描", false, async delegate { await RunPlan(false, false); }); scan.SetBounds(12, 258, 132, 33);
            ConfigureButton(accept, "独立验收 · 10 轮", false, async delegate { await RunPlan(true, true); }); accept.SetBounds(152, 258, 134, 33);
            l = LabelText("双向均从零开始 · 扫描限 ±15% · 不写 Flash", 8, Muted); l.SetBounds(12, 299, 279, 20); p.Controls.AddRange(new Control[] { baseline, scan, accept, l });
            p = Card("03  恢复与数据", 634, 112, left);
            ConfigureButton(restore, "恢复连接时基线 / RAM", false, async delegate { await Perform(async delegate { await bench.RestoreBaselineAsync(); FillEditors(bench.Snapshot().CurrentPid); }); }); restore.SetBounds(12, 33, 274, 31);
            ConfigureButton(export, "导出成绩 / 波形 / AI 摘要", false, ExportData); export.SetBounds(12, 71, 274, 31); p.Controls.AddRange(new Control[] { restore, export });
            actionButtons.AddRange(new Button[] { apply, home, read, baseline, scan, accept, restore, selectedApply });

            Panel statusPanel = new Panel { Dock = DockStyle.Top, Height = 108, BackColor = Color.White, Padding = new Padding(13) };
            state.Font = new Font(Font.FontFamily, 18, FontStyle.Bold, GraphicsUnit.Pixel); state.ForeColor = Accent; state.SetBounds(14, 7, 900, 29); state.Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right;
            detail.SetBounds(14, 40, 900, 24); detail.ForeColor = Ink; detail.Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right;
            readiness.SetBounds(14, 69, 900, 30); readiness.ForeColor = Muted; readiness.Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right; statusPanel.Controls.AddRange(new Control[] { state, detail, readiness });
            Panel chartPanel = new Panel { Dock = DockStyle.Top, Height = 348, BackColor = Color.White, Padding = new Padding(0, 6, 0, 0) };
            Panel chartTop = new Panel { Dock = DockStyle.Top, Height = 38 };
            metrics.SetBounds(14, 0, 720, 33); metrics.ForeColor = Muted; metrics.Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right;
            plotMode.DropDownStyle = ComboBoxStyle.DropDownList; plotMode.Items.AddRange(new string[] { "角度 / 目标", "误差 / ±0.2°", "速度 / 目标速度", "力矩请求 / 反馈" }); plotMode.SelectedIndex = 0; plotMode.SetBounds(chartTop.Width - 192, 4, 178, 28); plotMode.Anchor = AnchorStyles.Top | AnchorStyles.Right; plotMode.SelectedIndexChanged += delegate { plot.Mode = plotMode.SelectedIndex; plot.Invalidate(); };
            chartTop.Controls.AddRange(new Control[] { metrics, plotMode }); plot.Dock = DockStyle.Fill; chartPanel.Controls.Add(plot); chartPanel.Controls.Add(chartTop);
            TabControl tabs = new TabControl { Dock = DockStyle.Fill, Padding = new Point(16, 7) };
            mainTabs = tabs;
            TabPage trialsPage = new TabPage("试验成绩"), analysisPage = new TabPage("对比与建议"), channelPage = new TabPage("72 通道"), logPage = new TabPage("命令 / 事件");
            foreach (TabPage page in new TabPage[] { trialsPage, analysisPage, channelPage, logPage }) { page.BackColor = Color.White; page.Padding = new Padding(6); tabs.TabPages.Add(page); }
            ConfigureGrid(records); records.Dock = DockStyle.Fill; records.MultiSelect = false; records.SelectionMode = DataGridViewSelectionMode.FullRowSelect;
            foreach (string c in new string[] { "#", "参数组", "方向", "最终稳定 ms", "超调 °", "稳态 °", "板端完整", "波形覆盖", "全部通过" }) records.Columns.Add(c, c);
            records.Columns[0].FillWeight = 30; records.Columns[1].FillWeight = 100; records.SelectionChanged += ShowSelection;
            Panel selectedPanel = new Panel { Dock = DockStyle.Bottom, Height = 38 };
            ConfigureButton(selectedApply, "回零并应用选中组 / RAM", false, async delegate { TrialRecord tr = SelectedRecord(); if (tr != null) await Perform(async delegate { await bench.ApplyAsync(tr.Gains); FillEditors(bench.Snapshot().CurrentPid); }); }); selectedApply.SetBounds(0, 3, 236, 31);
            Button liveButton = MakeButton("查看实时曲线", false, delegate { records.ClearSelection(); plot.Frames = live; plot.Caption = "实时预览 · 主机相对时间"; plot.Invalidate(); }); liveButton.SetBounds(245, 3, 140, 31); selectedPanel.Controls.AddRange(new Control[] { selectedApply, liveButton });
            resultDetails.Multiline = true; resultDetails.ReadOnly = true; resultDetails.Height = 78; resultDetails.Dock = DockStyle.Bottom; resultDetails.ScrollBars = ScrollBars.Vertical; resultDetails.BackColor = Color.White; resultDetails.BorderStyle = BorderStyle.None;
            trialsPage.Controls.Add(records); trialsPage.Controls.Add(resultDetails); trialsPage.Controls.Add(selectedPanel);
            ConfigureGrid(channels); channels.Dock = DockStyle.Fill; channels.Columns.Add("Index", "通道"); channels.Columns.Add("Name", "含义 / 单位"); channels.Columns.Add("Value", "原始值"); for (int i = 0; i < 72; i++) channels.Rows.Add("I" + i, Protocol.ChannelNames[i], "—"); channelPage.Controls.Add(channels);
            foreach (TextBox text in new TextBox[] { eventLog, analysis }) { text.Multiline = true; text.ReadOnly = true; text.Dock = DockStyle.Fill; text.ScrollBars = ScrollBars.Vertical; text.BackColor = Color.White; text.BorderStyle = BorderStyle.None; text.Font = new Font("Microsoft YaHei UI", 13F, FontStyle.Regular, GraphicsUnit.Pixel); }
            logPage.Controls.Add(eventLog); analysisPage.Controls.Add(analysis);
            TabPage aiPage = BuildAiTab(); aiTab = aiPage; tabs.TabPages.Add(aiPage);
            tabs.SelectedIndexChanged += delegate { chartPanel.Visible = tabs.SelectedTab != aiPage; };
            Button aiEntry = MakeButton("AI 自动调参", true, delegate { tabs.SelectedTab = aiPage; }); aiEntry.SetBounds(850, 11, 130, 33); toolbar.Controls.Add(aiEntry);
            right.Controls.Add(tabs); right.Controls.Add(chartPanel); right.Controls.Add(statusPanel);
            workspace.Controls.Add(right); workspace.Controls.Add(left);
            directory.Dock = DockStyle.Bottom; directory.Height = 27; directory.Padding = new Padding(18, 0, 8, 0); directory.ForeColor = Muted; directory.Font = new Font(Font.FontFamily, 11, FontStyle.Regular, GraphicsUnit.Pixel); directory.TextAlign = ContentAlignment.MiddleLeft;
            Controls.Add(workspace); Controls.Add(directory); Controls.Add(toolbar); Controls.Add(header);
            foreach (ComboBox combo in new ComboBox[] { ports, gainKey, plotMode }) { combo.DrawMode = DrawMode.OwnerDrawFixed; combo.ItemHeight = 22; combo.DrawItem += DrawComboItem; }
        }
        static Label SetLabel(Label label, string text, float size, Color color) { label.Text = text; label.Font = new Font("Microsoft YaHei UI", size * 4F / 3F, FontStyle.Regular, GraphicsUnit.Pixel); label.ForeColor = color; label.TextAlign = ContentAlignment.MiddleRight; return label; }
        void DrawComboItem(object sender, DrawItemEventArgs e)
        { e.DrawBackground(); ComboBox combo = (ComboBox)sender; if (e.Index >= 0) using (Brush brush = new SolidBrush(e.ForeColor)) e.Graphics.DrawString(combo.Items[e.Index].ToString(), Font, brush, e.Bounds.Left + 4, e.Bounds.Top + 3); e.DrawFocusRectangle(); }
        void ConfigureGrid(DataGridView grid)
        {
            grid.BackgroundColor = Color.White; grid.BorderStyle = BorderStyle.None; grid.ReadOnly = true; grid.AllowUserToAddRows = false; grid.AllowUserToDeleteRows = false; grid.RowHeadersVisible = false; grid.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill; grid.SelectionMode = DataGridViewSelectionMode.FullRowSelect;
            grid.EnableHeadersVisualStyles = false; grid.ColumnHeadersDefaultCellStyle.BackColor = Color.FromArgb(233, 241, 245); grid.ColumnHeadersDefaultCellStyle.ForeColor = Ink; grid.ColumnHeadersHeight = 32; grid.RowTemplate.Height = 29; grid.GridColor = Border; grid.DefaultCellStyle.SelectionBackColor = Color.FromArgb(221, 241, 240); grid.DefaultCellStyle.SelectionForeColor = Ink;
        }
        void RefreshPorts()
        {
            string before = ports.SelectedItem as string; ports.Items.Clear(); ports.Items.AddRange(System.IO.Ports.SerialPort.GetPortNames().OrderBy(x => x).ToArray()); if (before != null && ports.Items.Contains(before)) ports.SelectedItem = before; else if (ports.Items.Count > 0) ports.SelectedIndex = 0;
        }
        async Task Perform(Func<Task> action)
        {
            if (localBusy) return;
            localBusy = true;
            try { await action(); }
            catch (Exception e) { logs.Enqueue("操作未完成：" + e.Message); detail.Text = e.Message; if (previewPath == null && !shuttingDown) MessageBox.Show(this, e.Message, "操作未完成", MessageBoxButtons.OK, MessageBoxIcon.Information); }
            finally { localBusy = false; }
        }
        Pid ReadEditors()
        {
            double[] values = new double[6]; for (int i = 0; i < 6; i++) if (!double.TryParse(editors[i].Text, NumberStyles.Float, CultureInfo.InvariantCulture, out values[i])) throw new InvalidOperationException(Protocol.Keys[i] + " 请输入有效数值。"); Pid p = new Pid(values); Protocol.ValidatePid(p); return p;
        }
        void FillEditors(Pid p)
        {
            if (p == null) return; editing = true; for (int i = 0; i < 6; i++) editors[i].Text = p.Values[i].ToString("G9", CultureInfo.InvariantCulture); editing = false; pidSignature = string.Join(",", p.Values.Select(x => x.ToString("G9", CultureInfo.InvariantCulture)).ToArray());
        }
        async Task RunPlan(bool acceptance, bool onlyBaseline)
        {
            await Perform(async delegate {
                WorkbenchSnapshot s = bench.Snapshot(); if (s.CurrentPid == null) throw new InvalidOperationException("请先连接并读取真实六参。");
                double[] changes = onlyBaseline ? new double[] { 0 } : offsets.Text.Split(new char[] { ',', '，', ';', ' ' }, StringSplitOptions.RemoveEmptyEntries).Select(x => double.Parse(x, CultureInfo.InvariantCulture)).ToArray();
                TrialPlan plan = Evaluation.BuildPlan(s.CurrentPid, gainKey.SelectedIndex, changes, acceptance ? 5 : onlyBaseline ? 1 : (int)repeats.Value, acceptance, condition.Text.Trim());
                records.ClearSelection(); plot.Frames = live; plot.Caption = "实时预览 · 主机相对时间";
                await bench.RunAsync(plan);
                FillEditors(bench.Snapshot().CurrentPid);
            });
        }
        async void ExportData()
        {
            using (FolderBrowserDialog dialog = new FolderBrowserDialog()) { dialog.Description = "选择导出目录（每次创建独立子目录）"; if (dialog.ShowDialog(this) != DialogResult.OK) return; string destination = Path.Combine(dialog.SelectedPath, "YawExport_" + DateTime.Now.ToString("yyyyMMdd_HHmmss_fff")); await Perform(async delegate { await Task.Run(delegate { bench.Export(destination); }); MessageBox.Show(this, "已导出：\r\n" + destination, "导出完成"); }); }
        }
        TrialRecord SelectedRecord()
        { if (snapshot == null || records.SelectedRows.Count == 0) return null; int index = records.SelectedRows[0].Index; return index >= 0 && index < snapshot.Records.Count ? snapshot.Records[index] : null; }
        void ShowSelection(object sender, EventArgs e)
        {
            TrialRecord tr = SelectedRecord(); if (tr == null) return;
            plot.Frames = tr.Frames; plot.Caption = ((tr.Mode ?? "").Contains("DEMO") ? "演示数据 / " : "实机记录 / ") + tr.Candidate + " / " + (tr.Direction > 0 ? "+30°" : "−30°") + " · 主机相对时间"; plot.Invalidate();
            StepResult r = tr.Result; resultDetails.Text = r == null ? "无完整板端成绩" : string.Format(CultureInfo.InvariantCulture, "{0}  /  ID {1}  /  首次入带 {2} ms  /  最终稳定 {3} ms\r\n六参：{4}\r\n{5}", tr.TrialId, r.Id, r.FirstMs, r.SettleMs, string.Join(", ", tr.Gains.Values.Select(x => x.ToString("G8", CultureInfo.InvariantCulture)).ToArray()), tr.Assessment == null ? "待评估" : string.Join("；", tr.Assessment.Issues.ToArray()));
            Assessment a = tr.Assessment; metrics.Text = a == null ? "待评估" : "板端成绩 " + (a.BoardComplete ? "完整" : "不完整") + "     主机波形 " + (a.WaveComplete ? "覆盖完整" : "存在缺口") + "     硬条件 " + (a.Pass ? "全部通过" : "未全部通过");
        }
        void Tick(object sender, EventArgs e)
        {
            snapshot = bench.Snapshot(); WorkbenchSnapshot s = snapshot;
            if (lastSession != (s.SessionDirectory ?? "")) { lastSession = s.SessionDirectory ?? ""; live.Clear(); plot.Frames = live; plot.Caption = "实时预览 · 主机相对时间"; recordCount = -1; pidSignature = ""; resultDetails.Clear(); metrics.Text = "等待本会话试验记录"; }
            connection.Text = s.Connected ? (s.Demo ? "演示模式 · 全部为模拟数据" : "串口已连接 · 921600 / 8N1") : "未连接 · 参数以板端回读为准"; connection.ForeColor = s.Demo ? Color.FromArgb(255, 213, 124) : Color.White;
            connect.Text = s.Connected ? "断开连接" : "连接设备"; connect.Enabled = !s.Busy && !localBusy; demo.Enabled = !s.Busy && !localBusy; ports.Enabled = !s.Connected && !localBusy;
            state.Text = (s.Demo ? "[演示] " : "") + (string.IsNullOrEmpty(s.Stage) ? "等待连接" : s.Stage) + (s.Total > 0 ? "    " + s.Completed + " / " + s.Total : ""); detail.Text = s.Detail ?? "";
            string health = s.Latest == null ? "连接仅读取状态；按现有遥控流程准备并使能 Yaw。" : Protocol.HealthReason(s.Latest);
            readiness.Text = string.IsNullOrEmpty(health) ? string.Format(CultureInfo.InvariantCulture, "在线 / 使能 · 反馈年龄 {0} ms · 驱动 / 电机温度 {1} / {2} °C · 开始前仍须 CONSOLE_READY=1", s.Latest.Values[19], s.Latest.Values[46], s.Latest.Values[47]) : health;
            readiness.ForeColor = string.IsNullOrEmpty(health) ? Accent : Color.FromArgb(156, 91, 30);
            foreach (Button b in actionButtons) b.Enabled = s.Connected && !s.Busy && !localBusy; export.Enabled = !s.Busy && !localBusy && (s.Records.Count > 0 || s.Connected); selectedApply.Enabled = selectedApply.Enabled && SelectedRecord() != null;
            foreach (Control input in new Control[] { gainKey, offsets, repeats, condition, editors[0], editors[1], editors[2], editors[3], editors[4], editors[5] }) input.Enabled = !s.Busy && !localBusy;
            if (s.CurrentPid != null) { for (int i = 0; i < 6; i++) readbacks[i].Text = s.CurrentPid.Values[i].ToString("G7", CultureInfo.InvariantCulture); if (pidSignature == "") FillEditors(s.CurrentPid); }
            directory.Text = "数据自动保存：" + (s.SessionDirectory ?? "尚无会话") + "     ·     RX " + s.FramesReceived + " 帧 / 无效行 " + s.InvalidLines;
            if (s.Latest != null)
            {
                if (live.Count == 0 || live[live.Count - 1].HostMs != s.Latest.HostMs) { live.Add(s.Latest); if (live.Count > 600) live.RemoveRange(0, 100); }
                if (plot.Frames == null) { plot.Frames = live; plot.Caption = "实时预览 · 主机相对时间"; }
                if (++channelTick % 5 == 0) for (int i = 0; i < 72; i++) channels.Rows[i].Cells[2].Value = s.Latest.Values[i].ToString("G9", CultureInfo.InvariantCulture);
            }
            if (recordCount != s.Records.Count)
            {
                recordCount = s.Records.Count; records.Rows.Clear();
                foreach (TrialRecord tr in s.Records)
                {
                    StepResult r = tr.Result; Assessment a = tr.Assessment;
                    int idx = records.Rows.Add(records.Rows.Count + 1, tr.Candidate, (tr.Direction > 0 ? "+30°" : "−30°") + " / " + tr.Repeat, r == null ? "—" : r.SettleMs.ToString("0"), r == null ? "—" : r.Overshoot.ToString("0.0000"), r == null ? "—" : r.Sse.ToString("0.0000"), a != null && a.BoardComplete ? "完整" : "缺失", a != null && a.WaveComplete ? "覆盖" : "缺口", a != null && a.Pass ? "通过" : "未通过");
                    records.Rows[idx].DefaultCellStyle.ForeColor = a != null && a.Pass ? Accent : Ink;
                }
                records.ClearSelection(); analysis.Text = s.Records.Count == 0 ? "先以当前六参完成一次双向基线。对比只使用完整、匹配且可追溯的成绩。\r\n可到 AI自动调参 页配置模型接口，自动进行试验、比较和重复验收。" : Evaluation.Summary(s.Records);
                if (s.Records.Count > 0) { records.Rows[s.Records.Count - 1].Selected = true; ShowSelection(null, EventArgs.Empty); }
            }
            string line; int drained = 0; while (drained++ < 100 && logs.TryDequeue(out line)) { eventLog.AppendText(line + "\r\n"); } if (eventLog.TextLength > 100000) eventLog.Text = eventLog.Text.Substring(eventLog.TextLength - 60000);
            plot.Invalidate();
            UpdateAiUi(s);
            if (previewPath != null) renderTicks++;
            if (previewPath != null && renderTicks == 12 && s.Connected && s.Demo)
            {
                if (aiPreview) { aiSource.SelectedIndex = 1; condition.Text = "DEMO / synthetic plant / no physical hardware"; mainTabs.SelectedTab = aiTab; aiStart.PerformClick(); }
                else baseline.PerformClick();
            }
            bool previewFinished = aiPreview ? renderTicks > 12 && autoTune == null && !localBusy && lastAutoTune.Stage != "未开始" : s.Records.Count >= 2 && !s.Busy;
            if (previewPath != null && (previewFinished || renderTicks >= (aiPreview ? 1600 : 350)))
            {
                Directory.CreateDirectory(Path.GetDirectoryName(previewPath));
                using (Bitmap b = new Bitmap(Width, Height))
                {
                    DrawToBitmap(b, new Rectangle(0, 0, Width, Height));
                    // Native ComboBox WM_PRINT omits selected text in DrawToBitmap.
                    // Complete that same control rendering in this application's preview export.
                    using (Graphics g = Graphics.FromImage(b)) using (Brush ink = new SolidBrush(Ink))
                        foreach (ComboBox combo in new ComboBox[] { ports, gainKey, plotMode, aiStyle, aiSource })
                            if (combo.Visible && combo.SelectedItem != null)
                            {
                                Point p = PointToClient(combo.PointToScreen(new Point(5, 6)));
                                p.Offset((Width - ClientSize.Width) / 2, Height - ClientSize.Height - (Width - ClientSize.Width) / 2);
                                g.DrawString(combo.SelectedItem.ToString(), Font, ink, p);
                            }
                    b.Save(previewPath, System.Drawing.Imaging.ImageFormat.Png);
                }
                Close();
            }
        }
    }

    public sealed class PlotControl : Control
    {
        public List<Frame> Frames;
        public int Mode;
        public string Caption = "实时预览 · 主机相对时间";
        public PlotControl() { DoubleBuffered = true; BackColor = Color.White; }
        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e); Graphics g = e.Graphics; g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
            Rectangle area = new Rectangle(64, 31, Math.Max(10, Width - 87), Math.Max(10, Height - 77));
            using (Font font = new Font("Microsoft YaHei UI", 11, FontStyle.Regular, GraphicsUnit.Pixel)) using (Brush muted = new SolidBrush(Color.FromArgb(104, 120, 137))) using (Pen grid = new Pen(Color.FromArgb(229, 236, 241)))
            {
                g.DrawString(Caption ?? "", font, muted, 14, 4);
                if (Frames == null || Frames.Count < 2) { g.DrawString("等待数据 · 原始采集独立于实时绘图", font, muted, 100, 95); return; }
                Frame[] data = Frames.ToArray(); int n = data.Length; long start = data[0].HostMs; double duration = Math.Max(1000, data[n - 1].HostMs - start);
                double min = Mode == 1 ? -0.4 : 0, max = Mode == 1 ? 0.4 : 1;
                foreach (Frame f in data) { double a = Value(f, false), b = Value(f, true); min = Math.Min(min, Math.Min(a, b)); max = Math.Max(max, Math.Max(a, b)); }
                double pad = Math.Max(Mode == 3 ? 0.005 : 0.1, (max - min) * 0.09); min -= pad; max += pad;
                Func<double, float> y = v => area.Bottom - (float)((v - min) / (max - min) * area.Height);
                Func<Frame, float> x = f => area.Left + (float)((f.HostMs - start) / duration * area.Width);
                for (int i = 0; i <= 4; i++) { int py = area.Top + area.Height * i / 4; g.DrawLine(grid, area.Left, py, area.Right, py); double v = max - (max - min) * i / 4; g.DrawString(v.ToString("0.###", CultureInfo.InvariantCulture), font, muted, 6, py - 7); }
                for (int i = 0; i <= 5; i++) { int px = area.Left + area.Width * i / 5; g.DrawLine(grid, px, area.Top, px, area.Bottom); g.DrawString((duration * i / 5000).ToString("0.0", CultureInfo.InvariantCulture), font, muted, px - 8, area.Bottom + 8); }
                g.DrawString("s", font, muted, area.Right + 4, area.Bottom + 8);
                if (Mode == 1) using (Pen band = new Pen(Color.FromArgb(200, 140, 57))) { band.DashStyle = System.Drawing.Drawing2D.DashStyle.Dash; g.DrawLine(band, area.Left, y(0.2), area.Right, y(0.2)); g.DrawLine(band, area.Left, y(-0.2), area.Right, y(-0.2)); }
                using (Pen actual = new Pen(Color.FromArgb(0, 128, 137), 2)) using (Pen target = new Pen(Color.FromArgb(228, 153, 54), 1.5F))
                {
                    target.DashStyle = System.Drawing.Drawing2D.DashStyle.Dash;
                    for (int i = 1; i < n; i++) { if (data[i].HostMs - data[i - 1].HostMs > 250) continue; g.DrawLine(actual, x(data[i - 1]), y(Value(data[i - 1], false)), x(data[i]), y(Value(data[i], false))); g.DrawLine(target, x(data[i - 1]), y(Value(data[i - 1], true)), x(data[i]), y(Value(data[i], true))); }
                    int ly = Height - 18; g.DrawLine(actual, 70, ly + 5, 90, ly + 5); g.DrawString(Mode == 3 ? "请求力矩 / Nm" : Mode == 2 ? "反馈速度 / °/s" : Mode == 1 ? "目标 − 实际 / °" : "实际角度 / °", font, muted, 96, ly); g.DrawLine(target, 260, ly + 5, 280, ly + 5); g.DrawString(Mode == 3 ? "反馈力矩" : Mode == 2 ? "目标速度" : Mode == 1 ? "零误差" : "目标角度", font, muted, 286, ly);
                }
            }
        }
        double Value(Frame f, bool secondary) { if (Mode == 1) return secondary ? 0 : f.Target - f.Angle; if (Mode == 2) return f.Values[secondary ? 2 : 3]; if (Mode == 3) return f.Values[secondary ? 6 : 5]; return secondary ? f.Target : f.Angle; }
    }
}
