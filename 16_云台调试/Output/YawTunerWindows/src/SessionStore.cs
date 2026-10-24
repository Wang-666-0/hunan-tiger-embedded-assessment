using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;
using System.Threading;

namespace YawTuner
{
    // One FIFO writer preserves RX/TX ordering. Queue pressure and disk errors are fatal,
    // rather than silently discarding the very data needed to validate an experiment.
    public sealed class SessionStore : IDisposable
    {
        private readonly object gate = new object();
        private readonly Queue<Action> queue = new Queue<Action>();
        private readonly Thread writer;
        private bool closing;
        private Exception failure;
        private StreamWriter raw, telemetry;
        public event Action<string> Failed;
        public readonly string DirectoryPath;
        public readonly bool Demo;
        private static readonly CultureInfo Invariant = CultureInfo.InvariantCulture;

        public SessionStore(string root, bool demo)
        {
            Demo = demo;
            string mode = demo ? "DEMO_模拟_非验收" : "HARDWARE_实机";
            DirectoryPath = Path.Combine(Path.GetFullPath(root), mode,
                DateTime.Now.ToString("yyyyMMdd_HHmmss_fff", Invariant) + "_" + Guid.NewGuid().ToString("N").Substring(0, 8));
            Directory.CreateDirectory(DirectoryPath);
            Directory.CreateDirectory(Path.Combine(DirectoryPath, "trials"));
            try
            {
                raw = new StreamWriter(Path.Combine(DirectoryPath, "raw.jsonl"), false, new UTF8Encoding(false));
                telemetry = new StreamWriter(Path.Combine(DirectoryPath, "telemetry.csv"), false, new UTF8Encoding(true));
                telemetry.Write("host_ms,mode");
                for (int i = 0; i < Protocol.ChannelNames.Length; i++) telemetry.Write("," + Csv(Protocol.ChannelNames[i]));
                telemetry.WriteLine();
                File.WriteAllText(Path.Combine(DirectoryPath, "session.json"), JsonUtil.Encode(new Dictionary<string, object>
                {
                    { "mode", mode }, { "created_utc", DateTime.UtcNow.ToString("o", Invariant) },
                    { "criteria", "真实±30度阶跃；最终持续进入±0.2度≤250ms；超调≤0.2度；完整5s；最后1s稳态" },
                    { "physical_transport", !demo }, { "hardware_validation", "not_verified" }, { "telemetry_columns", Protocol.ChannelNames }
                }), new UTF8Encoding(true));
                raw.Flush(); telemetry.Flush();
            }
            catch { if (raw != null) raw.Dispose(); if (telemetry != null) telemetry.Dispose(); throw; }
            writer = new Thread(WriteLoop); writer.IsBackground = true;
            writer.Name = "YawTuner disk writer"; writer.Start();
        }

        private void Enqueue(Action action)
        {
            lock (gate)
            {
                CheckFailure();
                if (closing) throw new ObjectDisposedException("SessionStore");
                if (queue.Count >= 100000) throw new IOException("磁盘写入队列达到上限，停止实验；没有静默丢弃数据。");
                queue.Enqueue(action); Monitor.PulseAll(gate);
            }
        }

        private void CheckFailure()
        {
            if (failure != null) throw new IOException("会话文件写入失败：" + failure.Message, failure);
        }

        public void WriteRaw(long hostMs, string direction, string line)
        {
            string encoded = JsonUtil.Encode(new Dictionary<string, object>
            {
                { "host_ms", hostMs }, { "utc", DateTime.UtcNow.ToString("o", Invariant) },
                { "direction", direction }, { "line", line }, { "demo", Demo }
            });
            Enqueue(delegate { raw.WriteLine(encoded); });
        }

        public void WriteFrame(Frame frame)
        {
            StringBuilder row = new StringBuilder(1024);
            row.Append(frame.HostMs.ToString(Invariant)).Append(',').Append(Demo ? "DEMO" : "HARDWARE");
            for (int i = 0; i < frame.Values.Length; i++) row.Append(',').Append(frame.Values[i].ToString("R", Invariant));
            string text = row.ToString();
            Enqueue(delegate { telemetry.WriteLine(text); });
        }

        public void SaveTrial(TrialRecord record)
        {
            string json = JsonUtil.Encode(record);
            string name = SafeName(record.TrialId) + ".json";
            Enqueue(delegate { File.WriteAllText(Path.Combine(DirectoryPath, "trials", name), json, new UTF8Encoding(true)); });
        }

        public void SaveSummaries(List<TrialRecord> records)
        {
            List<Dictionary<string, object>> metrics = new List<Dictionary<string, object>>();
            foreach (TrialRecord item in records)
                metrics.Add(new Dictionary<string, object>
                {
                    { "TrialId", item.TrialId }, { "RunId", item.RunId }, { "Acceptance", item.Acceptance },
                    { "Mode", item.Mode }, { "Candidate", item.Candidate }, { "Direction", item.Direction },
                    { "Repeat", item.Repeat }, { "StartedAt", item.StartedAt }, { "Condition", item.Condition },
                    { "Gains", item.Gains }, { "Result", item.Result }, { "Assessment", item.Assessment },
                    { "CapturedFrames", item.Frames.Count }, { "WaveformPath", "trials/" + SafeName(item.TrialId) + ".json" }
                });
            string json = JsonUtil.Encode(new Dictionary<string, object>
            {
                { "demo", Demo }, { "summary", Evaluation.Summary(records) }, { "trials", metrics }
            });
            string ai = Evaluation.BuildAiSummary(records);
            StringBuilder csv = new StringBuilder("trial_id,run_id,acceptance,mode,candidate,direction,repeat,board_id,state,board_complete,wave_complete,pass,settle_ms,overshoot_deg,sse_deg,issues\r\n");
            foreach (TrialRecord item in records)
            {
                StepResult r = item.Result; Assessment a = item.Assessment;
                csv.Append(Csv(item.TrialId)).Append(',').Append(Csv(item.RunId)).Append(',').Append(item.Acceptance ? "1" : "0").Append(',').Append(Csv(item.Mode)).Append(',').Append(Csv(item.Candidate)).Append(',')
                    .Append(item.Direction).Append(',').Append(item.Repeat).Append(',').Append(r == null ? "" : r.Id.ToString(Invariant)).Append(',')
                    .Append(Csv(r == null ? "" : r.State)).Append(',').Append(a != null && a.BoardComplete ? "1" : "0").Append(',')
                    .Append(a != null && a.WaveComplete ? "1" : "0").Append(',').Append(a != null && a.Pass ? "1" : "0").Append(',')
                    .Append(r == null ? "" : r.SettleMs.ToString("R", Invariant)).Append(',')
                    .Append(r == null ? "" : r.Overshoot.ToString("R", Invariant)).Append(',')
                    .Append(r == null ? "" : r.Sse.ToString("R", Invariant)).Append(',')
                    .Append(Csv(a == null ? "" : string.Join("；", a.Issues.ToArray()))).Append("\r\n");
            }
            string csvText = csv.ToString();
            Enqueue(delegate
            {
                File.WriteAllText(Path.Combine(DirectoryPath, "summary.json"), json, new UTF8Encoding(true));
                File.WriteAllText(Path.Combine(DirectoryPath, "summary.csv"), csvText, new UTF8Encoding(true));
                File.WriteAllText(Path.Combine(DirectoryPath, "AI_分析摘要.md"), ai, new UTF8Encoding(true));
            });
        }

        public void Flush()
        {
            using (ManualResetEvent done = new ManualResetEvent(false))
            {
                Enqueue(delegate { raw.Flush(); telemetry.Flush(); done.Set(); });
                while (!done.WaitOne(100))
                {
                    lock (gate) CheckFailure();
                    if (!writer.IsAlive) throw new IOException("磁盘写入线程已经停止。");
                }
                lock (gate) CheckFailure();
            }
        }

        public void Export(string destination)
        {
            Flush();
            string target = Path.GetFullPath(destination);
            if (string.Equals(target.TrimEnd('\\', '/'), DirectoryPath.TrimEnd('\\', '/'), StringComparison.OrdinalIgnoreCase)) return;
            if (target.StartsWith(DirectoryPath.TrimEnd('\\', '/') + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase))
                throw new ArgumentException("导出目录不能放在当前会话目录内。");
            Directory.CreateDirectory(target);
            Exception exportError = null;
            using (ManualResetEvent complete = new ManualResetEvent(false))
            {
                // Copy at a writer-queue boundary. New RX records wait behind this action,
                // making the exported raw/CSV files a consistent, flushed snapshot.
                Enqueue(delegate
                {
                    try
                    {
                        raw.Flush(); telemetry.Flush();
                        CopyFiles(DirectoryPath, target);
                    }
                    catch (Exception ex) { exportError = ex; }
                    finally { complete.Set(); }
                });
                while (!complete.WaitOne(100)) lock (gate) CheckFailure();
            }
            if (exportError != null) throw new IOException("导出失败（原会话保留）：" + exportError.Message, exportError);
        }

        public static void ExportClosed(string sourceDirectory, string destination)
        {
            string source = Path.GetFullPath(sourceDirectory), target = Path.GetFullPath(destination);
            if (!Directory.Exists(source)) throw new DirectoryNotFoundException("会话目录不存在：" + source);
            if (string.Equals(target.TrimEnd('\\', '/'), source.TrimEnd('\\', '/'), StringComparison.OrdinalIgnoreCase)) return;
            if (target.StartsWith(source.TrimEnd('\\', '/') + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase))
                throw new ArgumentException("导出目录不能放在原会话目录内。");
            Directory.CreateDirectory(target); CopyFiles(source, target);
        }

        private static void CopyFiles(string sourceDirectory, string target)
        {
            foreach (string source in Directory.GetFiles(sourceDirectory, "*", SearchOption.AllDirectories))
            {
                string relative = source.Substring(sourceDirectory.Length).TrimStart('\\', '/');
                string output = Path.Combine(target, relative);
                Directory.CreateDirectory(Path.GetDirectoryName(output));
                using (FileStream input = new FileStream(source, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
                using (FileStream result = new FileStream(output, FileMode.CreateNew, FileAccess.Write, FileShare.Read)) input.CopyTo(result);
            }
        }

        private void WriteLoop()
        {
            try
            {
                while (true)
                {
                    Action action;
                    lock (gate)
                    {
                        while (queue.Count == 0 && !closing) Monitor.Wait(gate);
                        if (queue.Count == 0 && closing) break;
                        action = queue.Dequeue();
                    }
                    action();
                }
                raw.Flush(); telemetry.Flush();
            }
            catch (Exception ex)
            {
                lock (gate) { failure = ex; closing = true; Monitor.PulseAll(gate); }
                Action<string> handler = Failed;
                if (handler != null) handler("磁盘写入失败，实验已停止：" + ex.Message);
            }
            finally
            {
                try { raw.Dispose(); } catch { }
                try { telemetry.Dispose(); } catch { }
            }
        }

        private static string SafeName(string name)
        {
            foreach (char c in Path.GetInvalidFileNameChars()) name = name.Replace(c, '_');
            return name;
        }
        internal static string Csv(string value) { return "\"" + (value ?? "").Replace("\"", "\"\"") + "\""; }
        public void Dispose()
        {
            lock (gate) { closing = true; Monitor.PulseAll(gate); }
            if (writer != Thread.CurrentThread) writer.Join();
        }
    }

    // Small invariant JSON encoder avoids package installation and handles nonfinite
    // unavailable board measurements as JSON null (never invalid NaN tokens).
    internal static class JsonUtil
    {
        public static string Encode(object value) { StringBuilder b = new StringBuilder(); Append(b, value); return b.ToString(); }
        private static void Append(StringBuilder b, object value)
        {
            if (value == null) { b.Append("null"); return; }
            if (value is string || value is char) { Quote(b, value.ToString()); return; }
            if (value is bool) { b.Append((bool)value ? "true" : "false"); return; }
            if (value is double || value is float)
            {
                double d = Convert.ToDouble(value, CultureInfo.InvariantCulture);
                b.Append(double.IsNaN(d) || double.IsInfinity(d) ? "null" : d.ToString("R", CultureInfo.InvariantCulture)); return;
            }
            if (value is decimal || value is byte || value is sbyte || value is short || value is ushort || value is int || value is uint || value is long || value is ulong)
            { b.Append(Convert.ToString(value, CultureInfo.InvariantCulture)); return; }
            IDictionary dictionary = value as IDictionary;
            if (dictionary != null)
            {
                b.Append('{'); bool first = true;
                foreach (DictionaryEntry entry in dictionary)
                { if (!first) b.Append(','); first = false; Quote(b, Convert.ToString(entry.Key, CultureInfo.InvariantCulture)); b.Append(':'); Append(b, entry.Value); }
                b.Append('}'); return;
            }
            IEnumerable sequence = value as IEnumerable;
            if (sequence != null)
            {
                b.Append('['); bool first = true;
                foreach (object item in sequence) { if (!first) b.Append(','); first = false; Append(b, item); }
                b.Append(']'); return;
            }
            b.Append('{'); bool firstField = true;
            foreach (FieldInfo field in value.GetType().GetFields(BindingFlags.Instance | BindingFlags.Public))
            { if (!firstField) b.Append(','); firstField = false; Quote(b, field.Name); b.Append(':'); Append(b, field.GetValue(value)); }
            // Anonymous audit records expose public properties rather than fields.
            // Keep existing DTO field serialization unchanged (Frame has derived
            // properties that would duplicate its full channel array).
            if (Attribute.IsDefined(value.GetType(), typeof(System.Runtime.CompilerServices.CompilerGeneratedAttribute), false))
                foreach (PropertyInfo property in value.GetType().GetProperties(BindingFlags.Instance | BindingFlags.Public))
                {
                    if (!property.CanRead || property.GetIndexParameters().Length != 0 || property.GetGetMethod() == null) continue;
                    if (!firstField) b.Append(','); firstField = false; Quote(b, property.Name); b.Append(':'); Append(b, property.GetValue(value, null));
                }
            b.Append('}');
        }
        private static void Quote(StringBuilder b, string value)
        {
            b.Append('"');
            foreach (char c in value)
            {
                if (c == '"' || c == '\\') b.Append('\\').Append(c);
                else if (c == '\r') b.Append("\\r"); else if (c == '\n') b.Append("\\n"); else if (c == '\t') b.Append("\\t");
                else if (c < 32) b.Append("\\u").Append(((int)c).ToString("x4", CultureInfo.InvariantCulture)); else b.Append(c);
            }
            b.Append('"');
        }
    }
}
