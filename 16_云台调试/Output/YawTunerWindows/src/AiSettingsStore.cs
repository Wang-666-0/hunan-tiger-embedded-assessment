using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;

namespace YawTuner
{
    // This file belongs beside the application under private/, never inside an
    // experiment session. Only an explicit RememberKey opt-in persists a key.
    public static class AiSettingsStore
    {
        private static readonly byte[] Entropy = Encoding.UTF8.GetBytes("YawTuner.Windows.AiSettings.v1");
        public static void Save(string path, AiSettings input)
        {
            if (input == null) throw new ArgumentNullException("input");
            AiSettings settings = AiSettingsValidation.CopyAndValidate(input, false);
            Dictionary<string, object> values = new Dictionary<string, object> {
                { "version", 1 }, { "base_url", settings.BaseUrl }, { "api_style", settings.ApiStyle },
                { "model", settings.Model }, { "strict_schema", settings.StrictSchema },
                { "remember_key", settings.RememberKey }, { "timeout_seconds", settings.TimeoutSeconds }
            };
            if (settings.RememberKey && !string.IsNullOrEmpty(settings.ApiKey))
            {
                byte[] clear = Encoding.UTF8.GetBytes(settings.ApiKey);
                try { values.Add("encrypted_api_key", Convert.ToBase64String(ProtectedData.Protect(clear, Entropy, DataProtectionScope.CurrentUser))); }
                catch (CryptographicException) { throw new InvalidOperationException("无法使用当前 Windows 用户加密密钥；设置未保存。"); }
                finally { Array.Clear(clear, 0, clear.Length); }
            }
            string absolute = Path.GetFullPath(path);
            string directory = Path.GetDirectoryName(absolute);
            if (!Directory.Exists(directory)) Directory.CreateDirectory(directory);
            string temporary = absolute + "." + Guid.NewGuid().ToString("N") + ".tmp";
            try
            {
                File.WriteAllText(temporary, JsonUtil.Encode(values), new UTF8Encoding(false));
                if (File.Exists(absolute)) File.Replace(temporary, absolute, null); else File.Move(temporary, absolute);
            }
            finally { if (File.Exists(temporary)) File.Delete(temporary); }
        }

        public static AiSettings Load(string path)
        {
            if (!File.Exists(path)) return new AiSettings();
            if (new FileInfo(path).Length > 32768) throw new FormatException("AI 设置文件超过大小限制。");
            Dictionary<string, object> values = AiJsonReader.Object(AiJsonReader.Parse(File.ReadAllText(path, Encoding.UTF8), 32768));
            HashSet<string> allowed = new HashSet<string>(new[] { "version", "base_url", "api_style", "model", "strict_schema", "remember_key", "timeout_seconds", "encrypted_api_key" }, StringComparer.Ordinal);
            foreach (string key in values.Keys) if (!allowed.Contains(key)) throw new FormatException("AI 设置含未识别字段，已拒绝加载。");
            if (Number(values, "version") != 1) throw new FormatException("AI 设置版本无效。");
            AiSettings settings = new AiSettings {
                BaseUrl = AiJsonReader.String(values, "base_url"), ApiStyle = AiJsonReader.String(values, "api_style"), Model = AiJsonReader.String(values, "model"),
                StrictSchema = Boolean(values, "strict_schema"), RememberKey = Boolean(values, "remember_key")
            };
            double seconds = Number(values, "timeout_seconds");
            if (seconds != Math.Floor(seconds) || seconds < 15 || seconds > 120) throw new FormatException("AI 设置超时必须为 15 至 120 秒。");
            settings.TimeoutSeconds = (int)seconds;
            if (values.ContainsKey("encrypted_api_key"))
            {
                if (!settings.RememberKey) throw new FormatException("AI 设置的密钥保存标志不一致。");
                byte[] clear = null;
                try
                {
                    byte[] encrypted = Convert.FromBase64String(AiJsonReader.String(values, "encrypted_api_key"));
                    clear = ProtectedData.Unprotect(encrypted, Entropy, DataProtectionScope.CurrentUser);
                    settings.ApiKey = new UTF8Encoding(false, true).GetString(clear);
                }
                catch (CryptographicException) { throw new InvalidOperationException("无法以当前 Windows 用户解密已保存的 AI 密钥，请重新配置。"); }
                catch (FormatException) { throw new FormatException("已保存的 AI 加密密钥无效，请重新配置。"); }
                catch (DecoderFallbackException) { throw new FormatException("已保存的 AI 加密密钥无效，请重新配置。"); }
                finally { if (clear != null) Array.Clear(clear, 0, clear.Length); }
            }
            return AiSettingsValidation.CopyAndValidate(settings, false);
        }

        private static bool Boolean(Dictionary<string, object> values, string key)
        { object value = AiJsonReader.Value(values, key); if (!(value is bool)) throw new FormatException("AI 设置布尔字段无效。"); return (bool)value; }
        private static double Number(Dictionary<string, object> values, string key)
        { object value = AiJsonReader.Value(values, key); if (!(value is double)) throw new FormatException("AI 设置数值字段无效。"); return (double)value; }
    }

    internal static class AiSettingsValidation
    {
        internal static AiSettings CopyAndValidate(AiSettings input, bool requireCredentials)
        {
            AiSettings settings = new AiSettings {
                BaseUrl = (input.BaseUrl ?? "").Trim().TrimEnd('/'), ApiStyle = (input.ApiStyle ?? "").Trim().ToLowerInvariant(),
                Model = (input.Model ?? "").Trim(), ApiKey = (input.ApiKey ?? "").Trim(),
                StrictSchema = input.StrictSchema, RememberKey = input.RememberKey, TimeoutSeconds = input.TimeoutSeconds
            };
            Uri uri;
            if (!Uri.TryCreate(settings.BaseUrl, UriKind.Absolute, out uri) || uri.Scheme != Uri.UriSchemeHttps || string.IsNullOrEmpty(uri.Host) ||
                !string.IsNullOrEmpty(uri.UserInfo) || !string.IsNullOrEmpty(uri.Query) || !string.IsNullOrEmpty(uri.Fragment) ||
                settings.BaseUrl.IndexOf('\\') >= 0 || HasControl(settings.BaseUrl))
                throw new ArgumentException("AI Base URL 必须为不含用户信息、查询参数或片段的 HTTPS 地址。");
            if (settings.ApiStyle != "responses" && settings.ApiStyle != "chat_completions") throw new ArgumentException("AI API 风格必须为 responses 或 chat_completions。");
            if (settings.Model.Length > 200 || HasControl(settings.Model) || (requireCredentials && settings.Model.Length == 0)) throw new ArgumentException("请填写有效的模型名称；软件不会自动选择或替换模型。");
            if (settings.ApiKey.Length > 4096 || HasControl(settings.ApiKey) || (requireCredentials && settings.ApiKey.Length == 0)) throw new ArgumentException("请配置有效的 AI API Key。");
            if (settings.TimeoutSeconds < 15 || settings.TimeoutSeconds > 120) throw new ArgumentException("AI 请求超时必须为 15 至 120 秒。");
            return settings;
        }
        private static bool HasControl(string text) { foreach (char c in text) if (char.IsControl(c)) return true; return false; }
    }
}
