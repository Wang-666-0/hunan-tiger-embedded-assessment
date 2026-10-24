using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Http;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace YawTuner.Tests
{
    // All HTTP here uses an in-process message handler. No DNS, network, model
    // billing, serial port, firmware command or physical movement is involved.
    public static class AiApiTests
    {
        private static int checks;
        private const string Secret = "offline-fake-credential-never-log-this";
        private const string Scan = "{\"action\":\"scan\",\"parameter\":\"VKD\",\"offsets_percent\":[0,7,14],\"reason\":\"双向比较小幅增加微分。\"}";
        private const string Stop = "{\"action\":\"stop\",\"parameter\":\"\",\"offsets_percent\":[],\"reason\":\"本轮结束。\"}";
        private static void Check(bool value, string message) { checks++; if (!value) throw new Exception("AiApiTests: " + message); }
        private static Exception Failure(Action action, string name)
        {
            Exception found = null; try { action(); } catch (Exception error) { found = error; }
            Check(found != null, name);
            if (found != null) Check(found.ToString().IndexOf(Secret, StringComparison.Ordinal) < 0, name + " credential absent from error and inner exception");
            return found;
        }
        private static Pid Gains() { return new Pid(new double[] { 16.63, .12, .269, .0081, .003, .000209 }); }
        private static AiSettings Settings(string style)
        { return new AiSettings { ApiKey = Secret, Model = "user-chosen-test-model", ApiStyle = style, TimeoutSeconds = 15 }; }
        private static AiContext Context()
        {
            AiContext context = new AiContext { Demo = true, Anchor = Gains(), Incumbent = Gains(), Condition = "24V 空载", Decision = 1, TrialsUsed = 2, MaxTrials = 24, MaxChangePercent = 15 };
            TrialRecord record = new TrialRecord { Gains = Gains(), Direction = 1, Repeat = 1, Candidate = "INTERNAL_CANDIDATE_DO_NOT_SEND", TrialId = "INTERNAL_TRIAL_ID_DO_NOT_SEND",
                Result = new StepResult { Raw = "RAW_BOARD_MUST_NOT_LEAVE", Valid = true, State = "DONE", Reason = "NONE", Start = 0, Target = 30, WindowMs = 5000, SettleMs = 220, Overshoot = .19, Sse = .02, SseValid = true, BoardPass = true, CriteriaMatch = true, DirectStep = true },
                Assessment = new Assessment { BoardComplete = true, WaveComplete = true, Pass = true, TimePass = true, OvershootPass = true, SsePass = true, WaveSpanMs = 5000 } };
            double[] full = new double[72]; full[0] = 99887766; record.Frames.Add(new Frame { HostMs = 88776655, Values = full });
            context.Records.Add(record); return context;
        }
        private static string Responses(string proposal)
        {
            return JsonUtil.Encode(new Dictionary<string, object> { { "status", "completed" }, { "incomplete_details", null },
                { "output", new object[] { new Dictionary<string, object> { { "type", "message" }, { "status", "completed" }, { "role", "assistant" },
                    { "content", new object[] { new Dictionary<string, object> { { "type", "output_text" }, { "text", proposal } } } } } } } });
        }
        private static string Chat(string proposal)
        {
            return JsonUtil.Encode(new Dictionary<string, object> { { "choices", new object[] { new Dictionary<string, object> { { "finish_reason", "stop" },
                { "message", new Dictionary<string, object> { { "role", "assistant" }, { "refusal", null }, { "content", proposal } } } } } } });
        }
        private static AiProposal Ask(AiSettings settings, CaptureHandler handler, AiContext context, CancellationToken token)
        { using (HttpAiAdvisor advisor = new HttpAiAdvisor(settings, handler)) return advisor.ProposeAsync(context, token).GetAwaiter().GetResult(); }

        public static int Run()
        {
            checks = 0;
            ParserTests(); HttpTests(); SettingsTests(); ScriptTests();
            return checks;
        }

        private static void ParserTests()
        {
            AiProposal scan = AiProposalParser.Parse(Scan);
            Check(scan.Action == "scan" && scan.Parameter == "VKD" && scan.OffsetsPercent.Length == 3, "valid bounded scan");
            Check(AiProposalParser.Parse(Stop).Action == "stop", "valid stop");
            Check(AiProposalParser.Parse(Stop.Replace("stop", "verify")).Action == "verify", "valid verify");
            string[] invalid = {
                "```json\n" + Scan + "\n```", Scan + " extra", "[" + Scan + "]", "null", "",
                Scan.Replace("\"action\":\"scan\"", "\"action\":\"scan\",\"action\":\"stop\""),
                Scan.Replace("\"action\":\"scan\"", "\"action\":\"scan\",\"\\u0061ction\":\"scan\""),
                Scan.Replace("\"reason\":", "\"command\":\"YAW OFF\",\"reason\":"),
                Scan.Replace("\"reason\":", "\"gains\":[1,2,3,4,5,6],\"reason\":"),
                Scan.Replace("\"scan\"", "\"enable\""), Scan.Replace("\"VKD\"", "\"YAW OFF\""),
                Scan.Replace("[0,7,14]", "[0,7,14,15]"), Scan.Replace("[0,7,14]", "[7,14]"),
                Scan.Replace("[0,7,14]", "[0]"), Scan.Replace("[0,7,14]", "[]"), Scan.Replace("[0,7,14]", "[0,0,7]"),
                Scan.Replace("[0,7,14]", "[0,\"7\"]"), Scan.Replace("[0,7,14]", "[0,true]"), Scan.Replace("[0,7,14]", "[0,null]"),
                Scan.Replace("[0,7,14]", "[0,NaN]"), Scan.Replace("[0,7,14]", "[0,Infinity]"), Scan.Replace("[0,7,14]", "[0,1e999]"),
                Scan.Replace("[0,7,14]", "[0,15.001]"), Scan.Replace("[0,7,14]", "[0,-15.001]"),
                Scan.Replace("[0,7,14]", "[0,01]"), Scan.Replace("[0,7,14]", "[0,+1]"), Scan.Replace("[0,7,14]", "[0,.1]"),
                Scan.Replace("[0,7,14]", "[0,1.]"), Scan.Replace("[0,7,14]", "[0,1,]"),
                Stop.Replace("\"parameter\":\"\"", "\"parameter\":\"PKP\""), Stop.Replace("[]", "[0]"),
                Stop.Replace("本轮结束。", ""), Stop.Replace("本轮结束。", "bad\\nline"),
                Scan.Replace("\"reason\":", "\"Reason\":")
            };
            for (int i = 0; i < invalid.Length; i++)
            {
                int test = i;
                Exception error = Failure(delegate { AiProposalParser.Parse(invalid[test]); }, "reject invalid proposal " + test);
                Check(error is FormatException, "invalid proposal is a fixed format error " + test);
            }
            Check(AiProposalParser.Parse(Scan.Replace("[0,7,14]", "[-1.5e1,0,+1]".Replace("+1", "1e-2"))).OffsetsPercent[0] == -15, "valid JSON scientific numbers");
            Failure(delegate { AiProposalParser.Parse(Stop.Replace("本轮结束。", new string('x', 401))); }, "bound reason length");
            Failure(delegate { AiProposalParser.Parse(new string('x', 16385)); }, "bound proposal length");
        }

        private static void HttpTests()
        {
            foreach (string style in new[] { "responses", "chat_completions" })
            {
                foreach (bool strict in new[] { true, false })
                {
                    AiSettings settings = Settings(style); settings.StrictSchema = strict;
                    CaptureHandler handler = new CaptureHandler { Reply = style == "responses" ? Responses(Scan) : Chat(Scan) };
                    AiProposal result = Ask(settings, handler, Context(), CancellationToken.None);
                    Check(result.Action == "scan", style + " parses proposal");
                    Check(handler.Calls == 1, style + " exactly one call without retry");
                    Check(handler.Uri.AbsoluteUri == "https://api.openai.com/v1/" + (style == "responses" ? "responses" : "chat/completions"), style + " exact endpoint");
                    Check(handler.Authorization == "Bearer " + Secret, style + " authorization only in header");
                    Check(handler.Body.IndexOf(Secret, StringComparison.Ordinal) < 0, style + " no key in JSON body");
                    Check(handler.Body.IndexOf("RAW_BOARD_MUST_NOT_LEAVE", StringComparison.Ordinal) < 0 && handler.Body.IndexOf("99887766", StringComparison.Ordinal) < 0 && handler.Body.IndexOf("88776655", StringComparison.Ordinal) < 0, style + " no waveform/raw board text");
                    Check(handler.Body.IndexOf("INTERNAL_CANDIDATE_DO_NOT_SEND", StringComparison.Ordinal) < 0 && handler.Body.IndexOf("INTERNAL_TRIAL_ID_DO_NOT_SEND", StringComparison.Ordinal) < 0, style + " whitelist excludes extra trial strings");
                    Check(handler.Body.IndexOf("original_anchor_gains", StringComparison.Ordinal) >= 0 && handler.Body.IndexOf("reserved_independent_verification_trials", StringComparison.Ordinal) >= 0 && handler.Body.IndexOf("trials_remaining_for_search", StringComparison.Ordinal) >= 0, style + " global anchor and verification budget explicit");
                    Dictionary<string, object> body = AiJsonReader.Object(AiJsonReader.Parse(handler.Body, 262144));
                    Check(AiJsonReader.String(body, "model") == settings.Model && !body.ContainsKey("tools"), style + " preserves explicit model and exposes no tools");
                    Dictionary<string, object> format = style == "responses"
                        ? AiJsonReader.Object(AiJsonReader.Value(AiJsonReader.Object(AiJsonReader.Value(body, "text")), "format"))
                        : AiJsonReader.Object(AiJsonReader.Value(body, "response_format"));
                    Check(AiJsonReader.String(format, "type") == (strict ? "json_schema" : "json_object"), style + " selected structured output mode");
                    if (strict)
                    {
                        Dictionary<string, object> definition = style == "responses" ? format : AiJsonReader.Object(AiJsonReader.Value(format, "json_schema"));
                        Check((bool)AiJsonReader.Value(definition, "strict"), style + " strict schema enabled");
                        Dictionary<string, object> schema = AiJsonReader.Object(AiJsonReader.Value(definition, "schema"));
                        Check(!(bool)AiJsonReader.Value(schema, "additionalProperties") && AiJsonReader.Array(schema, "required").Count == 4, style + " exact four-field schema");
                    }
                    else Check(!format.ContainsKey("schema") && !format.ContainsKey("json_schema"), style + " explicit JSON mode has no hidden schema/fallback");
                }
            }
            AiSettings custom = Settings("responses"); custom.BaseUrl = "https://example.test/custom/v1/";
            CaptureHandler customHandler = new CaptureHandler { Reply = Responses(Stop) }; Ask(custom, customHandler, Context(), CancellationToken.None);
            Check(customHandler.Uri.AbsoluteUri == "https://example.test/custom/v1/responses", "custom HTTPS API prefix");

            string[] badResponses = {
                Responses(Scan).Replace("\"status\":\"completed\"", "\"status\":\"incomplete\""),
                Responses(Scan).Replace("\"incomplete_details\":null", "\"incomplete_details\":{\"reason\":\"max_output_tokens\"}"),
                Responses(Scan).Replace("\"type\":\"output_text\"", "\"type\":\"refusal\""),
                Responses(Scan).Replace("\"type\":\"message\"", "\"type\":\"function_call\""),
                Responses(Scan).Replace("\"role\":\"assistant\"", "\"role\":\"tool\""), Responses(""),
                Responses(Scan).Replace("\"status\":\"completed\",", "\"status\":\"completed\",\"status\":\"completed\","),
                "{\"status\":\"completed\",\"output\":[]}", Responses(Scan.Replace("[0,7,14]", "[0,99]")),
                "{\"error\":{\"message\":\"" + Secret + "\"}}", "", "not-json " + Secret,
                Responses(Stop.Replace("本轮结束。", Secret))
            };
            for (int i = 0; i < badResponses.Length; i++)
            {
                CaptureHandler handler = new CaptureHandler { Reply = badResponses[i] };
                Failure(delegate { Ask(Settings("responses"), handler, Context(), CancellationToken.None); }, "reject malformed/refused/incomplete Responses " + i);
                Check(handler.Calls == 1, "no fallback/retry for invalid Responses " + i);
            }
            string[] badChats = {
                Chat(Scan).Replace("\"finish_reason\":\"stop\"", "\"finish_reason\":\"length\""),
                Chat(Scan).Replace("\"refusal\":null", "\"refusal\":\"no\""),
                Chat(Scan).Replace("\"refusal\":null", "\"tool_calls\":[{}]"),
                Chat(Scan).Replace("\"refusal\":null", "\"function_call\":{}"),
                Chat(Scan).Replace("\"role\":\"assistant\"", "\"role\":\"user\""), Chat(""),
                "{\"choices\":[]}", "{\"choices\":[{},{}]}", Chat("```json\n" + Stop + "\n```"), Chat(Stop.Replace("本轮结束。", Secret))
            };
            for (int i = 0; i < badChats.Length; i++)
            {
                CaptureHandler handler = new CaptureHandler { Reply = badChats[i] };
                Failure(delegate { Ask(Settings("chat_completions"), handler, Context(), CancellationToken.None); }, "reject malformed/refused/truncated Chat " + i);
            }
            CaptureHandler nullTools = new CaptureHandler { Reply = Chat(Scan).Replace("\"refusal\":null", "\"refusal\":null,\"tool_calls\":null,\"function_call\":null") };
            Check(Ask(Settings("chat_completions"), nullTools, Context(), CancellationToken.None).Action == "scan" && nullTools.Calls == 1, "Chat permits explicitly null tool and function call metadata");
            CaptureHandler emptyTools = new CaptureHandler { Reply = Chat(Scan).Replace("\"refusal\":null", "\"refusal\":null,\"tool_calls\":[],\"function_call\":null") };
            Check(Ask(Settings("chat_completions"), emptyTools, Context(), CancellationToken.None).Action == "scan" && emptyTools.Calls == 1, "Chat permits an empty tool call list with null function call");
            foreach (HttpStatusCode status in new[] { HttpStatusCode.Unauthorized, HttpStatusCode.Forbidden, (HttpStatusCode)429, HttpStatusCode.InternalServerError, HttpStatusCode.Redirect })
            {
                CaptureHandler handler = new CaptureHandler { Status = status, Reply = Secret };
                Exception error = Failure(delegate { Ask(Settings("responses"), handler, Context(), CancellationToken.None); }, "HTTP status " + (int)status);
                Check(error.Message.Contains(((int)status).ToString()), "safe numeric HTTP status retained");
                Check(handler.Calls == 1, "HTTP status no retry/redirect");
            }
            CaptureHandler tooBig = new CaptureHandler { Reply = new string('x', 262145) };
            Failure(delegate { Ask(Settings("responses"), tooBig, Context(), CancellationToken.None); }, "reject oversized HTTP response");
            CaptureHandler unknownLengthBig = new CaptureHandler { Content = new StreamContent(new MemoryStream(Encoding.UTF8.GetBytes(new string('x', 262145)))) };
            Failure(delegate { Ask(Settings("responses"), unknownLengthBig, Context(), CancellationToken.None); }, "reject streamed oversized HTTP response");
            CaptureHandler network = new CaptureHandler { NetworkError = true };
            Failure(delegate { Ask(Settings("responses"), network, Context(), CancellationToken.None); }, "network exception does not reveal credential");
            CaptureHandler invalidUtf8 = new CaptureHandler { Content = new ByteArrayContent(new byte[] { 0xff, 0xfe }) };
            Failure(delegate { Ask(Settings("responses"), invalidUtf8, Context(), CancellationToken.None); }, "reject invalid UTF8");
            using (CancellationTokenSource canceled = new CancellationTokenSource())
            {
                canceled.Cancel(); CaptureHandler handler = new CaptureHandler { Reply = Responses(Stop) };
                Exception error = Failure(delegate { Ask(Settings("responses"), handler, Context(), canceled.Token); }, "pre-canceled request");
                Check(error is OperationCanceledException && handler.Calls == 0, "pre-canceled request never reaches handler");
            }
            // The fake stream intentionally ignores ReadAsync's cancellation token.
            // Closing the HTTP response must still break the pending body read.
            using (CancellationTokenSource canceled = new CancellationTokenSource())
            {
                BlockingReadStream stream = new BlockingReadStream();
                CaptureHandler handler = new CaptureHandler { Content = new StreamContent(stream) };
                using (HttpAiAdvisor advisor = new HttpAiAdvisor(Settings("responses"), handler))
                {
                    Task<AiProposal> task = advisor.ProposeAsync(Context(), canceled.Token);
                    Check(stream.ReadStarted.WaitOne(3000), "response body read began");
                    canceled.Cancel();
                    Check(SpinWait.SpinUntil(delegate { return task.IsCompleted; }, 3000), "cancel closes noncooperative HTTP body stream");
                    Exception error = Failure(delegate { task.GetAwaiter().GetResult(); }, "body cancellation surfaced");
                    Check(error is OperationCanceledException && stream.Closed, "body cancellation preserves cancellation semantics");
                }
            }
            BlockingReadStream timedStream = new BlockingReadStream();
            CaptureHandler timedHandler = new CaptureHandler { Content = new StreamContent(timedStream) };
            using (HttpAiAdvisor advisor = new HttpAiAdvisor(Settings("responses"), timedHandler))
            {
                Task<AiProposal> task = advisor.ProposeAsync(Context(), CancellationToken.None);
                Check(SpinWait.SpinUntil(delegate { return task.IsCompleted; }, 20000), "configured 15-second deadline terminates noncooperative body stream");
                Exception error = Failure(delegate { task.GetAwaiter().GetResult(); }, "body deadline surfaced");
                Check(error is TimeoutException && timedStream.Closed, "body deadline distinguished from user cancellation");
            }
            foreach (string url in new[] { "http://api.example.test/v1", "http://127.0.0.1:8080/v1", "https://user:pass@example.test/v1", "https://example.test/v1?key=x", "https://example.test/v1#fragment", "not-a-url", "https://example.test\\v1" })
            {
                AiSettings settings = Settings("responses"); settings.BaseUrl = url;
                Failure(delegate { using (HttpAiAdvisor advisor = new HttpAiAdvisor(settings, new CaptureHandler())) { } }, "reject insecure/malformed API URL");
            }
            foreach (int seconds in new[] { 0, 14, 121 })
            {
                AiSettings settings = Settings("responses"); settings.TimeoutSeconds = seconds;
                Failure(delegate { using (HttpAiAdvisor advisor = new HttpAiAdvisor(settings, new CaptureHandler())) { } }, "reject invalid timeout");
            }
            AiSettings noModel = Settings("responses"); noModel.Model = "";
            Failure(delegate { using (HttpAiAdvisor advisor = new HttpAiAdvisor(noModel, new CaptureHandler())) { } }, "no implicit model default");
            AiSettings noKey = Settings("responses"); noKey.ApiKey = "";
            Failure(delegate { using (HttpAiAdvisor advisor = new HttpAiAdvisor(noKey, new CaptureHandler())) { } }, "missing key rejected");
            AiContext secretContext = Context(); secretContext.Condition = Secret;
            CaptureHandler notSent = new CaptureHandler { Reply = Responses(Stop) };
            Failure(delegate { Ask(Settings("responses"), notSent, secretContext, CancellationToken.None); }, "credential accidentally placed in context is not sent");
            Check(notSent.Calls == 0, "credential context rejected before HTTP");
        }

        private static void SettingsTests()
        {
            string directory = Path.Combine(Path.GetTempPath(), "YawTuner_AI_OfflineTests_" + Guid.NewGuid().ToString("N"));
            string path = Path.Combine(directory, "private", "ai-settings.json");
            try
            {
                Check(AiSettingsStore.Load(path).Model == "" && AiSettingsStore.Load(path).ApiKey == "", "missing settings uses no model/key default");
                AiSettings settings = Settings("responses");
                AiSettingsStore.Save(path, settings);
                string text = File.ReadAllText(path);
                Check(text.IndexOf(Secret, StringComparison.Ordinal) < 0 && text.IndexOf("encrypted_api_key", StringComparison.Ordinal) < 0, "key never persisted without explicit opt-in");
                AiSettings loaded = AiSettingsStore.Load(path);
                Check(loaded.ApiKey == "" && !loaded.RememberKey && loaded.Model == settings.Model, "non-secret settings roundtrip");
                settings.RememberKey = true; AiSettingsStore.Save(path, settings);
                text = File.ReadAllText(path);
                Check(text.IndexOf(Secret, StringComparison.Ordinal) < 0 && text.IndexOf("encrypted_api_key", StringComparison.Ordinal) >= 0, "remembered key uses encrypted field only");
                Check(AiSettingsStore.Load(path).ApiKey == Secret && AiSettingsStore.Load(path).RememberKey, "DPAPI CurrentUser decrypt roundtrip");
                settings.RememberKey = false; AiSettingsStore.Save(path, settings);
                Check(File.ReadAllText(path).IndexOf("encrypted_api_key", StringComparison.Ordinal) < 0 && AiSettingsStore.Load(path).ApiKey == "", "unchecking remember removes prior credential");
                File.WriteAllText(path, "{\"api_key\":\"" + Secret + "\"}");
                Failure(delegate { AiSettingsStore.Load(path); }, "plaintext credential setting rejected without echo");
                File.WriteAllText(path, "{\"version\":1,\"version\":1}");
                Failure(delegate { AiSettingsStore.Load(path); }, "duplicate setting keys rejected");
                settings.RememberKey = true; AiSettingsStore.Save(path, settings);
                Dictionary<string, object> values = AiJsonReader.Object(AiJsonReader.Parse(File.ReadAllText(path), 32768));
                values["encrypted_api_key"] = Convert.ToBase64String(new byte[] { 1, 2, 3, 4 });
                File.WriteAllText(path, JsonUtil.Encode(values));
                Failure(delegate { AiSettingsStore.Load(path); }, "corrupt DPAPI ciphertext rejected");
            }
            finally
            {
                if (File.Exists(path)) File.Delete(path);
                string privateDirectory = Path.GetDirectoryName(path);
                if (Directory.Exists(privateDirectory)) Directory.Delete(privateDirectory);
                if (Directory.Exists(directory)) Directory.Delete(directory);
            }
        }

        private static void ScriptTests()
        {
            using (ScriptedAiAdvisor advisor = new ScriptedAiAdvisor())
            {
                AiContext context = Context();
                Check(advisor.ProposeAsync(context, CancellationToken.None).Result.Parameter == "VKD", "demo begins scripted KD scan");
                context.ProbeOnly = true;
                Check(advisor.ProposeAsync(context, CancellationToken.None).Result.Action == "stop", "demo probe only stops without experiment");
                context.ProbeOnly = false; context.Decision = 2;
                Check(advisor.ProposeAsync(context, CancellationToken.None).Result.Action == "stop", "demo incomplete direction pair cannot verify");
                context.Records.Add(new TrialRecord { Gains = Gains(), Direction = -1, Assessment = new Assessment { Pass = true } });
                Check(advisor.ProposeAsync(context, CancellationToken.None).Result.Action == "verify", "demo complete passing direction pair can verify");
                Check(advisor.ProposeAsync(context, CancellationToken.None).Result.Reason.Contains("非真实 AI"), "script explicitly identified");
                context.Demo = false;
                Failure(delegate { advisor.ProposeAsync(context, CancellationToken.None).GetAwaiter().GetResult(); }, "script refuses hardware context");
            }
        }

        private sealed class CaptureHandler : HttpMessageHandler
        {
            internal string Reply = "", Body, Authorization;
            internal Uri Uri;
            internal int Calls;
            internal HttpStatusCode Status = HttpStatusCode.OK;
            internal HttpContent Content;
            internal bool NetworkError;
            protected override async Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken token)
            {
                Calls++; Uri = request.RequestUri; Authorization = request.Headers.Authorization.ToString();
                Body = await request.Content.ReadAsStringAsync();
                if (NetworkError) throw new HttpRequestException("Untrusted endpoint echoes " + Secret);
                return new HttpResponseMessage(Status) { Content = Content ?? new StringContent(Reply, Encoding.UTF8, "application/json") };
            }
        }

        private sealed class BlockingReadStream : Stream
        {
            private readonly TaskCompletionSource<int> pending = new TaskCompletionSource<int>();
            internal readonly ManualResetEvent ReadStarted = new ManualResetEvent(false);
            internal bool Closed;
            public override bool CanRead { get { return true; } }
            public override bool CanSeek { get { return false; } }
            public override bool CanWrite { get { return false; } }
            public override long Length { get { throw new NotSupportedException(); } }
            public override long Position { get { throw new NotSupportedException(); } set { throw new NotSupportedException(); } }
            public override Task<int> ReadAsync(byte[] buffer, int offset, int count, CancellationToken token) { ReadStarted.Set(); return pending.Task; }
            protected override void Dispose(bool disposing) { Closed = true; pending.TrySetException(new ObjectDisposedException("Untrusted stream " + Secret)); base.Dispose(disposing); }
            public override int Read(byte[] buffer, int offset, int count) { throw new NotSupportedException(); }
            public override void Flush() { }
            public override long Seek(long offset, SeekOrigin origin) { throw new NotSupportedException(); }
            public override void SetLength(long value) { throw new NotSupportedException(); }
            public override void Write(byte[] buffer, int offset, int count) { throw new NotSupportedException(); }
        }
    }
}
