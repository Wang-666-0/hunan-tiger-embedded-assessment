using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

namespace YawTuner
{
    public sealed class AiSettings
    {
        public string BaseUrl = "https://api.openai.com/v1";
        public string ApiStyle = "responses";
        public string Model = "";
        public string ApiKey = "";
        public bool StrictSchema = true;
        public bool RememberKey;
        public int TimeoutSeconds = 60;
    }

    public sealed class AutoTuneOptions
    {
        public string Condition = "";
        public int MaxTrials = 24;
        public int MaxDecisions = 4;
        public double MaxChangePercent = 15;
    }

    public sealed class AiContext
    {
        public Pid Anchor, Incumbent;
        public string Condition;
        public bool Demo, ProbeOnly;
        public int Decision, TrialsUsed, MaxTrials;
        public double MaxChangePercent;
        public List<TrialRecord> Records = new List<TrialRecord>();
    }

    public sealed class AiProposal
    {
        public string Action, Parameter, Reason, Raw;
        public double[] OffsetsPercent = new double[0];
    }

    public interface IAiAdvisor : IDisposable
    {
        Task<AiProposal> ProposeAsync(AiContext context, CancellationToken token);
    }

    public sealed class AutoTuneSnapshot
    {
        public bool Running, Verified;
        public string Stage = "未开始", Detail = "", Directory;
        public int TrialsUsed, Decisions;
        public Pid Anchor, Best;
    }
}
