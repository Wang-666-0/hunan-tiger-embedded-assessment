using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace YawTuner
{
    public interface ITransport : IDisposable
    {
        bool IsOpen { get; }
        bool IsDemo { get; }
        event Action<string> LineReceived;
        event Action<string> Failed;
        void Open();
        void Send(string command);
        void Close();
    }

    public sealed class Pid
    {
        public double[] Values = new double[6];
        public Pid() { }
        public Pid(double[] values) { Values = (double[])values.Clone(); }
        public Pid Copy() { return new Pid(Values); }
    }

    public sealed class Frame
    {
        public long HostMs;
        public double[] Values;
        public double Angle { get { return Values[0]; } }
        public double Target { get { return Values[1]; } }
        public uint Stamp { get { return (uint)Values[63]; } }
        public uint StepId { get { return (uint)Values[24]; } }
        public int Phase { get { return (int)Values[25]; } }
        public Pid Gains { get { return new Pid(new double[] { Values[7], Values[8], Values[9], Values[10], Values[11], Values[12] }); } }
    }

    public sealed class Reply
    {
        public string Raw;
        public string Kind;
        public Dictionary<string, string> Fields = new Dictionary<string, string>();
        public string Get(string key) { string value; return Fields.TryGetValue(key, out value) ? value : ""; }
        public double Number(string key) { double value; return double.TryParse(Get(key), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out value) ? value : double.NaN; }
    }

    public sealed class StepResult
    {
        public uint Id;
        public string State, Reason, Raw;
        public double Start, Target, WindowMs, FirstMs, SettleMs, Overshoot, Sse, SsePeak, Samples, GapMs, Fault;
        public bool Valid, SseValid, BoardPass, CriteriaMatch, DirectStep;
        public Pid Gains;
    }

    public sealed class Assessment
    {
        public bool BoardComplete, WaveComplete, Pass, TimePass, OvershootPass, SsePass;
        public double WaveSpanMs, MaxHostGapMs, MaxFeedbackGapMs;
        public int UniqueFeedbackFrames;
        public List<string> Issues = new List<string>();
    }

    public sealed class TrialRecord
    {
        public string TrialId, Candidate, Mode, StartedAt, Condition, RunId;
        public bool Acceptance;
        public int Direction, Repeat;
        public Pid Gains;
        public StepResult Result;
        public Assessment Assessment;
        public List<Frame> Frames = new List<Frame>();
    }

    public sealed class TrialSpec
    {
        public string Candidate;
        public int Direction, Repeat;
        public Pid Gains;
    }

    public sealed class TrialPlan
    {
        public string Condition = "";
        public bool Acceptance;
        public List<TrialSpec> Trials = new List<TrialSpec>();
    }

    public sealed class WorkbenchSnapshot
    {
        public bool Connected, Demo, Busy;
        public string Stage, Detail, SessionDirectory, LastError;
        public Frame Latest;
        public Pid CurrentPid, Baseline;
        public Reply Status, StepCheck;
        public int Completed, Total;
        public long FramesReceived, InvalidLines;
        public List<TrialRecord> Records = new List<TrialRecord>();
    }
}
