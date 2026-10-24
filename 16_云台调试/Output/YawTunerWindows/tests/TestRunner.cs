using System;

namespace YawTuner.Tests
{
    public static class TestRunner
    {
        public static int Main(string[] args)
        {
            try
            {
                bool aiOnly = Array.IndexOf(args, "--ai-only") >= 0;
                int count = aiOnly ? AutoTuneTests.Run() : ProtocolTests.Run() + EngineTests.Run() + AiApiTests.Run() + AutoTuneTests.Run();
                if (Array.IndexOf(args, "--scenario") >= 0) count += AutoTuneTests.RunScenario();
                Console.WriteLine("PASS: " + count + " offline protocol, assessment, AI HTTP and automation tests. No physical serial port opened or real model API called.");
                return 0;
            }
            catch (Exception error) { Console.Error.WriteLine(error); return 1; }
        }
    }
}
