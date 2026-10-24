using System;
using System.IO;
using System.Threading;
using System.Windows.Forms;

namespace YawTuner
{
    internal static class Program
    {
        [STAThread]
        private static int Main(string[] args)
        {
            Thread.CurrentThread.CurrentCulture = System.Globalization.CultureInfo.InvariantCulture;
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            bool demo = Array.IndexOf(args, "--demo") >= 0;
            bool aiPreview = Array.IndexOf(args, "--ai-preview") >= 0;
            string preview = null;
            for (int i = 0; i + 1 < args.Length; i++) if (args[i] == "--preview" || args[i] == "--ai-preview") preview = Path.GetFullPath(args[i + 1]);
            Application.ThreadException += delegate(object sender, ThreadExceptionEventArgs e) { MessageBox.Show(e.Exception.Message, "Yaw 调参工作台"); };
            MainForm form = new MainForm(demo || preview != null, preview, aiPreview);
            Application.Run(form);
            return 0;
        }
    }
}
