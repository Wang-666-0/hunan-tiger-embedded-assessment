using System;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Threading;

namespace YawTuner
{
    // SerialPort is deliberately owned by a dedicated reader, never the UI thread.
    public sealed class SerialTransport : ITransport
    {
        private readonly string portName;
        private readonly int baudRate;
        private readonly object sendGate = new object();
        private SerialPort port;
        private Thread reader;
        private volatile bool running;
        private int failureRaised;
        public event Action<string> LineReceived;
        public event Action<string> Failed;
        public bool IsDemo { get { return false; } }
        public bool IsOpen { get { return running && port != null && port.IsOpen; } }

        public SerialTransport(string name, int baud)
        {
            if (string.IsNullOrWhiteSpace(name)) throw new ArgumentException("请选择串口。");
            if (baud < 1200 || baud > 4000000) throw new ArgumentOutOfRangeException("baud");
            portName = name; baudRate = baud;
        }

        public static string[] Ports()
        {
            string[] names = SerialPort.GetPortNames();
            Array.Sort(names, StringComparer.OrdinalIgnoreCase);
            return names;
        }

        public void Open()
        {
            lock (sendGate)
            {
                if (running) throw new InvalidOperationException("串口已经打开。");
                failureRaised = 0;
                port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
                port.Encoding = Encoding.ASCII;
                port.Handshake = Handshake.None;
                port.DtrEnable = false; port.RtsEnable = false;
                port.ReadTimeout = 200; port.WriteTimeout = 1000;
                port.ReadBufferSize = 65536;
                try
                {
                    port.Open(); port.DiscardInBuffer(); port.DiscardOutBuffer();
                    // Empty line only resynchronizes the firmware line parser; it cannot move an axis.
                    port.Write("\r\n");
                    running = true;
                    reader = new Thread(ReadLoop); reader.IsBackground = true;
                    reader.Name = "YawTuner serial RX"; reader.Start();
                }
                catch { port.Dispose(); port = null; throw; }
            }
        }

        public void Send(string command)
        {
            if (command == null || command.IndexOfAny(new char[] { '\r', '\n', '\0' }) >= 0)
                throw new ArgumentException("不允许多行串口命令。");
            lock (sendGate)
            {
                if (!IsOpen) throw new IOException("串口已经关闭。");
                port.Write(command + "\r\n");
            }
        }

        private void ReadLoop()
        {
            byte[] bytes = new byte[4096];
            StringBuilder line = new StringBuilder(1024);
            try
            {
                while (running)
                {
                    int count;
                    try { count = port.Read(bytes, 0, bytes.Length); }
                    catch (TimeoutException) { continue; }
                    for (int i = 0; i < count && running; i++)
                    {
                        byte b = bytes[i];
                        if (b == 13 || b == 10)
                        {
                            if (line.Length != 0)
                            {
                                string complete = line.ToString(); line.Length = 0;
                                Action<string> handler = LineReceived;
                                if (handler != null) handler(complete);
                            }
                        }
                        else
                        {
                            if (b < 32 || b > 126) throw new IOException("收到非 ASCII 数据，请核对波特率和固件。");
                            if (line.Length >= 16384) throw new IOException("串口单行超过 16 KB，停止接收以避免错帧。");
                            line.Append((char)b);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                if (running && Interlocked.Exchange(ref failureRaised, 1) == 0)
                {
                    Action<string> handler = Failed;
                    if (handler != null) handler("串口接收失败：" + ex.Message);
                }
            }
            finally { running = false; }
        }

        public void Close()
        {
            running = false;
            lock (sendGate)
            {
                if (port != null)
                {
                    try { port.Close(); } catch (IOException) { }
                    catch (InvalidOperationException) { }
                }
            }
            if (reader != null && reader != Thread.CurrentThread) reader.Join(1500);
            lock (sendGate)
            {
                if (port != null) { port.Dispose(); port = null; }
            }
            reader = null;
        }

        public void Dispose() { Close(); }
    }
}
