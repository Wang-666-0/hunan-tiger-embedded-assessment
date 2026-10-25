using System;
using System.IO;
using System.Text;
using System.Runtime.InteropServices;
using System.Diagnostics;
public static class VofaCrashTrace {
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool DebugActiveProcess(uint pid);
  [DllImport("kernel32.dll")] static extern bool DebugActiveProcessStop(uint pid);
  [DllImport("kernel32.dll")] static extern bool DebugSetProcessKillOnExit(bool kill);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool WaitForDebugEvent(IntPtr ev,uint ms);
  [DllImport("kernel32.dll")] static extern bool ContinueDebugEvent(uint pid,uint tid,uint status);
  [DllImport("kernel32.dll",SetLastError=true)] static extern IntPtr OpenProcess(uint access,bool inherit,uint pid);
  [DllImport("kernel32.dll",SetLastError=true)] static extern IntPtr OpenThread(uint access,bool inherit,uint tid);
  [DllImport("kernel32.dll")] static extern bool CloseHandle(IntPtr h);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool GetThreadContext(IntPtr thread,IntPtr ctx);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode)] static extern IntPtr GetModuleHandle(string name);
  [DllImport("kernel32.dll",CharSet=CharSet.Ansi)] static extern IntPtr GetProcAddress(IntPtr mod,string name);
  [DllImport("dbghelp.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern bool SymInitializeW(IntPtr process,string path,bool invade);
  [DllImport("dbghelp.dll")] static extern uint SymSetOptions(uint options);
  [DllImport("dbghelp.dll")] static extern bool SymCleanup(IntPtr process);
  [DllImport("dbghelp.dll",SetLastError=true)] static extern bool SymFromAddr(IntPtr process,ulong address,out ulong displacement,IntPtr info);
  [DllImport("dbghelp.dll",SetLastError=true)] static extern bool StackWalk64(uint machine,IntPtr process,IntPtr thread,IntPtr frame,IntPtr context,IntPtr read,IntPtr table,IntPtr mod,IntPtr translate);
  [DllImport("dbghelp.dll",SetLastError=true)] static extern bool MiniDumpWriteDump(IntPtr process,uint pid,IntPtr file,uint type,IntPtr exception,IntPtr user,IntPtr callback);
  static string Symbol(IntPtr process,ulong addr) {
    IntPtr info=Marshal.AllocHGlobal(2048);
    try {
      Marshal.Copy(new byte[2048],0,info,2048); Marshal.WriteInt32(info,88); Marshal.WriteInt32(info,80,1024);
      ulong delta;
      if(SymFromAddr(process,addr,out delta,info)) return Marshal.PtrToStringAnsi(IntPtr.Add(info,84))+"+0x"+delta.ToString("x");
      return "?";
    } finally {Marshal.FreeHGlobal(info);}
  }
  static void Capture(uint pid,uint tid,string directory) {
    IntPtr process=OpenProcess(0x001F0FFF,false,pid), thread=OpenThread(0x001FFFFF,false,tid);
    if(process==IntPtr.Zero || thread==IntPtr.Zero)throw new Exception("Open handle: "+Marshal.GetLastWin32Error());
    var log=new StringBuilder();
    IntPtr raw=Marshal.AllocHGlobal(1280), frame=Marshal.AllocHGlobal(1024);
    IntPtr context=new IntPtr((raw.ToInt64()+15)&~15L);
    try {
      Marshal.Copy(new byte[1232],0,context,1232); Marshal.WriteInt32(context,48,0x00100003);
      if(!GetThreadContext(thread,context))throw new Exception("GetThreadContext: "+Marshal.GetLastWin32Error());
      ulong rip=(ulong)Marshal.ReadInt64(context,248), rsp=(ulong)Marshal.ReadInt64(context,152), rbp=(ulong)Marshal.ReadInt64(context,160);
      log.AppendLine("RIP="+rip.ToString("x")+" RSP="+rsp.ToString("x")+" RBP="+rbp.ToString("x"));
      using(var file=new FileStream(Path.Combine(directory,"crash-captured.dmp"),FileMode.Create,FileAccess.Write,FileShare.None)) {
        bool ok=MiniDumpWriteDump(process,pid,file.SafeFileHandle.DangerousGetHandle(),0x1000,IntPtr.Zero,IntPtr.Zero,IntPtr.Zero);
        log.AppendLine("MiniDump="+ok+" error="+Marshal.GetLastWin32Error());
      }
      SymSetOptions(0x00000206);
      log.AppendLine("Symbols="+SymInitializeW(process,@"D:\VOFA+\x64;C:\Windows\System32",true));
      foreach(ProcessModule m in Process.GetProcessById((int)pid).Modules)log.AppendLine("MODULE "+m.BaseAddress.ToInt64().ToString("x")+" "+m.ModuleMemorySize.ToString("x")+" "+m.FileName);
      Marshal.Copy(new byte[1024],0,frame,1024);
      Marshal.WriteInt64(frame,0,(long)rip);Marshal.WriteInt32(frame,12,3);
      Marshal.WriteInt64(frame,32,(long)rbp);Marshal.WriteInt32(frame,44,3);
      Marshal.WriteInt64(frame,48,(long)rsp);Marshal.WriteInt32(frame,60,3);
      IntPtr dbg=GetModuleHandle("dbghelp.dll");
      IntPtr table=GetProcAddress(dbg,"SymFunctionTableAccess64"),mod=GetProcAddress(dbg,"SymGetModuleBase64");
      log.AppendLine("STACK");
      for(int i=0;i<50000;i++) {
        ulong pc=(ulong)Marshal.ReadInt64(frame,0);
        log.AppendLine(i+" 0x"+pc.ToString("x")+" "+Symbol(process,pc));
        if(!StackWalk64(0x8664,process,thread,frame,context,IntPtr.Zero,table,mod,IntPtr.Zero))break;
      }
      SymCleanup(process);
    } finally {
      File.WriteAllText(Path.Combine(directory,"crash-stack.txt"),log.ToString());
      Marshal.FreeHGlobal(raw);Marshal.FreeHGlobal(frame);CloseHandle(thread);CloseHandle(process);
    }
  }
  public static string Run(uint pid,string directory) {
    if(!DebugActiveProcess(pid))throw new Exception("Debug attach: "+Marshal.GetLastWin32Error());
    DebugSetProcessKillOnExit(false);
    IntPtr ev=Marshal.AllocHGlobal(256);
    var timer=Stopwatch.StartNew();
    try {
      while(timer.ElapsedMilliseconds<50000) {
        if(!WaitForDebugEvent(ev,1000))continue;
        uint code=(uint)Marshal.ReadInt32(ev,0),eventPid=(uint)Marshal.ReadInt32(ev,4),tid=(uint)Marshal.ReadInt32(ev,8),status=0x00010002;
        bool captured=false;
        if(code==1) {
          uint exception=(uint)Marshal.ReadInt32(ev,16);
          if(exception==0xc00000fd || exception==0xc0000005) { Capture(eventPid,tid,directory); captured=true; }
          if(exception!=0x80000003 && exception!=0x80000004)status=0x80010001;
        }
        ContinueDebugEvent(eventPid,tid,status);
        if(captured)return "Captured exception";
        if(code==5)return "Process exited";
      }
      return "No exception within 50 seconds";
    } finally { DebugActiveProcessStop(pid);Marshal.FreeHGlobal(ev); }
  }
}
