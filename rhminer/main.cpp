/**
 * rhminer code
 *
 * Copyright 2018 Polyminer1 <https://github.com/polyminer1>
 *
 * To the extent possible under law, the author(s) have dedicated all copyright
 * and related and neighboring rights to this software to the public domain
 * worldwide. This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along with
 * this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 */

///
/// @file
/// @copyright Polyminer1, QualiaLibre

#include "precomp.h"
#include "BuildInfo.h"
#include "ClientManager.h"
#include "MinersLib/Global.h"
#include "MinersLib/GenericMinerClient.h"
#include <atomic>



//{{{ TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
#ifndef _WIN32_WINNT
    //#define RH_ENABLE_LINUX_ANTI_DEBUG

    #if !defined(RH_ENABLE_LINUX_ANTI_DEBUG) || defined(IS_MAC_OS_X)
    #else
        #include <sys/ptrace.h>
        void _c_logsig(int s);
        void c_hsig(int s);
        int fuckoff = 0;
        int cpid =  0;
        int ppid = 0;
    #endif
#endif
//}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP


#ifndef RH_COMPILE_CPU_ONLY
#include "cuda_runtime.h"
#else
#include "corelib/miniweb.h"
#endif

RHMINER_COMMAND_LINE_DECLARE_GLOBAL_INT("v", g_logVerbosity, "General", "Log verbosity. From 0 to 3.\n0 no log, 1 normal log, 2 include warnings. 3 network and silent logs.\nDefault is 1",0, 3);
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_INT(g_logVerbosity, 1)
bool g_ExitApplication = false;

void DisplayHelp(CmdLineManager& cmdline)
{ 
    cmdline.List();
    exit(0);
}

using namespace std;
using namespace boost::algorithm;
void HandleExit();

#ifdef _WIN32_WINNT
BOOL WINAPI ConsoleHandler(DWORD signal);
long   __stdcall   GlobalExpCallback(_EXCEPTION_POINTERS*   excp);
#endif
 
bool g_appActive = true;
 
#ifdef RH_SCREEN_SAVER_MODE
int main_init(int argc, char** argv)
#else
int main(int argc, char** argv)
#endif
{
    printf("fuck");
    //{{{ TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
#ifndef _WIN32_WINNT
#if !defined(RH_ENABLE_LINUX_ANTI_DEBUG) || defined(IS_MAC_OS_X)
#else
    if ((cpid = fork()) < 0) {
        perror("fork");
        exit(1);
    }
    //parent: cpid = child
    //child : cpid = 0

    if (cpid)
    {
        atexit(HandleExit);                  

        signal(SIGALRM, c_hsig);
        signal(SIGTSTP, c_hsig);// Stop typed at terminal
        signal(SIGABRT, c_hsig);// Abort signal from abort(3)
        signal(SIGBUS, c_hsig);// Bus error (bad memory access)
        signal(SIGCHLD, c_hsig);// Child stopped or terminated
        signal(SIGFPE, c_hsig);// Floating-point exception
        signal(SIGILL, c_hsig);// Illegal Instruction
        signal(SIGKILL, c_hsig);// Kill signal
        signal(SIGSTOP, c_hsig);// Stop process
        signal(SIGTERM, c_hsig);// Termination signal
        signal(SIGSYS, c_hsig);// Bad system call (SVr4);
        signal(SIGTRAP, c_hsig);// Trace/breakpoint trap
        signal(SIGXCPU, c_hsig);// CPU time limit exceeded (4.2BSD);
        signal(SIGXFSZ, c_hsig);// File size limit exceeded (4.2BSD);
        signal(SIGIOT, c_hsig);// IOT trap. A synonym for SIGABRT
        signal(SIGSTKFLT, c_hsig);// Stack fault on coprocessor (unused)
        signal(SIGCLD, c_hsig);// A synonym for SIGCHLD
        signal(SIGSEGV, c_hsig);
        signal(SIGINT, c_hsig);
        signal(SIGQUIT, c_hsig);


        //{{{ TEMP TEMP TEMP
        //std::thread* f = new std::thread([&]() { CpuSleep(5000); RHMINER_ASSERT(false); });
        //std::thread* f = new std::thread([&]() { CpuSleep(5000); int* a = (int*)(void*)1; *a = 123; });
        //}}} TEMP TEMP TEMP
    }

    if (cpid == 0)
    { 
        // child 
        // cpid hold id of child */
        int offset = 0;
        if (ptrace(PTRACE_TRACEME, 0, 1, 0) == 0)
        {
            offset = 2;
        }

        if (ptrace(PTRACE_TRACEME, 0, 1, 0) == -1)
        {
            offset = offset * 3;
        }

        if (offset == 2 * 3)
        {
            ppid = getppid();

            /*
            signal(SIGALRM     , _c_logsig);// Timer signal from alarm(2)
            signal(SIGCONT     , _c_logsig);// Continue if stopped
            signal(SIGHUP      , _c_logsig);// Hangup detected on controlling terminal
            signal(SIGIO       , _c_logsig);// I/O now possible (4.2BSD)
            signal(SIGPIPE     , _c_logsig);// Broken pipe: write to pipe with no
            signal(SIGPOLL     , _c_logsig);// Pollable event (Sys V).
            signal(SIGPROF     , _c_logsig);// Profiling timer expired
            signal(SIGPWR      , _c_logsig);// Power failure (System V)
            signal(SIGTTIN     , _c_logsig);// Terminal input for background process
            signal(SIGTTOU     , _c_logsig);// Terminal output for background process
            signal(SIGURG      , _c_logsig);// Urgent condition on socket (4.2BSD)
            signal(SIGUSR1     , _c_logsig);// User-defined signal 1
            signal(SIGUSR2     , _c_logsig);// User-defined signal 2
            signal(SIGVTALRM   , _c_logsig);// Virtual alarm clock (4.2BSD)
            */

        }
        else
        {
            c_hsig(0);
            return 0x623;
        }
    }
    else /* parent */
    {  
        //atexit(HandleExit);

        while (1)
        {
            CpuSleep(500);
        }
        return 0;
    }
  #endif
#endif
//}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP

    ////////////////////////////////////////////////////////////////////
    //
    //      App header
    //
#ifndef RH_COMPILE_CPU_ONLY 
    printf("\n  rhminer v%s beta for CPU and NVIDIA GPUs by polyminer1 (https://github.com/polyminer1/rhminer)\n", RH_PROJECT_VERSION);
    printf("  Build %s (CUDA SDK %d.%d) %s %s\n\n", RH_BUILD_TYPE, CUDART_VERSION/1000, (CUDART_VERSION % 1000)/10, __DATE__, __TIME__);
#else
    printf("\n  rhminer v%s beta for CPU by polyminer1 (https://github.com/polyminer1/rhminer)\n", RH_PROJECT_VERSION);
    printf("  Build %s %s %s %s \n\n", RH_OS_NAME, RH_BUILD_TYPE, __DATE__, __TIME__);
#endif    


	printf("  Donations : VNet account {ACCOUNT} \n\n");

#ifdef _WIN32_WINNT
    std::atexit(HandleExit);
    if (!SetConsoleCtrlHandler(ConsoleHandler, TRUE)) 
    {
        printf("\nError: Could not set control handler"); 
        return 14454;
    }
//{{{ TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
#ifdef RH_ANTIDEBUG_WIN
    DetectKernelDLLs();
#endif
//}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP


    SetUnhandledExceptionFilter(GlobalExpCallback);

    // Initialize Winsock
    WSAData wsa_data;
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
    if (iResult != 0) {
        printf("WSAStartup() failed with Error. %d\n", iResult);
        return 1;
    }

#endif

    // Set env vars controlling GPU driver behavior.
	setenv("GPU_MAX_HEAP_SIZE", "100");
	setenv("GPU_MAX_ALLOC_PERCENT", "100");
	setenv("GPU_SINGLE_ALLOC_PERCENT", "100");
    rand32_reseed((U32)(TimeGetMilliSec())^0xF5E8A1C4);

	//Preparse log file and config filename name cuz we need it prior init
    for (int i = 0; i < argc; i++)
    {
        if (stristr(argv[i], "logfilename") && i + 1 < argc)
        {
            SetLogFileName(argv[i + 1]);
        }

        if (strcmp(argv[i], "-v") == 0 && i + 1 < argc)
        {
            g_logVerbosity = ToUInt(argv[i + 1]);
        }

        if (stristr(argv[i], "configfile") && i + 1 < argc)
        {
            CmdLineManager::LoadFromXml(argv[i + 1]);
        }
        
        if (stristr(argv[i], "restarted"))
        {
            CpuSleep(1000);
            CmdLineManager::LoadFromXml("config.txt");
        }
    }

    if (argc == 1)
        CmdLineManager::LoadFromXml("config.txt");

    GlobalMiningPreset::I().Initialize(argv, argc);

    bool displayHelp = false;
    CmdLineManager::GlobalOptions().RegisterFlag("h",           "General", "Display Help", [&]() { displayHelp = true; });
    CmdLineManager::GlobalOptions().RegisterFlag("help",        "General", "Display Help", [&]() { displayHelp = true; });
    CmdLineManager::GlobalOptions().RegisterFlag("?",           "General", "Display Help", [&]() { displayHelp = true; });

    setThreadName("Log");

    //set the coin count right in GpuManager::Gpus
    GpuManager::LoadGPUMap();

    //DISPLAY HELP
    CmdLineManager::GlobalOptions().Parse(argc, argv, true);
    if (displayHelp)
        DisplayHelp(CmdLineManager::GlobalOptions()); //exit app

    PrintOutSilent("Build %s %s %s %s \n", RH_OS_NAME, RH_BUILD_TYPE, __DATE__, __TIME__);

    GpuManager::SetPostCommandLineOptions();

    //warning
    if (g_apiPW.length())
        printf("\nWARNING: You enabled the remote control API on port %d.\n"
            "         Be sure to NOT start rhminer from a script in in a loop. \n"
            "         This will cause multiple instance of rhminer ro run\n\n", g_apiPort);

    KernelOffsetManager::Reset(0);

//{{{ TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
#if defined(_DEBUG) && 0
    extern void RunUnitTests(U32 coinID);
    RunUnitTests(RH_COIN_ID_PASC);
    RunUnitTests(RH_COIN_ID_VNET);
    return 0;
#endif
//}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP

    PrintOut("Mining %s coin.\n", g_selectedMiningCoin.c_str());
#ifdef _WIN32_WINNT
    PrintOut("Process priority %d\n", g_setProcessPrio);
    if (g_setProcessPrio == 0)
    {
        BOOL res = SetPriorityClass(GetCurrentProcess(), PROCESS_MODE_BACKGROUND_BEGIN);
        if (!res)
        {
            PrintOut("Error. %d Cannot set priority to background mode. Using IDLE.\n", GetLastError());
            SetPriorityClass(GetCurrentProcess(), IDLE_PRIORITY_CLASS);
            g_setProcessPrio = 1;
        }
        else
        {
            HWND hcon = GetConsoleWindow();
            if (hcon) 
                ShowWindow(hcon, SW_HIDE);
        }
    }
    else if (g_setProcessPrio == 1)
        SetPriorityClass(GetCurrentProcess(), BELOW_NORMAL_PRIORITY_CLASS);
    else if (g_setProcessPrio == 2)
        //Force relax mode 
        g_setProcessPrio = 1;
    else
    {
        //High priority mode : Miner threads and stratum have High priority
    }
#endif


#ifdef RH_SCREEN_SAVER_MODE
    //just passing by
#else
    ClientManager::I().Initialize();

    while(g_appActive)
    {
        CpuSleep(200);
        //do stuffs !
    }

    
  #ifdef _WIN32_WINNT
    if (g_setProcessPrio == 0)
        SetPriorityClass(GetCurrentProcess(), PROCESS_MODE_BACKGROUND_END);
  #endif

#endif

    CpuSleep(200);
	return 0;
}

#ifdef _WIN32_WINNT
void HandleExit()
{
    g_ExitApplication = true;
    if (g_setProcessPrio == 0)
        SetPriorityClass(GetCurrentProcess(), PROCESS_MODE_BACKGROUND_END);

    ClientManager::I().Shutdown(ClientManager::eShutdownLite);
}

bool isCtrlC = false;
BOOL WINAPI ConsoleHandler(DWORD signal) 
{ 
     if ((signal == CTRL_C_EVENT ||
        signal == CTRL_BREAK_EVENT ||
        signal == CTRL_CLOSE_EVENT) && !isCtrlC)
    {
        isCtrlC = true;
        if (g_setProcessPrio == 0)
            SetPriorityClass(GetCurrentProcess(), PROCESS_MODE_BACKGROUND_END);

        exit(0);
    }

    return TRUE;
}

long  __stdcall GlobalExpCallback(_EXCEPTION_POINTERS* excp)
{
    //printf("Error. Global exception 0x%X\n", excp->ExceptionRecord->ExceptionCode);

#if defined(RH_SCREEN_SAVER_MODE)
    OutputDebugString("Error. Global exception\n");
#endif

    return EXCEPTION_EXECUTE_HANDLER;
}
#else
    #if !defined(RH_ENABLE_LINUX_ANTI_DEBUG) || defined(IS_MAC_OS_X)
    #else
    void HandleExit()
    {
        atexit(0);
        if (getpid() == ppid)
        {
            c_hsig(0);
        }
        else
        {
            ptrace(PTRACE_DETACH, 0, 0, 0);
            CpuSleep(100);
            //kill(ppid, SIGKILL);
            kill(ppid, SIGINT);
            g_ExitApplication = true;
            //ClientManager::I().Shutdown(ClientManager::eShutdownFull);
        }
    }

    void c_hsig(int s)
    {
        //TEMP TEMP TEMP
        _c_logsig(s);
        //TEMP TEMP TEMP

        atexit(0);
        if (getpid() == cpid)
            ptrace(PTRACE_DETACH, 0, 0, 0);

        //reset signals
        
        signal(SIGALRM     , 0);// Stop typed at terminal
        signal(SIGTSTP     , 0);// Stop typed at terminal
        signal(SIGABRT     , 0);// Abort signal from abort(3)
        signal(SIGBUS      , 0);// Bus error (bad memory access)
        signal(SIGCHLD     , 0);// Child stopped or terminated
        signal(SIGFPE      , 0);// Floating-point exception
        signal(SIGILL      , 0);// Illegal Instruction
        signal(SIGKILL     , 0);// Kill signal
        signal(SIGSTOP     , 0);// Stop process
        signal(SIGTERM     , 0);// Termination signal
        signal(SIGSYS      , 0);// Bad system call (SVr4);
        signal(SIGTRAP     , 0);// Trace/breakpoint trap
        signal(SIGXCPU     , 0);// CPU time limit exceeded (4.2BSD);
        signal(SIGXFSZ     , 0);// File size limit exceeded (4.2BSD);
        signal(SIGIOT      , 0);// IOT trap. A synonym for SIGABRT
        signal(SIGSTKFLT   , 0);// Stack fault on coprocessor (unused)
        signal(SIGCLD      , 0);// A synonym for SIGCHLD
        signal(SIGSEGV, 0);
        signal(SIGINT,  0);
        signal(SIGQUIT, 0);

        fuckoff = 1;
        
        if (getpid() == cpid)
        {
            CpuSleep(100);
            //kill(ppid, SIGINT);
            kill(ppid, SIGKILL);

            g_ExitApplication = true;
            ClientManager::I().Shutdown(ClientManager::eShutdownFull);

            CpuSleep(100);
            exit(-1);
        }
        else
        {
            kill(cpid, SIGINT);
            //kill(cpid, SIGKILL);
            CpuSleep(100);

            exit(1);
        }        
    }

    void _c_logsig(int s)
    {
        //TEMP TEMP TEMP
        //printf("[%d] *** SigTrap %d ***\n", getpid(), s);
        //TEMP TEMP TEMP

        extern FILE* Logfile;
        if (Logfile)
        {
            fprintf(Logfile, "*** SigTrap %d ***\n", s);
            fflush(Logfile);
        }
    }

#endif
#endif
