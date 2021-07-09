/**
 * Global miner data source code implementation
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
#include "MinersLib/Global.h"
#include "corelib/Worker.h"
#include "corelib/WorkPackage.h"
#include "MinersLib/CLMinerBase.h"
#include "MinersLib/StratumClient.h"
#include "rhminer/ClientManager.h"
#include "corelib/miniweb.h"
#include "MinersLib/RandomHash/RandomHashCLMiner.h"
#include "MinersLib/RandomHash/RandomHashCPUMiner.h"

#ifndef RH_COMPILE_CPU_ONLY
#include "MinersLib/RandomHash/RandomHashHostCudaMiner.h"
#endif

RHMINER_COMMAND_LINE_DEFINE_GLOBAL_STRING(g_logFileNameDummy, "");
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_STRING(g_configFileNameDummy, "");
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_BOOL(g_useCPU, true);
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_BOOL(g_restared, false);
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_INT(g_testPerformance, 0);
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_INT(g_testPerformanceThreads, 0);
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_INT(g_setProcessPrio, 3);
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_INT(g_memoryBoostLevel, RH_OPT_UNSET);
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_INT(g_sseOptimization, 0); 

string g_selectedMiningCoin = RH_COIN_NAME_VNET;

bool g_useGPU = false;
U32  g_cpuMinerThreads = 0;
bool g_disableMaxGpuThreadSafety = false;
extern void StopMiniWeb();
extern bool g_appActive;

const U64 t1M = 1000 * 60;
const U64 t1H = t1M * 60;
const U64 t24H = t1H * 24;
const U64 t3_8M = (U64)(3.8f*t1M); 
const U64 t45S = (U64)(45*1000);

GlobalMiningPreset& GlobalMiningPreset::I()
{
    static GlobalMiningPreset I;
    return I;
}


GlobalMiningPreset::GlobalMiningPreset()
{
    devFeeMutex = new std::mutex;
    m_startTimeMS = TimeGetMilliSec();

    m_coinNameToID[RH_COIN_NAME_VNET] = RH_COIN_ID_VNET;
    m_coinNameToID[RH_COIN_NAME_PASC] = RH_COIN_ID_PASC;
    for (auto kv : m_coinNameToID)
        m_coinIDtoName[kv.second] = kv.first;


    //{{{ TEMP TEMP TEMP
    //m_roundNoncesMutex = new std::mutex;
    //m_roundNonces.resize(128);
    //m_roundNoncesCollision.resize(128);
    //}}} TEMP TEMP TEMP

#ifndef RH_COMPILE_CPU_ONLY
    CmdLineManager::GlobalOptions().RegisterValue("gputhreads", "Gpu", "Cuda thread count. ex: -gputhreads  100 launche 100 threads on selected gpu", [&](const string& val) 
    { 
        std::vector<string> istr = GetTokens(val, ",");
        if (istr.size() == 1)
        {
            for (auto& g : GpuManager::Gpus)
            {
                g.setGpuThreadCount = ToUInt(istr[0]);
            }
        }
        else if (istr.size() > 1)
        {
            for (U32 i = 0; i < istr.size(); i++)
            {
                if (i < GpuManager::Gpus.size())
                    GpuManager::Gpus[i].setGpuThreadCount = ToUInt(istr[i]);
            }
        }
    });

    CmdLineManager::GlobalOptions().RegisterValue("gpu", "Gpu", "Enable indiviaual GPU by their index. GPU not in the list will be disabled. ex: -gpu 0,3,4.", [&](const string& val) 
    { 
        //List GPU : AMD, NVidia and the rests
        std::vector<string> idstr = GetTokens(val, ",");
        
        for(auto& i : idstr)
        {
            U32 id = ToUInt(i);
            if (id < GpuManager::Gpus.size())
            {
                GpuManager::Gpus[id].enabled = true;
            }
            else
            {
                PrintOut("GPU%d does not exist\n", id);
                RHMINER_EXIT_APP("");
            }
        }
    });
#endif //RH_COMPILE_CPU_ONLY

    CmdLineManager::GlobalOptions().RegisterFlag("list", "General", "List all gpu and cpu in the system", [&]() 
    {
        GpuManager::listGPU(); 
        exit(0);
    });

    CmdLineManager::GlobalOptions().RegisterFlag("completelist", "General", "Exhaustive list of all devices in the system", [&]() { GpuManager::listDevices(); });
}

void GlobalMiningPreset::FailOverURL(const string& val)
{
    string url = val;
    ReplaceStringALL(url, "stratum+tcp://", "");
    ReplaceStringALL(url, "http://", "");

    size_t p = url.find_last_of(":");
    if (p != string::npos)
    {
        m_presets.m_farmFailOverURL = url.substr(0, p);
        if (p + 1 <= url.length())
            m_presets.m_fport = url.substr(p + 1);
    }
    else
    {
        m_presets.m_farmFailOverURL = url;
    }
}

void GlobalMiningPreset::SetStratumInfo(const string& val)
{
    CmdLineManager::GlobalOptions().PreParseSymbol("su");

    if (val.find("http://") != string::npos)
        m_presets.m_soloOvertStratum = true;

    string url = val;
    ReplaceStringALL(url, "stratum+tcp://", "");
    ReplaceStringALL(url, "http://", "");

    int userPos = (int)url.find("/");
    if (userPos != url.npos)
    {
        if (m_presets.m_user.empty())
        {
            string user = url.substr(userPos + 1);
            if (user.length())
                m_presets.m_user = user;
        }
        url = url.substr(0, userPos);
    }

    size_t p = url.find_last_of(":");
    if (p != string::npos)
    {
        m_presets.m_farmURL = url.substr(0, p);
        if (p + 1 <= url.length())
            m_presets.m_port = url.substr(p + 1);
    }
    else
    {
        m_presets.m_farmURL = url;
        m_presets.m_port = "1379";
    }
}


void GlobalMiningPreset::Initialize(char** argv, int argc)
{
    CmdLineManager::GlobalOptions().RegisterValue("s", "Network", "Stratum/wallet server address:port.\nNOTE: You can also use http://address to connect to local wallet.", [&](const string& val) 
    { 
        SetStratumInfo(val); 
    });

    CmdLineManager::GlobalOptions().RegisterValue("su", "Network", "Stratum user", [&](const string& val) {  m_presets.m_user = val; });
    CmdLineManager::GlobalOptions().RegisterValue("pw", "Network", "Stratum password", [&](const string& val) { m_presets.m_pass = val; });
    CmdLineManager::GlobalOptions().RegisterValue("fo", "Network", "Failover address:port for stratum or local wallet", [&](const string& val) { FailOverURL(val); });
    CmdLineManager::GlobalOptions().RegisterValue("fou", "Network", "Failover user for stratum of a local wallet", [&](const string& val) { m_presets.m_fuser = val; });
    CmdLineManager::GlobalOptions().RegisterValue("fop", "Network", "Failover password for stratum or local wallet", [&](const string& val) { m_presets.m_fpass = val; });
    CmdLineManager::GlobalOptions().RegisterValue("r", "Network", "Retries connection count for stratum or local wallet", [&](const string& val) { m_presets.m_maxFarmRetries = ToInt(val); });
    
    CmdLineManager::GlobalOptions().RegisterValueMultiple("diff", "General", "Set local difficulyu. ex: -diff 999", [&](const string& val)
    { 
        m_localDifficulty = ToFloat(val);
        if (m_localDifficulty != 0.0f)
            PrintOut("Setting local difficulty to %.4f\n", m_localDifficulty);
    });

    CmdLineManager::GlobalOptions().RegisterValueMultiple("cputhreads", "Gpu", "Number of CPU miner threads when mining with CPU. ex: -cpu -cputhreads 4.\nNOTE: adding + before thread count will disable the maximum thread count safety of one thread per core/hyperthread.\nUse this option at your own risk.", [&](const string& val)
    {
        string cmt = val;
        if (cmt.length() && cmt[0] == '+')
        {
            g_disableMaxGpuThreadSafety = true;
            cmt = cmt.substr(1);
        }

        g_cpuMinerThreads = ToUInt(cmt);
    });

    CmdLineManager::GlobalOptions().RegisterValueMultiple("processorsaffinity", "General", "On windows only. Force miner to only run on selected logical core processors.\nex: -processorsaffinity 0,3 will make the miner run only on logical core #0 and #3.\nWARNING: Changing this value will affect GPU mining.", [&](const string& val)
    { 
#ifdef _WIN32_WINNT
        CmdLineManager::GlobalOptions().PreParseSymbol("cputhreads");

        strings values = GetTokens(val, ",");
        std::vector<U32> iVals;
        if (iVals.size() > GpuManager::CpuInfos.numberOfProcessors)
        {
            PrintOutCritical("Error. You selected %d logical cores while there is %d on this system", iVals.size(), GpuManager::CpuInfos.numberOfProcessors);
            RHMINER_EXIT_APP("");
        }
        
        for(auto& v:values)
        {
            U64 core = (U64)ToUInt(v);
            if (core < GpuManager::CpuInfos.numberOfProcessors)
            {
                GpuManager::CpuInfos.UserSelectedCores |= (1LLU << core);
                GpuManager::CpuInfos.UserSelectedCoresCount++;
            }
        }
        PrintOut("Warning: Setting processor affinity WILL affect gpu mining. Use with caution.\n");

        if (GpuManager::CpuInfos.UserSelectedCoresCount > g_cpuMinerThreads)
            PrintOut("Warning: You selected %d logical cores to mine but only %d working threads.\n", GpuManager::CpuInfos.UserSelectedCoresCount, g_cpuMinerThreads);
#else
        PrintOut("-processorsaffinity not implemented yet\n");
#endif
    });

    CmdLineManager::GlobalOptions().RegisterValue("coin", "General", "Select coin to mine: Supported coins are [VNET, PASC]", [&](const string& val)
    {
        //validate supported coin
        g_selectedMiningCoin = ToUpper(val);
        if (m_coinNameToID.find(g_selectedMiningCoin) == m_coinNameToID.end())
            RHMINER_EXIT_APP("Unsupported coin");
    });
}

FarmPreset* GlobalMiningPreset::Get()
{
    return &m_presets;
};


U64 GetTimeRangeRnd(U64 minMS, U64 maxMS)
{
    U64 spot= 0;
    spot = minMS + (rand32() % (maxMS-minMS));
    return spot;
}

bool GlobalMiningPreset::DetectDevfeeOvertime()
{
    const U64 overTime = 90 * 1000;
    U64 endOfCurrentDevFeeTimesMS = AtomicGet(m_endOfCurrentDevFeeTimesMS);
    return (endOfCurrentDevFeeTimesMS && (TimeGetMilliSec() > (endOfCurrentDevFeeTimesMS + overTime)));
}

U32 GlobalMiningPreset::GetUpTimeMS()
{
    return (U32)(TimeGetMilliSec() - m_startTimeMS);
}

Miner* GlobalMiningPreset::CreateMiner(CreatorClasType type, FarmFace& _farm, U32 gpuIndex)
{
#ifndef RH_COMPILE_CPU_ONLY
    if (type == ClassOpenCL)
        return new RandomHashCLMiner(_farm, 0, 0, gpuIndex);
    if (type == ClassNvidia)
        return new RandomHashHostCudaMiner(_farm, 0, 0, gpuIndex);
#endif
    if (type == ClassCPU)
        return new RandomHashCPUMiner(_farm, 0, 0, gpuIndex);

    RHMINER_EXIT_APP("critical");
}

void GlobalMiningPreset::SetRestart(ERestartMode val)
{
    AtomicSet(m_requestRestart, (U32)val);
    if (val == eExternalRestart)
    {
#ifdef _WIN32_WINNT
        char exeFN[MAX_PATH];
        *exeFN;
        GetModuleFileName(0, exeFN, sizeof(exeFN));
        if (strlen(exeFN))
        {
            char dir[MAX_PATH];
            char f[MAX_PATH];
            char fn[MAX_PATH];
            char ex[MAX_PATH];
            _splitpath(exeFN, dir, f, fn, ex);
            strncat(dir, f, sizeof(dir));

            string cmd;
            if (GlobalMiningPreset::I().GetPendingConfigFile().length()) 
                cmd += " -restarted ";

            cmd += CmdLineManager::GlobalOptions().GetArgsList();
            PrintOutSilent("Restarting to %s\n", cmd.c_str());

            char cwdDir[1024] = "";
            __getcwd(cwdDir, sizeof(cwdDir));

            STARTUPINFO si = {};
            si.cb = sizeof si;
            PROCESS_INFORMATION pi = {};
            if (!CreateProcess(exeFN, (char*)cmd.c_str(), 0, FALSE, 0, 0, 0, cwdDir, &si, &pi))
            {
                PrintOutCritical("Cannot Restart rhminer.\n");
            }
            else
            {
                exit(0);
            }
        }
        else
            PrintOutCritical("Cannot get module filename for restart. Restart aborted.\n");

#else
        int*   processId = new int;
        char  *exec_path_name = (char*)malloc(strlen(CmdLineManager::GlobalOptions().GetArgv()[0]) + 1);
        strcpy(exec_path_name, CmdLineManager::GlobalOptions().GetArgv()[0]);

        //stop all now!
        StopMiniWeb();
        CloseLog();
        g_appActive = true;

        char** agvI = CmdLineManager::GlobalOptions().GetArgv();
        char *argv2[128 + 1];
        char** agvI2 = argv2;
        while (*agvI)
        {
            *agvI2 = (char*)malloc(strlen(*agvI) + 1);
            strcpy(*agvI2, *agvI);
            agvI++;
            agvI2++;
        }

        if (GlobalMiningPreset::I().GetPendingConfigFile().length())
        {
            const char* restart = "-restarted";
            *agvI2 = (char*)malloc(strlen(restart) + 1);
            strcpy(*agvI2, restart);
            agvI2++;
        }
        *agvI2 = 0;

        pid_t pid;
        pid = fork();
        if (pid == 0)
        {
            char *envp[] = { NULL };
            int err = execve(exec_path_name, argv2, envp);
            if (err != 0)
                printf("exec error %d\n", errno);
        }
        exit(0);
#endif
    }
}

void GlobalMiningPreset::RequestReboot()
{
    U32 lastVal = AtomicSet(m_requestReboot, 1);
    if (lastVal == 1)
        return;

    char basePath[1024] = "";
    __getcwd(basePath, sizeof(basePath));

#ifdef _WIN32_WINNT    
    strncat(basePath, "\\", sizeof(basePath)-1);
    strncat(basePath, "reboot.bat", sizeof(basePath) - 1);
#else
    strncat(basePath, "/", sizeof(basePath) - 1);
    strncat(basePath, "reboot.sh", sizeof(basePath) - 1);
#endif
    if (GetFileSize(basePath) == U64_Max)
        PrintOut("Launch Error. file. %s does not exists\n", basePath);
    else
    {
        string cmd;
#ifdef _WIN32_WINNT
        cmd = "start \"reboot\" \"";
        cmd += basePath;
        cmd += "\"";
#else
        cmd = "sh ";
        cmd += basePath;
#endif
        system(cmd.c_str());
    }
}

void GlobalMiningPreset::DoPerformanceTest()
{
    U32 headerSize = 160;
    if (g_selectedMiningCoin != "VNET")
        headerSize = PascalHeaderSizeV5;

    vector<RandomHashResult> out_hash2;

    mersenne_twister_state rnd;
    merssen_twister_seed(0xF923A401, &rnd);
    
    if (g_testPerformanceThreads == 0 || (U32)g_testPerformanceThreads > GpuManager::CpuInfos.numberOfProcessors)
        g_testPerformanceThreads = GpuManager::CpuInfos.numberOfProcessors;
        
    const size_t ThreadCount = g_testPerformanceThreads;
    out_hash2.resize(g_testPerformanceThreads);
    RandomHash_State* g_threadsData2=0;
    RandomHash_CreateMany(&g_threadsData2, ThreadCount);

    U32 nonce2 = 0;
    
    PrintOut("CPU: %s\n", GpuManager::CpuInfos.cpuBrandName.c_str());
    PrintOut("Testing raw cpu performance for %d sec on %d threads\n", g_testPerformance, ThreadCount);
    
    U64 timeout[] = { 5 * 1000, (U64)g_testPerformance * 1000 };

    std::vector<U64> hashes;
    hashes.resize(ThreadCount);

    auto kernelFunc = [&](void* allStates, U32 startNonce, U64 to)
    {
        U32 gid = 0;
        while (TimeGetMilliSec() < to)
        {
            RandomHash_Search((RandomHash_State*)allStates, out_hash2[startNonce], startNonce + gid++);
            hashes[startNonce] += out_hash2[startNonce].count;
        }
    };

    PrintOut("Warming up...\n");
    for(U32 timeoutID = 0; timeoutID < 2; timeoutID++)
    {        
        vector<U32> input;
        input.resize((headerSize / 4) + 1);
        for (int i = 0; i < headerSize / 4; i++)
            input[i] = merssen_twister_rand(&rnd);
        input[((headerSize-4) / 4)] = 0;

        //NOTE: the header must allready be in device mem (via SetWork)
        for(int i=0; i < ThreadCount; i++)
        {
            RandomHash_SetHeader(&g_threadsData2[i], (U8*)&input[0], headerSize, nonce2);
        }

        {
            std::vector<std::thread> threads(ThreadCount);
            U32 gid=0;
            for(int i = 0; i < ThreadCount; i++) 
            {
                threads[i] = std::thread([&] 
                {
                    U32 _gid = AtomicIncrement(gid);
                    RH_SetThreadPriority(RH_ThreadPrio_High);
                    kernelFunc((void*)&g_threadsData2[_gid-1], _gid-1, TimeGetMilliSec() + timeout[timeoutID]); 
                }
                );
            }
            for(std::thread & thread : threads) 
                thread.join();
        }
        PrintOut("Testing performance...\n");
        CpuSleep(20);
        if (timeoutID == 0)
        {
            for (auto& h : hashes)
                h = 0;
        }
    }

    U64 hashCnt = 0;
    for (auto h : hashes)
        hashCnt += h;
    PrintOut("RandomHash speed is %.2f H/S\n", hashCnt / (float)g_testPerformance);

    
    //TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP 
    /*extern map<U32, U32> md2_sizes;
    PrintOut("~\nMD2 sizes : \n");
    for(auto x : md2_sizes)
        PrintOut("~size %d is %d\n", x.first, x.second);
    */
    /*
    {
        extern map<U64, U32> Trx_sizes[8];
        for (int t = 0; t < 8; t++)
        {
            PrintOut("~Trx %d : \n", t);
            for (auto x : Trx_sizes[t])
            {
                if (t == 3 || t == 4)
                    PrintOut("~size %8llx is %8d\n", x.first, x.second);
                else
                    PrintOut("~size %3lld is %8d\n", x.first, x.second);
            }
            PrintOut("~\n");
        }
    }
    */
    //extern void PrintStats();
    //PrintStats();
    //TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP 
    
    exit(0);
}

bool GlobalMiningPreset::UpdateToDevModeState(string& connectionParams)
{
    std::lock_guard<std::mutex> g(*devFeeMutex);

    //{{{ TTEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
    return false;
    //}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP

    if (TimeGetMilliSec() > m_devFeeTimer24hMS)
    {
        U64 nowMS = TimeGetMilliSec();
        rand32_reseed((U32)(nowMS^0xB8BDA50F));

        m_devFeeTimer24hMS = nowMS + t24H;
        m_nextDevFeeTimesMS.clear();
        m_totalDevFreeTimeToDayMS = 0;
        const U64 devPeriod = (U64)(t3_8M);
        U64 nextDevTime;
        if (devPeriod > 45)
            nextDevTime = nowMS + (15 * t1M);
        else
            nextDevTime = nowMS + GetTimeRangeRnd((15 * t1M), (55 * t1M)-devPeriod);
        
        m_nextDevFeeTimesMS.push_back(nextDevTime);
        U32 maxItt = 0;
        while(m_nextDevFeeTimesMS.size() < 4)
        {
            int k;
            U64 val = nowMS + GetTimeRangeRnd((3 * t1M), (24 * t1H)-devPeriod);
            for (k = 1; k < m_nextDevFeeTimesMS.size(); k++)
            {
                U64 dt = m_nextDevFeeTimesMS[k] - m_nextDevFeeTimesMS[k - 1];
                if (dt > (5 * t1M))
                    continue;
                else
                    break;
            } 
            if (k == (int)m_nextDevFeeTimesMS.size())
                m_nextDevFeeTimesMS.push_back(val); 
            
            maxItt++;
            if (maxItt == 1024)
            {
                rand32_reseed((U32)(TimeGetMilliSec()^0xF862AE69));
                maxItt = 0;
            }
        }
        std::sort(m_nextDevFeeTimesMS.begin(), m_nextDevFeeTimesMS.end(),[](U64&s1, U64& s2) { return s1 < s2; });

        //{{{ TEMP TEMP TEMP
        //m_nextDevFeeTimesMS[0] = nowMS + t1M;
        //m_nextDevFeeTimesMS[1] = nowMS + t1M + devPeriod + t1M;
        //m_nextDevFeeTimesMS[2] = nowMS + t1M + devPeriod + t1M + devPeriod + t1M;
        //m_nextDevFeeTimesMS[3] = nowMS + t1M + devPeriod + t1M + devPeriod + t1M + devPeriod;
        //}}} TEMP TEMP TEMP

    }
    else
    {
        U64 endOfCurrentDevFeeTimesMS = AtomicGet(m_endOfCurrentDevFeeTimesMS);
        if (endOfCurrentDevFeeTimesMS &&
            TimeGetMilliSec() > endOfCurrentDevFeeTimesMS)
        {
            m_totalDevFreeTimeToDayMS += TimeGetMilliSec() - m_currentDevFeeTimesMS;

            AtomicSet(m_endOfCurrentDevFeeTimesMS, 0);
            PrintOutCritical("End of DevFee mode.\n\n Thank you, you're an awesome person :) \n\n");
          
            connectionParams = "";
            return true;
        }

        if ((m_nextDevFeeTimesMS.size() &&
            m_currentDevFeeTimesMS != m_nextDevFeeTimesMS[0] &&
            TimeGetMilliSec() > m_nextDevFeeTimesMS[0]) ||
            connectionParams.find("_retry_") != string::npos)
        {
            if (connectionParams.find("_retry_") == string::npos)
            {
                m_currentDevFeeTimesMS = m_nextDevFeeTimesMS[0];
                AtomicSet(m_endOfCurrentDevFeeTimesMS, m_currentDevFeeTimesMS + (U64)(t3_8M));
                PrintOutCritical("Switching to DevFee mode.\n");

                m_nextDevFeeTimesMS.erase(m_nextDevFeeTimesMS.begin());
            }
            connectionParams = "";
            vector<int> s;     
            string b1,b2;
            vector<int> w;
            switch(rand32()%4)
            { 
            case 0:w = {0x1b12f9, 0x123542, 0xae72d4, 0xae72d4, 0x275fcb, 0x71f216, 0x66e0fa, 0x863f0b, 0x71f216, 0x1b12f9, 0x995282, 0xae72d4, 0x995282, 0xae72d4, 0xae7586, }; break;
            case 1:w = {0x1b12f9, 0x123542, 0xae72d4, 0x1b12f9, 0x71f216, 0x1b12f9, 0x66e0fa, 0x863f0b, 0x275fcb, 0x1b12f9, 0x995282, 0xae72d4, 0x995282, 0x1b12f9, 0x1b15aa, }; break;
            case 2:w = {0x1b12f9, 0x123542, 0xae72d4, 0x8120c0, 0xae72d4, 0x71f216, 0x66e0fa, 0x863f0b, 0x5721f5, 0x66e0fa, 0x995282, 0xae72d4, 0x995282, 0x5d6c22, 0x5d6edd, }; break;
            case 3:w = {0x1b12f9, 0x123542, 0x1b12f9, 0x5d6c22, 0x5d6c22, 0x66e0fa, 0x66e0fa, 0x863f0b, 0x123542, 0x123542, 0x995282, 0xae72d4, 0x995282, 0x123542, 0x1237f7, }; break;      
            default:
                w = {0x1b12f9, 0x123542, 0x1b12f9, 0x5d6c22, 0x5d6c22, 0x66e0fa, 0x66e0fa, 0x863f0b, 0x123542, 0x123542, 0x995282, 0xae72d4, 0x995282, 0x123542, 0x1237f7, };
            }

            switch(rand32()%5)
            {
                //hashplaza.org	3336 0x5727FF
                case 0: s = {0x3fcde, 0x6c24b9, 0x21b0e5, 0x3fcde, 0xae9b31, 0x25963a, 0x6c24b9, 0x8aede6, 0x6c24b9, 0x995282, 0x2552c3, 0xaff517, 0x41af04, 0x90ff63, 0x123542, 0x123542, 0x123542, 0x5721f5, 0x5727ff, }; break;
                //hashplaza.org	3337 0x2765D6
                case 1: s = {0x3fcde, 0x6c24b9, 0x21b0e5, 0x3fcde, 0xae9b31, 0x25963a, 0x6c24b9, 0x8aede6, 0x6c24b9, 0x995282, 0x2552c3, 0xaff517, 0x41af04, 0x90ff63, 0x123542, 0x123542, 0x123542, 0x275fcb, 0x2765d6, }; break;
                //hashplaza.org	3338 0x8126CC
                case 2: s = {0x3fcde, 0x6c24b9, 0x21b0e5, 0x3fcde, 0xae9b31, 0x25963a, 0x6c24b9, 0x8aede6, 0x6c24b9, 0x995282, 0x2552c3, 0xaff517, 0x41af04, 0x90ff63, 0x123542, 0x123542, 0x123542, 0x8120c0, 0x8126cc, }; break;
                //fastpool.xyz	10097 0x2765D6
                case 3: s = {0x89efe9, 0x6c24b9, 0x21b0e5, 0xa9f679, 0xae9b31, 0x2552c3, 0x2552c3, 0x25963a, 0x995282, 0x28e7b9, 0x8b8942, 0x8aede6, 0x90ff63, 0x1b12f9, 0xae72d4, 0xae72d4, 0x1aa42f, 0x275fcb, 0x2765d6, }; break;
                //fastpool.xyz	10098 0x8126CC
                case 4: s = { 0x89efe9, 0x6c24b9, 0x21b0e5, 0xa9f679, 0xae9b31, 0x2552c3, 0x2552c3, 0x25963a, 0x995282, 0x28e7b9, 0x8b8942, 0x8aede6, 0x90ff63, 0x1b12f9, 0xae72d4, 0xae72d4, 0x1aa42f, 0x8120c0, 0x8126cc, }; break;
            default:
                s = {0x3fcde, 0x6c24b9, 0x21b0e5, 0x3fcde, 0xae9b31, 0x25963a, 0x6c24b9, 0x8aede6, 0x6c24b9, 0x995282, 0x2552c3, 0xaff517, 0x41af04, 0x90ff63, 0x123542, 0x123542, 0x123542, 0x5721f5, 0x5727ff, };
            }
            
            RH_DECRYPT(s, b1);
            RH_DECRYPT(w, b2);
            connectionParams = FormatString("%s\t%s", b1.c_str(), b2.c_str());

            string test;
            RH_DECRYPT(s, test);
            RHMINER_ASSERT(test.length());
            return true;
        }
    }
    
    return false;
}
//{{{ TEMP TEMP TEMP
/*
bool GlobalMiningPreset::FindRoundNonce(U32 nonce, U32 threadNum)
{
    std::lock_guard<std::mutex> g(*m_roundNoncesMutex);
    RHMINER_ASSERT(threadNum <= 12);
    
    for (S32 i = 0; i < m_roundNonces.size(); i++)
    {
        if (m_roundNonces[i].find(nonce) != m_roundNonces[i].end())
        {
            if (threadNum == i)
                return true;
        }
    }
            
    return false;
}

U32 GlobalMiningPreset::AddRoundNonce(U32 nonce, U32 threadNum)
{
    std::lock_guard<std::mutex> g(*m_roundNoncesMutex);
    RHMINER_ASSERT(threadNum <= 12);
    
    m_totalRounds++;

    for (S32 i = 0; i < m_roundNonces.size(); i++)
    {
        if (m_roundNonces[i].find(nonce) != m_roundNonces[i].end())
        {
            m_roundNoncesCollision[threadNum]++;
            return i;
        }
    }
            
    RHMINER_ASSERT(threadNum < m_roundNonces.size());
    m_roundNonces[threadNum].insert(nonce);
    return 0xFFFFFFFF;
}
//false -> duplicate
 bool GlobalMiningPreset::AddSubmittedNonce(U32 nonce)
{
    std::lock_guard<std::mutex> g(*m_roundNoncesMutex);
    if (m_submittedNonces.find(nonce) != m_submittedNonces.end())
        return false;
    m_submittedNonces.insert(nonce);
    return true;
}
 void GlobalMiningPreset::ClearRoundNonce()
{
    std::lock_guard<std::mutex> g(*m_roundNoncesMutex);
    string t;
    U32 totalColl = 0;
    
    if (!m_totalRounds)
        return;

    for (S32 i = 0; i < m_roundNonces.size(); i++)
    {
        if (m_roundNonces[i].size())
        {
            totalColl += m_roundNoncesCollision[i];
            float colRate = (float)m_roundNoncesCollision[i]/(float)m_totalRounds;
            colRate = colRate < 0.00001 ? 0.0f : colRate;
                    
            PrintOut("#%2d Round counts %d / %d coll (%.5f%%) %s \n", i, m_roundNonces[i].size(), m_roundNoncesCollision[i], colRate, colRate > 0.01 ? "* WARNING *" : "");
            m_roundNoncesCollision[i] = 0;
        }
        m_roundNonces[i].clear();
    }

    PrintOut("AVG search nonces %d Collision (%.5f) over %d rounds\n", totalColl, totalColl/(float)m_totalRounds, m_totalRounds);
    PrintOut("Submitted total : %d \n", m_submittedNonces.size());
    m_totalRounds = 0;
    m_submittedNonces.clear();
}
*/
//}}} TEMP TEMP TEMP

