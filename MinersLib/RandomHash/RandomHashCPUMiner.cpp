/**
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
#include "MinersLib/RandomHash/RandomHashCPUMiner.h"
#include "MinersLib/Algo/sph_sha2.h"
#include "MinersLib/Algo/sph_blake.h"
#include "rhminer/ClientManager.h"
#include "corelib/miniweb.h"

const U64 VentingMultiplyer = 3;
extern bool g_useGPU;
extern bool g_cpuDisabled;


RandomHashCPUMiner::RandomHashCPUMiner(FarmFace& _farm, unsigned globalWorkMult, unsigned localWorkSize, U32 gpuIndex) :
    GenericCLMiner(_farm, globalWorkMult, localWorkSize, gpuIndex),
    m_firstKernelCycleDone(false, false)
{
}

RandomHashCPUMiner::~RandomHashCPUMiner()
{
    RandomHash_DestroyMany(m_randomHash2Array, g_cpuMinerThreads);
}

void RandomHashCPUMiner::InitFromFarm(U32 relativeIndex)
{
    //NOTE: WE need to force the local WS to 64 for UpdateWorkSize
    const U32 g_cpuRoundsThread = 64;
    m_localWorkSize = 64;
    UpdateWorkSize(g_cpuRoundsThread * g_cpuMinerThreads); //64
    RandomHash_DestroyMany(m_randomHash2Array, g_cpuMinerThreads);
    RandomHash_CreateMany(&m_randomHash2Array, g_cpuMinerThreads);    

    static_assert (sizeof(CPUKernelData::DataPackage::m_work1) >= sizeof(RandomHashResult), "BadHeader");

    //Make all CPU miner threads
    for (U32 i=0; i < (U32)g_cpuMinerThreads; i++)
    {
        CPUKernelData* kdata = (CPUKernelData*)RH_SysAlloc(sizeof(CPUKernelData));
        memset(kdata, 0, sizeof(CPUKernelData));
        kdata->m_id = i;
        //kdata->m_packages[0].m_requestPause = true;
        kdata->m_thread = new std::thread([&,kdata] { RandomHashCpuKernel(kdata); });
        m_cpuKernels.push_back(kdata);
        kdata->m_thread->detach();
    }
}

void RandomHashCPUMiner::RandomHashCpuKernel(CPUKernelData* kernelData)
{
    char tname[64];
    snprintf(tname, 64, "Cpu%d", (int)kernelData->m_id);
    setThreadName(tname);

    if (g_setProcessPrio != 1)
    {
        if (kernelData->m_id == GpuManager::CpuInfos.numberOfProcessors-1) 
            RH_SetThreadPriority(RH_ThreadPrio_Normal);
        else
        {
            if (g_useCPU && !g_useGPU)
                RH_SetThreadPriority(RH_ThreadPrio_High);
        }
    }


    U32 workWindow = m_globalWorkSizePerCPUMiner;
    //U32 gid = (U32)KernelOffsetManager::Increment(workWindow) - workWindow;
    U32 gid = KernelOffsetManager::GetNextSearchNonce();
    bool paused = false;
    U64 oldID = U64_Max;
    U64 cpuMonitor = 0;
    
    //Debug
    std::unordered_set<U32> partialNoncesProcessed(65535);
    U32 roundCnt = 0;
    U32 internalCollision = 0;

    while(!kernelData->m_abortThread)
    {
        RHMINER_RETURN_ON_EXIT_FLAG();
        U64 packageID = AtomicGet(kernelData->m_packageID);
        CPUKernelData::DataPackage* packageData = &kernelData->m_packages[packageID % CPUKernelData::PackagesCount];

        //handle internal pause
        if (oldID != packageID && paused)
        {
            paused = false;
        }

        //handle pause request from ::Pause()
        if (packageData->m_requestPause)
        {
            packageData->m_requestPause = 0;
            paused = true;
        }

       
        U32 workCount= 1;
        if (!paused)
        {
            if (oldID != packageID)
            {
                //gid = (U32)packageData->m_rndVal;
                {
                    if (!packageData->m_headerSize)
                    {
                        CpuSleep(100);
                        continue;
                    }

                    //{{{ TEMP TEMP TEMP
                    /*
                    {
                        std::lock_guard<std::mutex> g(*GlobalMiningPreset::I().m_roundNoncesMutex);
                        PrintOut("#%2d Partial nonce collision : %d over %d partials\n", kernelData->m_id, internalCollision, partialNoncesProcessed.size());
                        partialNoncesProcessed.clear();
                        internalCollision = 0;
                        roundCnt = 0;
                    }
                    */
                    //}}} TEMP TEMP TEMP

                    RandomHash_SetHeader(&m_randomHash2Array[kernelData->m_id], packageData->m_header.asU8,(U32)packageData->m_headerSize, (U32)packageData->m_nonce2);
                }
            }            

            if (*GpuManager::CpuInfos.pEnabled == false)
            {
                CpuSleep(100);
                continue;
            }

            //set start nonce here
            U32* work;
            RandomHashResult* resultv2 = (RandomHashResult*)packageData->m_work1;

            {
                U32 _n = m_randomHash2Array[kernelData->m_id].m_partiallyComputedNonceHeader;                
                if (_n == U32_Max)
                {
                    //search new nonce for this round
                    gid = KernelOffsetManager::GetNextSearchNonce();
                }
                /*
                else
                {
                    if (std::find(partialNoncesProcessed.begin(), partialNoncesProcessed.end(), _n) != partialNoncesProcessed.end())
                    {
                        PrintOut("Duplicate re-entering/Partial nonce found %x\n", _n);
                        internalCollision++;

                        //maybe reset the m_partiallyComputedNonceHeader here by force
                    }
                    else
                    {
                        partialNoncesProcessed.insert(_n);
                    }
                }
                */

                RandomHash_Search(&m_randomHash2Array[kernelData->m_id], *resultv2, gid);
                workCount = resultv2->count;
            }
//{{{ TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
#ifdef RH_ANTIDEBUG_WIN
            if (kernelData->m_id == 0)
            {
                if (!cpuMonitor)
                    cpuMonitor = TimeGetMilliSec() + 150;

                if (TimeGetMilliSec() > cpuMonitor)
                {
                    CheckRemoteDebuggerPresentAPI();
                    DetectDebuger_slow();
                    DetectHardwareBreakpoints_slow();
                    cpuMonitor = TimeGetMilliSec() + 150;
                }
            }
#endif
//}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP

            for(U32 w = 0; w < workCount; w++)
            {
                {
                    work = resultv2->hashes[w];
#ifdef RH_SCREEN_SAVER_MODE
                    if (w == 1 || w == 3)
                        ::ScreensaverFeed(resultv2->nonces[w]);
#endif
                    //{{{ TEMP TEMP TEMP
                    /*
                    roundCnt++;
                    U32 dup = GlobalMiningPreset::I().AddRoundNonce(resultv2->nonces[w], kernelData->m_id);
                    if (dup != U32_Max)
                    {
                        PrintOut("***** Duplicate nonce %x found on thread %d %s\n", resultv2->nonces[w], dup, dup ==kernelData->m_id ? "*":"" );
                        //RHMINER_ASSERT(dup == U32_Max);
                    }
                    */
                    //}}} TEMP TEMP TEMP
                }

                if (RH_swap_u32(*work) <= packageData->m_target)
                {
                    //Swapb256
                    U32 tmp[4] = { work[0], work[1], work[2], work[3] };
                    work[0] = RH_swap_u32(work[7]);
                    work[1] = RH_swap_u32(work[6]);
                    work[2] = RH_swap_u32(work[5]);
                    work[3] = RH_swap_u32(work[4]);
                    work[4] = RH_swap_u32(tmp[3]);
                    work[5] = RH_swap_u32(tmp[2]);
                    work[6] = RH_swap_u32(tmp[1]);
                    work[7] = RH_swap_u32(tmp[0]);
                    if (IsHashLessThan_32(work, packageData->m_targetFull))
                    {
                        std::vector<U64> foundNonce;
                        {
                            U32 nn = resultv2->nonces[w];
                            foundNonce.push_back(nn);
                        }

                        //{{{ TEMP TEMP TEMP
                        /*
                        bool ress = GlobalMiningPreset::I().AddSubmittedNonce(resultv2->nonces[w]);
                        if (!ress)
                            PrintOut("DUPLICATE submit %x\n", resultv2->nonces[w]);

                        if (workCount > 1)
                            PrintOut("Nonce %d/%d\n", w, workCount);
                        */
                        //}}} TEMP TEMP TEMP

                        SolutionSptr solPtr = MakeSubmitSolution(foundNonce, packageData->m_nonce2, true);
                        m_farm.submitProof(solPtr);
                        //resetOFfset = true;

                        //pause all solutions until next package in solo
                        if (kernelData->m_isSolo)
                        {
                            paused = true;
                        }

#ifdef RH_SCREEN_SAVER_MODE
                        extern void ScreensaverFoundNonce(U32 nonce);
                        ScreensaverFoundNonce((U32)foundNonce[0]);
#endif
                    }
                }
            }
        }
        else
        {
            CpuSleep(20);
        }
        oldID = packageID;

        AtomicAdd(kernelData->m_hashes, workCount);

    }
    AtomicSet(kernelData->m_abortThread, U32_Max);
}

void RandomHashCPUMiner::UpdateWorkSize(U32 absoluteVal)
{
    if (!absoluteVal)
        return;

    if (m_globalWorkSize && m_globalWorkSize != absoluteVal)
        RHMINER_EXIT_APP("Cpu miner cannot set an arbitrary worksize\n");    

    CLMinerBase::UpdateWorkSize(absoluteVal);

    m_globalWorkSize = absoluteVal;

    //spread WORK among all cpu miners -> absoluteVal = x * g_cpuMinerThreads
    m_globalWorkSizePerCPUMiner = m_globalWorkSize / g_cpuMinerThreads;
    if (!m_globalWorkSizePerCPUMiner)
        m_globalWorkSizePerCPUMiner = 1;
}


bool RandomHashCPUMiner::init(const WorkPackageSptr& work)
{
    m_isInitialized = true;
    //start hashrate counting
    if (m_hashCountTime == U64_Max)
        m_hashCountTime = TimeGetMilliSec();

    m_lastHashReading.resize(g_cpuMinerThreads);
    for (int i= 0;i < m_lastHashReading.size(); i++)
        m_lastHashReading[i] = 0;

    U32 seed;
    if (work->m_startNonce != 0)
        seed = (U32)work->m_startNonce;
    else
        seed = rand32();

    merssen_twister_seed_fast(seed, &m_rnd32);
    //merssen_twister_seed_fast(0x19873456, &m_rnd32); //for debug purpose

    return true;
}

bool RandomHashCPUMiner::configureGPU()
{
    return true;
}

PrepareWorkStatus RandomHashCPUMiner::PrepareWork(const WorkPackageSptr& workTempl, bool reuseCurrentWP)
{
    PrepareWorkStatus workStatus = GenericCLMiner::PrepareWork(workTempl, reuseCurrentWP);    
    
    //in case we're pause, the workStatus will be PrepareWork_Nothing, BUT we need to restart the cpu kernel...
    Guard g(m_pauseMutex);
    if (workStatus == PrepareWork_Nothing && m_isPaused == 1)
    {        
        CpuSleep(20);
    }
    else if (workStatus == PrepareWork_NewWork)
    {
        WorkPackageSptr wp = m_currentWp;
        SendWorkPackageToKernels(wp.get(), false);
    }
    
    m_isPaused = 0;
    return workStatus;
}

void RandomHashCPUMiner::SendWorkPackageToKernels(WorkPackage* wp, bool requestPause)
{
    // makeup CPU work pakcage
    const U32 target = m_currentWp->GetDeviceTargetUpperBits();
    U32 savedNonce2 = wp->m_nonce2;
    U64 nextPackage = 0;

    //reser search nonce itt
    KernelOffsetManager::ResetSearchNonce(merssen_twister_rand_fast(&m_rnd32));
    
    for (auto& data : m_cpuKernels)
    {
        data->m_isSolo = wp->m_isSolo;
        nextPackage = (AtomicGet(data->m_packageID) + 1);
        CPUKernelData::DataPackage* kernelData = &data->m_packages[nextPackage % CPUKernelData::PackagesCount];

        memset(kernelData, 0, sizeof(CPUKernelData::DataPackage));
        kernelData->m_requestPause = !!requestPause;
        kernelData->m_headerSize = wp->m_fullHeader.size();
        memcpy(kernelData->m_header.asU8, &wp->m_fullHeader[0], wp->m_fullHeader.size());
        RHMINER_ASSERT(wp->m_fullHeader.size() <= sizeof(kernelData->m_header.asU8));

#ifdef RH_RANDOMIZE_NONCE2
        if (!wp->m_isSolo && m_currentWp->m_extranoncePos)
        {
            //inject new n2
            kernelData->m_nonce2 = merssen_twister_rand_fast(&m_rnd32);
            U64 n264 = m_currentWp->ComputeNonce2((U32)kernelData->m_nonce2);
            //U32 offset = (U32)(m_currentWp->m_coinbase1.length() + m_currentWp->m_nonce1.length()) / 2;
            U32 offset = m_currentWp->m_extranoncePos;
            string n2str = toHex(n264);
            bytes xbfr = fromHex(n2str);
            memcpy(kernelData->m_header.asU8 + offset, &xbfr[0], sizeof(U64));
        }
#else
        kernelData->m_nonce2 = savedNonce2;
#endif
        //randomize start
        kernelData->m_rndVal = merssen_twister_rand_fast(&m_rnd32);

        if (wp->m_isSolo)
            memcpy(kernelData->m_targetFull, wp->m_soloTargetPow.data(), 32);
        else
            memcpy(kernelData->m_targetFull, wp->m_deviceBoundary.data(), 32);

        kernelData->m_target = target;
        
        RHMINER_ASSERT(wp->m_jobID.length() < sizeof(kernelData->m_workID)-1);
        memcpy(&kernelData->m_workID[0], wp->m_jobID.c_str(), wp->m_jobID.length()+1);
        kernelData->m_workID[wp->m_jobID.length()] = 0;

        //set next wp
        AtomicSet(data->m_packageID, nextPackage);
    }
    //PrintOutSilent("Pushing package %llu as work %s to kernels\n", (nextPackage % CPUKernelData::PackagesCount), wp->m_jobID.c_str());
}

void RandomHashCPUMiner::PauseCpuKernel()
{
    Guard g(m_pauseMutex);
    if (m_isPaused)
    {
        return;
    }
    
    //send pause package !
    WorkPackageSptr wp = m_currentWp;
    SendWorkPackageToKernels(wp.get(), true);
    m_isPaused = 1;
}

void RandomHashCPUMiner::Pause()
{
    GenericCLMiner::Pause();
    PauseCpuKernel();
}

void RandomHashCPUMiner::Kill()
{
    for (auto& k : m_cpuKernels)
    {
        k->m_abortThread = true;
    }
    CpuSleep(150);
    
    GenericCLMiner::Kill();
}



//unjam the queueKernel method
void RandomHashCPUMiner::SetWork(WorkPackageSptr _work)
{ 
    GenericCLMiner::SetWork(_work);
}

void RandomHashCPUMiner::QueueKernel()
{
    //wait 100ms
    S32 cnt = 5;
    while (!m_cpuKernels[0]->m_abortThread && cnt >= 0)
    {
        CpuSleep(20);
        cnt--;
        RHMINER_RETURN_ON_EXIT_FLAG();
    }

}

U64 RandomHashCPUMiner::GetHashRatePerSec() 
{
    U64 rate = 0;
    if (m_hashCountTime < U64_Max)
    {
        for(U32 i=0; i < m_cpuKernels.size(); i++)
        {
            U64 dt = 1;
            U64 kHash = AtomicGet(m_cpuKernels[i]->m_hashes);
            if (kHash > m_lastHashReading[i])
            {
                dt = kHash - m_lastHashReading[i];
                m_lastHashReading[i] = kHash;
            }
            rate += dt;
        }
    }

    return rate;
}

void RandomHashCPUMiner::AddHashCount(U64 hashes)
{ 
    m_hashCountTime = TimeGetMilliSec();
}

