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

#pragma once

#include "MinersLib/GenericCLMiner.h"
#include "MinersLib/CPUMiner.h"
#include "MinersLib/RandomHash/RandomHash.h"

class RandomHashCPUMiner: public GenericCLMiner
{
    
public:
    RandomHashCPUMiner(FarmFace& _farm, unsigned globalWorkMult, unsigned localWorkSize, U32 gpuIndex);
    ~RandomHashCPUMiner();

    virtual bool init(const WorkPackageSptr& work);
    virtual void InitFromFarm(U32 relativeIndex);
    static bool configureGPU();
    virtual PlatformType GetPlatformType() { return PlatformType_CPU; }

    virtual void Pause();
    virtual void Kill();
    virtual void SetWork(WorkPackageSptr _work);


protected:
    
    vector<CPUKernelData*>   m_cpuKernels; //paged alloc
    Event  m_firstKernelCycleDone;
    U32    m_setWorkComming = 0;
    U32    m_waitingForKernel = 1;
    U32    m_lastIttCount = 0;
    U32    m_isPaused = 0;
    std::mutex  m_pauseMutex;
    U32    m_globalWorkSizePerCPUMiner = 0;
    mersenne_twister_state   m_rnd32;

    //Cut cl miner stuff
    virtual KernelCodeAndFuctions GetKernelsCodeAndFunctions() { return KernelCodeAndFuctions(); }
    virtual void ClearKernelOutputBuffer() {}
    virtual void EvalKernelResult() {}

    //generic CPU mining
    virtual PrepareWorkStatus PrepareWork(const WorkPackageSptr& workTempl, bool reuseCurrentWP = false);
    virtual void SendWorkPackageToKernels(WorkPackage* wp, bool requestPause = false);
    virtual void QueueKernel();
    virtual void AddHashCount(U64 hashes);
    virtual U64 GetHashRatePerSec();
    std::vector<U64> m_lastHashReading;

    void PauseCpuKernel();
    void UpdateWorkSize(U32 absoluteVal);
    void RandomHashCpuKernel(CPUKernelData* kernelData); //The Kernel
    RandomHash_State* m_randomHash2Array = 0;    
};

