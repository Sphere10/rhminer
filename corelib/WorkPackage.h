/**
 * Generic command line parser
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
#if defined(_WIN32_WINNT) || defined(IS_MAC_OS_X)
#include <json/json.h>
#else
#include <jsoncpp/json/json.h>
#endif

#include "corelib/MicroRTTI.h"
#include <condition_variable>
#include "FixedHash.h"
#include "utils.h"

//Randomize nonce2
#define RH_RANDOMIZE_NONCE2

struct WorkSolution;

class WorkPackage : public BaseRTTI
{
    MICRO_RTTI_ROOT(WorkPackage)
public:
    WorkPackage();
    explicit WorkPackage(bool isSolo) { m_isSolo = isSolo; }
    explicit WorkPackage(const WorkPackage& c);
    ~WorkPackage();
    virtual WorkPackage* Clone() = 0;

    void            Init(const string& job, const h256& prevHash, const string& coinbase1, const string& coinbase2, const string& nTime, bool cleanWork, const string& nonce1, U32 nonce2Size, U64 extranonce, const string& server);
    void            ComputeWorkDiff(double& diff);
    bool            IsSame(WorkPackage* work);
    U32             GetDeviceTargetUpperBits();
    U64             GetDeviceTargetUpperBits64();
    void            ComputeTargetBoundary();
    bool            IsEmpty();
    bool            Eval(WorkSolution* solPtr);
    h256            RebuildNonce(U64 nonce);

    virtual void    UpdateHeader() = 0;
    virtual U64	    ComputeNonce2(U32 nonce2) = 0;

    static void     ComputeTargetBoundary(h256& boundary, double& diff, double diffMultiplyer);

    string          m_jobID;
    string          m_ntime;   
    U64             m_initTimeMS = 0;
    h256            m_prev_hash;
    U64             m_startNonce = 0; 
    mutable h256    m_boundary;                
    mutable h256    m_deviceBoundary;
    mutable h256    m_soloTargetPow;
    double          m_workDiff = 1.0;          
    double          m_deviceDiff = 1.0;
    bool            m_localyGenerated = false; 
    U32             m_nonce2 = U32_Max;
    bytes           m_fullHeader;
    string          m_coinbase1;
    string          m_coinbase2;
    string          m_nonce1;
    unsigned        m_nonce2Size = 0; 
    bool            m_clean = false;
    U64             m_nonce2_64 = 0;
    string          m_server;
    std::set<U64>   m_submittedNonces;

    //solo stuff
    bool            m_isSolo = false;    

    //header's dynamic field pos
    U32             m_extranoncePos = 0;
    U32             m_noncePos = 0;
};

typedef std::shared_ptr<WorkPackage> WorkPackageSptr;

//--------------------------------------------------------------------------------------------------------------------------------------
//PascalCoin WorkPackage
class PascalWorkPackage : public WorkPackage
{
    MICRO_RTTI(PascalWorkPackage, WorkPackage)
public:
    PascalWorkPackage():WorkPackage() {}
    explicit PascalWorkPackage(bool isSolo) :WorkPackage(isSolo) {}
    explicit PascalWorkPackage(const WorkPackage& c) :WorkPackage(c) {}

    virtual WorkPackage* Clone();
    virtual void    UpdateHeader();
    virtual U64	    ComputeNonce2(U32 nonce2);
};

//--------------------------------------------------------------------------------------------------------------------------------------

class VNetWorkPackage : public WorkPackage
{
    MICRO_RTTI(VNetWorkPackage, WorkPackage)
public:
    VNetWorkPackage() :WorkPackage() {}
    explicit VNetWorkPackage(bool isSolo) :WorkPackage(isSolo) {}
    explicit VNetWorkPackage(const WorkPackage& c) :WorkPackage(c) {}

    virtual WorkPackage* Clone();
    virtual void    UpdateHeader();
    virtual U64	    ComputeNonce2(U32 nonce2);

    U32             m_minerNonce = 0;
    U32             m_minerTagPos = 0;
    U32             m_minerTagSize = 0;
    U32             m_timeStamp = 0;
};


struct WorkSolution
{
    U64                 GetCurrentEvaluatingNonce();
    void                SetCurrentEvaluatingNonceIndex(U32 i);
    bool                Eval() { return m_work->Eval(this);  }
        
    h256                m_calcHash;
    std::vector<U64>    m_results;
    U32                 m_gpuIndex = 0;
    bool                m_isFromCpuMiner = false;
    WorkPackageSptr     m_work;

private:
    U32                 _eval_current_result_index = 0; //index used to travers m_results and eval each value

};
typedef std::shared_ptr<WorkSolution> SolutionSptr;


struct WorkPackageFactory
{
    static WorkPackageSptr Create(string coinName, bool isSolo)
    {
        if (coinName == RH_COIN_NAME_PASC)
            return WorkPackageSptr(new PascalWorkPackage(isSolo));
        if (coinName == RH_COIN_NAME_VNET)
            return WorkPackageSptr(new VNetWorkPackage(isSolo));
        return 0;
    }
};
