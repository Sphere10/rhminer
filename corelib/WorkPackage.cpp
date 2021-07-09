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

#include "precomp.h"

#include "corelib/WorkPackage.h"
#include "corelib/rh_endian.h"
#include "corelib/Log.h"
#include "MinersLib/Algo/sph_sha2.h"
#include "MinersLib/Algo/sph_blake.h"
#include "MinersLib/RandomHash/RandomHash.h"

using namespace std;
using namespace chrono;


MICRO_RTTI_DEFINE(WorkPackage)
MICRO_RTTI_DEFINE(PascalWorkPackage)
MICRO_RTTI_DEFINE(VNetWorkPackage)

WorkPackage::~WorkPackage()
{

}

WorkPackage::WorkPackage()
{
}

WorkPackage::WorkPackage(const WorkPackage& c)
{
    m_jobID = c.m_jobID;
    m_ntime = c.m_ntime;
    m_initTimeMS = c.m_initTimeMS;
    m_prev_hash = c.m_prev_hash;
    m_startNonce = c.m_startNonce;
    m_boundary = c.m_boundary;
    m_deviceBoundary = c.m_deviceBoundary;
    m_soloTargetPow = c.m_soloTargetPow;
    m_workDiff = c.m_workDiff;
    m_deviceDiff = c.m_deviceDiff;
    m_localyGenerated = c.m_localyGenerated;
    m_nonce2 = c.m_nonce2;
    m_fullHeader = c.m_fullHeader;
    m_coinbase1 = c.m_coinbase1;
    m_coinbase2 = c.m_coinbase2;
    m_nonce1 = c.m_nonce1;
    m_nonce2Size = c.m_nonce2Size;
    m_clean = c.m_clean;
    m_nonce2_64 = c.m_nonce2_64;
    m_isSolo = c.m_isSolo;
    m_server = c.m_server;
    m_extranoncePos = c.m_extranoncePos;
    m_noncePos = c.m_noncePos;
}

U64 WorkSolution::GetCurrentEvaluatingNonce()
{ 
    RHMINER_ASSERT(_eval_current_result_index < m_results.size());
    return m_results[_eval_current_result_index];
}

void WorkSolution::SetCurrentEvaluatingNonceIndex(U32 i)
{ 
    RHMINER_ASSERT(i < m_results.size());
    _eval_current_result_index = i; 
}


void WorkPackage::Init(const string& job, const h256& prevHash, const string& coinbase1, const string& coinbase2, const string& nTime, bool cleanWork, const string& nonce1, U32 nonce2Size, U64 extranonce, const string& server)
{
    m_prev_hash = prevHash;
    m_jobID = job;
    m_startNonce = extranonce;
    m_workDiff = 1.0;
    m_coinbase1 = coinbase1;
    m_coinbase2 = coinbase2;
    m_ntime = nTime;
    m_clean = cleanWork;
    m_nonce1 = nonce1;
    m_nonce2Size = nonce2Size;
    m_server = server;
    m_initTimeMS = TimeGetMilliSec();
}


void WorkPackage::ComputeWorkDiff(double& diff)
{
    if (diff == 0)
    {
        double d64, dcut64;

        d64 = 1.0f * truediffone;

        dcut64 = le256todouble(m_boundary.data());
        if (!dcut64)
            dcut64 = 1;
        
        m_workDiff = d64 / dcut64;
        diff = m_workDiff;
    }
    else    
    {
        m_workDiff = diff;
    }

    m_deviceDiff = m_workDiff;
}

void WorkPackage::ComputeTargetBoundary(h256& boundary, double& diff, double diffMultiplyer)
{
    static_assert(sizeof(boundary) == 32, "wrong");

    byte* targetPtr = boundary.data();

    U64 *data64, h64;
    double d64, dcut64;

    if (diff == 0.0)
    {
        PrintOut("Error. Invalid diff in work package");
        diff = 1.0;
    }

    d64 = diffMultiplyer * truediffone;
    d64 /= diff;

    dcut64 = d64 / bits192;
    h64 = (U64)dcut64;
    data64 = (U64 *)(targetPtr + 24);
    *data64 = h64;
    dcut64 = (double)h64;
    dcut64 *= bits192;
    d64 -= dcut64;

    dcut64 = d64 / bits128;
    h64 = (U64)dcut64;
    data64 = (U64 *)(targetPtr + 16);
    *data64 = h64;
    dcut64 = (double)h64;
    dcut64 *= bits128;
    d64 -= dcut64;

    dcut64 = d64 / bits64;
    h64 = (U64)dcut64;
    data64 = (U64 *)(targetPtr + 8);
    *data64 = h64;
    dcut64 = (double)h64;
    dcut64 *= bits64;
    d64 -= dcut64;

    h64 = (U64)d64;
    data64 = (U64 *)(targetPtr);
    *data64 = h64;
}



//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

extern void PascalHashV3(void *state, const void *input);
bool WorkPackage::IsSame(WorkPackage* work)
{
    RHMINER_ASSERT(m_fullHeader.size())
    return (m_jobID == work->m_jobID && 
            m_ntime == work->m_ntime && 
            m_fullHeader == work->m_fullHeader);
}

U32  WorkPackage::GetDeviceTargetUpperBits()
{
    return *(U32*)(m_deviceBoundary.data() + 28);
}

U64  WorkPackage::GetDeviceTargetUpperBits64()
{
    return *(U64*)(m_deviceBoundary.data() + 24);
}

bool WorkPackage::IsEmpty()
{
    if (m_fullHeader.size())
        return !std::any_of(m_fullHeader.begin(), m_fullHeader.end(), [](const char& c) { return c != '0'; });
    else
        return !m_prev_hash.isValid();
}



void WorkPackage::ComputeTargetBoundary()
{
    m_deviceDiff = m_workDiff;
    ComputeTargetBoundary(m_boundary, m_workDiff, 1.0f); 
    ComputeTargetBoundary(m_deviceBoundary, m_deviceDiff, 1.0f);
}

bool WorkPackage::Eval(WorkSolution* solPtr)
{
    if (solPtr->m_isFromCpuMiner)
        return true;

    U64 nonce = solPtr->GetCurrentEvaluatingNonce();

    solPtr->m_calcHash = RebuildNonce(nonce);
    bool res = IsHashLessThan(solPtr->m_calcHash, m_boundary);

    if (res)
    {
        return true;
    }

    return false;
}

h256 WorkPackage::RebuildNonce(U64 nonce)
{
    h256 solution;
    h256 solutionTmp;

    RandomHash_State opt;
    RandomHashResult res;
    RandomHash_Create(&opt);
    RandomHash_SetHeader(&opt, &m_fullHeader[0], (U32)m_fullHeader.size(), m_nonce2);
    RandomHash_Search(&opt, res, (U32)nonce);
    swab256((void*)solution.data(), (void*)&res.hashes[0]);
    RandomHash_Destroy(&opt);


    return solution;
}


//-------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------
WorkPackage* PascalWorkPackage::Clone()
{
    return new PascalWorkPackage(*this);
}

U64 PascalWorkPackage::ComputeNonce2(U32 nonce2)
{
    //updated injected nonce2
    string n2str;
    n2str.reserve(10);
    n2str = FormatString("%X", nonce2);
    while (n2str.length() < 8)
        n2str += '0';

    U64 nonce2_64;
    static_assert(sizeof(nonce2_64) <= 8, "fatal error");
    memcpy(&nonce2_64, n2str.c_str(), 8);
    return nonce2_64;
}

void PascalWorkPackage::UpdateHeader()
{
    m_nonce2_64 = ComputeNonce2(m_nonce2);

    string noncePlaceHolder;
    string payload;
    noncePlaceHolder = toHex((U64)0);

    string nonce2 = toHex(m_nonce2_64);
    payload = m_nonce1 + nonce2; //inject new nonce2 to reduce uncle rate when solomining

    if (m_coinbase1.length() == 0 || m_nonce1.length() == 0)
        throw RH_Exception("Coinbase data is wrong.");

    // 256 -> 180bytes
    // 283 -> 194bytes
    // 384 -> 244bytes
    // 512 -> 314bytes
    if (m_coinbase1.length() != 180)
    {
        RHMINER_EXIT_APP("Private key length is too long. Please chose a mining key with encryption type secp256k1.\n");
    }

    string headerStr;
    headerStr = m_coinbase1 + payload + m_coinbase2 + noncePlaceHolder;

    m_fullHeader = fromHex(headerStr, WhenError::Throw);

    if (m_fullHeader.size() != PascalHeaderSizeV5)
    {
        //sometimes the wallet send incorect package
        throw RH_Exception("Incorrect coinbase/wallet data.\n");
    }

    h32 ntime(m_ntime);
    U32 ntimeIdx = 48;
    ntimeIdx = (PascalHeaderSizeV5 / 4) - 2;

    ((uint32_t*)m_fullHeader.data())[ntimeIdx + 0] = RH_swap_u32(*(U32*)ntime.data());
    ((uint32_t*)m_fullHeader.data())[ntimeIdx + 1] = 0;
}


//-------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------
WorkPackage* VNetWorkPackage::Clone()
{
    return new VNetWorkPackage(*this);
}


U64 VNetWorkPackage::ComputeNonce2(U32 nonce2)
{
    mersenne_twister_state nonce2Rnd;
    merssen_twister_seed_fast(nonce2, &nonce2Rnd);
    U64 n2 = merssen_twister_rand_fast(&nonce2Rnd);
    n2 <<= 32;
    n2 |= merssen_twister_rand_fast(&nonce2Rnd);
    return n2;
}

void VNetWorkPackage::UpdateHeader()
{
    m_nonce2_64 = ComputeNonce2(m_nonce2);
    RHMINER_ASSERT(m_noncePos != 0);
    RHMINER_ASSERT(m_extranoncePos!= 0);

    m_fullHeader = fromHex(m_coinbase1, WhenError::Throw);

    DebugOut("Extra nonce %llx\n", m_nonce2_64);

    U8* head = static_cast<U8*>(m_fullHeader.data());
    reinterpret_cast<U64*>(head + m_extranoncePos)[0] = m_nonce2_64;
    //static_cast<U32*>(head + m_noncePos-4)[0] = RH_swap_u32(*(U32*)ntime.data());
    reinterpret_cast<U32*>(head + m_noncePos)[0] = 1;
    
}
