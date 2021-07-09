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
#include "StratumClientPASC.h"
#include "RandomHash/Common.h"

void StratumClientPASC::ProcessMiningNotifySolo(Json::Value& jsondata)
{
    string retVal;
    Json::Value arayparams = jsondata.get("params", Json::Value::null);
    Json::Value params = arayparams.get((Json::Value::ArrayIndex)0, 0);

    U32 i = 0;
    U32 block = params.get("block", 0).asUInt();
    U64 timestamp = params.get("timestamp", 0).asUInt64();
    if (block != m_lastBlock || timestamp != m_lastTimestamp)
    {
        bool cleanFlag = false;
        if (m_lastBlock != block || timestamp - m_lastTimestamp > 61 || TimeGetMilliSec() > m_cleanTime)
        {
            cleanFlag = true;
            m_cleanTime = TimeGetMilliSec() + 61 * 1000;
        }

        m_lastBlock = block;
        m_lastTimestamp = timestamp;

        U32 version = params.get("version", 0).asUInt();
        string coinbase1 = params.get("part1", "").asString(); //part 1
        string payload = params.get("payload_start", "").asString();
        string coinbase2 = params.get("part3", "").asString(); //part 3
        U64 target = params.get("target", 0).asUInt64();
        string targetPOWstr = params.get("target_pow", "").asString();

        string nTime = toHex((U32)timestamp);

        string nonce1;
        char LocalPayloadData[67] = { 0 };
        strncpy(LocalPayloadData, "rhminer.rhminer.rhminer.rhminer.rhminer.rhminer.rhminer.polyminer1", sizeof(LocalPayloadData) - 1);
        if (g_extraPayload.length())
        {
            //filter
            for (auto& c : g_extraPayload)
            {
                if (c < 32 || c > 126)
                    c = 32;
            }
            memset(LocalPayloadData, 32, 66 - 11);
            memcpy(LocalPayloadData, g_extraPayload.c_str(), RH_Min((size_t)(66 - 11), g_extraPayload.length()));
            LocalPayloadData[66] = 0;
        }

        if (payload.length() > 52)
        {
            //NOTE: there is a bug in the wallet where it will resent the last submited payload in the next mining notify. 
            //      If this error recure more than 2 times in a row, just restart the wallet.
            RHMINER_EXIT_APP("Error. Deamon/Wallet miner name is too long. Set a name under 26 caracters.\nNOTE, if this error persist, just restart the demaon/wallet.");
        }
        else
        {
            m_nonce1 = payload + toHex((void*)LocalPayloadData, 26 - (payload.length() / 2), false);
        }

        h256 targetPow;
        h256 soloTargetPow;
        h256 normBoud = h256(fromHex(targetPOWstr));
        swab256((void*)targetPow.data(), normBoud.data());
        soloTargetPow = targetPow;

        //Compute difficulty in float
        double d64, dcut64;
        d64 = 1.0f * truediffone;
        dcut64 = le256todouble(targetPow.data());
        if (!dcut64)
            dcut64 = 1;
        float newDiff = (float)(d64 / dcut64);
        SetStratumDiff(newDiff);

        //reset nonce !
        m_authorized = true;
        if (cleanFlag)
            RequestCleanNonce2();

        //send work to miners
        WorkPackageSptr newWork = InstanciateWorkPackage();
        newWork->Init(toHex(++m_soloJobId), h256("0000000000000000000000000000000000000000000000000000000000000000"), coinbase1, coinbase2, nTime, cleanFlag, m_nonce1, m_nonce2Size, m_extraNonce, m_active->host);
        newWork->m_soloTargetPow = soloTargetPow;
        newWork->m_extranoncePos = (U32)(coinbase1.length() + m_nonce1.length()) / 2;
        SendWorkToMiners(newWork);
    }
}

void StratumClientPASC::RespondMiningSubmitSolo(Json::Value& stratumData, U32 gpuIndex)
{
    string errorStr = stratumData.get("error", "").asString();
    if (errorStr.length())
    {
        PrintOutCritical("Solution rejected by %s. Reason :%s\n\n", m_active->HostDescr(), errorStr.c_str());
        m_farm->AddRejectedSolution((U32)gpuIndex);
    }

    Json::Value resData = stratumData.get("result", Json::Value::null);
    if (!resData.isNull())
    {
        string pow = resData.get("pow", "").asString();
        string payload = resData.get("payload", "").asString();
        U64 timeStamp = resData.get("timestamp", 0).asUInt64();

        if (pow.length() && payload.length() && timeStamp)
        {
            PrintOutCritical("Solution accepted by %s\n", m_active->HostDescr());
            string block = resData.get("block", "").asString();
            PrintOutCritical("Found block %s !!! Pow is %s\n\n", block.c_str(), pow.c_str());

            m_farm->AddAcceptedSolution((U32)gpuIndex);
            m_lastSubmitTime = TimeGetMilliSec();

            Guard g(m_stsMutex);
            if (timeStamp > m_submittedTimestamp)
                m_submittedTimestamp = timeStamp;
        }
        else
        {
            //error
            PrintOutCritical("Solution rejected by %s. Reason :Deamon/wallet sent incorect submit result\n\n", m_active->HostDescr());
            m_farm->AddRejectedSolution((U32)gpuIndex);
        }
    }
}

void StratumClientPASC::CallSubmit(SolutionSptr solution)
{
    if (!m_connected)
        return;

    string params;
    WorkPackage* cbwp = solution->m_work.get();
    RHMINER_ASSERT(cbwp);

    U64 currentNonce = solution->GetCurrentEvaluatingNonce();
    RHMINER_ASSERT(currentNonce <= U32_Max);

    if (IsSoloMining())
    {
        U32 nTimeV = ToUIntX(cbwp->m_ntime);

        char payload[64];
        memcpy(payload, &solution->m_work->m_fullHeader[90], 34);
        payload[34] = 0;
        PrintOut("Submiting solution %u for %s\n", currentNonce, GpuManager::Gpus[solution->m_gpuIndex].gpuName.c_str());

        params = FormatString("{\"payload\":\"%s\",\"timestamp\":%d,\"nonce\":%d}",
            toHex((void*)&solution->m_work->m_fullHeader[90], 34).c_str(),
            nTimeV,
            currentNonce);

        CallJsonMethod("miner-submit", params, solution->m_gpuIndex);
    }
    else
    {
        char tstr[32];
        GetSysTimeStrF(tstr, sizeof(tstr), "%H:%M:%S", false);
        string nonceHex = toHex(currentNonce);

        PrintOut("Nonce %llX found on %s for job %s at %s. Submitting to %s\n", currentNonce, GpuManager::Gpus[solution->m_gpuIndex].gpuName.c_str(), cbwp->m_jobID.c_str(), tstr, m_active->HostDescr());

        RHMINER_ASSERT(cbwp->m_nonce2 != U32_Max);
        params = FormatString("\"%s\",\"%s\",\"%llx\",\"%s\",\"%s\"",
            m_active->user.c_str(),
            cbwp->m_jobID.c_str(),
            cbwp->m_nonce2_64,
            cbwp->m_ntime.c_str(),
            nonceHex.c_str());

        CallJsonMethod("mining.submit", params, solution->m_gpuIndex);
    }
}
