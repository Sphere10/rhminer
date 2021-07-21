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
#include "StratumClientVNet.h"
#include "RandomHash/Common.h"

/*
* New Miner Block
* ---------------
* ATM we're not fixed on one Mining Block structure so the mining package fields are
* sent, thru json RPC, with a description of it's structure in the field "Config.BlockTemplate".
* This struture is(must) represent what the NewMinerBlockSerializer class do.
*
* The field convension is as follow:
*	- int/uint are all 32bit uint
*	- Timestamp, Nonce, MinerNonce, NodeNonce, version...  are 32bit uint
*	- ExtraNonce is always 64bit uint
*	- The 2 hash field (MerkelRoot, PreviousBlockHash) are bytes arrray represented as an hexa
*	  string (without 0x). Size may varry.
*	- MinerTag must be smaller than Config.TagSize
*
* Nonce search space:
*  To minimize nonce dupliates and double works accross the entire mining network, the nonce search
*  space is devided into 4 dimensions : Node | Miner | CPU/GPU | Nonce
*  - NodeNonce :  By convention, this is a the Node GUID. It's unique amongst all nodes in the network.
*				   It cannot be modified by the miner client.
*	- MinerNonce : By convention, this is a the miner client LUID. It can be the client's IP, as long as
*				   it's unique amongst all miners. It cannot be modified by the miner client.
*	- ExtraNonce : This 64bit search dimention is indented to prevent the Nonce search sub-space to be
*				   enterely scanned within the block time limit. It must be unique amongst all
*				   computing unit on the system. It should be modified by the miner.
*	- Nonce :      The actual nonce sub-space to search. It must be modified by the miner.
*
*	To maximize randomness, and minimize dead-pockets(pockets of search space that are skipped due
*	to "bad" entropy) it is strongly recomended to sparsly distribute the 4 search nonces,
*	inside the mining block.
*
*	Until we settle on a NewMinerBlock structure, we can add/remove/reorder any field without having
*	to recompile RHminer. The only requirement is that you provide TagSize and BlockTemplate
*	in the Config object
*
*/
void StratumClientVNet::ProcessMiningNotifySolo(Json::Value& jsondata)
{
    try
    {
        string retVal;
        Json::Value arayparams = jsondata.get("params", Json::Value::null);
        Json::Value params = arayparams.get((Json::Value::ArrayIndex)0, 0);

        string targetPOWstr = params.get("TargetPOW", "").asCString();

        // Uncomment this to simulate a high-diff workpackage and receive "miner.update_difficulty"
        //targetPOWstr = "00000000000000000000000000000000000000000000000000000000010fffff";

        Json::Value config = params.get("Config", Json::Value::null);
        string nTime;
        U32 timeStamp = 0;
        U32 timeStampPos = 0;
        string workID;
        U32 minerTagPos = 0;
        U32 startNonce = 0;
        U32 noncePos = 0;
        U32 microBlockNumnerPos = 0;
        U16 microBlockNumner = 0;
        U32 minerTagSize = 64;
        strings blockTemplate;
        string blockTemplateStr;
        if (config != Json::Value::null)
        {
            m_RealtimeTargetMaxTime = (U32)config.get("maxtime", 0).asUInt();

            minerTagSize = (U32)config.get("tagsize", 0).asUInt();
            if (ToUpper(config.get("hashalgo", "").asCString()) != "RH2")
            {
                PrintOutCritical("Unsupported algo\n");  
                return; 
            }
            /*
            blockTemplateStr = config.get("blocktemplate", "").asCString();
            blockTemplate = GetTokens(blockTemplateStr, ",");
            if (blockTemplate.size() == 0)
                RHMINER_EXIT_APP("Wrong block template\n");
            */
        }

        //if (blockTemplateStr.length() == 0)
        //    RHMINER_EXIT_APP("Error. Deamon/Wallet miner name sent badly constructed work package.");

        string header;
        auto ProcessUintField = [&](U32 val) { U32 v = val; return toHex((void*)&v, 4, false); };
        auto ProcessUint16Field = [&](U32 val) { U16 v = (U16)val; return toHex((void*)&v, 2, false); };
        auto ProcessUint64Field = [&](U64 val) { U64 v = val; return toHex((void*)&v, 8, false); };
        auto ProcessStringField = [&](string val) { return toHex((void*)val.c_str(), val.length(), false); };
        //auto IsHexArray  = [&](string fieldname) { return (fieldname == "PrevMinerElectionHeader" || fieldname == "BlockPolicy" || fieldname == "KernelID" || fieldname == "Signature") ? true : false; };
        //auto Is64Bits = [&](string fieldname) { return fieldname.find("Account") != std::string::npos ? true : false; };
        
        auto AddMinerTagFilling = [&](string minerTag) {
            U32 tagSize = (U32)minerTag.length();
            if (tagSize > minerTagSize)
            {
                PrintOutCritical("Miner tag must be smaller than %d bytes\n", minerTagSize);
                return string("");
            }
            if (tagSize == minerTagSize)
                return minerTag;
    
            U32 remainingSize = minerTagSize - tagSize;
            string filler = minerTag;
            filler += MakeSpaces(remainingSize);
            RHMINER_ASSERT(filler.length() == minerTagSize);

            return filler;
        };

        header.reserve(1024);
        header += ProcessUintField(params.get("Version", 0).asUInt());
        header += params.get("PrevMinerElectionHeader", "").asString();
        microBlockNumner = (U16)params.get("PreviousMinerMicroBlockNumber", 0).asUInt();
        header += ProcessUint16Field(microBlockNumner);
        microBlockNumnerPos = (header.length() / 2)-2;
        header += ProcessUintField(params.get("CompactTarget", 0).asUInt());
        U32 paddingSize = (U32)params.get("PADDING", 0).asUInt();
        header += MakeSpaces(paddingSize * 2, '0');
        header += params.get("BlockPolicy", "").asString();
        header += params.get("KernelID", "").asString();
        header += ProcessUint64Field(params.get("MinerRewardAccount", 0).asUInt64()); 
        header += ProcessUint64Field(params.get("DevRewardAccount", 0).asUInt64()); 
        header += ProcessUint64Field(params.get("InfrastructureRewardAccount", 0).asUInt64());
        header += params.get("Signature", "").asString(); 
        string fullTag = AddMinerTagFilling(params.get("MinerTag", "").asString());
        header += ProcessStringField(fullTag);
        minerTagPos = (header.length() / 2) - minerTagSize;
        timeStamp = params.get("TimeStamp", 0).asUInt();
        header += ProcessUintField(timeStamp);
        timeStampPos = (header.length() / 2) - 4;
        startNonce = params.get("Nonce", 0).asUInt();
        header += ProcessUintField(startNonce);
        noncePos = (header.length() / 2) - 4;

        workID = params.get("WorkID", "0").asString();
        nTime = toHex((U32)timeStamp);

        /*
        //PrintOutSilent("Mining block template : %s\n", blockTemplateStr.c_str());
        for (auto const& fieldName : blockTemplate)
        {
            Json::Value fieldVal = params.get(fieldName, Json::Value::null);
            if (IsHexArray(fieldName))
                header += fieldVal.asString();
            else if (Is64Bits(fieldName))
                header += ProcessUint64Field(fieldVal.asUInt64());
            else if (fieldVal.isUInt())
                header += ProcessUintField(fieldVal.asUInt());
            else if (fieldVal.isString())
                header += ProcessStringField(fieldVal.asString());
            else
            {
                PrintOutCritical("Unsuported field type in block header\n");
                return;
            }

            if (fieldName == "WorkID")
                workID = params.get(fieldName, "0").asString();

            if (fieldName == "MinerTag")
            {
                string fullTag = AddMinerTagFilling(fieldVal.asString());
                header += ProcessStringField(fullTag);
                minerTagPos = (header.length() / 2) - minerTagSize;
            }

            if (fieldName == "ExtraNonce")
            {
                extraNoncePos = (header.length() / 2)-4;
                //Json saved it as 32bit ! Add another 32b here
                header += ProcessUintField(0xAAAAAAAA);
            }

            if (fieldName == "PADDING")
            {
                U32 paddingSize = (U32)params.get(fieldName, 0).asUInt();
                header += MakeSpaces(paddingSize*2, '0');
            }

            if (fieldName == "Nonce")
                noncePos = (header.length() / 2)-4;

            //Header ends on timestamp field
            if (fieldName == "Timestamp")
            {
                timeStamp = params.get("Timestamp", 0).asUInt();
                nTime = toHex((U32)timeStamp);
            }
        }
*/

        //TODO: Get ridd of targetPOW in MinerBlockSurogate and calc POW from compactTarget, HERE,  using MonilaTarget algo in c++
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

        m_authorized = true;

        //send work to miners
        WorkPackageSptr newWork = InstanciateWorkPackage();
        newWork->m_extranoncePos = 0; //DISABLED
        newWork->m_noncePos = noncePos;
        newWork->as<VNetWorkPackage>()->m_microBlockPos = microBlockNumnerPos;
        newWork->as<VNetWorkPackage>()->m_microBlockNumber = microBlockNumner;
        newWork->as<VNetWorkPackage>()->m_minerTagPos = minerTagPos;
        newWork->as<VNetWorkPackage>()->m_minerTagSize = minerTagSize;
        newWork->as<VNetWorkPackage>()->m_timeStampPos = timeStampPos;
        newWork->as<VNetWorkPackage>()->m_timeStamp = timeStamp;

        if ((header.size() / 2) != MaxMiningHeaderSize)
        {
            RHMINER_EXIT_APP("Error: Wrong Header size.\n");
        }

        //Cram all in coinbase1 (as a hack)
        newWork->Init(workID, h256("0000000000000000000000000000000000000000000000000000000000000000"), header, "", nTime, false, m_nonce1, m_nonce2Size, startNonce, m_active->host);
        newWork->m_soloTargetPow = soloTargetPow;
        SendWorkToMiners(newWork);
    }
    catch (std::exception const& _e)
    {
        RHMINER_PRINT_EXCEPTION_EX("error", _e.what());
    }
    catch (...)
    {
        PrintOutCritical("exception\n");
    }
}


bool StratumClientVNet::ProcessMiningNotify(Json::Value& params)
{
    PrintOutCritical("Not implemented yet...\n");
    int i = 0;
    string job = params.get((Json::Value::ArrayIndex)i++, "").asString();
    string prevHash = params.get((Json::Value::ArrayIndex)i++, "").asString();
    string hasTries = (params.size() == 10) ? params.get((Json::Value::ArrayIndex)i++, "").asString() : "";
    string coinbase1 = params.get((Json::Value::ArrayIndex)i++, "").asString();
    string coinbase2 = params.get((Json::Value::ArrayIndex)i++, "").asString();
    Json::Value merkelArray = params.get((Json::Value::ArrayIndex)i++, 0);
    string bbver = params.get((Json::Value::ArrayIndex)i++, "").asString();
    string nBit = params.get((Json::Value::ArrayIndex)i++, "").asString();
    string nTime = params.get((Json::Value::ArrayIndex)i++, "").asString();
    bool   cleanWork = params.get((Json::Value::ArrayIndex)i++, false).asBool();

    if (prevHash.empty())
        prevHash = "0000000000000000000000000000000000000000000000000000000000000000";

    {
        //some pool dont process authorization
        if (!m_authorized)
            m_authorized = true;

        //regardless of cleanWork, we clean nonc2 every 15 min ! Duno what is the best value here ?
        static U64 firstTimeCalled = TimeGetMilliSec();
        cleanWork |= (TimeGetMilliSec() - firstTimeCalled > (15 * 60 * 1000)) &&
            ((TimeGetMilliSec() - m_lastNonce2CleanTime) > (15 * 60 * 1000));

        if (cleanWork)
            RequestCleanNonce2();

        {
            WorkPackageSptr newWork = InstanciateWorkPackage();
            newWork->Init(job, h256(prevHash), coinbase1, coinbase2, nTime, cleanWork, m_nonce1, m_nonce2Size, m_extraNonce, m_active->host);
            newWork->m_extranoncePos = 0;
            SendWorkToMiners(newWork);
        }

        return true;
    }
    return false;
}

//TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP 
U64 _lastAcceptedSubmitTime = 0;
//TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP 
void StratumClientVNet::RespondMiningSubmitSolo(Json::Value& stratumData, U32 gpuIndex)
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
        string result = resData.get("SolutionResult", "").asString();
        U64 blockNumber = resData.get("BlockNumber", "").asUInt64();
        U64 timeStamp = resData.get("TimeStamp", 0).asUInt64();
        if (result.length() && timeStamp)
        {
            if (ToUpper(result) == "ACCEPTED")
            {
                PrintOutCritical("Solution accepted by %s\n", m_active->HostDescr());
                PrintOutCritical("Found block %lld !!! \n\n", blockNumber);

                m_farm->AddAcceptedSolution((U32)gpuIndex);
                m_lastSubmitTime = TimeGetMilliSec();

                Guard g(m_stsMutex);
                if (timeStamp > m_submittedTimestamp)
                    m_submittedTimestamp = timeStamp;

                //TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP 
                _lastAcceptedSubmitTime = TimeGetMilliSec();
                //TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP 
            }
            else
            {
                //Rejected
                PrintOutCritical("Solution rejected by %s. Reason : %s\n\n", m_active->HostDescr(), result.c_str());
                m_farm->AddRejectedSolution((U32)gpuIndex);
            }
        }
        else
        {
            //error
            PrintOutCritical("Solution rejected by %s. Reason :Deamon/wallet sent incorect submit result\n\n", m_active->HostDescr());
            m_farm->AddRejectedSolution((U32)gpuIndex);
        }
    }
    else
    {
        //error
        PrintOutCritical("Solution rejected by %s. Reason :Deamon/wallet sent incorect wrong data\n\n", m_active->HostDescr());
        m_farm->AddRejectedSolution((U32)gpuIndex);
    }
}

void StratumClientVNet::CallSubmit(SolutionSptr solution)
{
    if (!m_connected)
        return;

    string params;
    VNetWorkPackage* cbwp = solution->m_work->as<VNetWorkPackage>();
    RHMINER_ASSERT(cbwp);

    U64 currentNonce = solution->GetCurrentEvaluatingNonce();
    RHMINER_ASSERT(currentNonce <= U32_Max);

    if (IsSoloMining())
    {
        bytes minerTag;
        minerTag.resize(cbwp->m_minerTagSize);
        memcpy(&minerTag[0], &solution->m_work->m_fullHeader[0] + cbwp->m_minerTagPos, cbwp->m_minerTagSize);

        DebugOut(">> Header %s\n", toHex(&solution->m_work->m_fullHeader[0], 256, false).c_str());
        
        auto deviceName = GpuManager::Gpus[solution->m_gpuIndex].gpuName.c_str();

        //TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP 
        //PrintOut("Submiting solution %u at %u\n", currentNonce, cbwp->m_timeStamp);
        PrintOut("Submiting solution %u (id %s) in %2.4f seconds, at %.12u, with diff %s (dt submit %3.4f) for micro block %d\n", currentNonce, cbwp->m_jobID.c_str(), (TimeGetMilliSec() - cbwp->m_initTimeMS)/1000.0, cbwp->m_timeStamp, DiffToStr((float)cbwp->m_workDiff), (TimeGetMilliSec() - _lastAcceptedSubmitTime)/1000.0f, int(cbwp->m_microBlockNumber));
        //TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP 

        params = FormatString("{\"WorkID\":%s, \"MinerTag\":\"%s\",\"Time\":%u,\"Nonce\":%u}",
            cbwp->m_jobID.c_str(),
            toHex((void*)&minerTag[0], minerTag.size()).c_str(),
            cbwp->m_timeStamp,
            currentNonce);

        CallJsonMethod("miner.submit", params, solution->m_gpuIndex);
    }
    else
    {
        //TODO: test stratum , somehow !!!
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

//set_difficulty
void StratumClientVNet::ProcessSetDiffSolo(Json::Value& responseObject)
{
    Json::Value arayparams = responseObject.get("params", Json::Value::null);
    Json::Value params = arayparams.get((Json::Value::ArrayIndex)0, 0);
    string targetPOWstr = params.get("TargetPOW", "").asCString();
    U32 newMicroBlockNumber = params.get("MicroBlockNumber", "").asUInt();
    U32 timeStamp = params.get("TimeStamp", "").asUInt();
    
    h256 targetPow;
    h256 soloTargetPow;
    h256 normBoud = h256(fromHex(targetPOWstr));
    swab256((void*)targetPow.data(), normBoud.data());
    soloTargetPow = targetPow;
        
    //NOTE SURE we need this...
    double d64, dcut64;
    d64 = 1.0f * truediffone;
    dcut64 = le256todouble(targetPow.data());
    if (!dcut64)
        dcut64 = 1;
    float newDiff = (float)(d64 / dcut64);
    SetStratumDiff(newDiff);
    PrintOut("Updating header\n");
    
    //m_current->m_localyGenerated = true;
    m_current->m_initTimeMS = TimeGetMilliSec();
    m_current->m_soloTargetPow = soloTargetPow;
    m_current->as<VNetWorkPackage>()->m_microBlockPos;
    m_current->as<VNetWorkPackage>()->m_timeStamp = timeStamp;
    m_current->m_ntime = toHex((U32)timeStamp);
    m_current->as<VNetWorkPackage>()->m_microBlockNumber = newMicroBlockNumber;    
    SendWorkToMiners(m_current);

}


