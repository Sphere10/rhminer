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
#include "StratumClient.h"


class StratumClientPASC : public StratumClient
{
public:
    StratumClientPASC(const StratumInit& initData) :StratumClient(initData) {}

    virtual const char* GetMinerSubmitRpcName() { return "miner-submit"; }
    virtual const char* GetMinerNotifyRpcName() { return "miner-notify"; }
    virtual bool ProcessMiningNotify(Json::Value& arrayParam);
    virtual void CallSubmit(SolutionSptr solution);
    virtual void ProcessMiningNotifySolo(Json::Value& arrayParam);
    virtual void RespondMiningSubmitSolo(Json::Value& stratumData, U32 gpuIndex);

protected:
    U32 m_RealtimeTargetMaxTime = 0;
};



