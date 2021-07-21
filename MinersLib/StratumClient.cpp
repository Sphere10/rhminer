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
#include "StratumClient.h"
#include "corelib/Log.h"
#include "MinersLib/Global.h"
#include "corelib/CommonData.h"
#include "corelib/boostext.h"
#include "corelib/miniweb.h"
#include "corelib/rh_endian.h"
#include "MinersLib/RandomHash/Common.h"

/*
see 
https://nextbigthings.info/secured-tls-connection-using-boost-asio-and-openssl-for-windows/

Examples : https://www.boost.org/doc/libs/1_53_0/doc/html/boost_asio/example/ssl/client.cpp
#include <boost/asio/ssl.hpp>
*/

#ifndef _WIN32_WINNT
#include <sys/socket.h>
#endif

RHMINER_COMMAND_LINE_DEFINE_GLOBAL_BOOL(g_DisableAutoReconnect, false);
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_INT(g_maxConsecutiveSubmitErrors, 10);
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_BOOL(g_forceSequentialNonce, false)
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_BOOL(g_disableCachedNonceReuse, false)
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_STRING(g_extraPayload, "")
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_INT(g_apiPort, 7111)
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_STRING(g_apiPW, "")
RHMINER_COMMAND_LINE_DEFINE_GLOBAL_INT(g_workTimeout, 60);
extern bool g_useGPU;

using boost::asio::ip::tcp;


StratumClient::StratumClient(const StratumInit& initData )
	: Worker("Net"),
      m_socket(m_io_service),
      m_nonce2Rand(std::random_device{}())
{
    //find any gpu enabled
    for (auto&g : GpuManager::Gpus)
        if (g.enabled && g.gpuType != GpuType_CPU)
        {
            g_useGPU = true; 
            break;
        }

    m_started = false;
    m_workID = 0;
    m_sessionRandomID = h256(rand32());

    //init nonce2 random 
    m_nonce2Rand.seed((U32)TimeGetMilliSec()^rand32());

	//m_minerType = initData.m;
	m_primary.host = initData.host;
	m_primary.port = initData.port;
	m_primary.user = initData.user;
	m_primary.pass = initData.pass;

	m_active = &m_primary;

	m_authorized = false;
	m_connected = false;
	m_maxRetries = initData.retries;
    m_maxRetriesDEV = 4;
	m_email = initData.email;
    m_miningCoin = initData.coin;
	m_farm = initData.f;
    m_soloMining = initData.soloOverStratum;

    if (!m_soloMining)
    {
        if (m_primary.user.length() == 0 && m_primary.pass.length() == 0 && g_testPerformance == false)
        {
            RHMINER_EXIT_APP("Error: No credential provided.\n");
        }
    }
    m_nextWorkDifficulty = GlobalMiningPreset::I().m_localDifficulty;
}

StratumClient::~StratumClient()
{
}

bool StratumClient::IsSoloMining() 
{ 
    return m_soloMining && !GlobalMiningPreset::I().IsInDevFeeMode();
}

void StratumClient::SetFailover(string const & host, string const & port, string const & user, string const & pass)
{
    if (!user.length())
    {
	    m_failover.host = host;
	    m_failover.port = port;
	    m_failover.user = m_active->user;
	    m_failover.pass = m_active->pass;        
    }
    else
    {
	    m_failover.host = host;
	    m_failover.port = port;
	    m_failover.user = user;
	    m_failover.pass = pass;
    }
}

void StratumClient::Write(tcp::socket& socket, boost::asio::streambuf& buff)
{
    if (!m_connected)
        return;

    try
    {
        size_t size = buff.size();
        std::string s((std::istreambuf_iterator<char>(&buff)), std::istreambuf_iterator<char>());            

        PrintOutSilent("send : %s\n", s.c_str());

        std::ostream os(&buff);
        os << s;
    
        write(socket, buff);
    }
    catch (std::exception const& _e) 
    {
        RHMINER_PRINT_EXCEPTION_EX("Network Error", _e.what());
        //Reconnect(3000);
        m_socket.close(); 
        m_io_service.reset();
    }
}

void StratumClient::GetUserPW(string& user, string& pw)
{
    user = m_active->user;
    pw = m_active->pass;
}

void StratumClient::StartWorking()
{
    if (!m_started || m_state == WorkerState::Stopped)
    {
        if (m_state == WorkerState::Stopped)
            m_running = true;

        m_started = true;
        Worker::StartWorking();
    }
}

void StratumClient::SetDevFeeCredentials(const string& param)
{
    if (param.length())
    {
        strings params = GetTokens(param, "\t");
	    m_devFee.host = params[0];
        m_devFee.port = params[1];
	    m_devFee.user = params[2];
        m_devFee.pass = "";
        m_active = &m_devFee;
    }
    else
        m_active = &m_primary;
    
    m_devFeeConnectionMode = true;
}

void StratumClient::Connect()
{
    m_workID = 0;
    Preconnect();
    
    if (m_connected && m_lastActiveHost != m_active->host)
    {
        m_retries = 0;
        m_lastActiveHost = m_active->host;
    }

    boost::asio::ip::tcp::socket::bytes_readable command;
    boost::system::error_code ec;
    m_socket.io_control(command, ec);

    if (ec)
    {
        RHMINER_PRINT_EXCEPTION_EX(FormatString("Error connecting to %s", m_lastActiveHost.c_str()), "");
        m_socket.close();
        m_connected = false;
    }

    if (m_connected)
    {
        m_devFeeConnectionMode = false;
        m_retries = 0;

        OnPostConnect();
    }
}

void StratumClient::Reconnect(U32 preSleepTimeMS)
{
    Guard l(m_reconnMutex);

    if (m_farm->isMining())
    {
        m_farm->Pause();
	}

    bool wasConnected = m_connected;

	m_authorized = false;
	m_connected = false;
    m_workID = 0;
    //dont wait to much in devfee mode
    if (GlobalMiningPreset::I().IsInDevFeeMode())
        preSleepTimeMS = 1000;

    if (m_devFeeConnectionMode)
        CpuSleep(500);
    else if (preSleepTimeMS)
    {
        CpuSleep(preSleepTimeMS);
    }
	
    if (m_state != WorkerState::Stopping && m_state != WorkerState::Stopped)
    {
        m_retries++;
        if (m_retries >= m_maxRetries && !m_devFeeConnectionMode)
        {
            if (m_failover.host.empty())
            {
                RHMINER_EXIT_APP("Max retry count reached, exiting...");
            }
            else
            {
                if (m_active == &m_primary)
                {
                    m_retries = 0;
                    m_active = &m_failover;
                }
                else 
                {
                    RHMINER_EXIT_APP("Max retry count reached with failover address, exiting...");
                }
            }
        }

        if (wasConnected)
        {
            if (g_DisableAutoReconnect && !m_devFeeConnectionMode)
            {
                PrintOut("Disable auto reconnect is on. Exiting miner...\n");
                RHMINER_EXIT_APP("");
            }
            else
            {
                m_dropConnectionCount++;
                m_dropConnectionLastTime = TimeGetMilliSec();

                if (!preSleepTimeMS)
                    m_sleepBeforeConnectMS = 3*1000;

                m_socket.close(); 
                m_io_service.reset();
            }
        }
    }

    //update devfee
    if (GlobalMiningPreset::I().IsInDevFeeMode())
    {
        string x;
        if (m_retries > m_maxRetriesDEV)
            x = "_retry_";

        bool res = GlobalMiningPreset::I().UpdateToDevModeState(x);
        if (m_retries > m_maxRetriesDEV && res)
        {
            m_retries = 0;
            SetDevFeeCredentials(x);
        }

        if (!GlobalMiningPreset::I().IsInDevFeeMode())
            m_active = &m_primary;
    }
}

void StratumClient::StopFarming()
{
    if (m_farm->isMining())
	{
        m_farm->Pause();
	}
}

void StratumClient::Disconnect()
{
    StopFarming();

	m_connected = false;
	m_running = false;

	m_socket.close();
}

string StratumClient::ReadLineFromServer()
{
    string response;
    RH_ReadLine(m_socket, m_responseBuffer, "\n");

    std::istream is(&m_responseBuffer);
    getline(is, response);

    PrintOutSilent("recv : %s\n", response.c_str());

    return response;
}

//With No try/catch
void StratumClient::ProcessServerCommands()
{
    std::string response;
    response = "go";
    m_processedResponsesCount = 0;
    while (response.length())
    {
        response = ReadLineFromServer();

        if (!response.empty())
        {
            m_lastReceivedCommandTime = TimeGetMilliSec();

            if (!response.empty() && response.size() > 2 && response[0] == 0)
                response = (const char*)&response[1];

            //some deamon will send a \r at the end !?!?
            if (response.back() == '\r')
                response = response.substr(0, response.length() - 1);

            //handle coinotron's invalid request
            if (response.find("\"Invalid Request\"") != string::npos)
                throw RH_Exception("Invalid Request");

            if (response.front() == '{' && response.back() == '}')
            {
                Json::Value responseObject;
                Json::Reader reader;
                m_lastReceivedLine = response;
                if (reader.parse(response.c_str(), responseObject))
                {
                    ProcessReponse(responseObject);
                    m_processedResponsesCount++;
                }
                else
                {
                    PrintOut("Error. Parsing response failed: %s\n", reader.getFormattedErrorMessages().c_str());
                    throw RH_Exception("Stratum.Json");
                }
            }
        }
    }
}

void StratumClient::WorkLoop()
{
    if (g_useCPU && !g_useGPU && g_setProcessPrio != 1)
        RH_SetThreadPriority(RH_ThreadPrio_High);

    while (m_running)
    {
        while(!m_connected)
        {
            try 
            {
                Connect();
            }     
            catch (std::exception const& _e) 
            {
                RHMINER_PRINT_EXCEPTION_EX("Connection error",  _e.what());
                Reconnect(3000);
            }
            catch (...) 
            {
                PrintOutCritical("Connection exception\n");
                Reconnect(3000);
            }
        }

        try
        {
            ProcessServerCommands();
            if (m_state != WorkerState::Started)
                break;
        }
        catch (std::exception const& _e) 
        {
            if (!m_devFeeConnectionMode)
                RHMINER_PRINT_EXCEPTION_EX("Network Error",  _e.what());
            
            Reconnect(3000);
        }
        catch (...) 
        {
            PrintOutCritical("Network exception\n");
            Reconnect(3000);
        }
    }
    CpuSleep(200);
}

void StratumClient::CallJsonMethod(string methodName, string params, U64 gpuIndexOrRigID, string additionalCallParams, bool dontPutID)
{
    Guard g(m_CallJsonMethodMutex);
    m_workID++;

    m_lastCalleJSondMethod[m_workID] = MethodInfos(methodName, gpuIndexOrRigID);
    
    std::ostream os(&m_requestBuffer);
    os << "{";
    if (!m_jsonrpcVersion.empty())
    {
        os << "\"jsonrpc\":\"" << m_jsonrpcVersion << "\",";
    }

    string IDstr = "";
    if (!dontPutID)
        IDstr = FormatString("\"id\":%d,", m_workID);
    
    if (!additionalCallParams.empty() && additionalCallParams.back() != ',')
        additionalCallParams += ",";

    os << "\"params\":[" << params << "]," << IDstr << additionalCallParams << "\"method\":\"" << methodName << "\"}\n";
    
    Write(m_socket, m_requestBuffer);
}

void StratumClient::CallMiningSubscribeMethod()
{
    if (!IsSoloMining())
    {
        if (!m_sessionID.empty())
            CallJsonMethod("mining.subscribe", FormatString("\"%s\", \"%s\"",  m_userAgent.c_str(), m_sessionID.c_str()));
        else
            CallJsonMethod("mining.subscribe", FormatString("\"%s\"", m_userAgent.c_str()));

        CallJsonMethod("mining.authorize", FormatString("\"%s\", \"%s\"", m_active->user.c_str(), m_active->pass.c_str()));
    }
}

void StratumClient::OnPostConnect()
{
    m_nextWorkDifficulty = GlobalMiningPreset::I().m_localDifficulty;
    m_lastReceivedCommandTime = TimeGetMilliSec();

    m_nonce2 = GetNewNonce2();
    {
        Guard g(m_CallJsonMethodMutex);
        m_lastCalleJSondMethod.clear();
    }
    
    CallMiningSubscribeMethod();

    RHMINER_ONLY_ONCE_CODE_BEGIN()
    if (g_apiPort)
        StartMiniWeb();
    RHMINER_ONLY_ONCE_CODE_END()
}


void StratumClient::Preconnect()
{
    if (m_devFeeConnectionMode || GlobalMiningPreset::I().IsInDevFeeMode())
        CpuSleep(1000);
    else if (m_sleepBeforeConnectMS)
    {
        PrintOut("Reconnecting in %u seconds...\n", m_sleepBeforeConnectMS/1000);
        CpuSleep(m_sleepBeforeConnectMS);
    }    
    
    if (!GlobalMiningPreset::I().IsInDevFeeMode())
    {
        if (!m_soloMining)
            PrintOut("User: '%s' PW: '%s'\n", m_active->user.c_str(), m_active->pass.c_str());
    }

    if (IsSoloMining())
        PrintOut("Solomining on deamon %s\n", m_active->HostDescr());
    else
        PrintOut("Connecting to stratum server %s\n", m_active->HostDescr());

    tcp::resolver r(m_io_service);
    tcp::resolver::query q(m_active->host, m_active->port);
    tcp::resolver::iterator endpoint_iterator = r.resolve(q);
    tcp::resolver::iterator end;

    boost::system::error_code error = boost::asio::error::host_not_found;
    while (error && endpoint_iterator != end)
    {
        m_socket.close();
        m_socket.connect(*endpoint_iterator++, error);
    }

    if (error)
    {
        RHMINER_PRINT_EXCEPTION_EX(FormatString("Error. Could not connect to server %s", m_active->HostDescr()), "Retrying...");
        Reconnect(5 * 1000);
    }
    else
    {
        m_sleepBeforeConnectMS = 0;
        m_connected = true;
        if (!m_farm->isMining())
        {
            m_farm->start();
        }
   }
}

//set_extranonce
void StratumClient::ProcessExtranonce(Json::Value& responseObject)
{
    Json::Value params = responseObject.get("params", Json::Value::null);
    if (!params.isArray())
        params = responseObject.get("result", Json::Value::null);

    RHMINER_ASSERT(params.isArray());

    std::string enonce = params.get((Json::Value::ArrayIndex)0, "").asString();
    PrintOut("Extranonce set to %s\n", enonce.c_str());

    for (auto i = enonce.length(); i < 16; ++i)
        enonce += "0";

    m_extraNonce = ToUInt64X(enonce);

    g_disableCachedNonceReuse = true;
    g_forceSequentialNonce = true;
}


bool StratumClient::ValidateNewWork(WorkPackageSptr& work)
{
    if (work->m_localyGenerated)
        return true;

    Guard g(m_workBacklogMutex);
    auto backLog = std::find_if(m_workBacklog.begin(), m_workBacklog.end(), [&](const PastWorkInfo& i) { return work->m_prev_hash == i.hash; });
    if (backLog != m_workBacklog.end())
    {
        if (m_previous.get() && backLog->hash != m_previous->m_prev_hash)
        {
            auto dt = (TimeGetMilliSec() - backLog->recvTime)/1000;
            PrintOutWarning("Stale work %s found in back log dating %lu seconds\n", work->m_prev_hash.hex().c_str(), dt);
            return false;
        }        
    }
    else
    {
        //remove the oldest one
        if (m_workBacklog.size() == MaxBackLogCount)
            m_workBacklog.erase(m_workBacklog.begin());
        m_workBacklog.push_back(PastWorkInfo {work->m_prev_hash, TimeGetMilliSec(), ToUIntX(work->m_ntime)} );
    }

    return true;
}


U32 StratumClient::GetNewNonce2()
{   
    U32 n2 = std::uniform_int_distribution<U32>{}(m_nonce2Rand);
    return n2;
}


void StratumClient::MiningNotify(Json::Value& responseObject)
{
    bool res = false;
    Json::Value arrayParam = responseObject.get("params", Json::Value::null);
    if (arrayParam.isArray())
    {
        //mining.notify
        res = ProcessMiningNotify(arrayParam);
    }

    if (!res)
    {
        PrintOutWarning("mining.notify Json responce error : %s\b", responseObject.toStyledString().c_str());
    }
}

ServerCredential* StratumClient::GetCurrentCred() 
{ 
    if (m_active == &m_failover)
        return &m_failover;
    return &m_primary;
}

bool StratumClient::GetCurrentWorkInfo(h256& out_header)
{
    Guard g(m_currentWorkMutex);
    if (m_current.get() != nullptr)
    {
        out_header = m_current->m_prev_hash;
        return !m_current->IsEmpty();
    }
    return false;
}


void StratumClient::RequestCleanNonce2()
{
    //dont reset nonce to 0 to prevent search collision
    Guard g(m_currentWorkMutex);
    m_lastNonce2CleanTime = TimeGetMilliSec();
    m_nonce2 = GetNewNonce2();

    KernelOffsetManager::Init(0);
}

//init a newly received wp. Called from mining.notify
void StratumClient::SendWorkToMiners(WorkPackageSptr wp)
{    
    //dont req a new nonce for all miner will clone and re-init this worl template
    InitializeWP(wp);

    //print status
    string ids = wp->m_jobID;
    U64 ts = ToUIntX(wp->m_ntime);    

    //PrintOutCritical("Received new Work %s. Difficulty is %s\n", ids.c_str(), DiffToStr((float)wp->m_workDiff));
    PrintOutCritical("Received new Work %s. Difficulty is %s at %d\n", ids.c_str(), DiffToStr((float)wp->m_workDiff), (U32)ToIntX(wp->m_ntime));

    //Propagate the workpackage to all miners
    m_farm->SetWork(InstanciateWorkPackage(&wp));
}


void StratumClient::PrepareWorkInternal(WorkPackageSptr wp)
{
    wp->UpdateHeader();
    if (!wp->m_localyGenerated)
        wp->ComputeWorkDiff(m_nextWorkDifficulty);
    wp->ComputeTargetBoundary();
}

void StratumClient::InitializeWP(WorkPackageSptr wp)
{
    //because pascal have no prev_hash, we put something unique in there
    bytes fakePreHash((size_t)32, (byte)0);
    ((U32*)fakePreHash.data())[0] = (U32)ToIntX(wp->m_ntime);
    ((U32*)fakePreHash.data())[1] = (U32)ToIntX(wp->m_jobID);
    ((U32*)fakePreHash.data())[2] = RH_crc32(wp->m_coinbase1.c_str());
    ((U32*)fakePreHash.data())[3] = RH_crc32(wp->m_coinbase2.c_str());

    wp->m_prev_hash = h256(fakePreHash);

    Guard g(m_currentWorkMutex);
    bool isLocalWork = wp->m_localyGenerated;

    //miner will pass a null work
    if (!isLocalWork)
    {
        m_previous = m_current;
        m_current = InstanciateWorkPackage(&wp);
    }
    m_current->m_submittedNonces.clear();

    wp->m_nonce2 = m_nonce2;

    PrepareWorkInternal(wp);

    ValidateNewWork(wp);
}

//mining.subscribe
void StratumClient::RespondSubscribe(Json::Value& responseObject, U64 gpuIndex)
{
    Json::Value params = responseObject.get("result", Json::Value::null);

    if (params.isArray())
    {
        Json::Value::ArrayIndex i = 0; 
        Json::Value param1 = params[i];
        if (param1.isArray())
        {
            Json::Value::ArrayIndex size = param1.size();
            for (i = 0; i < size; i++)
            {
                Json::Value param11 = param1[0];
                string name;
                if (param11.isArray())
                {
                    name = param1[i][0].asString();
                    if (name == "mining.notify")
                    {
                        m_sessionID = param1[i][1].asString();
                        break;
                    }
                }
                else
                {
                    name = param1[i].asString();
                    if (name == "mining.notify")
                    {
                        m_sessionID = params[0][1].asString();
                        break;
                    }
                }
            }
 
            m_nonce1 = params.get((Json::Value::ArrayIndex)1, "").asString();
            m_nonce2Size = params.get((Json::Value::ArrayIndex)2, "").asUInt();
        }
    }
}



//set_difficulty
void StratumClient::ProcessSetDiff(Json::Value& responseObject)
{
    Json::Value params = responseObject.get("params", Json::Value::null);
    double stratDiff = params.isArray() ? params[0].asDouble() : responseObject.asDouble();
    SetStratumDiff((float)stratDiff);
}

void StratumClient::ProcessSetDiffSolo(Json::Value& arrayParam)
{
    ProcessSetDiff(arrayParam);
}



void StratumClient::SetStratumDiff(float stratDiff)
{
    if (GlobalMiningPreset::I().m_localDifficulty != 0.0f)
    {
        PrintOut("Ignoring stratum difficulty of %.8f over locally chosen difficulty of %s \n", (float)stratDiff, DiffToStr((float)m_nextWorkDifficulty));
    }
    else
    {
        m_nextWorkDifficulty = stratDiff;
        if (m_nextWorkDifficulty <= 0.0000000001)
            m_nextWorkDifficulty = 0.000000001;
    }
}


bool StratumClient::HandleMiningSubmitResponceResult(Json::Value& responseObject, string& errorStr, U64 lastMethodCallTime)
{
    bool succeded = false;
    try
    {
        errorStr = "";
        succeded = responseObject.get("result", false).asBool();
        if (!succeded)
        {
            Json::Value erra = JsonGetSafeArray(responseObject, "error");
            if (erra.isArray())
            {
                string p1;
                string p0 = erra.get((Json::Value::ArrayIndex)0, "").asString();
                if (erra.size() > 1)
                {
                    p1 = erra.get((Json::Value::ArrayIndex)1, "").asString();
                    errorStr = p1.length() ? (p0 + ":" + p1) : p1;
                    if (errorStr.length() == 0)
                        errorStr = p0;
                }
            }
            else if (erra.isString())
            {
                errorStr = JsonGetSafeArray(responseObject, "error").asString();
            }

            if (errorStr.empty())
            {
                errorStr = JsonGetSafeString(erra, "message");
                if (errorStr.empty())
                    errorStr = JsonGetSafeString(erra, "error");
                if (errorStr.empty())
                    errorStr = JsonGetSafeString(erra, "reason");
                if (errorStr.empty())
                    errorStr = erra.toStyledString();
            }
        }
    }
    catch (...)
    {
        errorStr = "exp";
        succeded = false;
    }
    ReplaceStringALL(errorStr, "\n", " ");
    ReplaceStringALL(errorStr, "\r", " ");
    return succeded;
}

void StratumClient::RespondMiningSubmit(Json::Value& responseObject, U64 gpuIndex, U64 lastMethodCallTime)
{
    //process mining.submit responce
    string errorStr;
    U32 calltime = (U32)(TimeGetMilliSec() - lastMethodCallTime);

    if (calltime > 40)
        calltime -= 40;

    bool succeded = HandleMiningSubmitResponceResult(responseObject, errorStr, lastMethodCallTime);
    if (succeded)
    {
        if (!GlobalMiningPreset::I().IsInDevFeeMode()) 
        {
            PrintOutCritical("Share accepted by %s in %u ms \n\n", m_active->HostDescr(),calltime);
            m_farm->AddAcceptedSolution((U32)gpuIndex);
            m_lastSubmitTime = TimeGetMilliSec();
        }
    }
    else
    {
        if (!GlobalMiningPreset::I().IsInDevFeeMode())
        {
            PrintOutCritical("Share REJECTED by %s in %u ms. Reason :%s\n\n", m_active->HostDescr(), calltime, errorStr.c_str());
            m_farm->AddRejectedSolution((U32)gpuIndex);
        }
    }
}

void StratumClient::RespondAuthorize(Json::Value& responseObject, U64 gpuIndex)
{
    Json::Value res = responseObject.get("result", Json::Value::null);
    if (res.isNull())
    {
        Json::Value error = responseObject.get("error", Json::Value::null);
        if (error.isNull())
            m_authorized = true;
        else
        {
            m_authorized = false;
        }
    }
    else
    {
        if (res.isBool())
            m_authorized = res.asBool();
        else
            m_authorized = true; 
    }

    if (!m_authorized)
    {
        if (m_active != &m_devFee)
        {
            if (m_active->user.length())
                PrintOut("%s is not autorized to connect to stratum server %s. If you intended to mine on local wallet, put http:// \n",m_active->user.c_str(), m_active->HostDescr());
            else
                PrintOut("Not autorized to connect to stratum server %s. If you intended to mine on local wallet, put http:// \n",m_active->HostDescr());
        }

        Disconnect();
        
        if (m_active != &m_devFee)
        {
            RHMINER_EXIT_APP("Connection refused");
        }
        else
        {
            PrintOut("Connection refused\n");
            CpuSleep(20*1000);
        }
    }
    else
    {
        if (!GlobalMiningPreset::I().IsInDevFeeMode())
        {
            if (m_active->user.length())
                PrintOut("%s is autorized on stratum server %s\n", m_active->user.c_str(), m_active->HostDescr());
            else
                PrintOut("Autorized on stratum server %s\n", m_active->HostDescr());
        }
    } 
}

void StratumClient::WriteGenericAnwser(const string& line, U32 workID)
{
    std::ostream os(&m_requestBuffer);
    string resultStr;
    if (line.find("\"method\"") == string::npos)
        resultStr = "\"result\":";
        
    if (workID)
        os << "{\"error\":null,\"id\":" << workID << ",\"result\":" << line << "}\n";
    else
        os << "{\"error\":null,\"id\":null,"<< resultStr << line << "}\n";
    Write(m_socket, m_requestBuffer);
}

void StratumClient::RespondExtraNonceSubscribe(const string& method, Json::Value& responseObject, U64 extraData)
{
    //not sure its valid anymore to put thee 2 process here
    if (method == "mining.set_difficulty")
    {
        ProcessSetDiff(responseObject);
    }
    else if (method == "mining.notify")
    {
        MiningNotify(responseObject);
    }
    else
    {
        PrintOut("Error in extranone\n");
    }
}


void StratumClient::ProcessReponse(Json::Value& responseObject)
{
    Json::Value error = responseObject.get("error", Json::Value::null);
    Json::Value params;
    U64 gpuIndex = U64_Max;
    string lastMethodName;
    U64 lastMethodCallTime = 0;
    string method = responseObject.get("method", Json::Value::null).asString();
    Json::Value idVal = responseObject.get("id", Json::Value::null);
    bool resultPresent = responseObject.isMember("result"); 
    U32 id = 0;
    
    if (!idVal.isIntegral())
    {
        id = U32_Max;
    }
    else
    {
        id = idVal.asUInt();
        if (!resultPresent)
        {
            //sync the id with the server
            Guard g(m_CallJsonMethodMutex);
            if (id > 0 && (U32)id > m_workID)
                m_workID = id;
        }
        else
        {
            if (!IsSoloMining())
            {
                if (m_processedResponsesCount == 0)
                    lastMethodName = "mining.subscribe";
            }
        }
    }

    //get rpc version
    if (m_jsonrpcVersion.empty())
    {
        m_jsonrpcVersion = responseObject.get("jsonrpc", "").asString();
    }

    if (id > 0 && resultPresent)
    {
        {
            Guard g(m_CallJsonMethodMutex);
            auto fnd = m_lastCalleJSondMethod.find(id);
            if (fnd != m_lastCalleJSondMethod.end())
            {
                lastMethodName = fnd->second.metName;
                lastMethodCallTime = fnd->second.callTime;
                gpuIndex = fnd->second.gpuIndex;

                m_lastCalleJSondMethod.erase(fnd);
            }
        }
        if (lastMethodName == GetMinerSubmitRpcName())
        {
            RespondMiningSubmitSolo(responseObject, (U32)gpuIndex);
            return;
        }
        else if (lastMethodName == "mining.subscribe")
        {
            RespondSubscribe(responseObject, gpuIndex);
            return;
        }
        else if (lastMethodName == "mining.extranonce.subscribe")
        {
            RespondExtraNonceSubscribe(method, responseObject, gpuIndex);
            return;
        }
        else if (lastMethodName == "mining.authorize")
        {
            RespondAuthorize(responseObject, gpuIndex);
            
            return;
        }
        else if (lastMethodName == "mining.submit")
        {
            RespondMiningSubmit(responseObject, gpuIndex, lastMethodCallTime);
            return;
        }
    }

    if (method == GetMinerNotifyRpcName())
    {
        if (m_lastReceivedLine != m_lastReceivedMiningNotify)
        {
            m_lastReceivedMiningNotify = m_lastReceivedLine;
            ProcessMiningNotifySolo(responseObject);
        }
    }
    //Process PascalCoin Stratum
    else if (method == "mining.notify")
    {
        if (m_lastReceivedLine != m_lastReceivedMiningNotify)
        {
            m_lastReceivedMiningNotify = m_lastReceivedLine;

            MiningNotify(responseObject);
        }
    }
    else if (method == "miner.update_difficulty")
    {
        ProcessSetDiffSolo(responseObject);
    }
    else if (method == "mining.set_difficulty")
    {
        ProcessSetDiff(responseObject);
    }
    else if (method == "mining.set_extranonce")
    {
        ProcessExtranonce(responseObject);
    }
    else if (method == "client.get_version")
    {
        WriteGenericAnwser("\"" + m_userAgent + "\"" , m_workID++);
    }
    else 
    {
        if (!(resultPresent && responseObject.isMember("error") && error.isNull()))
        {
            if (method.length())
            {
                PrintOut("Error. Stratum method %s is unknown for server %s. Json content = %s\n",method.c_str(), m_active->HostDescr(), responseObject.toStyledString().c_str());
            }
            else
            {
                if (!responseObject.isMember("result"))
                {
                    PrintOut("Error : Stratum result is unexpected for server %s. Json content = %s\n",m_active->HostDescr(), responseObject.toStyledString().c_str());
                    PrintOut("      : Stratum stack :\n");
                    Guard g(m_CallJsonMethodMutex);
                    for(auto& s : m_lastCalleJSondMethod)
                        PrintOut("%3u: %s at %u\n", s.first, s.second.metName.c_str(), s.second.callTime);
                }
            }
        }
    }
}

bool StratumClient::Submit(SolutionSptr solution)
{
    U64 count = solution->m_results.size();    
    U32 start = 0;

    //no point in submiting all solutions, just pick one at random
    if (IsSoloMining() && count > 1)
    {
        start = rand32() % count;
        count = start+1;
    }

    for (U32 i = start; i < count; i++)
    {
        solution->SetCurrentEvaluatingNonceIndex(i);
        U64 currentNonce = solution->GetCurrentEvaluatingNonce();
        if (currentNonce)
        {
            {
                Guard g(m_currentWorkMutex); 
                if (m_current->m_submittedNonces.find(currentNonce) != m_current->m_submittedNonces.end())
                {
                    m_farm->AddFailedSolution(solution->m_gpuIndex);
                    PrintOut("Nonce %X on %s is DUPLICATE. Submit aborted.\n", currentNonce, GpuManager::Gpus[solution->m_gpuIndex].gpuName.c_str());
                    continue;
                }
                else
                    m_current->m_submittedNonces.insert(currentNonce);
            }

            bool res = solution->Eval();
            if (res)
            { 
                //validate workpackage time
                {
                    Guard g(m_stsMutex);
                    U64 ts = ToUIntX(solution->m_work->m_ntime);

                    if (ts < m_submittedTimestamp)
                    {
                        PrintOut("Found solution %u dated from %s but a solution was found at %s. Submit aborted\n", solution->GetCurrentEvaluatingNonce(), toString(ts).c_str(), toString(m_submittedTimestamp).c_str());
                        return true;
                    }
                }

                //prevent cross-over share submissions
                if (m_active->host != solution->m_work->m_server)
                    return true;

                CallSubmit(solution);
            }
            else
            {
                m_farm->AddFailedSolution(solution->m_gpuIndex);
                PrintOut("Nonce %X on %s is INVALID\n", currentNonce, GpuManager::Gpus[solution->m_gpuIndex].gpuName.c_str());
            }
        }
    }

    return true;
}


WorkPackageSptr StratumClient::InstanciateWorkPackage(WorkPackageSptr* cloneFrom)
{
    if (cloneFrom)
    {
        //WorkPackage* cloned = (*cloneFrom).get();
        //RHMINER_ASSERT(cloned);
        //return WorkPackageSptr(new WorkPackage(*cloned));
        return WorkPackageSptr((*cloneFrom)->Clone());
    }
    else
    {
        return WorkPackageFactory::Create(m_miningCoin, IsSoloMining());
    }    
}

