/**
 * Miniweb
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
#include "corelib/utils.h"

extern void StartMiniWeb();
extern void StopMiniWeb();
extern void SetMiniWebData(const string& data, const string& ethManData);

#define RH_DECRYPT(venc, output) \
{ \
    auto _lp = [](int n, int p, int mod) \
    { \
        int result = 1; \
        for (; p; p >>= 1) \
        { \
            if (p & 1) \
                result = (1LL * result * n) % mod; \
            n = (1LL * n * n) % mod; \
        } \
        return result; \
    }; \
    output = ""; \
    int dec; \
    for (int i = 0; i < venc.size()-1; i++) \
    { \
        dec = _lp(venc[i], 10249909, 11580379); \
        output += (char)dec; \
    } \
    output += "\0"; \
}


//{{{ TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
//ANTI DEBUG
//https://github.com/x04/BamewareBase/blob/master/Bameware%20Base%20Shared/Source/Protection/AntiDebug.cpp


//#define RH_ANTIDEBUG_WIN 

#if !defined(_WIN32_WINNT) || defined(_DEBUG)
    #undef RH_ANTIDEBUG_WIN
#endif

#ifdef RH_ANTIDEBUG_WIN
#undef RH_DECRYPT
#define RH_DECRYPT(venc, output)                           \
{                                                  \
    auto _lp = [](int n, int p, int mod)           \
    {                                              \
        int result = 1;                            \
        for (; p; p >>= 1)                         \
        {                                          \
            if (p & 1)                             \
                result = (1LL * result * n) % mod; \
            n = (1LL * n * n) % mod;               \
        }                                          \
        return result;                             \
    };                                             \
    output = "";                                   \
    int dec;                                       \
    U32 _c = venc[venc.size()-2];                  \
    for (int i = 0; i < venc.size()-1; i++)        \
    {                                              \
        dec = _lp(venc[i], 10249909, 11580379);    \
        output += (char)dec;                       \
        _c += (char)dec;                           \
    }                                              \
    if (venc[venc.size()-1] != _c)                 \
        exit(__LINE__);                            \
    output += "\0";                                \
}                                                  \


#include "winternl.h"
typedef NTSTATUS(NTAPI *pfnNtQueryInformationProcess)(
    _In_      HANDLE           ProcessHandle,
    _In_      UINT             ProcessInformationClass,
    _Out_     PVOID            ProcessInformation,
    _In_      ULONG            ProcessInformationLength,
    _Out_opt_ PULONG           ReturnLength
    );
typedef BOOL (WINAPI *pfnIsDP)();
typedef BOOL (WINAPI *pfnsCheckRemoteDebuggerPresent)(HANDLE, PBOOL);

/*
        //Kernel32.dll 0x259A6A
        U32 estring_kernel32[] = {0x7a03ea, 0x8a49db, 0xaff517, 0x224b72, 0x8a49db, 0x25963a, 0x123542, 0x5d6c22, 0x995282, 0x620dcd, 0x25963a, 0x25963a, 0x259a6a, };
        //ntdll.dll 0x2599C2
        U32 estring_ntdll [] = {0x224b72, 0xa9f679, 0x620dcd, 0x25963a, 0x25963a, 0x995282, 0x620dcd, 0x25963a, 0x25963a, 0x2599c2, };
        //NtQueryInformationProcess 0x21BB22
        U32 estring_qip [] = {0x584498, 0xa9f679, 0x354f61, 0x9bd359, 0x8a49db, 0xaff517, 0x8b8942, 0x209d9, 0x224b72, 0x89efe9, 0x2552c3, 0xaff517, 0x4fef7a, 0x6c24b9, 0xa9f679, 0x206787, 0x2552c3, 0x224b72, 0xac3c16, 0xaff517, 0x2552c3, 0x37c525, 0x8a49db, 0x21b0e5, 0x21b0e5, 0x21bb22, };
        //IsDebuggerPresent 0xA9FD3B
        U32 estring_idp [] = {0x209d9, 0x21b0e5, 0x8543b9, 0x8a49db, 0x4f8f4f, 0x9bd359, 0x41af04, 0x41af04, 0x8a49db, 0xaff517, 0xac3c16, 0xaff517, 0x8a49db, 0x21b0e5, 0x8a49db, 0x224b72, 0xa9f679, 0xa9fd3b, };
        //CheckRemoteDebuggerPresent 0xAA00C9
        U32 estring_crdp [] = {0x858cfe, 0x3fcde, 0x8a49db, 0x37c525, 0x8d963e, 0x7bab60, 0x8a49db, 0x4fef7a, 0x2552c3, 0xa9f679, 0x8a49db, 0x8543b9, 0x8a49db, 0x4f8f4f, 0x9bd359, 0x41af04, 0x41af04, 0x8a49db, 0xaff517, 0xac3c16, 0xaff517, 0x8a49db, 0x21b0e5, 0x8a49db, 0x224b72, 0xa9f679, 0xaa00c9, };
*/

inline void DetectKernelDLLs()
{
    vector<int> testEnc = {0x7a03ea, 0x8a49db, 0xaff517, 0x224b72, 0x8a49db, 0x25963a, 0x123542, 0x5d6c22, 0x995282, 0x620dcd, 0x25963a, 0x25963a, 0x259a6a, };
    string kernel32;

    RH_DECRYPT(testEnc, kernel32);
    //prevent dlls in gwd + modul-path
    HMODULE m = LoadLibrary(kernel32.c_str());
    char fn[512] = {0};
    GetModuleFileName(m, fn, 512);
    char m_fn[512] = {0};
    GetModuleFileName(0, m_fn, 512);
    char* f = strrchr(fn, '\\');
    char* p = strrchr(m_fn, '\\');
    *p = 0;
    *f = 0;
    if (__stricmp(fn, m_fn) == 0 || strlen(fn) == 0)
        exit(__LINE__);

    FreeLibrary(m);
}

inline void DetectDebuger_slow()
{
    string sIsDebuggerPresent;
    string kernel32;
    vector<int> testEnc1 = {0x209d9, 0x21b0e5, 0x8543b9, 0x8a49db, 0x4f8f4f, 0x9bd359, 0x41af04, 0x41af04, 0x8a49db, 0xaff517, 0xac3c16, 0xaff517, 0x8a49db, 0x21b0e5, 0x8a49db, 0x224b72, 0xa9f679, 0xa9fd3b, };

    {
        vector<int> testEnc = {0x7a03ea, 0x8a49db, 0xaff517, 0x224b72, 0x8a49db, 0x25963a, 0x123542, 0x5d6c22, 0x995282, 0x620dcd, 0x25963a, 0x25963a, 0x259a6a, };
        RH_DECRYPT(testEnc, kernel32);
        RH_DECRYPT(testEnc1, sIsDebuggerPresent);
    }

    HMODULE hNtDll = LoadLibrary(kernel32.c_str());
    pfnIsDP pIsDP = (pfnIsDP)GetProcAddress(hNtDll, sIsDebuggerPresent.c_str());
  
    if (pIsDP())
        exit(__LINE__);

    FreeLibrary(hNtDll);
}

inline void CheckRemoteDebuggerPresentAPI()
{
    string sCheckRemoteDebuggerPresent;
    string kernel32;

    
    vector<int> testEnc1 = {0x858cfe, 0x3fcde, 0x8a49db, 0x37c525, 0x8d963e, 0x7bab60, 0x8a49db, 0x4fef7a, 0x2552c3, 0xa9f679, 0x8a49db, 0x8543b9, 0x8a49db, 0x4f8f4f, 0x9bd359, 0x41af04, 0x41af04, 0x8a49db, 0xaff517, 0xac3c16, 0xaff517, 0x8a49db, 0x21b0e5, 0x8a49db, 0x224b72, 0xa9f679, 0xaa00c9, };
    {
        vector<int> testEnc = {0x7a03ea, 0x8a49db, 0xaff517, 0x224b72, 0x8a49db, 0x25963a, 0x123542, 0x5d6c22, 0x995282, 0x620dcd, 0x25963a, 0x25963a, 0x259a6a, };
        RH_DECRYPT(testEnc, kernel32);
        RH_DECRYPT(testEnc1, sCheckRemoteDebuggerPresent);
    }
    

    HMODULE hNtDll = LoadLibrary(kernel32.c_str());
    pfnsCheckRemoteDebuggerPresent psRpDp = (pfnsCheckRemoteDebuggerPresent)GetProcAddress(hNtDll, sCheckRemoteDebuggerPresent.c_str());

    BOOL debugger_present = TRUE;
    psRpDp(GetCurrentProcess(), &debugger_present);
    if (debugger_present)
        exit(__LINE__);
    FreeLibrary(hNtDll);
}

inline void DetectHardwareBreakpoints_slow()
{
    CONTEXT ctx = {};
    ctx.ContextFlags = CONTEXT_DEBUG_REGISTERS;
    if (GetThreadContext(GetCurrentThread(), &ctx))
    {
        if (ctx.Dr0 != 0 || ctx.Dr1 != 0 || ctx.Dr2 != 0 || ctx.Dr3 != 0)
        {
            exit(__LINE__);
        }
    }
}



#endif //RH_ANTIDEBUG
//}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
