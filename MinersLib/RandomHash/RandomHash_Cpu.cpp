/**
 * RandomHash source code implementation
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
/// @copyright Polyminer1

#include "precomp.h"


////////////////////////////////////////////////////
// opt
//#define RH2_STRIDE_PACKMODE //debug only
#define RH2_STRIDE_USE_MEMCPY
//#define RH2_ENABLE_TRANSFO0_MMX128 //slower
#if !defined(_WIN32_WINNT)
    #define RH2_ENABLE_MERSSEN_12_SSE4
    #define RH2_ENABLE_MERSSEN_INTERLEAVE
    //#define RH2_ENABLE_EXPAND_MERSSEN_USING_FAST4 
    #define RH2_ENABLE_EXPAND_MERSSEN_INPLACE
#else
    #define RH2_ENABLE_MERSSEN_12_SSE4 
    //#define RH2_ENABLE_MERSSEN_INTERLEAVE             //slower
    //#define RH2_ENABLE_EXPAND_MERSSEN_USING_FAST4     //slower
    #define RH2_ENABLE_EXPAND_MERSSEN_INPLACE
#endif
#define RH2_ENABLE_TRANSFO_INPLACE
#define RH2_ENABLE_COMPACT_TRANSFO_67
#define RH2_ENABLE_MEM_ZERO_X_USE_MMX
#define RH2_AVOID_MMX_FOR_INPLACE_MEMCPY
#define RH2_ENABLE_CACHE
#define RH2_ENABLE_PREFLIGHT_CACHE
#define RH2_ENABLE_PREFLIGHT_4X
#define RH2_ENABLE_SHA256_PRERUN

//force UT
#ifdef _DEBUG
    #define RHMINER_DEBUG_RANDOMHASH_UNITTEST
#endif

#ifndef _DEBUG
    #undef RHMINER_DEBUG_RANDOMHASH_UNITTEST
#endif

#ifdef RHMINER_NO_SIMD
    #define RH2_ENABLE_MERSSEN_INTERLEAVE
    #undef RH2_ENABLE_EXPAND_MERSSEN_INPLACE
    #undef RH2_ENABLE_MEM_ZERO_X_USE_MMX
    #undef RH2_ENABLE_PREFLIGHT_4X
#endif

#ifndef RH2_ENABLE_PREFLIGHT_CACHE
    #undef RH2_ENABLE_PREFLIGHT_4X
#endif

// TODO: Make RH2_ENABLE_PREFLIGHT_4X work for VNet")
// TODO: Make RH2_ENABLE_PREFLIGHT_CACHE work for VNet")

#include "MinersLib/RandomHash/RandomHash.h"
#include "MinersLib/RandomHash/RandomHash_MurMur3_32.h"  

#include "MinersLib/RandomHash/RandomHash_Blake2b.h" 
#include "MinersLib/RandomHash/RandomHash_Blake2s.h"
#include "MinersLib/RandomHash/RandomHash_Grindahl512.h"
#include "MinersLib/RandomHash/RandomHash_Haval_5_256.h" 
#include "MinersLib/RandomHash/RandomHash_MD5.h"
#include "MinersLib/RandomHash/RandomHash_RadioGatun32.h"
#include "MinersLib/RandomHash/RandomHash_RIPEMD160.h" 
#include "MinersLib/RandomHash/RandomHash_RIPEMD256.h"
#include "MinersLib/RandomHash/RandomHash_RIPEMD320.h"
#include "MinersLib/RandomHash/RandomHash_SHA2_256.h"
#include "MinersLib/RandomHash/RandomHash_SHA2_512.h"
#include "MinersLib/RandomHash/RandomHash_SHA3_512.h"
#include "MinersLib/RandomHash/RandomHash_Snefru_8_256.h"
#include "MinersLib/RandomHash/RandomHash_Tiger2_5_192.h"
#include "MinersLib/RandomHash/RandomHash_Whirlpool.h"

//--------------- RandomHash2 -------------------
#include "MinersLib/RandomHash/RandomHash_Ghost.h"
#include "MinersLib/RandomHash/RandomHash_Ghost3411.h"
#include "MinersLib/RandomHash/RandomHash_MD2.h"
#include "MinersLib/RandomHash/RandomHash_MD4.h"
#include "MinersLib/RandomHash/RandomHash_Panama.h"
#include "MinersLib/RandomHash/RandomHash_HAS160.h"
#include "MinersLib/RandomHash/RandomHash_SHA.h"
#include "MinersLib/RandomHash/RandomHash_Grindahl256.h"
#include "MinersLib/RandomHash/RandomHash_Haval.h"
#include "MinersLib/RandomHash/RandomHash_RIPEMD.h"
#include "MinersLib/RandomHash/RandomHash_RIPEMD128.h"
//--------------- RandomHash2 -------------------


#define RH_FULLDEBUG_CPU 
#ifdef RHMINER_RELEASE
    #undef RH_FULLDEBUG_CPU
#endif

//{{{ TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
//#include "MinersLib\Global.h"
#ifdef RH_FULLDEBUG_CPU
//#define RH_FULLDEBUG_CPU_PRINTOUT DebugOut
#define RH_FULLDEBUG_CPU_PRINTOUT(...) 
//#define RH_FULLDEBUG_PRINT(...) PrintOut(__VA_ARGS__); 
#define RH_FULLDEBUG_PRINT(...) 

//{if (itt == 3716) _PDBN;}
thread_local U32 itt = 0;
thread_local U32 _n;
thread_local U32 _maxAllocIndex = 0;
thread_local U32 _hash;
thread_local std::map<U8*, U32> _inputPtr;
#define BoolToStr(B) ((B) ? "-1":"0")

U32 _GetInputPtr(U8* i)
{
    auto fnd = _inputPtr.find(i);
    if (fnd == _inputPtr.end())
        return 0xFFFF;
    else
        return (*fnd).second;
}
inline U32 RH_xcrc(U8* data, size_t s)
{
    /*
    U32 cval = 0xABD13D59;
    while (s-- != 0)
        cval ^= ROTL32((*data++ * 0xBFD06116), s);
    */
    MurmurHash3_x86_32_State state;
    MurmurHash3_x86_32_Init(0, &state);
    MurmurHash3_x86_32_Update(data, s, &state);
    U32 cval = MurmurHash3_x86_32_Finalize(&state);
    return cval;

}
inline U32 RH_xcrcArray(RH_StrideArrayStruct& inputs)
{
    U32 cval = 0;
    RH_STRIDEARRAY_FOR_EACH_BEGIN(inputs)
    {
        cval += RH_xcrc(RH_STRIDE_GET_DATA8(strideItrator), RH_STRIDE_GET_SIZE(strideItrator));
    }
    RH_STRIDEARRAY_FOR_EACH_END(inputs)
    
    return cval;
}
U32 RandomHash_ChecksumArray(RH_StrideArrayStruct& inputs)
{
    U32 csum;
    MurmurHash3_x86_32_State state;
    MurmurHash3_x86_32_Init(0, &state);

    RH_STRIDEARRAY_FOR_EACH_BEGIN(inputs)
    {
        MurmurHash3_x86_32_Update(RH_STRIDE_GET_DATA8(strideItrator), RH_STRIDE_GET_SIZE(strideItrator), &state);
    }
    RH_STRIDEARRAY_FOR_EACH_END(inputs)
    csum = MurmurHash3_x86_32_Finalize(&state);
	return csum;
}
string _ArrayStr(RH_StrideArrayStruct& arr)
{
    string res = FormatString("[%3d]", RH_STRIDEARRAY_GET_SIZE(arr));
    {
        RH_STRIDEARRAY_FOR_EACH_BEGIN(arr)
        {
            RH_STRIDE_CHECK_INTEGRITY(strideItrator);

            res += FormatString("%08x_", RH_xcrc(RH_STRIDE_GET_DATA8(strideItrator), RH_STRIDE_GET_SIZE(strideItrator)));
            res += FormatString("%02d_", RH_STRIDE_GET_INDEX(strideItrator));
            //res += FormatString("%06d_", RH_STRIDE_GET_SIZE(strideItrator));
        }
        RH_STRIDEARRAY_FOR_EACH_END(arr)
    }
    
    extern U32 RandomHash_ChecksumArray(RH_StrideArrayStruct& inputs);
    res += FormatString("[csum = %x]", RandomHash_ChecksumArray(arr));
    return res;
}
string _StrideStr(RH_StridePtr stride)
{
    string res;
    res += FormatString("%08x", RH_xcrc(RH_STRIDE_GET_DATA8(stride), RH_STRIDE_GET_SIZE(stride)));
    
    return res;
}
extern std::string toHex(void* p, size_t len, bool withEndl);
string _Bytes2Hex(RH_StridePtr stridePtr, bool x = false)
{
    string res;
    {
        U32 ssize = RH_STRIDE_GET_SIZE(stridePtr);
        U32 off = 0;
        if (ssize > 20)
        {
            off = ssize - 16;
            ssize = 16;
        }
        res = FormatString("[%3d]:%s, ", RH_STRIDE_GET_SIZE(stridePtr), toHex(RH_STRIDE_GET_DATA8(stridePtr)+off, ssize, false).c_str());
    }
    
    return ToUpper(res);
}
string _Bytes2HexPart(RH_StridePtr stridePtr, U32 removeLast)
{
    //TEMP TEMP TEMP
    removeLast = 0;
    //TEMP TEMP TEMP
    string res;
    {
        U32 ssize = removeLast > RH_STRIDE_GET_SIZE(stridePtr) ? RH_STRIDE_GET_SIZE(stridePtr) : RH_STRIDE_GET_SIZE(stridePtr) - removeLast;
        res = FormatString("[%3d]:%s, ", RH_STRIDE_GET_SIZE(stridePtr), toHex(RH_STRIDE_GET_DATA8(stridePtr), ssize, false).c_str());
    }
    
    return ToUpper(res);
}
string _ArrayBytes2Hex(RH_StrideArrayStruct& arr, bool x = false)
{
    string res= FormatString("[%3d]:", RH_STRIDEARRAY_GET_SIZE(arr));
    {
        RH_STRIDEARRAY_FOR_EACH_BEGIN(arr)
        {
            RH_STRIDE_CHECK_INTEGRITY(strideItrator);

            res += _Bytes2Hex(strideItrator);
        }
        RH_STRIDEARRAY_FOR_EACH_END(arr)
    }
    
    if (res.length() > 6000)
    {
        res.resize(6000);
        res += "...";
    }
    
    return ToUpper(res);
}


#ifdef RH2_STRIDE_PACKMODE
string PakListBytes2Hex(RH_StrideArrayStruct& arr)
{
    U32 ssize = RH_STRIDEARRAY_GET_SIZE(arr);
    string res= FormatString("[%3d]: sizes ", ssize);
    {
        for(int i=0; i < ssize; i++)
            res += FormatString("%d, ", arr.packedSizes[i]);
        res += FormatString(" - idx ");
        for(int i=0; i < ssize; i++)
            res += FormatString("%d, ", arr.packedIdx[i]);
    }
    
    if (res.length() > 6000)
    {
        res.resize(6000);
        res += "...";
    }
    
    return ToUpper(res);
}
#else
#define PakListBytes2Hex _ArrayBytes2Hex
#endif

#define _ListBytes2Hex _ArrayBytes2Hex
#else
    #define RH_FULLDEBUG_CPU_PRINTOUT(...)
    #define RH_FULLDEBUG_PRINT(...) 
#endif //RH_FULLDEBUG_CPU


//#define RH_MAKE_STATS
#ifdef RH_MAKE_STATS
std::mutex*              g_sm = new std::mutex;
U32 g_stridesAllocIndexStat = 0;
map<U32, U32> g_LNumNeighboursStat;
U32 g_LNumNeighbours_cnt=0;
U32 g_algoUSed_cnt = 0;
U32 g_algoUSed[] = { 0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0 };
U32    c_AlgoSizeStat[] = { 0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0 };
U32 g_maxArraySizeRoundStat[RH2_MAX_N + 1] = { 0, 0, 0, 0, 0 };
U32 g_maxArraySizeParentRoundStat[RH2_MAX_N + 1] = { 0, 0, 0, 0, 0 };


    map<U32, U32> md2_sizes;
    map<U64, U32> Trx_sizes[8];
    /*
    {
        std::lock_guard<std::mutex> g(*g_sm);
        (Trx_sizes[r])[nextChunkSize]++;
    }

    {
        extern map<U64, U32> Trx_sizes[8];
        for (int t = 0; t < 8; t++)
        {
            PrintOut("~Trx %d : \n", t);
            for(auto x : Trx_sizes[t])
                PrintOut("~size %3d is %8d\n", x.first, x.second);
            PrintOut("~\n");
        }
    }
    */
    /*
    extern map<U32, U32> md2_sizes;
    PrintOut("~\nMD2 sizes : \n");
    for(auto x : md2_sizes)
        PrintOut("~size %d is %d\n", x.first, x.second);
    */


#define RH_MAKE_STATS_ALGO_USED \
        { \
            std::lock_guard<std::mutex> g(*g_sm); \
            g_algoUSed[rndHash]++; \
            g_algoUSed_cnt++; \
        } \

#define RH_MAKE_STATS_LNumNeighbours \
        { \
            std::lock_guard<std::mutex> g(*g_sm); \
            g_LNumNeighboursStat[LNumNeighbours]++; \
            g_LNumNeighbours_cnt++; \
        } \

#define RH_MAKE_STATS_MAX_STRIDE_SIZE \
        {std::lock_guard<std::mutex> g(*g_sm); \
        if (in_state->m_stridesAllocIndex > g_stridesAllocIndexStat) \
            g_stridesAllocIndexStat = in_state->m_stridesAllocIndex; \
        } \

#define RH_MAKE_STATS_MAX_ALGO_SIZE \
        { \
            std::lock_guard<std::mutex> g2(*g_sm); \
            c_AlgoSizeStat[rndHash] = RH_STRIDE_GET_SIZE(output);  \
        } \


#define RH_MAKE_STATS_MAX_ROUNDOUTPUT_SIZE(Strides, Round) \
        { \
            std::lock_guard<std::mutex> g2(*g_sm); \
            if (RH_STRIDEARRAY_GET_SIZE(Strides) > g_maxArraySizeRoundStat[Round]) \
                g_maxArraySizeRoundStat[Round] = RH_STRIDEARRAY_GET_SIZE(Strides); \
        } \

#define RH_MAKE_STATS_MAX_PARENTROUNDOUTPUT_SIZE(Strides, Round) \
        { \
            std::lock_guard<std::mutex> g2(*g_sm); \
            if (RH_STRIDEARRAY_GET_SIZE(Strides) > g_maxArraySizeParentRoundStat[Round]) \
                g_maxArraySizeParentRoundStat[Round] = RH_STRIDEARRAY_GET_SIZE(Strides); \
        } \

 
void PrintStats()
{
    std::lock_guard<std::mutex> g(*g_sm);
    PrintOut("Stats : \n  Count %d\n", g_algoUSed_cnt);
    for (int i = 0; i < RHMINER_ARRAY_COUNT(g_algoUSed); i++)
    {
        ////////////TEMP TEMP TEMP RHMINER_ASSERT(g_algoUSed[i] != 0);
        PrintOut("Algo %2d = %d\n", i, g_algoUSed[i]);
    }
    PrintOut("~const U32    c_AlgoSize[] = { ");
    for (auto c : c_AlgoSizeStat)
        PrintOut("~%d, ", c);
    PrintOut("~}\n");
    PrintOut("#define RH2_STRIDE_BANK_SIZE %d\n", RHMINER_CEIL(g_stridesAllocIndexStat, 4096));
    PrintOut("~const U32 g_maxArraySizeRounds[] = {");
    for (auto v : g_maxArraySizeRoundStat)
        PrintOut("~%d, ", v);
    PrintOut("~};\n");
    PrintOut("~const U32 g_maxArraySizeParentRounds[] = {");
    for (auto v : g_maxArraySizeParentRoundStat)
        PrintOut("~%d, ", v);
    PrintOut("~};\n");

    PrintOut("LNumNeighbours : %d\n", g_LNumNeighbours_cnt);
    for (auto v : g_LNumNeighboursStat)
        PrintOut("Size %d is %d\n", v.first, v.second);
}
#else
#define RH_MAKE_STATS_ALGO_USED
#define RH_MAKE_STATS_MAX_STRIDE_SIZE 
#define RH_MAKE_STATS_MAX_ALGO_SIZE
#define RH_MAKE_STATS_MAX_ROUNDOUTPUT_SIZE(Strides, Round)
#define RH_MAKE_STATS_MAX_PARENTROUNDOUTPUT_SIZE(Strides, Round)
#define RH_MAKE_STATS_LNumNeighbours 

#endif
//}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP


#include "corelib/CommonData.h"

#include "MinersLib/RandomHash/RandomHash_inl.h"


void RandomHash_Free(void* ptr)
{
    if (ptr)
        RH_SysFree(ptr);
}

void RandomHash_Alloc(void** out_ptr, size_t size)
{
    *out_ptr = RH_SysAlloc(size);
    RHMINER_ASSERT(*out_ptr);
}


const U32    c_AlgoSize[] = { 20, 32, 64, 48, 16, 20, 28, 32, 32, 32, 64, 32, 64, 20, 16, 20, 24, 28, 32, 
                              16, 20, 24, 28, 32, 16, 20, 24, 28, 32, 28, 32, 36, 48, 64, 16, 16, 16, 32, 
                              32, 16, 16, 20, 32, 40, 20, 20, 28, 32, 48, 64, 28, 32, 28, 32, 48, 64, 16, 
                              32, 16, 20, 24, 16, 20, 24, 16, 20, 24, 16, 20, 24, 16, 20, 24, 16, 20, 24, 64, 28,32 };
static thread_local U64                 c_target = 0xFFFFFFFFFFFFFfFF;
#define GetTarget()                     c_target

void RandomHash_CreateMany(RandomHash_State** outPtr, U32 count)
{
    RandomHash_Alloc((void**)outPtr, sizeof(RandomHash_State)*count);
    RH_CUDA_ERROR_CHECK();

    for (U32 i = 0; i < count; i++)
    {
        RandomHash_Create(&(*outPtr)[i]);
    }
} 


/*
void AllocateArray(RH_StrideArrayStruct& arrayData, int count)
{
    static_assert(sizeof(void*) == sizeof(U64), "Incorrect ptr size");
    U32 size = sizeof(RH_StrideArrayStruct);
    RandomHash_Alloc((void**)&arrayData, size);
    //PLATFORM_MEMSET(arrayData, 0, size);
    RH_STRIDEARRAY_RESET(arrayData);

#ifndef RH_DISABLE_RH_ASSERTS
    RH_STRIDEARRAY_GET_MAXSIZE(arrayData) = count;
#endif
}
*/

/*`
void RandomHash_RoundDataAlloc(RH_RoundData* rd, int round)
{
    RH_ASSERT(round < RH2_MAX_N + 1)
    PLATFORM_MEMSET(rd, 0, sizeof(RH_RoundData));

    //AllocateArray(rd->roundOutputs,RH2_StrideArrayCount);
    //AllocateArray(rd->parenAndNeighbortOutputs,RH2_StrideArrayCount);
}

void RandomHash_RoundDataInit(RH_RoundData* rd, int round)
{
    //RH_ASSERT(rd->roundOutputs || (round == 0 && rd->roundOutputs == 0));
    //RH_STRIDEARRAY_RESET(rd->roundOutputs);
    //RH_STRIDEARRAY_RESET( rd->parenAndNeighbortOutputs );
}
*/

inline void RandomHash_Initialize(RandomHash_State* state)
{
    /*
    RandomHash_RoundDataInit(&state->m_data[0], 0);
    RandomHash_RoundDataInit(&state->m_data[1], 1);
    RandomHash_RoundDataInit(&state->m_data[2], 2);
    RandomHash_RoundDataInit(&state->m_data[3], 3);
    RandomHash_RoundDataInit(&state->m_data[4], 4);
    */

    //state->m_data[RH2_MAX_N].io_results = state->m_data[RH2_MAX_N].roundOutputs;
    state->m_workBytes = state->m_precalcPart2;



    /*
    state->m_strideID = 0;
    RH_STRIDEARRAY_RESET(state->m_round5Phase2PrecalcArray);

    //restert allocating at the begening of the buffer in midstate round
    if (state->m_isCachedOutputs)
    {
        state->m_isMidStateRound = true;

        RH_ASSERT(state->m_stridesAllocIndex);
        if (state->m_stridesAllocMidstateBarrierNext != RH2_STRIDE_BANK_SIZE) 
        {
            //Preserve Right, use left
            state->m_stridesAllocIndex = 0;
            state->m_stridesAllocMidstateBarrier = state->m_stridesAllocMidstateBarrierNext;

#ifdef RHMINER_DEBUG_STRIDE_INTEGRITY_CHECK
            //init left
            memset(state->m_pStridesInstances + 0, (U8)0xBA, state->m_stridesAllocMidstateBarrierNext); 
            U64* check = (U64*)(state->m_pStridesInstances + RH2_STRIDE_BANK_SIZE);
            RHMINER_ASSERT(*check == 0xFF55AA44BB8800DDLLU);
#endif            
        }
        else 
        {
            //preserve LEFT, use right
            const U32 ReqDelta = 4096;
            state->m_stridesAllocIndex = RHMINER_ALIGN(state->m_stridesAllocIndex, 4096) + ReqDelta;
            state->m_stridesAllocMidstateBarrierNext = state->m_stridesAllocMidstateBarrier;
            state->m_stridesAllocMidstateBarrier = RH2_STRIDE_BANK_SIZE;


#ifdef RHMINER_DEBUG_STRIDE_INTEGRITY_CHECK
            //init right
            memset(state->m_pStridesInstances + state->m_stridesAllocIndex, (U8)0xBA, RH2_STRIDE_BANK_SIZE - state->m_stridesAllocIndex);   
            U64* check = (U64*)(state->m_pStridesInstances + RH2_STRIDE_BANK_SIZE);
            RHMINER_ASSERT(*check == 0xFF55AA44BB8800DDLLU);
#endif
        }
    }
    else
    {
        state->m_stridesAllocMidstateBarrierNext = RH2_STRIDE_BANK_SIZE;
        state->m_stridesAllocMidstateBarrier = 0;
    }
*/
        state->m_strideID = 0;
        state->m_stideDataFlipFlop = state->m_stideDataFlipFlop ? 0 : 1;
        state->m_pPartiallyComputedOutputsClone = &state->m_stideDBuffer[state->m_stideDataFlipFlop].m_partiallyComputedOutputsClone;
        state->m_pStridesInstances = state->m_stideDBuffer[state->m_stideDataFlipFlop].m_stridesInstances;
        state->m_stridesAllocIndex = 0;

#ifdef RHMINER_DEBUG_STRIDE_INTEGRITY_CHECK
        memset(state->m_stideDBuffer[state->m_stideDataFlipFlop].m_stridesInstances, (U8)0xBA, RH2_STRIDE_BANK_SIZE);
        U64* check = (U64*)(((U8*)state->m_stideDBuffer[state->m_stideDataFlipFlop].m_stridesInstances) + RH2_STRIDE_BANK_SIZE);
        RHMINER_ASSERT(*check == 0xFF55AA44BB8800DDLLU);
#endif
}


void RandomHash_Create(RandomHash_State* state)
{
    U32 ajust = 0;
#ifdef RHMINER_DEBUG_STRIDE_INTEGRITY_CHECK
    ajust = sizeof(U64);
#endif    

    RandomHash_Alloc((void**)&state->m_stideDBuffer[0].m_stridesInstances, RH2_STRIDE_BANK_SIZE + ajust);
    RandomHash_Alloc((void**)&state->m_stideDBuffer[1].m_stridesInstances, RH2_STRIDE_BANK_SIZE + ajust);
    //AllocateArray(state->m_stideDBuffer[0].m_partiallyComputedOutputsClone, RH2_StrideArrayCount);
    //AllocateArray(state->m_stideDBuffer[1].m_partiallyComputedOutputsClone, RH2_StrideArrayCount);

#ifdef RHMINER_DEBUG_STRIDE_INTEGRITY_CHECK
    U64* check = (U64*)(((U8*)state->m_stideDBuffer[0].m_stridesInstances) + RH2_STRIDE_BANK_SIZE);
    *check = 0xFF55AA44BB8800DDLLU;
    check = (U64*)(((U8*)state->m_stideDBuffer[1].m_stridesInstances) + RH2_STRIDE_BANK_SIZE);
    *check = 0xFF55AA44BB8800DDLLU;
#endif

    RH_CUDA_ERROR_CHECK();
    state->m_stideDataFlipFlop = 0;    

    
    //state->m_stridesAllocMidstateBarrierNext = 0;
    /*
    RandomHash_RoundDataAlloc(&state->m_data[0], 0);
    RandomHash_RoundDataAlloc(&state->m_data[1], 1);
    RandomHash_RoundDataAlloc(&state->m_data[2], 2);
    RandomHash_RoundDataAlloc(&state->m_data[3], 3);
    RandomHash_RoundDataAlloc(&state->m_data[4], 4);
    //RandomHash_RoundDataAlloc(&state->m_data[5], 5);
    */
    //AllocateArray(state->m_round5Phase2PrecalcArray, 31);
    //RH_STRIDEARRAY_RESET(state->m_round5Phase2PrecalcArray);
    
    
    state->m_isNewHeader = true;
    //state->m_partiallyComputedOutputs = 0;

    /*state->m_isCachedOutputs = false;
    state->m_midStateNonce = 0xFFFFFFFF;
    state->m_isMidStateRound = false;
    state->m_data[1].first_round_consume = false;
    state->m_data[2].first_round_consume = false;
    state->m_data[3].first_round_consume = false;
    state->m_data[4].first_round_consume = false;
    state->m_data[5].first_round_consume = false;*/

    RandomHash_Initialize(state);
}


void RandomHash_Destroy(RandomHash_State* state)
{
    /*RandomHash_RoundDataUnInit(&state->m_data[0], 0);
    RandomHash_RoundDataUnInit(&state->m_data[1], 1);
    RandomHash_RoundDataUnInit(&state->m_data[2], 2);
    RandomHash_RoundDataUnInit(&state->m_data[3], 3);
    RandomHash_RoundDataUnInit(&state->m_data[4], 4);
    //RandomHash_RoundDataUnInit(&state->m_data[5], 5);
    */
   
    //state->m_isCachedOutputs = false;
    RandomHash_Free(state->m_stideDBuffer[0].m_stridesInstances);
    RandomHash_Free(state->m_stideDBuffer[1].m_stridesInstances);
    //RandomHash_Free(state->m_stideDBuffer[0].m_partiallyComputedOutputsClone);
    //RandomHash_Free(state->m_stideDBuffer[1].m_partiallyComputedOutputsClone);
}

void RandomHash_DestroyMany(RandomHash_State* stateArray, U32 count)
{
    if (stateArray)
    {
        for (U32 i = 0; i < count; i++)
            RandomHash_Destroy(&stateArray[i]);
        RandomHash_Free(stateArray);
    }
} 

/*
void RandomHash_RoundDataUnInit(RH_RoundData* rd, int round)
{
    RandomHash_Free(rd->roundOutputs);
}
*/

void RandomHash_SetHeader(RandomHash_State* state, U8* sourceHeader, U32 headerSize, U32 nonce2) 
{
    RHMINER_ASSERT(headerSize < MaxMiningHeaderSize);
    U8* targetInput = state->m_header;
    
    //state->m_isCachedOutputs = false;
    state->m_isNewHeader = true;
    state->m_headerSize = headerSize;

    //cache
#ifdef RH2_ENABLE_CACHE
    state->m_partiallyComputedCount = 0;
    state->m_partiallyComputedRound = U32_Max;
    state->m_partiallyComputedNonceHeader = U32_Max;
#endif

#ifdef RH2_ENABLE_PREFLIGHT_CACHE
    if (headerSize == PascalHeaderSizeV5)
    {
        state->m_isCalculatePreflight = 1;
        state->m_preflightData.endCut = 8;
    }
    else 
        state->m_isCalculatePreflight = 0;
#endif

    memcpy(targetInput, sourceHeader, state->m_headerSize);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    inline RH_StridePtr RH_StrideArrayAllocOutput(RandomHash_State* state, U32 initialSize) 
    {
   
        RH_ASSERT((size_t(state->m_stridesAllocIndex) % 32) == 0);
        RH_ASSERT((initialSize % 4) == 0);
        RH_StridePtr stride = state->m_pStridesInstances + (state->m_stridesAllocIndex/4);
        RH_ASSERT((size_t(stride) % 32) == 0);

    #ifdef RHMINER_DEBUG_STRIDE_INTEGRITY_CHECK
        if (state->m_strideID)
            RH_ASSERT(*(stride - 1) == 0xBABABABA);
    #endif

        state->m_stridesAllocIndex += initialSize + RH_IDEAL_ALIGNMENT;
        RH_ASSERT(state->m_stridesAllocIndex < RH2_STRIDE_BANK_SIZE);

        RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|   Stride Alloc #%02d           %d (+%d)\n", _n++, _hash, state->m_strideID, state->m_stridesAllocIndex, initialSize);
    
        //CpuYield();

    #ifdef RHMINER_DEBUG_STRIDE_INTEGRITY_CHECK
        RH_STRIDE_SET_INDEX(stride, state->m_strideID);
        state->m_strideID++;
    #endif

        return stride;
    }

    inline void RH_StrideArrayGrow(RandomHash_State* state, RH_StridePtr stride, U32 growSize)
    {
        /*if (state->m_isMidStateRound)
        {
            RHMINER_ASSERT(state->m_stridesAllocIndex + growSize+8 < state->m_stridesAllocMidstateBarrier);
        }
        */
        RH_ASSERT((growSize % 4) == 0);
        state->m_stridesAllocIndex += growSize;
        RH_ASSERT(state->m_stridesAllocIndex < RH2_STRIDE_BANK_SIZE);

        //CpuYield();
        RH_STRIDE_SET_SIZE(stride, RH_STRIDE_GET_SIZE(stride) + growSize);
        RH_STRIDE_INIT_INTEGRITY(stride);
    }


    inline void RH_StrideArrayClose(RandomHash_State* state, RH_StridePtr stride)
    {
        /*
        if (state->m_isMidStateRound)
        {
            RHMINER_ASSERT(RHMINER_ALIGN(state->m_stridesAllocIndex, 32) + 8 < state->m_stridesAllocMidstateBarrier);
        }
        */
        U32 ss = 0;
#ifdef RHMINER_DEBUG_STRIDE_INTEGRITY_CHECK
        ss += sizeof(U64);
#endif

        RH_ASSERT(RHMINER_ALIGN(state->m_stridesAllocIndex + ss, 32) > state->m_stridesAllocIndex);
        state->m_stridesAllocIndex = RHMINER_ALIGN(state->m_stridesAllocIndex + ss, 32);
        RH_ASSERT(state->m_stridesAllocIndex < RH2_STRIDE_BANK_SIZE);
        RH_ASSERT((size_t(state->m_stridesAllocIndex) % 32) == 0);
        RH_ASSERT((size_t(state->m_stridesAllocIndex) % 4) == 0);

        RH_STRIDE_CHECK_INTEGRITY(stride);
    }


inline U32 GetLastDWordLE(RH_StridePtr in_stride)
{
    U32 size = RH_STRIDE_GET_SIZE(in_stride);
    RH_ASSERT(size > 4);
    //return RH_swap_u32( *(U32*) (RH_STRIDE_GET_DATA(in_stride)+ size - 4));
    return *(RH_STRIDE_GET_DATA(in_stride) + ((size - 4)/4));
}

inline void SetLastDWordLE(RH_StridePtr in_stride, U32 newNonce)
{
    U32 size = RH_STRIDE_GET_SIZE(in_stride);
    RH_ASSERT(size > 4);
    RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Round.            Replaceing last %8x by %8x\n", _n++, _hash, 9, *(RH_STRIDE_GET_DATA(in_stride) + ((size - 4) / 4)), newNonce);
    *(RH_STRIDE_GET_DATA(in_stride) + ((size - 4)/4)) = newNonce;
}


    void RandomHash_Expand(RandomHash_State* state, RH_StridePtr input, U32 seed, int round, int ExpansionFactor)
    {
        U32 inputSize = RH_STRIDE_GET_SIZE(input);
#if defined(RH2_ENABLE_EXPAND_MERSSEN_USING_FAST4)
    #ifndef RH2_ENABLE_EXPAND_MERSSEN_INPLACE
        U32 mt4_sequence = 0;
        {
            U64 mt10;
            U64 mt32;
            U32 pval = (0x6c078965 * (seed ^ seed >> 30) + 1);
            mt10 = ((U64)pval) << 32 | seed;
            
            mt32 = 0x6c078965 * (pval ^ pval >> 30) + 2;
            pval = (U32)mt32;
            pval = 0x6c078965 * (pval ^ pval >> 30) + 3;
            mt32 |= ((U64)pval) << 32;
            pval = 0x6c078965 * (pval ^ pval >> 30) + 4;
            U32 mt5 = pval;

            U32 fval= 5;
            while (fval < MERSENNE_TWISTER_PERIOD)
                pval = 0x6c078965 * (pval ^ pval >> 30) + fval++;

            //future
            fval = 0x6c078965 * (pval ^ pval >> 30) + MERSENNE_TWISTER_PERIOD;
            // 0 ---
            pval = M32((U32)mt10);
            pval |= L31(mt10>>32);
            pval = fval ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);

            pval ^= pval >> 11; 
            pval ^= pval << 7 & 0x9d2c5680; 
            pval ^= pval << 15 & 0xefc60000; 
            pval ^= pval >> 18;

            //store
            mt4_sequence |= (pval % 8);
            mt4_sequence <<= 8;

            // 1 ---
            fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD+1;
            pval = M32(mt10>>32);
            pval |= L31((U32)mt32);
            pval = fval ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);

            pval ^= pval >> 11; 
            pval ^= pval << 7 & 0x9d2c5680; 
            pval ^= pval << 15 & 0xefc60000; 
            pval ^= pval >> 18;

            //store
            mt4_sequence |= (pval % 8);
            mt4_sequence <<= 8;

            // 2 ---
            fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD+2;
            pval = M32((U32)mt32);
            pval |= L31(mt32>>32);
            pval = fval ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);

            pval ^= pval >> 11; 
            pval ^= pval << 7 & 0x9d2c5680; 
            pval ^= pval << 15 & 0xefc60000; 
            pval ^= pval >> 18;

            //store
            mt4_sequence |= (pval % 8);
            mt4_sequence <<= 8;

            // 3 ---
            fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD+3;
            pval = M32(mt32>>32);
            pval |= L31(mt5);
            pval = fval ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);

            pval ^= pval >> 11; 
            pval ^= pval << 7 & 0x9d2c5680; 
            pval ^= pval << 15 & 0xefc60000; 
            pval ^= pval >> 18;

            //store
            mt4_sequence |= (pval % 8);
            mt4_sequence = RH_swap_u32(mt4_sequence);
        }
    #else
        //-------------------
        U32 rndIdx = 0;
        U64 mt10;
        U64 mt32;
        U32 pval = (0x6c078965 * (seed ^ seed >> 30) + 1);
        mt10 = ((U64)pval) << 32 | seed;
            
        mt32 = 0x6c078965 * (pval ^ pval >> 30) + 2;
        pval = (U32)mt32;
        pval = 0x6c078965 * (pval ^ pval >> 30) + 3;
        mt32 |= ((U64)pval) << 32;
        pval = 0x6c078965 * (pval ^ pval >> 30) + 4;
        U32 mt5 = pval;

        U32 fval= 5;
        while (fval < MERSENNE_TWISTER_PERIOD)
            pval = 0x6c078965 * (pval ^ pval >> 30) + fval++;

        //-------------------
    #endif //inplace
#else // RH2_ENABLE_EXPAND_MERSSEN_USING_FAST4
    #ifndef RH2_ENABLE_EXPAND_MERSSEN_INPLACE
        mersenne_twister_state   rndGenExpand;
        merssen_twister_seed_fast_partial(seed, &rndGenExpand, 4);

        //Faster with precalc
        merssen_twister_rand_fast_partial_4(&rndGenExpand);
#if defined(RH2_ENABLE_MERSSEN_INTERLEAVE) && (!defined(RHMINER_NO_SSE4) || defined(RHMINER_NO_SIMD)) 
        rndGenExpand.index -= 2;
#else
        rndGenExpand.index--;
#endif
        
#else
        __m128i r1;
        {
            __m128i f1;
            __m128i c1 = _mm_cvtsi32_si128(0x9d2c5680);
            __m128i c2 = _mm_cvtsi32_si128(0xefc60000);
            c1 = _mm_shuffle_epi32(c1, 0);
            c2 = _mm_shuffle_epi32(c2, 0);
            U32  pval, pval2, pvaln, ip1_l;
            pval = seed;

            // --------- init ----------
            //one group of 4
            r1 = _mm_cvtsi32_si128(pval);
            pval = 0x6c078965 * (pval ^ pval >> 30) + 1;
            r1 = _mm_insert_epi32_M(r1, pval, 1);
            pval = 0x6c078965 * (pval ^ pval >> 30) + 2;
            r1 = _mm_insert_epi32_M(r1, pval, 2);
            pval = 0x6c078965 * (pval ^ pval >> 30) + 3;
            r1 = _mm_insert_epi32_M(r1, pval, 3);
            ip1_l = 0x6c078965 * (pval ^ pval >> 30) + 4; //i+1 of the 4th element

            //jump to forward constant
            pval2 = 5;
            pval = ip1_l;
            while (pval2 <= MERSENNE_TWISTER_PERIOD)
            {
                pval = 0x6c078965 * (pval ^ pval >> 30) + pval2++;
            }

            //one group of 4 future constants
            f1 = _mm_cvtsi32_si128(pval);
            pval = 0x6c078965 * (pval ^ pval >> 30) + 398;
            f1 = _mm_insert_epi32_M(f1, pval, 1);
            pval = 0x6c078965 * (pval ^ pval >> 30) + 399;
            f1 = _mm_insert_epi32_M(f1, pval, 2);
            pval = 0x6c078965 * (pval ^ pval >> 30) + 400;
            f1 = _mm_insert_epi32_M(f1, pval, 3);

            //---------- seed ----------
            //seed
            RH_MT_ST(r1, f1, 0, 1);
            RH_MT_ST(r1, f1, 1, 2);
            RH_MT_ST(r1, f1, 2, 3);
            pval = _mm_extract_epi32_M(r1, 3);
            pval2 = ip1_l;
            pvaln = _mm_extract_epi32_M(f1, 3);
            pval = M32(pval) | L31(pval2);
            pval = pvaln ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);
            r1 = _mm_insert_epi32_M(r1, pval, 3);

            f1 = _mm_srli_epi32(r1, 11);
            r1 = _mm_xor_si128(r1, f1);
            f1 = _mm_slli_epi32(r1, 7);
            //c = _mm_cvtsi32_si128(0x9d2c5680);
            //c = _mm_shuffle_epi32(c, 0);
            f1 = _mm_and_si128(f1, c1);
            r1 = _mm_xor_si128(r1, f1);
            f1 = _mm_slli_epi32(r1, 15);
            //c = _mm_cvtsi32_si128(0xefc60000);
            //c = _mm_shuffle_epi32(c, 0);
            f1 = _mm_and_si128(f1, c2);
            r1 = _mm_xor_si128(r1, f1);
            f1 = _mm_srli_epi32(r1, 18);
            r1 = _mm_xor_si128(r1, f1);
        }
    #endif //RH2_ENABLE_EXPAND_MERSSEN_INPLACE
#endif
        size_t sizeExp = inputSize + ExpansionFactor * RH2_M;

        RH_StridePtr output = input;

        S64 bytesToAdd = sizeExp - inputSize;
        RH_ASSERT((bytesToAdd % 2) == 0);
        RH_ASSERT(RH_STRIDE_GET_SIZE(output) != 0);
        RH_ASSERT(RH_STRIDE_GET_SIZE(output) < RH2_StrideSize);
        RH_STRIDE_INIT_INTEGRITY(input);

        U8* outputPtr = RH_STRIDE_GET_DATA8(output);
        
        while (bytesToAdd > 0)
        {
            U32 nextChunkSize = RH_STRIDE_GET_SIZE(output);
            U8* nextChunk = outputPtr + nextChunkSize;
            if (nextChunkSize > bytesToAdd)
            {
                nextChunkSize = (U32)bytesToAdd;
            }

            RH_StrideArrayGrow(state, output, nextChunkSize);

            RH_ASSERT(nextChunk + nextChunkSize < ((U8*)(void*)output) + RH2_StrideSize);
#ifdef RH2_ENABLE_EXPAND_MERSSEN_USING_FAST4
#ifndef RH2_ENABLE_EXPAND_MERSSEN_INPLACE
            U32 r = (U8)mt4_sequence;
            mt4_sequence >>= 8;
#else
            //----------------------------------
            switch (rndIdx)
            {
            case 0:
                // 0 ---
                fval = 0x6c078965 * (pval ^ pval >> 30) + MERSENNE_TWISTER_PERIOD;
                pval = M32((U32)mt10);
                pval |= L31(mt10 >> 32);
                break;
            case 1:
                // 1 ---
                fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD + 1;
                pval = M32(mt10 >> 32);
                pval |= L31((U32)mt32);
                break;
            case 2:
                fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD + 2;
                pval = M32((U32)mt32);
                pval |= L31(mt32 >> 32);
                break;
            case 3:
                fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD + 3;
                pval = M32(mt32 >> 32);
                pval |= L31(mt5);
                break;
            }
            rndIdx++;
            pval = fval ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);

            pval ^= pval >> 11;
            pval ^= pval << 7 & 0x9d2c5680;
            pval ^= pval << 15 & 0xefc60000;
            pval ^= pval >> 18;
            U32 r = (pval % 8);
#endif
#else
#ifndef RH2_ENABLE_EXPAND_MERSSEN_INPLACE
            U32 random = merssen_twister_rand_fast_partial_4(&rndGenExpand);
            U32 r = random % 8;
#else
            U32 r = _mm_extract_epi32_M(r1, 0);
            r1 = _mm_bsrli_si128(r1, 4);
            r = r % 8;
#endif
#endif
            RH_ASSERT((nextChunkSize & 1) == 0);
            RH_ASSERT(r >= 0 && r <= 7);
#ifdef RH2_ENABLE_TRANSFO_INPLACE

            U32 halfSize = nextChunkSize >> 1;
            switch (r)
            {
                case 0:
                {
                    halfSize = (*(U32*)(outputPtr + nextChunkSize - 4));

                    if (!halfSize)
                        halfSize = 1;

                    for (U32 i = 0; i < nextChunkSize; i++)
                    {
                        halfSize ^= halfSize << 13;
                        halfSize ^= halfSize >> 17;
                        halfSize ^= halfSize << 5;
                        *nextChunk = outputPtr[halfSize % nextChunkSize];
                        nextChunk++;
                    }
                }
                break;
                case 1:
                {
                    memcpy(nextChunk, outputPtr + halfSize, halfSize);
                    memcpy(nextChunk + halfSize, outputPtr, halfSize);
                }
                break;
                case 2:
                {
                    U8* srcHead = outputPtr;
                    U8* srcEnd = srcHead + halfSize;
                    U8* srcTail = &outputPtr[nextChunkSize - 1];
                    U8* tail = &nextChunk[nextChunkSize - 1];
                    while (srcHead < srcEnd)
                    {
                        *nextChunk = *srcTail;
                        *tail = *srcHead;
                        nextChunk++;
                        tail--;
                        srcHead++;
                        srcTail--;
                    }
                }
                break;
                case 3:
                {
                    U32 left = 0;
                    U32 right = (int)halfSize;
                    while (left < halfSize)
                    {
                        *nextChunk = outputPtr[left++];
                        nextChunk++;
                        *nextChunk = outputPtr[right++];
                        nextChunk++;
                    }
                }
                break;
                case 4:
                {
                    U8* left = outputPtr;
                    U8* right = outputPtr + halfSize;
                    while (halfSize)
                    {
                        *nextChunk = *right;
                        nextChunk++;
                        *nextChunk = *left;
                        nextChunk++;
                        right++;
                        left++;
                        halfSize--;
                    }
                }
                break;
                case 5:
                {
                    S32 itt = 0;
                    S32 ritt = nextChunkSize - 1;
                    S32 fitt = 0;
                    while (fitt < halfSize)
                    {
                        //first half
                        nextChunk[fitt] = outputPtr[itt] ^ outputPtr[itt + 1];
                        itt += 2;
                        //second half
                        nextChunk[fitt + halfSize] = outputPtr[fitt] ^ outputPtr[ritt];
                        fitt++;
                        ritt--;
                    }
                }
                break;
                case 6:
                {
                    for (int i = 0; i < nextChunkSize; i++)
                        nextChunk[i] = ROTL8(outputPtr[i], nextChunkSize - i);
                }
                break;
                case 7:
                {
                    for (int i = 0; i < nextChunkSize; i++)
                        nextChunk[i] = ROTR8(outputPtr[i], nextChunkSize - i);
                }
                break;
        }
#else
            switch (r)
            {
            case 0: Transfo0_2(nextChunk, nextChunkSize, outputPtr); break;
            case 1: Transfo1_2(nextChunk, nextChunkSize, outputPtr); break;
            case 2: Transfo2_2(nextChunk, nextChunkSize, outputPtr); break;
            case 3: Transfo3_2(nextChunk, nextChunkSize, outputPtr); break;
            case 4: Transfo4_2(nextChunk, nextChunkSize, outputPtr); break;
            case 5: Transfo5_2(nextChunk, nextChunkSize, outputPtr); break;
            case 6: Transfo6_2(nextChunk, nextChunkSize, outputPtr); break;
            case 7: Transfo7_2(nextChunk, nextChunkSize, outputPtr); break;
            }
#endif

            RH_STRIDE_CHECK_INTEGRITY(output);
            RH_ASSERT(RH_STRIDE_GET_SIZE(output) < RH2_StrideSize);
            bytesToAdd = bytesToAdd - nextChunkSize;
        }
       

        RH_StrideArrayClose(state, output);
        RH_ASSERT(sizeExp == RH_STRIDE_GET_SIZE(output));
        RH_STRIDE_CHECK_INTEGRITY(output);
    }



    void RandomHash_Compress(RandomHash_State* state, RH_StrideArrayStruct& inputs, RH_StridePtr Result, U32 seed)
    {
        U32 rval;
        mersenne_twister_state   rndGenCompress;
        merssen_twister_seed_fast_partial(seed, &rndGenCompress, 204);
        
        //Faster with precalc
        merssen_twister_rand_fast_partial_204(&rndGenCompress);

#if defined(RH2_ENABLE_MERSSEN_INTERLEAVE) && (!defined(RHMINER_NO_SSE4) || defined(RHMINER_NO_SIMD)) 
        rndGenCompress.index -= 2;
#else
        rndGenCompress.index--;
#endif

        RH_STRIDE_SET_SIZE(Result, 100);
        U8* resultPtr = RH_STRIDE_GET_DATA8(Result);
        U32 inoutSize = RH_STRIDEARRAY_GET_SIZE(inputs);
        RH_StridePtr source;
        for (size_t i = 0; i < 100; i++)
        {
#ifdef RH2_STRIDE_PACKMODE
            U32 rndArray = merssen_twister_rand_fast_partial(&rndGenCompress, 204) % inoutSize;
            U8* asrc = inputs.packedData + inputs.packedIdx[rndArray];
            U32 sourceSize = inputs.packedSizes[rndArray];

            rval = merssen_twister_rand_fast_partial(&rndGenCompress, 204);
            resultPtr[i] = asrc[rval % sourceSize];
#else
            source = RH_STRIDEARRAY_GET(inputs, merssen_twister_rand_fast_partial(&rndGenCompress, 204) % inoutSize);
            U32 sourceSize = RH_STRIDE_GET_SIZE(source);

            rval = merssen_twister_rand_fast_partial(&rndGenCompress, 204);
            resultPtr[i] = RH_STRIDE_GET_DATA8(source)[rval % sourceSize];
#endif
        }
/*
        string ss, sd;
        RH_StridePtr ls = RH_STRIDEARRAY_GET(inputs, 0);
        U32 avg = 0;
        U32 mxdst = 0;

        for (size_t i = 0; i < 100; i++)
        {
            source = RH_STRIDEARRAY_GET(inputs, merssen_twister_rand_fast_partial(&rndGenCompress, 204) % inoutSize);
            U32 sourceSize = RH_STRIDE_GET_SIZE(source);
            ss += FormatString("%d, ", sourceSize);
            S64 aval = std::abs((S64)(size_t(source) - size_t(ls)));
            if (aval > mxdst)
                mxdst = aval;
            sd += FormatString("%llu, ", aval); 
            ls = source;
            avg += (U32)aval;

            rval = merssen_twister_rand_fast_partial(&rndGenCompress, 204);
            resultPtr[i] = RH_STRIDE_GET_DATA8(source)[rval % sourceSize];
        }
        U32 tot = 0;
        U32 mx = 0;
        ls = RH_STRIDEARRAY_GET(inputs, 0);
        //sd = "";
        ss = "";
        for (int i = 0; i < RH_STRIDEARRAY_GET_SIZE(inputs); i++)
        {
            source = RH_STRIDEARRAY_GET(inputs, i);
            ss += FormatString("%d, ", RH_STRIDE_GET_SIZE(source));
            tot += RH_STRIDE_GET_SIZE(source);
            if (RH_STRIDE_GET_SIZE(source) > mx)
                mx = RH_STRIDE_GET_SIZE(source);

            //S64 aval = std::abs((S64)(size_t(source) - size_t(ls)));
            //sd += FormatString("%llu, ", aval); 
            //ls = source;
            //avg += (U32)aval;
        }
        PrintOut("Sizes tot %8d max %8d- [%d] %s\n", tot, mx, RH_STRIDEARRAY_GET_SIZE(inputs), ss.c_str());
        PrintOut("Dist  avg %8d max %8d- [%d] %s\n\n", avg/100, mxdst, RH_STRIDEARRAY_GET_SIZE(inputs), sd.c_str());
*/
    }

    inline void ComputeVeneerRound(RandomHash_State* state, RH_StrideArrayStruct& in_strideArray, RH_StridePtr finalHash)
    {
//{{{ TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
#ifdef RH_FULLDEBUG_CPU
        bytes tmpR1;
        tmpR1.resize(RH_STRIDEARRAY_GET_SIZE(in_strideArray) + 1);
        memcpy(&tmpR1[0], &in_strideArray, RH_STRIDEARRAY_GET_SIZE(in_strideArray));
#endif
//}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP

        //LSeed := GetLastDWordLE(ARoundOutputs[High(ARoundOutputs)]);
        U32 seed = GetLastDWordLE(RH_STRIDEARRAY_GET(in_strideArray, RH_STRIDEARRAY_GET_SIZE(in_strideArray)-1));
        RandomHash_Compress(state, in_strideArray, state->m_workBytes, seed);
        RH_ASSERT(RH_STRIDE_GET_SIZE(state->m_workBytes) < WorkBytesSize);


        RandomHash_SHA2_256(state->m_workBytes, finalHash, false, false);

        //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d ComputeVeneerRound         %s -> 32:%s\n", _n++, _hash, 99,  _Bytes2Hex(&tmpR1[0]).c_str(), toHex(finalHash,32).c_str() );
        RH_FULLDEBUG_PRINT("Close seed = %X, compress = %s, sha256 = %s}\n""=========================================================================================\n", seed, _Bytes2Hex(state->m_workBytes, true).c_str(), (toHex(finalHash,32)).c_str(), true);

    }

    /*
    inline void RandomHash_Phase_1_push(RandomHash_State* state, int in_round)
    {
        RH_ASSERT(in_round >= 2);
        RH_ASSERT(in_round <= RH2_MAX_N+1);

        RH_ASSERT(state->m_data[in_round - 1].backup_io_results == 0);
        state->m_data[in_round-1].backup_io_results = state->m_data[in_round-1].io_results;
    
        //skip-//skip-cachedOutput
        state->m_data[in_round - 1].io_results = state->m_data[in_round].parenAndNeighbortOutputs;
    }

    inline void RandomHash_Phase_1_pop(RandomHash_State* state, int in_round)
    {
        RH_ASSERT(in_round >= 2);
        RH_ASSERT(in_round <= RH2_MAX_N+1);

        state->m_data[in_round-1].io_results = state->m_data[in_round-1].backup_io_results;

    }


    inline void RandomHash_Phase_2_push(RandomHash_State* state, int in_round)
    {
        RH_ASSERT(in_round >= 2);
        RH_ASSERT(in_round <= RH2_MAX_N+1);
        state->m_data[in_round-1].backup_io_results = state->m_data[in_round-1].io_results;
        state->m_data[in_round-1].io_results = state->m_data[in_round].parenAndNeighbortOutputs;
    }


    inline void RandomHash_Phase_2_pop(RandomHash_State* state, int in_round)         
    {
        RH_ASSERT(in_round >= 2);
        RH_ASSERT(in_round <= RH2_MAX_N+1);

        state->m_data[in_round-1].io_results = state->m_data[in_round-1].backup_io_results;
    }
    */


    //void CalculateRoundOutputs(const ABlockHeader: TBytes; round: Int32; APCachedHash : PCachedHash; out ARoundOutputs : TArray<TBytes>) : Boolean;
    bool CalculateRoundOutputs(RandomHash_State* state, U32 in_round, RH_StrideArrayStruct& roundOutputs)
    {    
        U32 LSeed = 0;
        //if (round < 1) or (round > RH2_MAX_N) then    raise EArgumentOutOfRangeException.CreateRes(@SInvalidRound);   
        RH_ASSERT(in_round >= 1 && in_round <= RH2_MAX_N);
        RH_ASSERT(RH_STRIDE_GET_SIZE(state->m_roundInput) <= state->m_headerSize);

        //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d CalculateRoundOutput Start %2d-%s\n", _n++, _hash, in_round,  RH_STRIDEARRAY_GET_SIZE(state->m_roundInput), (state->m_roundInput).c_str() );
#ifdef RH2_ENABLE_CACHE
        RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Computing Round for input  nonce %x, in %s\n", _n++, _hash, in_round,  GetLastDWordLE(state->m_roundInput), _StrideStr(state->m_roundInput).c_str());

        //TODO: ************ find re-usable round3 outputs

        mersenne_twister_state   rndGen; // &state->m_data[in_round].rndGen

        if (state->m_partiallyComputedCount &&
            state->m_partiallyComputedRound == in_round &&
            GetLastDWordLE(state->m_roundInput) == state->m_partiallyComputedNonceHeader)
        {
            RH_STRIDEARRAY_RESET(roundOutputs);

            RH_StrideArrayPtr prevCompClone = &state->m_stideDBuffer[state->m_stideDataFlipFlop ? 0 : 1].m_partiallyComputedOutputsClone;
            RH_STRIDEARRAY_COPY_ALL(roundOutputs, *prevCompClone);
            RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d CACHE HIT. Reusing last comp  nonce %x  %s\n", _n++, _hash, in_round, GetLastDWordLE(state->m_roundInput), _ArrayStr(*prevCompClone).c_str() );

            /*
            for (auto& e : *state->m_partiallyComputedOutputs)
            {
                RH_StridePtr srcItem = &e[0];
                RH_StridePtr item = RH_StrideArrayAllocOutput(state, (RH_STRIDE_GET_SIZE(srcItem)));
                RH_StrideArrayClose(state, item);
                memcpy(item, srcItem, RH_STRIDE_GET_SIZE(srcItem) + 64 + 8);
                RH_STRIDEARRAY_PUSHBACK(roundOutputs, item);
                RH_STRIDE_CHECK_INTEGRITY(item);
            }
            */

            //}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
            /*
            RH_ASSERT(RH_STRIDEARRAY_GET_SIZE(roundOutputs) == RH_STRIDEARRAY_GET_SIZE(*prevCompClone));
            for(int a=0; a < RH_STRIDEARRAY_GET_SIZE(*prevCompClone); a++)            
            {
                RHMINER_ASSERT(memcmp(RH_STRIDE_GET_DATA8(RH_STRIDEARRAY_GET(roundOutputs, a)),
                                      RH_STRIDE_GET_DATA8(RH_STRIDEARRAY_GET(*prevCompClone, a)), 
                                      RH_STRIDEARRAY_GET_SIZE(*prevCompClone)) == 0);
            }
            */
            //}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP

            

            state->m_partiallyComputedCount = 0;
            state->m_partiallyComputedRound = U32_Max;
            state->m_partiallyComputedNonceHeader = U32_Max;
            return 0; //not completed
        }
        //{{{ TEMP TEMP TEMP
        /*
        else
        {
            bool dup = GlobalMiningPreset::I().FindRoundNonce(GetLastDWordLE(state->m_roundInput), in_round);
            if (dup)
            {
                //PrintOut("***** Duplicate input found on step 3(round.start)  is %d, %s\n", dup, _ArrayBytes2Hex(state->m_data[in_round].parenAndNeighbortOutputs).c_str());
                RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d ***DUPLICATE*** round %d       nonce %x  %s\n", _n++, _hash, in_round,dup, GetLastDWordLE(state->m_roundInput), "" );
            }
            else
                RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Proceding with round          nonce %x  %s\n", _n++, _hash, in_round, GetLastDWordLE(state->m_roundInput), "" );
        }
        */
        //}}} TEMP TEMP TEMP
#endif

        //LRoundOutputs = LDisposables.AddObject(TList<TBytes>.Create()) as TList<TBytes>;
        //LGen = LDisposables.AddObject(TMersenne32.Create(0)) as TMersenne32;

        //NOTE in_round starts at RH2_MAX_N
        if (in_round == 1)
        {
            RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Round. Phase 1 SHA2-256             ~input %s\n", _n++, _hash, in_round, _Bytes2HexPart(state->m_roundInput, 8).c_str());
            //LRoundInput = FHashAlg[SHA2_256_IX].ComputeBytes(ABlockHeader).GetBytes;

/*
#ifdef RH2_ENABLE_PREFLIGHT_CACHE
    #ifndef RH2_ENABLE_PREFLIGHT_4X
            RandomHash_SHA2_256_Part2(state->m_roundInput, state->m_preflightData, state->m_workBytes);
    #endif
#else
            RandomHash_SHA2_256(state->m_roundInput, state->m_workBytes);
#endif
*/

#ifdef RH2_ENABLE_PREFLIGHT_CACHE
    #ifndef RH2_ENABLE_PREFLIGHT_4X
            if (state->m_headerSize == PascalHeaderSizeV5)
                RandomHash_SHA2_256_Part2(state->m_roundInput, state->m_preflightData, state->m_workBytes);
            else
                RandomHash_SHA2_256(state->m_roundInput, state->m_workBytes);
    #else
            //act like no preflight for non PASC
            if (state->m_headerSize != PascalHeaderSizeV5)
                RandomHash_SHA2_256(state->m_roundInput, state->m_workBytes);
    #endif
#else
            RandomHash_SHA2_256(state->m_roundInput, state->m_workBytes);
#endif

            //LSeed := GetLastDWordLE( LRoundInput );
            //LGen.Initialize(LSeed);
            LSeed = GetLastDWordLE(state->m_workBytes);

            merssen_twister_seed_fast_partial(LSeed, &rndGen,12);
            
            RH_FULLDEBUG_PRINT("Round.Phase1      itt = %d, round = %d, Seed = %X, input = %s\n", ++itt, in_round, LSeed, _Bytes2Hex(state->m_workBytes).c_str() );
            //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Round. Phase 1 result      seed %8x. roundOutput %s\n", _n++, _hash, in_round, LSeed, _StrideStr(state->m_workBytes).c_str());
                
            RH_ASSERT(RH_STRIDE_GET_SIZE(state->m_roundInput) == state->m_headerSize);

        #ifdef RH2_ENABLE_SHA256_PRERUN
            LSeed = merssen_twister_rand_fast_partial_12(&rndGen) % RH2_MAX_ALGO;
#if defined(RH2_ENABLE_MERSSEN_INTERLEAVE) && (!defined(RHMINER_NO_SSE4) || defined(RHMINER_NO_SIMD)) 
            rndGen.index -= 2;
#else
            rndGen.index--;
#endif
        #endif
        }
        else
        {
            //RandomHash_Phase_1_push(state, in_round); //push parenAndNeighbortOutputs  
            RH_StrideArrayStruct parenAndNeighbortOutputs;
            RH_FULLDEBUG_PRINT("Round.RoundA BEGI itt = %d, round = %d, header = %s\n", ++itt, in_round, _Bytes2Hex(state->m_roundInput).c_str());

#ifdef RH2_ENABLE_PREFLIGHT_CACHE
            //Precalc part2 by 4X m_precalcPart2 as current m_workbyte
            //state->m_workBytes = state->m_precalcPart2;
            if (state->m_headerSize == PascalHeaderSizeV5)
            {
                if (in_round - 1 == 1)
                    RandomHash_SHA2_256_Part2(state->m_roundInput, state->m_preflightData, state->m_workBytes);
            }
#endif

            //if CalculateRoundOutputs(ABlockHeader, round - 1, APCachedHash, LParentOutputs) == true
            if (CalculateRoundOutputs(state, in_round - 1, parenAndNeighbortOutputs) == true)
            {
                ///RandomHash_Phase_1_pop(state, in_round);

                // Previous round was the final round, so just return it's value
                //ARoundOutputs = LParentOutputs;  <-- parenAndNeighbortOutputs
                RH_STRIDEARRAY_RESET(roundOutputs); 
                RH_STRIDEARRAY_COPY_ALL(roundOutputs, parenAndNeighbortOutputs);

                RH_MAKE_STATS_MAX_PARENTROUNDOUTPUT_SIZE(roundOutputs, in_round+1);
                
                RH_FULLDEBUG_PRINT("Round.RoundA _END itt = %d, round = %d, ARoundOutputs = %s, EXIT = TRUE } \n", ++itt, in_round, _ArrayBytes2Hex(parenAndNeighbortOutputs, true).c_str());
                //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Round. Phase 2 result EXIT seed %8x %s. roundOutput %2d-%s\n", _n++, _hash, in_round, 0, _ArrayBytes2Hex(parenAndNeighbortOutputs, true).c_str());
                return true;
            }
            else
            {
                RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Round. Phase 2 result      roundOutput %s\n", _n++, _hash, in_round, _ArrayBytes2Hex(parenAndNeighbortOutputs, true).c_str());
            }

            //RandomHash_Phase_1_pop(state, in_round);
            RH_FULLDEBUG_PRINT("Round.RoundA END  itt = %d, round = %d, LParentOutputs = %s} \n", ++itt, in_round, _ArrayBytes2Hex(parenAndNeighbortOutputs, true).c_str());

            // Add parent round outputs to this round outputs
            //LSeed = GetLastDWordLE(LParentOutputs[High(LParentOutputs)]);
            //LGen.Initialize(LSeed);
            LSeed = GetLastDWordLE( RH_STRIDEARRAY_GET(parenAndNeighbortOutputs, RH_STRIDEARRAY_GET_SIZE(parenAndNeighbortOutputs)-1) );
            merssen_twister_seed_fast_partial(LSeed, &rndGen, 12);

            //Faster with precalc
            merssen_twister_rand_fast_partial_12(&rndGen);
#if defined(RH2_ENABLE_MERSSEN_INTERLEAVE) && (!defined(RHMINER_NO_SSE4) || defined(RHMINER_NO_SIMD)) 
            rndGen.index -= 2;
#else
            rndGen.index--;
#endif            
            //LRoundOutputs.AddRange(LParentOutputs);
            RH_STRIDEARRAY_COPY_ALL(roundOutputs, parenAndNeighbortOutputs); 
            RH_MAKE_STATS_MAX_ROUNDOUTPUT_SIZE(roundOutputs, in_round);
            RH_STRIDEARRAY_RESET(parenAndNeighbortOutputs);            

            // Add neighbouring nonce outputs to this round outputs
            //LNumNeighbours := (LGen.NextUInt32 MOD (MAX_J - MIN_J)) + MIN_J;
            
            U32 LNumNeighbours = (merssen_twister_rand_fast_partial_12(&rndGen) % (RH2_MAX_J - RH2_MIN_J)) + RH2_MIN_J;
            RH_FULLDEBUG_PRINT("Round.RoundA ADNO itt = %d, round = %d, seed = %X, LNumNeighbours = %d, LRoundOutputs = %s \n", ++itt, in_round, LSeed, LNumNeighbours, _ListBytes2Hex(roundOutputs, true).c_str());
            
#ifdef RH2_ENABLE_PREFLIGHT_4X
            if (state->m_headerSize == PascalHeaderSizeV5)
            {
                //Precm_precalcPart2 as current m_workbyte alc part2 by 4X 
                state->m_workBytes = state->m_precalcPart2;
                U32* MT = &rndGen.MT[rndGen.index];
                // Max 7
                if (LNumNeighbours >= 4)
                {
                    SetLastDWordLE(state->m_roundInput, *MT);
                    RandomHash_SHA2_256_Part2_SSE_4x(state->m_roundInput, state->m_preflightData,
                        MT[1], MT[2], MT[3],
                        state->m_workBytes + 0 * WorkBytesSize32,
                        state->m_workBytes + 1 * WorkBytesSize32,
                        state->m_workBytes + 2 * WorkBytesSize32,
                        state->m_workBytes + 3 * WorkBytesSize32);
                    state->m_workBytes += 4 * WorkBytesSize32;
                    MT += 4;
                }
                switch (LNumNeighbours % 4)
                {
                case 3:
                case 2:
                    SetLastDWordLE(state->m_roundInput, *MT);
                    RandomHash_SHA2_256_Part2_SSE_4x(state->m_roundInput, state->m_preflightData,
                        MT[1], MT[2], MT[3],
                        state->m_workBytes + 0 * WorkBytesSize32,
                        state->m_workBytes + 1 * WorkBytesSize32,
                        state->m_workBytes + 2 * WorkBytesSize32,
                        state->m_workBytes + 3 * WorkBytesSize32);
                    state->m_workBytes += 4 * WorkBytesSize32;
                    MT += 4;
                    break;

                case 1:
                    SetLastDWordLE(state->m_roundInput, *MT);
                    RandomHash_SHA2_256_Part2(state->m_roundInput, state->m_preflightData, state->m_workBytes);
                    state->m_workBytes += WorkBytesSize32;
                    MT++;
                }

                state->m_workBytes = state->m_precalcPart2 - WorkBytesSize32;
            }
#endif //RH2_ENABLE_PREFLIGHT_4X

            RH_MAKE_STATS_LNumNeighbours

            for (U32 i = 0; i < LNumNeighbours; i++)
            {
                LSeed = merssen_twister_rand_fast_partial_12(&rndGen);
                //LNeighbourNonceHeader = SetLastDWordLE(ABlockHeader, LGen.NextUInt32); // change nonce
                SetLastDWordLE(state->m_roundInput, LSeed); //WARNING: ===> PAS code do not alter state->m_roundInput
                RH_FULLDEBUG_PRINT("Round.RoundB #%d{{ itt = %d, round = %d, NextSeed = %X = NONCE \n", i, ++itt, in_round, LSeed);
                
                //RandomHash_Phase_2_push(state, in_round); //push parenAndNeighbortOutputs

#ifdef RH2_ENABLE_PREFLIGHT_4X
                if (state->m_headerSize == PascalHeaderSizeV5)
                {
                    //Precm_precalcPart2 as current m_workbyte alc part2 by 4X   
                    state->m_workBytes += WorkBytesSize32;
                    RH_ASSERT(state->m_workBytes < state->m_precalcPart2 + sizeof(state->m_precalcPart2) / 4);
                }
#endif

                //LNeighbourWasLastRound = CalculateRoundOutputs(LNeighbourNonceHeader, round - 1, nil, LNeighborOutputs);
                bool LNeighbourWasLastRound = CalculateRoundOutputs(state, in_round - 1, parenAndNeighbortOutputs);


                //LRoundOutputs.AddRange(LNeighborOutputs);
                RH_STRIDEARRAY_PUSHBACK_MANY(roundOutputs, parenAndNeighbortOutputs); 
                RH_MAKE_STATS_MAX_ROUNDOUTPUT_SIZE(roundOutputs, in_round);
                RH_FULLDEBUG_PRINT("Round.RoundB outv itt = %d, round = %d, RoundOutput = %s \n", ++itt, in_round, _ArrayBytes2Hex(roundOutputs, true).c_str());
                RH_FULLDEBUG_PRINT("Round.RoundB #%d}} itt = %d, round = %d, IsLast = %s,RoundOutputSize = %d, LNeighborOutputs = %s \n", i, ++itt, in_round, BoolToStr(LNeighbourWasLastRound), RH_STRIDEARRAY_GET_SIZE(roundOutputs), _ArrayBytes2Hex(parenAndNeighbortOutputs, true).c_str());
                //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Inner Round                %d\n", _n, _hash, in_round, in_round);
                
#ifdef RH2_ENABLE_CACHE
                // If neighbour was a fully evaluated nonce, cache it for re-use
                if (LNeighbourWasLastRound)
                {
                    //NOTE: Queue ALL computed hashes for testing against target in CPU Miner Threads
                    //FCache.AddFullyComputed(LNeighbourNonceHeader, round - 1, ComputeVeneerRound(LNeighborOutputs))

                    if (state->out_hashes->count < RandomHashResult::MaxHashes)
                    {
                        //{{{ TEMP TEMP TEMP
                        /*
                        RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d fullycomputed                 nonce %x  %s\n", _n++, _hash, in_round, LSeed, _ArrayStr(parenAndNeighbortOutputs).c_str() );
                        U32 dup = GlobalMiningPreset::I().AddRoundNonce(LSeed, in_round);
                        if (dup != U32_Max)
                        {
                            PrintOut("***** Duplicate input found on step 1(inner.veneer) is %d, %s\n", dup, _ArrayStr(parenAndNeighbortOutputs).c_str());
                            //RHMINER_ASSERT(dup == U32_Max);
                        }
                        */
                        //}}} TEMP TEMP TEMP

                        state->out_hashes->nonces[state->out_hashes->count] = LSeed;

                        ComputeVeneerRound(state, parenAndNeighbortOutputs, state->out_hashes->hashes[state->out_hashes->count]);
                        state->out_hashes->count++;
                    }
                    else
                    {
                        //RH_ASSERT(false);
                        PrintOut("Warning: Missed internal hash.\n");
                    }
                    //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d fullycomputed To.Validate  %s nonce %x  hash = %s\n", _n++, _hash, in_round, _Bytes2Hex((U8*)&tmpR1[0]).c_str(), LSeed, toHex(state->out_hashes->hashes[state->out_hashes->count-1], 32).c_str() );
                }
                else
                {
                    //{{{ TEMP TEMP TEMP
                    /*
                    RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d PARTIALLY computed            nonce %x  %s\n", _n++, _hash, in_round, LSeed, _ArrayStr(parenAndNeighbortOutputs).c_str() );
                    U32 dup = GlobalMiningPreset::I().AddRoundNonce(LSeed, in_round);
                    if (dup != U32_Max)
                    {
                        PrintOut("***** Duplicate input found on step 2(Partial)      is %d, %s\n", dup, _ArrayStr(parenAndNeighbortOutputs).c_str());
                        //RHMINER_ASSERT(dup == U32_Max);
                    }
                    */
                    //}}} TEMP TEMP TEMP


                    //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d PartiallyComp              %s nonce %x  roundOutput %2d-%s\n", _n++, _hash, in_round-1, _Bytes2Hex((U8*)&tmpR1[0]).c_str(), LSeed, RH_STRIDEARRAY_GET_SIZE(parenAndNeighbortOutputs),  _ArrayStr(parenAndNeighbortOutputs,0).c_str());

                    //Queue partially comp rounds so the cpuMiner resume that when this round is done
                    //FCache.AddPartiallyComputed(LNeighbourNonceHeader, round - 1, LNeighborOutputs);                    
                    U32 pround = in_round - 1;
                    if ((pround >= 3) /*&& (state->m_partiallyComputedCount <= 10) allways keep the last one */ )
                    {
                        /*slower !!! if ((state->m_partiallyComputedLocalCount &&
                            pround >= state->m_partiallyComputedRound &&
                            RH_STRIDEARRAY_GET_SIZE(parenAndNeighbortOutputs) > RH_STRIDEARRAY_GET_SIZE(state->m_pPartiallyComputedOutputsClone))
                            || state->m_partiallyComputedLocalCount == 0)*/
                        {
                            state->m_partiallyComputedLocalCount++;
                            state->m_partiallyComputedCount++;
                            state->m_partiallyComputedRound = pround;
                            state->m_partiallyComputedNonceHeader = LSeed;


                            //fullup clone
                            RH_STRIDEARRAY_RESET(*(state->m_pPartiallyComputedOutputsClone));
                            RH_STRIDEARRAY_COPY_ALL(*(state->m_pPartiallyComputedOutputsClone), parenAndNeighbortOutputs);
                            /*
                            //test with sperated buffer
                            if (!state->m_partiallyComputedOutputs)
                                state->m_partiallyComputedOutputs = new vector<bytes>;
                            state->m_partiallyComputedOutputs->resize(RH_STRIDEARRAY_GET_SIZE(parenAndNeighbortOutputs));
                            U32 itt = 0;
                            RH_STRIDEARRAY_FOR_EACH_BEGIN(parenAndNeighbortOutputs)
                            {
                                U32 ssize = RH_STRIDE_GET_SIZE(strideItrator) + 64 + 8;
                                //RH_ASSERT((ssize % RH_IDEAL_ALIGNMENT) == 0);
                                bytes& stride = (*state->m_partiallyComputedOutputs)[itt];
                                stride.reserve(ssize + RH_IDEAL_ALIGNMENT);
                                stride.resize(ssize);
                                memcpy(&stride[0], strideItrator, ssize);
                                itt++;
                            }
                            RH_STRIDEARRAY_FOR_EACH_END(0)
                            */

                            //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d PartiallyComp              cnt %d, round %d, nonce %x, %s, roundOutput %2d-%s\n", _n++, _hash, in_round-1, state->m_partiallyComputedCount, in_round-1, LSeed, _Bytes2Hex((U8*)&tmpR1[0]).c_str(), RH_STRIDEARRAY_GET_SIZE(parenAndNeighbortOutputs),  _ArrayStr(parenAndNeighbortOutputs,0).c_str());
                            RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d PartiallyComp CLONE        cnt %d, round %d, nonce %x, %s, roundOutput %2d-%s\n", _n++, _hash, in_round - 1, state->m_partiallyComputedCount, in_round - 1, LSeed, "", RH_STRIDEARRAY_GET_SIZE(*state->m_pPartiallyComputedOutputsClone), _ArrayStr(*state->m_pPartiallyComputedOutputsClone).c_str());
                        }
                        /*else
                        {
                            RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d PartiallyComp CLONE SKIPP  cnt %d, round %d, nonce %x, %s, roundOutput %2d-%s\n", _n++, _hash, in_round - 1, state->m_partiallyComputedCount, in_round - 1, LSeed, "", RH_STRIDEARRAY_GET_SIZE(*state->m_pPartiallyComputedOutputsClone), _ArrayStr(*state->m_pPartiallyComputedOutputsClone).c_str());
                        }*/
                    }
                }
#endif//#ifdef RH2_ENABLE_CACHE

                RH_STRIDEARRAY_RESET(parenAndNeighbortOutputs);
                //RandomHash_Phase_2_pop(state, in_round);
            }
            
            LSeed = merssen_twister_rand_fast_partial_12(&rndGen);
            // Compress the parent/neighbouring outputs to form this rounds input
            //LRoundInput = Compress(LRoundOutputs.ToArray, LGen.NextUInt32);
            RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Round. Compress            seed %8x. Input %s\n", _n++, _hash, in_round, 0, _ArrayStr(roundOutputs).c_str());
            RandomHash_Compress(state, roundOutputs, state->m_workBytes, LSeed);
            RH_FULLDEBUG_PRINT("Round.RoundA  Zip itt = %d, round = %d, NextSeed = %X, LRoundInput = %s, LRoundOutputs = %s\n", ++itt, in_round, LSeed, _Bytes2Hex(state->m_workBytes, true).c_str(), _ListBytes2Hex(roundOutputs, true).c_str());
            //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Round. Compress            seed %8x. Output %s\n", _n++, _hash, in_round, LSeed, _StrideStr(state->m_workBytes).c_str());
            RH_ASSERT(RH_STRIDE_GET_SIZE(state->m_workBytes) <= 100);
        }

        // Select a random hash function and hash the input to find the output
        //LHashFunc = FHashAlg[LGen.NextUInt32 mod NUM_HASH_ALGO];
        U32 rndHash = (merssen_twister_rand_fast_partial_12(&rndGen) % RH2_MAX_ALGO);

        // input selection
        RH_StridePtr input = state->m_workBytes;
        RH_ASSERT(RH_STRIDE_GET_SIZE(state->m_workBytes) < WorkBytesSize);

        //allocate expanded output
        RH_StridePtr output = RH_StrideArrayAllocOutput(state, c_AlgoSize[rndHash]);

        RH_MAKE_STATS_ALGO_USED;
        RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Round. Hashing input       Algo %2x. ~input %s\n", _n++, _hash, in_round, rndHash, _Bytes2HexPart(state->m_workBytes, 8).c_str()); 
        switch(rndHash)
        {
            case RH2_Blake2B_160:       RandomHash_blake2b(input, output, 160); break; 
            case RH2_Blake2B_256:       RandomHash_blake2b(input, output, 256); break;
            case RH2_Blake2B_512:       RandomHash_blake2b(input, output, 512); break;
            case RH2_Blake2B_384:       RandomHash_blake2b(input, output, 384); break;
            case RH2_Blake2S_128:       RandomHash_blake2s(input, output, 128); break;
            case RH2_Blake2S_160:       RandomHash_blake2s(input, output, 160); break;
            case RH2_Blake2S_224:       RandomHash_blake2s(input, output, 224); break;
            case RH2_Blake2S_256:       RandomHash_blake2s(input, output, 256); break;
            case RH2_Gost              :RandomHash_Ghost(input, output); break;
            case RH2_GOST3411_2012_256 :RandomHash_Ghost3411(input, output, 256); break;
            case RH2_GOST3411_2012_512 :RandomHash_Ghost3411(input, output, 512); break;
            case RH2_Grindahl256       :RandomHash_Grindahl256(input, output); break;
            case RH2_Grindahl512       :RandomHash_Grindahl512(input, output); break;
            case RH2_HAS160            :RandomHash_HAS160(input, output); break;
            case RH2_Haval_3_128       :RandomHash_Haval3 (input, output, 128); break;
            case RH2_Haval_3_160       :RandomHash_Haval3 (input, output, 160); break;
            case RH2_Haval_3_192       :RandomHash_Haval3 (input, output, 192); break;
            case RH2_Haval_3_224       :RandomHash_Haval3 (input, output, 224); break;
            case RH2_Haval_3_256       :RandomHash_Haval3 (input, output, 256); break;
            case RH2_Haval_4_128       :RandomHash_Haval4 (input, output, 128); break;
            case RH2_Haval_4_160       :RandomHash_Haval4 (input, output, 160); break;
            case RH2_Haval_4_192       :RandomHash_Haval4 (input, output, 192); break;
            case RH2_Haval_4_224       :RandomHash_Haval4 (input, output, 224); break;
            case RH2_Haval_4_256       :RandomHash_Haval4 (input, output, 256); break;
            case RH2_Haval_5_128       :RandomHash_Haval_5_256 (input, output, 128); break;
            case RH2_Haval_5_160       :RandomHash_Haval_5_256 (input, output, 160); break;
            case RH2_Haval_5_192       :RandomHash_Haval_5_256 (input, output, 192); break;
            case RH2_Haval_5_224       :RandomHash_Haval_5_256 (input, output, 224); break;
            case RH2_Haval_5_256       :RandomHash_Haval_5_256 (input, output, 256); break;
            case RH2_Keccak_224        :_RandomHash_SHA3_512  (input, output, 224/8, false); break;
            case RH2_Keccak_256        :_RandomHash_SHA3_512  (input, output, 256/8, false); break;
            case RH2_Keccak_288        :_RandomHash_SHA3_512  (input, output, 288/8, false); break;
            case RH2_Keccak_384        :_RandomHash_SHA3_512  (input, output, 384/8, false); break;
            case RH2_Keccak_512        :_RandomHash_SHA3_512  (input, output, 512/8, false); break;
            case RH2_MD2               :RandomHash_MD2         (input, output); break;
            case RH2_MD5               :RandomHash_MD5         (input, output); break;
            case RH2_MD4               :RandomHash_MD4         (input, output); break;
            case RH2_Panama            :RandomHash_Panama      (input, output); break;
            case RH2_RadioGatun32      :RandomHash_RadioGatun32(input, output); break;
            case RH2_RIPEMD            :RandomHash_RIPEMD      (input, output); break;
            case RH2_RIPEMD128         :RandomHash_RIPEMD128   (input, output); break;
            case RH2_RIPEMD160         :RandomHash_RIPEMD160   (input, output); break;
            case RH2_RIPEMD256         :RandomHash_RIPEMD256   (input, output); break;
            case RH2_RIPEMD320         :RandomHash_RIPEMD320   (input, output); break;
            case RH2_SHA0              :RandomHash_SHA0        (input, output); break;
            case RH2_SHA1              :RandomHash_SHA1        (input, output); break;
            case RH2_SHA2_224          :RandomHash_SHA2_256    (input, output, true); break;                 
            case RH2_SHA2_256          :RandomHash_SHA2_256    (input, output, false); break;
            case RH2_SHA2_384          :RandomHash_SHA2_512    (input, output, SHA2_512_MODE_384); break;
            case RH2_SHA2_512          :RandomHash_SHA2_512    (input, output, SHA2_512_MODE_512); break;
            case RH2_SHA2_512_224      :RandomHash_SHA2_512    (input, output, SHA2_512_MODE_512_224); break;
            case RH2_SHA2_512_256      :RandomHash_SHA2_512    (input, output, SHA2_512_MODE_512_256); break;
            case RH2_SHA3_224          :RandomHash_SHA3_224    (input, output); break;
            case RH2_SHA3_256          :RandomHash_SHA3_256    (input, output); break;
            case RH2_SHA3_384          :RandomHash_SHA3_384    (input, output); break;
            case RH2_SHA3_512          :RandomHash_SHA3_512    (input, output); break;
            case RH2_Snefru_8_128      :RandomHash_Snefru_8_256(input, output, 16); break;
            case RH2_Snefru_8_256      :RandomHash_Snefru_8_256(input, output, 32); break;
            case RH2_Tiger_3_128       :RandomHash_Tiger(input, output, 3, 128, false); break;
            case RH2_Tiger_3_160       :RandomHash_Tiger(input, output, 3, 160, false); break;
            case RH2_Tiger_3_192       :RandomHash_Tiger(input, output, 3, 192, false); break;
            case RH2_Tiger_4_128       :RandomHash_Tiger(input, output, 4, 128, false); break;
            case RH2_Tiger_4_160       :RandomHash_Tiger(input, output, 4, 160, false); break;
            case RH2_Tiger_4_192       :RandomHash_Tiger(input, output, 4, 192, false); break;
            case RH2_Tiger_5_128       :RandomHash_Tiger(input, output, 5, 128, false); break;
            case RH2_Tiger_5_160       :RandomHash_Tiger(input, output, 5, 160, false); break;
            case RH2_Tiger_5_192       :RandomHash_Tiger(input, output, 5, 192, false); break;
            case RH2_Tiger2_3_128      :RandomHash_Tiger(input, output, 3, 128, true); break;
            case RH2_Tiger2_3_160      :RandomHash_Tiger(input, output, 3, 160, true); break;
            case RH2_Tiger2_3_192      :RandomHash_Tiger(input, output, 3, 192, true); break;
            case RH2_Tiger2_4_128      :RandomHash_Tiger(input, output, 4, 128, true); break;
            case RH2_Tiger2_4_160      :RandomHash_Tiger(input, output, 4, 160, true); break;
            case RH2_Tiger2_4_192      :RandomHash_Tiger(input, output, 4, 192, true); break;
            case RH2_Tiger2_5_128      :RandomHash_Tiger(input, output, 5, 128, true); break;
            case RH2_Tiger2_5_160      :RandomHash_Tiger(input, output, 5, 160, true); break;
            case RH2_Tiger2_5_192      :RandomHash_Tiger(input, output, 5, 192, true); break;
            case RH2_WhirlPool         :RandomHash_WhirlPool(input, output); break;
        }

        RH_MAKE_STATS_MAX_ALGO_SIZE

        RH_FULLDEBUG_PRINT("Round.RoundA  HAH itt = %d, round = %d, NextSeed = %X, LOutput = %s \n", ++itt, in_round, rndHash, _Bytes2Hex(output, true).c_str());
        RH_ASSERT(RH_STRIDE_GET_SIZE(state->m_workBytes) <= 100);
        RH_ASSERT(RH_STRIDE_GET_SIZE(output) == c_AlgoSize[rndHash]);
        RH_STRIDE_INIT_INTEGRITY(output);


        LSeed = merssen_twister_rand_fast_partial_12(&rndGen);
        // Memory-expand the output, add to output list and return output list
        //LOutput = Expand(LOutput, RH2_MAX_N - round, LGen.NextUInt32);
        RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Round. Expand              seed %8x. input %s \n", _n++, _hash, in_round, 0, _StrideStr(output).c_str());
        RandomHash_Expand(state, 
                               output, 
                               LSeed, 
                               in_round, 
                               RH2_MAX_N - in_round);
        RH_FULLDEBUG_PRINT("Round.RoundA  EXP itt = %d, round = %d, NextSeed = %X, ExpandFac = %d LOutput = %s \n", ++itt, in_round, LSeed, (RH2_MAX_N - in_round), _Bytes2Hex(output, true).c_str());
        //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Round. Expand              seed %8x. Output %s\n", _n++, _hash, in_round, LSeed, _StrideStr(output).c_str());

        RH_STRIDEARRAY_PUSHBACK(roundOutputs, output);

        bool finalResult = (in_round == RH2_MAX_N) || ((in_round >= RH2_MIN_N) && ((GetLastDWordLE(output) % RH2_MAX_N) == 0));
        if (in_round < 4)
        {
            //LRoundOutputs.Add(LOutput);
            //ARoundOutputs = LRoundOutputs.ToArray;
            //RH_STRIDEARRAY_RESET(state->m_data[in_round].io_results);
            //RH_STRIDEARRAY_COPY_ALL(state->m_data[in_round].io_results, roundOutputs);

            RH_MAKE_STATS_MAX_PARENTROUNDOUTPUT_SIZE(roundOutputs, in_round+1);

            RH_FULLDEBUG_PRINT("Round.RoundA  Res itt = %d, round = %d, ARoundOutputs = %d, LastDWord = %X, EXIT = %s \n",
                ++itt, in_round, RH_STRIDEARRAY_GET_SIZE(roundOutputs),
                GetLastDWordLE(output),
                BoolToStr(finalResult));

            // Determine if final round
            //RH_STRIDEARRAY_RESET(roundOutputs);
        }
        else
        {
            RH_FULLDEBUG_PRINT("Round.RoundA  Res itt = %d, round = %d, ARoundOutputs = %d, LastDWord = %X, EXIT = %s \n",
                ++itt, in_round, RH_STRIDEARRAY_GET_SIZE(roundOutputs),
                GetLastDWordLE(output),
                BoolToStr(finalResult));
        }
        return finalResult;
    }


    void RandomHash_Init(RandomHash_State* allStates, U32 startNonce)
    {
        CUDA_DECLARE_STATE();

        RH_FULLDEBUG_PRINT("Input %s, itt = %d\n", ToUpper(toHex(state->m_header, state->m_headerSize, false)).c_str(), itt);
        
        RH_ASSERT(state->m_headerSize > 0);

        RandomHash_Initialize(state);

        if (state->m_isNewHeader)
        {
            state->m_isNewHeader = false;
            RH_STRIDE_SET_SIZE(state->m_roundInput, state->m_headerSize);
            memcpy(RH_STRIDE_GET_DATA(state->m_roundInput), &state->m_header[0], state->m_headerSize); 
            RH_STRIDE_INIT_INTEGRITY(state->m_roundInput);

#ifdef RH2_ENABLE_PREFLIGHT_CACHE
            {
                if (state->m_isCalculatePreflight)
                {
                    state->m_isCalculatePreflight = 0;
                    RandomHash_SHA2_256_Part1(RH_STRIDE_GET_DATA(state->m_roundInput), RH_STRIDE_GET_SIZE(state->m_roundInput), state->m_preflightData);
                }
            }
#endif
        }
        else
        {
            //resume on last partially computed round. Else continue with same header but with provided nonce/gid
            if (state->m_partiallyComputedNonceHeader && state->m_partiallyComputedNonceHeader != U32_Max)
            {
                //state->m_partiallyComputedCount = 0;
                startNonce = state->m_partiallyComputedNonceHeader;
            }
        }
    
        //reset computed list
        state->out_hashes->count = 1;
        state->out_hashes->nonces[0] = startNonce;


        SetLastDWordLE(state->m_roundInput, startNonce);
    }
      

    void RandomHash_Search(RandomHash_State* in_state, RandomHashResult& out_hashes, U32 startNonce)
    {
//{{{ TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
#ifdef RH_FULLDEBUG_CPU
    _hash++;
#endif
//}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP

        //Reset cache if no partial-comp is found. If not, we're looping on the last part-count for ever !
        in_state->m_partiallyComputedLocalCount = 0;

        in_state->out_hashes = &out_hashes;
        RandomHash_Init(in_state, startNonce);
        
        RH_StrideArrayStruct roundOutputs;
        CalculateRoundOutputs(in_state, RH2_MAX_N, roundOutputs);

        //U8 tempStride[32+RH_IDEAL_ALIGNMENT];
        ComputeVeneerRound(in_state, roundOutputs, out_hashes.hashes[0]);
        //memcpy(out_hash, tempStride, 32);

        
        //reset if no partial found
        if(in_state->m_partiallyComputedLocalCount == 0)
        {            
            in_state->m_partiallyComputedCount = 0; 
            in_state->m_partiallyComputedRound = U32_Max;
            in_state->m_partiallyComputedNonceHeader = U32_Max;
            RH_STRIDEARRAY_RESET(*in_state->m_pPartiallyComputedOutputsClone);
            //RH_FULLDEBUG_CPU_PRINTOUT("#%2d %d|%d Post round                RESETING part-comp cache, using old-part-calc nonce %x\n", _n++, _hash, 99,in_state->m_partiallyComputedNonceHeader );
        }

        RH_ASSERT(out_hashes.count <= RandomHashResult::MaxHashes);
        RH_MAKE_STATS_MAX_STRIDE_SIZE
    }

//{{{ TEMP TEMP TEMP
#if defined(_DEBUG) && 0
    #include "RandomHash_DEV_UNITTEST.h"
#endif
//}}} TEMP TEMP TEMP