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

#ifndef RANDOM_HASH_OPTIMIZED_H
#define RANDOM_HASH_OPTIMIZED_H

#include "MinersLib/RandomHash/RandomHash_core.h"


using namespace std;

struct RandomHashResult
{
    typedef U32 Hash32[32/4];
    const static U32 MaxHashes = 128;

    Hash32  hashes[MaxHashes];
    U32     nonces[MaxHashes];
    U32     count;
    RandomHashResult() :count(0) {}

};

/*
struct RH_ALIGN(128) RH_RoundData
{
    mersenne_twister_state   rndGen; 
    RH_StrideArrayPtr        roundOutputs;
    RH_StrideArrayPtr        parenAndNeighbortOutputs;
    RH_StrideArrayPtr        io_results;
    RH_StrideArrayPtr        backup_io_results;
};
*/

struct RH_ALIGN(128) RandomHash_StrideDBuffer
{
    RH_StrideArrayStruct     m_partiallyComputedOutputsClone;
    RH_StridePtr             m_stridesInstances;
};

static const U32 WorkBytesSize = 256;
static const U32 WorkBytesSize32 = 256/4;
struct RH_ALIGN(128) RandomHash_State
{
    U8                                    m_header[MaxMiningHeaderSize];

    RH_ALIGN(64) U32                      m_precalcPart2[8*WorkBytesSize32];
#ifdef RHMINER_DEBUG_STRIDE_INTEGRITY_CHECK
    RH_ALIGN(64) U32                      m_roundInput[8 + (RH_IDEAL_ALIGNMENT + 512) / 4]; 
#else
    RH_ALIGN(64) U32                      m_roundInput[(RH_IDEAL_ALIGNMENT + 512) / 4];
#endif

    RH_ALIGN(64) SHA2_256_SavedState      m_preflightData;
    RH_ALIGN(64) RandomHash_StrideDBuffer m_stideDBuffer[2];
    //RH_ALIGN(64) RH_RoundData             m_data[RH2_MAX_N+1];
    U32                                   m_partiallyComputedCount;
    U32                                   m_partiallyComputedLocalCount;
    U32                                   m_partiallyComputedRound;
    U32                                   m_partiallyComputedNonceHeader;
    //vector<bytes>*                      m_partiallyComputedOutputs;
    RH_StrideArrayPtr                     m_pPartiallyComputedOutputsClone;
    RH_StridePtr                          m_pStridesInstances;
    U32                                   m_stideDataFlipFlop;
    U32                                   m_stridesAllocIndex; //current buffer round
    U32                                   m_strideID; //DEBUG only
    U32                                   m_isCalculatePreflight;
    U32*                                  m_workBytes;
    U32                                   m_isNewHeader;
    U32                                   m_headerSize;
    RandomHashResult*                     out_hashes; //holds FullyComputedRounds 
};


//External API functions
extern void RandomHash_CreateMany(RandomHash_State** outPtr, U32 count);
extern void RandomHash_DestroyMany(RandomHash_State* stateArray, U32 count);
extern void RandomHash_Create(RandomHash_State* state);
extern void RandomHash_Destroy(RandomHash_State* state);


//External alloc func
//extern void RandomHash_RoundDataAlloc(RH_RoundData* rd, int round);
//extern void RandomHash_RoundDataUnInit(RH_RoundData* rd, int round);
extern void RandomHash_SetHeader(RandomHash_State* state, U8* sourceHeader, U32 headerSize, U32 nonce2);

extern void RandomHash_Search(RandomHash_State* state, RandomHashResult& out_hash, U32 startNonce);



extern void RandomHash_Free(void* ptr);
extern void RandomHash_Alloc(void** out_ptr, size_t size);

#ifdef RH_SCREEN_SAVER_MODE
extern void ScreensaverFeed(U32 nonce);
#endif


#endif //RANDOM_HASH_OPTIMIZED_H


