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

#pragma once
#include "MinersLib/RandomHash/RandomHash_def.h"
#include "corelib/rh_endian.h"

#ifdef RHMINER_PLATFORM_CPU
extern int  g_memoryBoostLevel;

#define _RH_LD_LINE_128(i) r##i = RH_MM_LOAD128((__m128i *)(source)); 
#define RH_LD_LINE_128(i) r##i = RH_MM_LOAD128((__m128i *)(source)); source += sizeof(__m128i);

#endif //CPU


inline void RH_INPLACE_MEMCPY_128(U8* pDst, U8* pSrc, size_t byteCount)
{
    RH_ASSERT(((size_t)pDst % 32) == 0);
    RH_ASSERT(((size_t)pSrc % 32) == 0);
#ifndef RH2_AVOID_MMX_FOR_INPLACE_MEMCPY
    if (g_memoryBoostLevel == 1)
    {
        S32 n = RHMINER_CEIL(byteCount, sizeof(__m128i));
        __m128i r0;        
        __m128i* src = (__m128i *)(pSrc);
        __m128i* end = src + n;
        __m128i* dst = (__m128i *)(pDst);
        while (src < end)
        {
            r0 = RH_MM_LOAD128(src++);
            RH_MM_STORE128(dst++, r0);
        }
       //RH_MM_BARRIER();
    }
    else
#endif
        memcpy(pDst, pSrc, byteCount);
}



inline void RH_STRIDE_MEMCPY_ALIGNED_SIZE128(U8 *pDst, U8 *pSrc, size_t byteCount)
{
    RH_ASSERT(( (size_t)pDst % 8) == 0);
    RH_ASSERT(( (size_t)pSrc % 8) == 0);
    RH_ASSERT(( (size_t)pDst % 32) == 0);
    RH_ASSERT(( (size_t)pSrc % 32) == 0);

    RH_INPLACE_MEMCPY_128(pDst, pSrc, byteCount);
}



#ifdef RH2_STRIDE_PACKMODE  

    void inline RH_STRIDEARRAY_PUSHBACK(RH_StrideArrayStruct& strideArrayVar, RH_StridePtr stride)
    {
        U32 aidx = RH_STRIDEARRAY_GET_SIZE(strideArrayVar)++;
        RH_ASSERT(aidx < RH_STRIDEARRAY_GET_MAXSIZE(strideArrayVar));
        strideArrayVar.strides[aidx] = stride;

        U32 ssize = RH_STRIDE_GET_SIZE(stride);
        memcpy(&strideArrayVar.packedData[strideArrayVar.packedSize], RH_STRIDE_GET_DATA8(stride), ssize);        
        strideArrayVar.packedSizes[aidx] = ssize;    
        strideArrayVar.packedIdx[aidx] = strideArrayVar.packedSize;
        strideArrayVar.packedSize += ssize;

//TEMP TEMP TEMP 
#ifdef _DEBUG
    for(int i=0; i < RH_STRIDEARRAY_GET_SIZE(strideArrayVar); i++)
        RH_ASSERT(strideArrayVar.packedSizes[i] != 0);
    RH_ASSERT(ssize != 0);
#endif

    }
#else
    #define RH_STRIDEARRAY_PUSHBACK(strideArrayVar, stride)                     \
    {                                                                           \
        U32 _as = RH_STRIDEARRAY_GET_SIZE(strideArrayVar)++;                    \
        RH_ASSERT(_as < RH_STRIDEARRAY_GET_MAXSIZE(strideArrayVar));            \
        (strideArrayVar).strides[_as] = (stride);     \
    }
#endif


inline void RH_STRIDEARRAY_PUSHBACK_MANY(RH_StrideArrayStruct& strideArrayVar, RH_StrideArrayStruct& strideArrayVarSrc)
{
    U32 ssize = RH_STRIDEARRAY_GET_SIZE(strideArrayVarSrc);
    U32 dsize = RH_STRIDEARRAY_GET_SIZE(strideArrayVar);

#ifdef RH2_STRIDE_PACKMODE

    memcpy(&strideArrayVar.strides[dsize], &strideArrayVarSrc.strides[0], sizeof(strideArrayVarSrc.strides[0]) * ssize);
    RH_STRIDEARRAY_GET_SIZE(strideArrayVar) += ssize;

    //pack stuff
    RH_ASSERT(strideArrayVar.packedSize + strideArrayVarSrc.packedSize < RH_StrideArrayStruct::MaxPackedData);
    memcpy(&strideArrayVar.packedData[strideArrayVar.packedSize], strideArrayVarSrc.packedData, strideArrayVarSrc.packedSize);
    memcpy(&strideArrayVar.packedSizes[dsize], &strideArrayVarSrc.packedSizes[0], sizeof(strideArrayVarSrc.packedSizes[0]) * ssize);
    
    U32* newIdx = &strideArrayVar.packedIdx[dsize];
    memcpy(newIdx, &strideArrayVar.packedIdx[0], sizeof(strideArrayVarSrc.packedIdx[0]) * ssize);        
    for(int i=0; i < ssize; i++)
        newIdx[i] += strideArrayVar.packedSize;

    strideArrayVar.packedSize += strideArrayVarSrc.packedSize;
    
//TEMP TEMP TEMP 
#ifdef _DEBUG
    for(int i=0; i < RH_STRIDEARRAY_GET_SIZE(strideArrayVar); i++)
        RH_ASSERT(strideArrayVar.packedSizes[i] != 0);
#endif


#else
    RH_ASSERT(&strideArrayVar != &strideArrayVarSrc);
    RH_ASSERT(RH_STRIDEARRAY_GET_SIZE(strideArrayVar) + RH_STRIDEARRAY_GET_SIZE(strideArrayVarSrc) < RH_STRIDEARRAY_GET_MAXSIZE(strideArrayVar));
  #ifdef RH2_STRIDE_USE_MEMCPY
    memcpy(&strideArrayVar.strides[dsize], &strideArrayVarSrc.strides[0], sizeof(strideArrayVarSrc.strides[0]) * ssize);
    RH_STRIDEARRAY_GET_SIZE(strideArrayVar) += ssize;
  #else
    RH_STRIDEARRAY_FOR_EACH_BEGIN(strideArrayVarSrc)
    {                                                                           
        RH_STRIDEARRAY_PUSHBACK(strideArrayVar, strideItrator);  //TODO optimiz : one shot with memcpy
        RH_STRIDE_CHECK_INTEGRITY(strideItrator);
    }                                                                           
    RH_STRIDEARRAY_FOR_EACH_END(strideArrayVarSrc)
  #endif
#endif
} 

//according to this, packing all stride would make Expand faster for BIG strides
// Sizes tot = 47592 max  = 256    [cnt = 230] : 216, 216, 224, 240, 256, 144, 224, 256, 224, 256, 256, 168, 256, 208, 224, 256, 224, 160, 256, 216, 208, 212, 216, 216, 160, 212, 224, 220, 208, 208, 144, 208, 216, 224, 212, 224, 224, 176, 208, 212, 240, 240, 256, 212, 232, 212, 156, 80, 220, 216, 224, 208, 208, 208, 240, 216, 156, 208, 224, 256, 228, 228, 224, 220, 208, 176, 208, 208, 224, 208, 228, 208, 192, 208, 212, 152, 212, 216, 208, 240, 212, 212, 212, 144, 224, 208, 256, 220, 216, 212, 148, 220, 216, 216, 212, 144, 92, 212, 212, 208, 256, 240, 224, 144, 216, 256, 256, 156, 256, 216, 144, 256, 208, 216, 216, 148, 92, 240, 208, 152, 208, 208, 212, 160, 212, 208, 212, 256, 144, 208, 208, 216, 216, 160, 212, 224, 208, 152, 216, 208, 144, 84, 212, 212, 224, 216, 224, 224, 208, 208, 160, 208, 256, 212, 224, 216, 256, 144, 212, 212, 240, 212, 168, 216, 224, 220, 256, 220, 216, 256, 256, 152, 224, 216, 220, 224, 212, 216, 240, 176, 208, 216, 220, 212, 148, 80, 224, 	
// Dist  avg = 79880 max  = 323584 [cnt = 230] : 323584, 16576, 297856, 280160, 44224, 10656, 16544, 15296, 312928, 3776, 301664, 7264, 6272, 43872, 22464, 2784, 23776, 39328, 25184, 16800, 288, 294368, 295456, 12544, 16960, 16352, 1792, 3904, 13088, 7872, 3584, 13248, 33600, 323456, 286016, 47968, 18016, 309600, 301536, 19520, 290880, 314656, 7456, 4128, 20352, 1568, 298432, 295200, 294624, 291584, 3424, 8512, 38112, 10208, 544, 310400, 301728, 20864, 13728, 7616, 21632, 317696, 291040, 13760, 275104, 313792, 17216, 1120, 3232, 14464, 16128, 1088, 16832, 28256, 8320, 19168, 512, 10880, 11360, 9504, 11072, 10720, 5984, 11360, 2848, 23904, 320, 39776, 15296, 1120, 10016, 13344, 19680, 9952, 2912, 29504, 10976, 1216, 8640, 4128, 

void RH_STRIDEARRAY_COPY_ALL(RH_StrideArrayStruct& strideArrayVar, RH_StrideArrayStruct& strideArrayVarSrc)
{
    U32 srcCnt = RH_STRIDEARRAY_GET_SIZE(strideArrayVarSrc);
    RH_STRIDEARRAY_SET_SIZE(strideArrayVar, srcCnt);
    RH_ASSERT(&strideArrayVar != &strideArrayVarSrc);

#ifdef RH2_STRIDE_PACKMODE

    memcpy(&strideArrayVar.strides[0], &strideArrayVarSrc.strides[0], sizeof(strideArrayVarSrc.strides[0]) * srcCnt);

    //pack stuff
    U32 dstSize = RH_STRIDEARRAY_GET_SIZE(strideArrayVar);
    memcpy(&strideArrayVar.packedData[0], strideArrayVarSrc.packedData, strideArrayVarSrc.packedSize);
    memcpy(&strideArrayVar.packedSizes[0], &strideArrayVarSrc.packedSizes[0], sizeof(strideArrayVarSrc.packedSizes[0]) * srcCnt);
    
    U32* newIdx = &strideArrayVar.packedIdx[0];
    memcpy(newIdx, &strideArrayVar.packedIdx[0], sizeof(strideArrayVarSrc.packedIdx[0]) * srcCnt);        

    strideArrayVar.packedSize = strideArrayVarSrc.packedSize;

//TEMP TEMP TEMP 
#ifdef _DEBUG
    for(int i=0; i < RH_STRIDEARRAY_GET_SIZE(strideArrayVar); i++)
        RH_ASSERT(strideArrayVar.packedSizes[i] != 0);
    RH_ASSERT(srcCnt != 0);
#endif

#else
  #ifdef RH2_STRIDE_USE_MEMCPY
    memcpy(&(strideArrayVar).strides[0], &(strideArrayVarSrc).strides[0], sizeof((strideArrayVarSrc).strides[0]) * srcCnt);
  #else
    U32 i = 0;
    while(i < srcCnt)
    {
        (strideArrayVar).strides[i] = (strideArrayVarSrc).strides[i];
        RH_STRIDE_CHECK_INTEGRITY((strideArrayVar).strides[i]);
        i++;
    }
  #endif
#endif
}



#define RH_Accum_8() \
    U32 acc8_idx = 0;  \
    U64 acc8_buf = 0;  \

#define RH_Accum_8_Reset(p) \
    acc8_idx = 0;  \
    acc8_buf = 0;  \

#define RH_Accum_8_Add(chunk8, acc8_ptr)                    \
{                                                           \
    acc8_buf &= ~(0xFFLLU << (U64)(acc8_idx << 3));         \
    acc8_buf |= ((U64)(chunk8) << (U64)(acc8_idx << 3));    \
    acc8_idx++;                                             \
    if (acc8_idx == 8)                                      \
    {                                                       \
        *((U64*)acc8_ptr) = acc8_buf;                       \
        acc8_ptr += 8;                                      \
        acc8_idx = 0;                                       \
        acc8_buf = 0;                                       \
    }                                                       \
}


#define RH_Accum_8_Finish(acc8_ptr)      \
{                                       \
    RH_ASSERT(acc8_idx != 8);           \
    *((U64*)acc8_ptr) = acc8_buf;       \
}

//RH2: avg 64, MAX 128
#ifdef RH2_ENABLE_TRANSFO0_MMX128
inline void Transfo0_2(U8* nextChunk, U32 size, U8* source)
{
    RH_ASSERT(size <= 128);
    U32 rndState;
    rndState = (*(U32*)(source+size-4));

    if (!rndState)
        rndState = 1;

    __m128i r0,r1,r2,r3,r4,r5,r6,r7;
    switch(size/16)
    {
        case 8:
        case 7: r0 = RH_MM_LOAD128 ((__m128i *)(source+0*sizeof(__m128i)));
                r1 = RH_MM_LOAD128 ((__m128i *)(source+1*sizeof(__m128i)));
                r2 = RH_MM_LOAD128 ((__m128i *)(source+2*sizeof(__m128i)));
                r3 = RH_MM_LOAD128 ((__m128i *)(source+3*sizeof(__m128i)));
                r4 = RH_MM_LOAD128 ((__m128i *)(source+4*sizeof(__m128i)));
                r5 = RH_MM_LOAD128 ((__m128i *)(source+5*sizeof(__m128i)));
                r6 = RH_MM_LOAD128 ((__m128i *)(source+6*sizeof(__m128i)));
                r7 = RH_MM_LOAD128 ((__m128i *)(source+7*sizeof(__m128i)));
                break;
        case 6: r0 = RH_MM_LOAD128 ((__m128i *)(source+0*sizeof(__m128i)));
                r1 = RH_MM_LOAD128 ((__m128i *)(source+1*sizeof(__m128i)));
                r2 = RH_MM_LOAD128 ((__m128i *)(source+2*sizeof(__m128i)));
                r3 = RH_MM_LOAD128 ((__m128i *)(source+3*sizeof(__m128i)));
                r4 = RH_MM_LOAD128 ((__m128i *)(source+4*sizeof(__m128i)));
                r5 = RH_MM_LOAD128 ((__m128i *)(source+5*sizeof(__m128i)));
                r6 = RH_MM_LOAD128 ((__m128i *)(source+6*sizeof(__m128i)));
                break;
        case 5: r0 = RH_MM_LOAD128 ((__m128i *)(source+0*sizeof(__m128i)));
                r1 = RH_MM_LOAD128 ((__m128i *)(source+1*sizeof(__m128i)));
                r2 = RH_MM_LOAD128 ((__m128i *)(source+2*sizeof(__m128i)));
                r3 = RH_MM_LOAD128 ((__m128i *)(source+3*sizeof(__m128i)));
                r4 = RH_MM_LOAD128 ((__m128i *)(source+4*sizeof(__m128i)));
                r5 = RH_MM_LOAD128 ((__m128i *)(source+5*sizeof(__m128i)));
                break;
        case 4: r0 = RH_MM_LOAD128 ((__m128i *)(source+0*sizeof(__m128i)));
                r1 = RH_MM_LOAD128 ((__m128i *)(source+1*sizeof(__m128i)));
                r2 = RH_MM_LOAD128 ((__m128i *)(source+2*sizeof(__m128i)));
                r3 = RH_MM_LOAD128 ((__m128i *)(source+3*sizeof(__m128i)));
                r4 = RH_MM_LOAD128 ((__m128i *)(source+4*sizeof(__m128i)));
                break;
        case 3: r0 = RH_MM_LOAD128 ((__m128i *)(source+0*sizeof(__m128i)));
                r1 = RH_MM_LOAD128 ((__m128i *)(source+1*sizeof(__m128i)));
                r2 = RH_MM_LOAD128 ((__m128i *)(source+2*sizeof(__m128i)));
                r3 = RH_MM_LOAD128 ((__m128i *)(source+3*sizeof(__m128i)));
                break;
        case 2: r0 = RH_MM_LOAD128 ((__m128i *)(source+0*sizeof(__m128i)));
                r1 = RH_MM_LOAD128 ((__m128i *)(source+1*sizeof(__m128i)));
                r2 = RH_MM_LOAD128 ((__m128i *)(source+2*sizeof(__m128i)));
                break;
        case 1: r0 = RH_MM_LOAD128 ((__m128i *)(source+0*sizeof(__m128i)));
                r1 = RH_MM_LOAD128 ((__m128i *)(source+1*sizeof(__m128i)));
                break;
        case 0: r0 = RH_MM_LOAD128 ((__m128i *)(source+0*sizeof(__m128i)));
                break;
        default: RH_ASSERT(false);
    }

    U8* head = nextChunk;
    U8* end = head + size;
    //load work
    while(head < end)
    {
        uint32_t x = rndState;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        rndState = x;
        U32 d;
        #define RH_GB128_SSE4(chunk128, n)                                         \
            {                                                               \
                d = ((n) & 0x7)*8;                                          \
                switch((n)>>2)                                              \
                {                                                           \
                    case 0:b = _mm_extract_epi32_M(chunk128, 0)>>d; break;  \
                    case 1:b = _mm_extract_epi32_M(chunk128, 1)>>d; break;  \
                    case 2:b = _mm_extract_epi32_M(chunk128, 2)>>d; break;  \
                    case 3:b = _mm_extract_epi32_M(chunk128, 3)>>d; break;  \
                    default:                                                \
                        RH_ASSERT(false);                              \
                };                                                          \
            }

        U8 b;
        U32 val = x % size;
        U32 reg = val / 16;
        U32 n = val % 16;
        switch(reg)
        {
            case 7: RH_GB128_SSE4(r7, n)  break;
            case 6: RH_GB128_SSE4(r6, n)  break;
            case 5: RH_GB128_SSE4(r5, n)  break;
            case 4: RH_GB128_SSE4(r4, n)  break;
            case 3: RH_GB128_SSE4(r3, n)  break;
            case 2: RH_GB128_SSE4(r2, n)  break;
            case 1: RH_GB128_SSE4(r1, n)  break;
            case 0: RH_GB128_SSE4(r0, n)  break;
            default: RH_ASSERT(false);
        }
        
        *head = b;
        head++;
    }
}


#else


inline void Transfo0_2(U8* nextChunk, U32 size, U8* source)
{
    U32 rndState;
    rndState = (*(U32*)(source+size-4));

    if (!rndState)
        rndState = 1;

    for(U32 i=0; i < size; i++)
    {                
        rndState ^= rndState << 13;
        rndState ^= rndState >> 17;
        rndState ^= rndState << 5;
        *nextChunk = source[rndState % size];
        nextChunk++;
    }
}
#endif


void Transfo1_2(U8* nextChunk, U32 size, U8* outputPtr)
{
    U32 halfSize = size >> 1;
    RH_ASSERT((size % 2) == 0);


    memcpy(nextChunk, outputPtr + halfSize , halfSize);
    memcpy(nextChunk + halfSize, outputPtr, halfSize);
}

void Transfo2_2(U8* nextChunk, U32 size, U8* outputPtr)
{
    U32 halfSize = size >> 1;
    
    U8* srcHead = outputPtr;
    U8* srcEnd = srcHead + halfSize;
    U8* srcTail = &outputPtr[size - 1];
    U8* tail = &nextChunk[size - 1];
    U8* head = nextChunk;
    while(srcHead < srcEnd)
    {
        *head = *srcTail;
        *tail = *srcHead;
        head++;
        tail--;
        srcHead++;
        srcTail--;
    }
}


void Transfo3_2(U8* nextChunk, U32 size, U8* outputPtr)
{
    RH_ASSERT((size % 2) == 0);

    U32 halfSize = size >> 1;
    U32 left = 0;
    U32 right = (int)halfSize;
    U8* work = nextChunk;
    while(left < halfSize)
    {
        *work = outputPtr[left++];
        work++;
        *work = outputPtr[right++];
        work++;
    }

#ifdef RH_ENABLE_ASSERT
    {
        RH_ASSERT(size < RH2_StrideSize);
    }
#endif
    
}


void Transfo4_2(U8* nextChunk, U32 size, U8* outputPtr)
{
    RH_ASSERT((size % 2) == 0);
    U32 halfSize = (size >> 1);
    U8* left = outputPtr;
    U8* right = outputPtr + halfSize;
    U8* work = nextChunk;
    while (halfSize)
    {
        *work = *right;
        work++;
        *work = *left;
        work++;
        right++;
        left++;
        halfSize--;
    }
}

void Transfo5_2(U8* nextChunk, U32 size, U8* outputPtr)
{
    RH_ASSERT((size % 2) == 0);
    const U32 halfSize = size >> 1;
    S32 itt = 0;
    S32 ritt = size-1;
    size = 0;
    while(size < halfSize)
    {
        //first half
        nextChunk[size] = outputPtr[itt] ^ outputPtr[itt + 1];
        itt += 2;
        //second half
        nextChunk[size+halfSize] = outputPtr[size] ^ outputPtr[ritt];
        size++;
        ritt--;
    }
}



void Transfo6_2(U8* nextChunk, U32 size, U8* source)
{
    U32 i = 0;
#if defined(RH2_ENABLE_COMPACT_TRANSFO_67)
    while(i < size)
    {
        nextChunk[i] = ROTL8(source[i], size-i);
        i++;
    }
#else
    U8* end = nextChunk + (size >> 3)*8;
    while(((size_t)source % 8) != 0 && (i < size))
    {
        *nextChunk = ROTL8(*source, size-i);
        i++;
        nextChunk++;
        source++;
    }
    
    while(nextChunk < end)
    {
        U64 res = 0; 
        U64 buf = *(U64*)source;
        source += 8;

        U32 localSize = size - i;
        U64 b;
        
        b = (U8)(buf);
        b = ROTL8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8*0);
        res |= b;
        b = (U8)(buf);
        b = ROTL8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8*1);
        res |= b;
        b = (U8)(buf);
        b = ROTL8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8*2);
        res |= b;
        b = (U8)(buf);
        b = ROTL8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8*3);
        res |= b;
        b = (U8)(buf);
        b = ROTL8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8*4);
        res |= b;
        b = (U8)(buf);
        b = ROTL8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8*5);
        res |= b;
        b = (U8)(buf);
        b = ROTL8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8*6);
        res |= b;
        b = (U8)(buf);
        b = ROTL8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8*7);
        res |= b;

        i += 8;
        *(U64*)nextChunk = res;
        nextChunk += 8;
    }

    while(i < size)
    {
        *nextChunk = ROTL8(*source, size-i);
        i++;
        nextChunk++;
        source++;
    }
#endif //cudaarch
}


void Transfo7_2(U8* nextChunk, U32 size, U8* source)
{
    U32 i = 0;
#if defined(RH2_ENABLE_COMPACT_TRANSFO_67)
    while(i < size)
    {
        nextChunk[i] = ROTR8(source[i], size-i);
        i++;
    }
#else
    U8* end = nextChunk + (size >> 3)*8;
    while(((size_t)source % 8) != 0 && (i < size))
    {
        *nextChunk = ROTR8(*source, size-i);
        i++;
        nextChunk++;
        source++;
    }

    while(nextChunk < end)
    {
        U64 res = 0;
        U64 buf = *(U64*)source;
        source += 8;

        U32 localSize = size - i;
        U64 b;
        b = (U8)(buf);
        b = ROTR8((U8)b, localSize);
        localSize--;
        buf >>= 8;
        b <<= (8 * 0);
        res |= b;
        b = (U8)(buf);
        b = ROTR8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8 * 1);
        res |= b;
        b = (U8)(buf);
        b = ROTR8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8 * 2);
        res |= b;
        b = (U8)(buf);
        b = ROTR8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8 * 3);
        res |= b;
        b = (U8)(buf);
        b = ROTR8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8 * 4);
        res |= b;
        b = (U8)(buf);
        b = ROTR8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8 * 5);
        res |= b;
        b = (U8)(buf);
        b = ROTR8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8 * 6);
        res |= b;
        b = (U8)(buf);
        b = ROTR8((U8)b, localSize); 
        localSize--;
        buf >>= 8;
        b <<= (8 * 7);
        res |= b;

        i += 8;
        *(U64*)nextChunk = res;
        nextChunk += 8;
    }
    while(i < size)
    {
        *nextChunk = ROTR8(*source, size-i);
        i++;
        nextChunk++;
        source++;
    }
#endif 
}

//{{{ TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP
#if defined(RHMINER_ENABLE_SSE4)

//TODO: Perf test that 

inline void Transfo0_64(U8* nextChunk, U32 size, U8* source)
{
    RH_ASSERT(size == 64);
    U32 x = (*(U32*)(source+size-4));

    if (!x)
        x = 1;

    __m128i r0,r1,r2,r3;
    U32 tval;
    #define RH_TR0_RND_4X_64()                             \
    {                                                   \
        x ^= x << 13;                                   \
        x ^= x >> 17;                                   \
        x ^= x << 5;                                    \
        tval = source[x % size];                        \
        tval <<= 8;                                     \
        x ^= x << 13;                                   \
        x ^= x >> 17;                                   \
        x ^= x << 5;                                    \
        tval |= source[x % size];                       \
        tval <<= 8;                                     \
        x ^= x << 13;                                   \
        x ^= x >> 17;                                   \
        x ^= x << 5;                                    \
        tval |= source[x % size];                       \
        tval <<= 8;                                     \
        x ^= x << 13;                                   \
        x ^= x >> 17;                                   \
        x ^= x << 5;                                    \
        tval |= source[x % size];                       \
        tval = RH_swap_u32(tval); \
    }
    

    #define RH_TR0_PACKING_16X(R128) \
        RH_TR0_RND_4X_64(); \
        R128 = _mm_cvtsi32_si128(tval); \
        RH_TR0_RND_4X_64(); \
        R128 = _mm_insert_epi32_M(R128, tval, 1); \
        RH_TR0_RND_4X_64(); \
        R128 = _mm_insert_epi32_M(R128, tval, 2); \
        RH_TR0_RND_4X_64(); \
        R128 = _mm_insert_epi32_M(R128, tval, 3); \


    RH_TR0_PACKING_16X(r0);
    RH_TR0_PACKING_16X(r1);
    RH_TR0_PACKING_16X(r2);
    RH_TR0_PACKING_16X(r3);

    RH_MM_STORE128(((__m128i*)nextChunk)+0, r0);
    RH_MM_STORE128(((__m128i*)nextChunk)+1, r1);
    RH_MM_STORE128(((__m128i*)nextChunk)+2, r2);
    RH_MM_STORE128(((__m128i*)nextChunk)+3, r3);

    //__debugbreak();
}



void Transfo1_64(U8* nextChunk, U32 size, U8* outputPtr)
{
    RH_ASSERT(size == 64);

    __m128i r0,r1,r2,r3;

    r0 = RH_MM_LOAD128(((__m128i *)(outputPtr)) + 0 );
    r1 = RH_MM_LOAD128(((__m128i *)(outputPtr)) + 1 );
    r2 = RH_MM_LOAD128(((__m128i *)(outputPtr)) + 2 ); //half
    r3 = RH_MM_LOAD128(((__m128i *)(outputPtr)) + 3 );
    __m128i* dst = (__m128i*)nextChunk;
    RH_MM_STORE128(dst++, r2);
    RH_MM_STORE128(dst++, r3);
    RH_MM_STORE128(dst++, r0);
    RH_MM_STORE128(dst++, r1);
}


void Transfo2_64(U8* nextChunk, U32 size, U8* outputPtr)
{
    __m128i r0,r1,r2,r3,mask;
     mask = _mm_set_epi64x(0x0001020304050607ULL, 0x08090a0b0c0d0e0fULL);
    r0 = RH_MM_LOAD128 (((__m128i *)(outputPtr)) + 0 );
    r1 = RH_MM_LOAD128 (((__m128i *)(outputPtr)) + 1 );
    r2 = RH_MM_LOAD128 (((__m128i *)(outputPtr)) + 2 ); //half
    r3 = RH_MM_LOAD128 (((__m128i *)(outputPtr)) + 3);
    r0 = _mm_shuffle_epi8(r0, mask);
    r1 = _mm_shuffle_epi8(r1, mask);
    r2 = _mm_shuffle_epi8(r2, mask);
    r3 = _mm_shuffle_epi8(r3, mask);

    __m128i* dst = (__m128i*)nextChunk;
    RH_MM_STORE128(dst++, r3);
    RH_MM_STORE128(dst++, r2);
    RH_MM_STORE128(dst++, r1);
    RH_MM_STORE128(dst++, r0);
    
}

#define _mm_extract_epi64_M _mm_extract_epi64 

#define RH_TRX34_MERGE(reg0, reg2, i0, i1)           \
        {a0 = _mm_extract_epi32_M(reg0, i0);         \
        b0 = _mm_extract_epi32_M(reg2, i0);          \
        tmp = _mm_set1_epi32(a0);                    \
        tmp = _mm_insert_epi32_M(tmp, b0, 1);          \
        tmp = _mm_shuffle_epi8(tmp, mask);           \
        t0 = _mm_extract_epi64_M(tmp, 1);            \
                                                     \
        a0 = _mm_extract_epi32_M(reg0, i1);          \
        b0 = _mm_extract_epi32_M(reg2, i1);          \
        tmp = _mm_set1_epi32(a0);                    \
        tmp = _mm_insert_epi32_M(tmp, b0, 1);          \
        tmp = _mm_shuffle_epi8(tmp, mask);           \
        tmp = _mm_insert_epi64(tmp, t0, 0);          \
        RH_MM_STORE128(dst++, tmp);}                 \



void Transfo3_64(U8* nextChunk, U32 size, U8* outputPtr)
{
    RH_ASSERT((size % 2) == 0);

    //__debugbreak();
    U32 a0, b0;
    U64 t0;
    __m128i r0,r1,r2,r3,tmp, mask;
    mask = _mm_set_epi8(7,3,6,2,5,1,4,0, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);
    r0 = RH_MM_LOAD128 (((__m128i *)(outputPtr)) + 0 );
    r1 = RH_MM_LOAD128 (((__m128i *)(outputPtr)) + 1 );
    r2 = RH_MM_LOAD128 (((__m128i *)(outputPtr)) + 2 ); //half
    r3 = RH_MM_LOAD128 (((__m128i *)(outputPtr)) + 3);
    __m128i* dst = (__m128i*)nextChunk;

    RH_TRX34_MERGE(r0, r2, 0, 1);
    RH_TRX34_MERGE(r0, r2, 2, 3);
    RH_TRX34_MERGE(r1, r3, 0, 1);
    RH_TRX34_MERGE(r1, r3, 2, 3);

    //__debugbreak();
}


#define _mm_extract_epi64_M _mm_extract_epi64 

#define RH_TRX34_MERGE(reg0, reg2, i0, i1)           \
        {a0 = _mm_extract_epi32_M(reg0, i0);         \
        b0 = _mm_extract_epi32_M(reg2, i0);          \
        tmp = _mm_set1_epi32(a0);                    \
        tmp = _mm_insert_epi32_M(tmp, b0, 1);          \
        tmp = _mm_shuffle_epi8(tmp, mask);           \
        t0 = _mm_extract_epi64_M(tmp, 1);            \
                                                     \
        a0 = _mm_extract_epi32_M(reg0, i1);          \
        b0 = _mm_extract_epi32_M(reg2, i1);          \
        tmp = _mm_set1_epi32(a0);                    \
        tmp = _mm_insert_epi32_M(tmp, b0, 1);          \
        tmp = _mm_shuffle_epi8(tmp, mask);           \
        tmp = _mm_insert_epi64(tmp, t0, 0);          \
        RH_MM_STORE128(dst++, tmp);}                 \


#define RH_TRX67_4BYTES(R128, N, _ROT)             \
    val32 = _mm_extract_epi32_M(R128, N);    \
    res = 0;                                 \
    b = (U8)(val32);                         \
    b = _ROT((U8)b, size);                  \
    size--;                                  \
    val32 >>= 8;                             \
    b <<= (8*0);                             \
    res |= b;                                \
    b = (U8)(val32);                         \
    b = _ROT((U8)b, size);                  \
    size--;                                  \
    val32 >>= 8;                             \
    b <<= (8*1);                             \
    res |= b;                                \
    b = (U8)(val32);                         \
    b = _ROT((U8)b, size);                  \
    size--;                                  \
    val32 >>= 8;                             \
    b <<= (8*2);                             \
    res |= b;                                \
    b = (U8)(val32);                         \
    b = _ROT((U8)b, size);                  \
    size--;                                  \
    val32 >>= 8;                             \
    b <<= (8*3);                             \
    res |= b;                                \
    R128 = _mm_insert_epi32_M(R128, res, N); \


void Transfo6_64(U8* nextChunk, U32 size, U8* source)
{
    //__debugbreak();
    __m128i r0,r1,r2,r3;
    r0 = RH_MM_LOAD128 (((__m128i *)(source)) + 0 );
    r1 = RH_MM_LOAD128 (((__m128i *)(source)) + 1 );
    r2 = RH_MM_LOAD128 (((__m128i *)(source)) + 2 ); //half
    r3 = RH_MM_LOAD128 (((__m128i *)(source)) + 3 );
    
    U32 val32, res;
    U32 b;    

    RH_TRX67_4BYTES(r0, 0, ROTL8);
    RH_TRX67_4BYTES(r0, 1, ROTL8);
    RH_TRX67_4BYTES(r0, 2, ROTL8);
    RH_TRX67_4BYTES(r0, 3, ROTL8);
    RH_TRX67_4BYTES(r1, 0, ROTL8);
    RH_TRX67_4BYTES(r1, 1, ROTL8);
    RH_TRX67_4BYTES(r1, 2, ROTL8);
    RH_TRX67_4BYTES(r1, 3, ROTL8);
    RH_TRX67_4BYTES(r2, 0, ROTL8);
    RH_TRX67_4BYTES(r2, 1, ROTL8);
    RH_TRX67_4BYTES(r2, 2, ROTL8);
    RH_TRX67_4BYTES(r2, 3, ROTL8);
    RH_TRX67_4BYTES(r3, 0, ROTL8);
    RH_TRX67_4BYTES(r3, 1, ROTL8);
    RH_TRX67_4BYTES(r3, 2, ROTL8);
    RH_TRX67_4BYTES(r3, 3, ROTL8);

    __m128i* dst = (__m128i*)nextChunk;
    RH_MM_STORE128(dst++, r0);
    RH_MM_STORE128(dst++, r1);
    RH_MM_STORE128(dst++, r2);
    RH_MM_STORE128(dst++, r3);
     //__debugbreak();

}

void Transfo7_64(U8* nextChunk, U32 size, U8* source)
{    
    __m128i r0,r1,r2,r3;
    r0 = RH_MM_LOAD128 (((__m128i *)(source)) + 0 );
    r1 = RH_MM_LOAD128 (((__m128i *)(source)) + 1 );
    r2 = RH_MM_LOAD128 (((__m128i *)(source)) + 2 ); //half
    r3 = RH_MM_LOAD128 (((__m128i *)(source)) + 3 );
    
    U32 val32, res;
    U32 b;


    RH_TRX67_4BYTES(r0, 0, ROTR8);
    RH_TRX67_4BYTES(r0, 1, ROTR8);
    RH_TRX67_4BYTES(r0, 2, ROTR8);
    RH_TRX67_4BYTES(r0, 3, ROTR8);
    RH_TRX67_4BYTES(r1, 0, ROTR8);
    RH_TRX67_4BYTES(r1, 1, ROTR8);
    RH_TRX67_4BYTES(r1, 2, ROTR8);
    RH_TRX67_4BYTES(r1, 3, ROTR8);
    RH_TRX67_4BYTES(r2, 0, ROTR8);
    RH_TRX67_4BYTES(r2, 1, ROTR8);
    RH_TRX67_4BYTES(r2, 2, ROTR8);
    RH_TRX67_4BYTES(r2, 3, ROTR8);
    RH_TRX67_4BYTES(r3, 0, ROTR8);
    RH_TRX67_4BYTES(r3, 1, ROTR8);
    RH_TRX67_4BYTES(r3, 2, ROTR8);
    RH_TRX67_4BYTES(r3, 3, ROTR8);

    __m128i* dst = (__m128i*)nextChunk;
    RH_MM_STORE128(dst++, r0);
    RH_MM_STORE128(dst++, r1);
    RH_MM_STORE128(dst++, r2);
    RH_MM_STORE128(dst++, r3);

}
#endif //sse4
//}}} TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP TEMP


