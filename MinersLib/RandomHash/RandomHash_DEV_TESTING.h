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
/// @file
/// @copyright QualiaLibre
#ifndef RANDOM_HASH_BASE_H
#define RANDOM_HASH_BASE_H

#if defined(_MSC_VER) 
#pragma warning(disable:4250)
#endif

//#define RHMINER_DEBUG_STRIDE_INTEGRITY_CHECK
//#define RHMINER_DEBUG_STRIDE_ARRAY_MURMUR3
//#define RHMINER_DEBUG_RANDOMHASH_UNITTEST_CUDA
//#define RHMINER_DEBUG_RANDOMHASH_UNITTEST
//#define RHMINER_DEBUG_RANDOMHASH_ENABLE_TIMINGS
//#define _____CUT_CUDA_COMPIL_TO_WORK_ON_CPU_____


//#define RHMINER_RANDOMHASH_RHMINER_SSE4
//#define RHMINER_DEBUG_RANDOMHASH_STATS
//#define RHMINER_DEBUG_TEST_REF
//#define RHMINER_DEBUG_PRINT_OUT_ALGO_STEPS
//#define RHMINER_DEBUG_RANDOMHASH
//#define RHMINER_DEBUG_RANDOMHASH_REFERENCE_TEST_STEPS
//#define RHMINER_DEBUG_RANDOMHASH_TIMINGS_TEST

#ifdef RH_DEBUG_TEST_ALGO
    //test algo
    {
        bytes in2 = fromHex("90010000CA022000BB718B4B00D6F74478C332F5FB310507E55A9EF9B38551F63858E3F7C86DBD00200006F69AFAE8A6B0735B6ACFCC58B7865FC8418897C530211F19140C9F95F2453240420F00000000000400030000000008617364662E506F6C796D696E657231506F6C796D696E6572315062373861313533661A8F7BAAC08AF918B363C57FB54948BFA1C8EBB6F8DF88D53CE096839516C663E3B0C44298FC1C149AFBF4C8996FB92427AE41E4649B934CA495991B7852B855000000005ACB925BE72C2D31");
        string result2 = "519063c65dc5e7e0ad204c18bdcf492fde1a01ac885eab371c7aa4a5cf648497";
        U8 output2[32];
        U8 input2[PascalHeaderSize];
        RHMINER_ASSERT(in2.size() == PascalHeaderSize);
        memset(output2, 0, sizeof(output2));
        memcpy(input2, &in2[0], in2.size());
                        
        RandomHash_SetHeader(&m_randomHashArray[0], &input2[0], 0x20b7436d);
        RandomHash_Search(&m_randomHashArray[0], output2, ((U32*)input2)[(PascalHeaderSize/4)-1]);
        RHMINER_ASSERT(memcmp(toHex((void*)output2, 32).c_str(), result2.c_str(), 32) == 0);
    }
#endif


//auto-config
#ifdef RHMINER_DEBUG_RANDOMHASH_UNITTEST_CUDA
    #undef RHMINER_DEBUG_RANDOMHASH_TIMINGS_TEST 
#endif

#ifdef RHMINER_DEBUG_RANDOMHASH_REFERENCE_TEST_STEPS
  #ifdef __CUDA_ARCH__
    #define RH_DEBUG_OUT_STEPS(...) {if (KERNEL_GET_GID() == 0) printf(__VA_ARGS__);}
  #else
    #define RH_DEBUG_OUT_STEPS(...) PrintOutCritical(__VA_ARGS__);
  #endif
#else
    #define RH_DEBUG_OUT_STEPS(...) 
#endif

#if defined(RHMINER_DEBUG) || defined(RHMINER_DEBUG_DEV)
    
    CUDA_DECL_HOST_AND_DEVICE
    inline void CUDA_SYM(RandomHash_PrintHex)(void* ptr, size_t size, bool cutOutput, bool addEndl)
    {
        const char hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
        U8* s = (U8*)ptr;
        char buf[408];
        char* b = buf;
        const char* b_end = buf + sizeof(buf)-1;
        for (int i = 0; i < (int)size; i++)
        {
            *b = hex[s[i] >> 4]; 
            b++;
            *b = hex[s[i] & 0xF]; 
            b++;

            if (b >= b_end)
            {
                *b = 0;
                KERNEL_LOG("%s", buf);
                if (cutOutput)
                    KERNEL_LOG("\n");
                b = buf;
            }
        }
        *b = 0;
        
        if (addEndl)
            {KERNEL_LOG("%s\n", buf);}
        else
            {KERNEL_LOG("%s", buf);}

        if (cutOutput && !addEndl)
            KERNEL_LOG("\n");
    }

#else
    #define CU_RandomHash_PrintHex(...)
    #define RandomHash_PrintHex(...)
    #define RH_CUDA_ERROR_CHECK() 
    #define RH_CUDA_ERROR_CHECK_EX(e) 
#endif


#ifdef RHMINER_DEBUG_RANDOMHASH
    #ifndef RHMINER_PLATFORM_GPU
        #define RH_DEBUG_OUT(...) DebugOut(__VA_ARGS__)
    #else
        #define RH_DEBUG_OUT printf
    #endif
#else
    #define RH_DEBUG_OUT(...)
#endif


#if defined(RHMINER_DEBUG_PRINT_OUT_ALGO_STEPS) && !defined(RANDOMHASH_CUDA)
    #undef RH_DEBUG_OUT
    #define RH_DEBUG_OUT
    #define RH_PRINT_STEP(...) { DebugOut(__VA_ARGS__); }

    extern U32 g_RandomHashStep;
    extern const char* RH_GET_NEXT_STEP();
    extern void RH_STEP(U32 s);
    extern const char* bufferToHex(U8* buf, size_t len);
#else
    #define RH_PRINT_STEP(...) {}
    #define RH_GET_NEXT_STEP() ""
    #define RH_STEP(x)
#endif

#ifdef RHMINER_DEBUG_RANDOMHASH_REFERENCE_TEST_STEPS
    #define _RHDS(...) __VA_ARGS__
#else
    #define _RHDS(...) 
#endif



#endif //RANDOM_HASH_BASE_H