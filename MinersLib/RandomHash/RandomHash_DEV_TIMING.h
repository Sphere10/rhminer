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

#ifdef RHMINER_DEBUG_RANDOMHASH_ENABLE_TIMINGS
    extern U64 TimeGetMicroSec();
    #define RH_GET_MILLISEC() (TimeGetMicroSec()/1000)
    #define RHMINER_DEBUG_RANDOMHASH_TIMINGS_BEGIN()                   U64 _start = RH_GET_MILLISEC();
    #ifdef RANDOMHASH_CUDA
        #define RHMINER_DEBUG_RANDOMHASH_TIMINGS_END(THREADS)     {    CUDA_SAFE_CALL(cudaDeviceSynchronize()); \
                                                                    U32 _dt = RH_GET_MILLISEC() - _start; \
                                                                    KERNEL_LOG("GPU%d AVG Time is %.2f ms per thread(%.2f H/S for %d threads). Total run %d ms\n", GetDeviceID(), (_dt /float(THREADS)), (float(THREADS)*1000.0f/(float)_dt), THREADS, _dt);}
    #else
        static std::mutex* _mtx = new std::mutex;
        static U64 accum = 0;
        static U32 accumCnt = 0;
        static U64 lastTime = 0;
        static U64 totalAccumStart = 0;
        static U32 totalAccum = 0;
        void DoProfile(U64 start, U32 THREADS)
        {   
            U64 _dt = RH_GET_MILLISEC() - start;
            {
                std::lock_guard<std::mutex> _x(*_mtx);
                if (!totalAccumStart)
                    totalAccumStart = RH_GET_MILLISEC();
                accum += _dt;
                accumCnt++;
                totalAccum++;
                if (!lastTime)
                    lastTime = RH_GET_MILLISEC();
                if (RH_GET_MILLISEC() > lastTime + 1000)
                {
                    float dt = float(RH_GET_MILLISEC() - lastTime);
                    float avg = (accum / (float)accumCnt/(float)THREADS);
                    lastTime = RH_GET_MILLISEC();
                    PrintOut("GPU%d AVG Time is %.2f ms per thread(%.2f H/S for %d threads AVG %.2f H/s). Total run %.2f ms\n", 
                        GetDeviceID(), 
                        avg, 
                        (float(accumCnt)*1000.0f/(float)dt), 
                        THREADS, 
                        (float(totalAccum)*1000.0f/(float)(RH_GET_MILLISEC() - totalAccumStart + 1)), 
                        dt);
                    accum = 0;
                    accumCnt = 0;
                }
            }
        }

        #define RHMINER_DEBUG_RANDOMHASH_TIMINGS_END(THREADS)     DoProfile(_start, THREADS);
#endif

#endif
