/**
 * RandomHash2 reference source code implementation
 *
 * Copyright 2019 Polyminer1 <https://github.com/polyminer1>
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


#ifdef POLY_DEBUG_RANDOMHASH
    U32 g_RandomHashStep = 0;
    const char* RH_GET_NEXT_STEP()
    {
        U32 nstep = g_RandomHashStep++;
        return FormatString("RH_STEP(%-3d);", nstep);
    }
    void RH_STEP(U32 s)
    {
        if (g_RandomHashStep - 1 == s)
        {
            int a = 0; a; //PUT BP here to track one steps in particular
        }
    }
    
    const char* bufferToHex(U8* buf, size_t len) 
    { 
        return FormatString("%s", toHex(buf, len).c_str()); 
    }
#endif



//#define RH_DEBUG_OUT_STEPS_ENABLE
#ifdef RH_DEBUG_OUT_STEPS_ENABLE
#define RH_DEBUG_OUT_STEPS(...) DebugOut(__VA_ARGS__);
#else
#define RH_DEBUG_OUT_STEPS(...) 
#endif

#define RH_PRINT_STEP(...) 
#define RH_STRIDE_STATS(...)


//TEMP
inline string IntToStr(U32 v) { return toString(v); }
thread_local U32 _rndCnt = 1;

static U32 GetNextRnd(mersenne_twister_state& gen, const char* title) 
{
    U32 val = merssen_twister_rand(&gen);
    RH_DEBUG_OUT_STEPS("- Random #%d Val %u    %s\n",  _rndCnt++, val, title);
    return val;
}

using namespace std;
class RandomHash_REF
{
public:
    const U32 MIN_N = 2; // Min-number of hashing rounds required to compute a nonce
    const U32 MAX_N = 4; // Max-number of hashing rounds required to compute a nonce
    const U32 MIN_J = 1; // Min-number of dependent neighbouring nonces required to evaluate a nonce round
    const U32 MAX_J = 8; // Max-number of dependent neighbouring nonces required to evaluate a nonce round
    const U32 M = 64;    // The memory expansion unit (in bytes)
    const U32 NUM_HASH_ALGO = 77;
    const U32 SHA2_256_IX = 47;

    bytes           m_workBytes;

#ifdef POLY_DEBUG_RANDOMHASH_STATS
    //stats
    U32 m_itterationCount = 0;
    struct StatInfos
    {
        U32 m_maxInputsCount=0;
        U32 m_maxWorkSize = 0;
        U32 m_maxUsedMem = 0;
        U32 m_longestResult = 0;
    };
    StatInfos m_stats[2];
    U32 GetStats(const vector<bytes>& inputs, int isNotInput)
    {
        U32 tot = 0;
        for (auto& i : inputs)
        {
            if (i.size() > m_stats[isNotInput].m_longestResult)
                m_stats[isNotInput].m_longestResult = (U32)i.size();

            tot += (U32)i.size();
        }
        
        if (inputs.size() > m_stats[isNotInput].m_maxInputsCount)
            m_stats[isNotInput].m_maxInputsCount = (U32)inputs.size();

        if (tot > m_stats[isNotInput].m_maxUsedMem)
            m_stats[isNotInput].m_maxUsedMem = tot;

        if (m_workBytes.size() > m_stats[isNotInput].m_maxWorkSize)
            m_stats[isNotInput].m_maxWorkSize = (U32)m_workBytes.size();

        return tot;
    }

#endif

    RandomHash_REF()
    {
        
    }

    inline void Reseed(mersenne_twister_state& gen, U32 seed, const char* tittle)
    {
        RH_DEBUG_OUT_STEPS("- Reseed with %u %s\n", seed, tittle);
        //&m_gen.seed(seed);
        merssen_twister_seed(seed, &gen); 
    }

    bytes ChangeNonce(const bytes& blockHeader, U32 nonce)
    {
        // IMPORTANT: Prior to RandomHash, when calculating the proof-of-work the hashable digest
        // includes the nonce at a variable offset. Post RandomHash, the hashable digest
        // includes the nonce at fixed offset 4.
        RH_ASSERT(blockHeader.size() >= 8);
        const int LNonceOffset = 4;

        // clones and changes nonce in blockHeader (by determining offset of nonce)
        bytes newBlockHeader = blockHeader;
        // Overwrite the nonce in little-endian
        newBlockHeader[LNonceOffset+0] = U8(nonce);
        newBlockHeader[LNonceOffset+1] = (nonce >> 8) & 255;
        newBlockHeader[LNonceOffset+2] = (nonce >> 16) & 255;
        newBlockHeader[LNonceOffset+3] = (nonce >> 24) & 255;

        return newBlockHeader;
    }

    U32 Checksum(const bytes& input)
    {
        // standard MurMu3 algorithm (key, len, seed, outHash32)
        U32 csum = MurmurHash3_x86_32_Fast(&input[0], input.size();
        //RH_DEBUG_OUT("Checksum of %d = %u\n", input.size(), csum);
        return csum;
    }
    
    U32 Checksum(const vector<bytes>& inputs )
    {
        // standard MurMu3 algorithm run over list of inputs
		m_murmur3->Initialize();		
        for(auto&i : inputs)
            m_murmur3->TransformBytes(i);

        U32 csum = m_murmur3->TransformFinal()->GetUInt32();
        //RH_DEBUG_OUT("Checksum of %d inputs is %u\n", inputs.size(), csum);

		return csum;
    }

    void Expand(const bytes& input, int ExpansionFactor, bytes& Result)
    {
        RH_DEBUG_OUT_STEPS("- Expand header %u factor %d\n", m_murmur3->ComputeBytes(input)->GetUInt32(), ExpansionFactor); 

        U32 seed = Checksum(input);
        mersenne_twister_state gen;
        Reseed(gen, seed, "Expand");
        
        //TEMP TEMP TEMP TEMP COMMENT
        //size_t sizeExp = input.size() + ExpansionFactor * M;
        size_t sizeExp = input.size() + ExpansionFactor * M;
        //TEMP TEMP TEMP TEMP COMMENT

        bytes output = input;
        S64 bytesToAdd = sizeExp - input.size();

        //RH_DEBUG_OUT("Expand of %d bytes to %u (Exp Fac %d) \n", input.size(), sizeExp, ExpansionFactor);

        while (bytesToAdd > 0)
        {
            bytes nextChunk = output;
            if ((S64)nextChunk.size() > bytesToAdd)
                nextChunk.resize((size_t)bytesToAdd);

            //TEMP TEMP TEMP TEMP COMMENT
            //POLY_ASSERT(nextChunk.size() < RH_StrideSize);
            //TEMP TEMP TEMP TEMP COMMENT

            U32 random = GetNextRnd(gen, "ExpFunc");
            size_t isOdd  = (nextChunk.size() % 2);
            size_t halfSize = nextChunk.size() / 2;
            U32 sizeIsOdd = nextChunk.size() % 2;
            size_t size = nextChunk.size();
            size_t i = 0;
#ifdef POLY_DEBUG_RANDOMHASH_TOUCH_ALL_ALGOS
            random = (g_expandAlgoItt++) % 8;
#endif
            switch(random % 8)
            {
                case 0: //No-Op         (e.g. input = 123456   output = 123456)
                {
                    U32 rndState = m_murmur3->ComputeBytes(nextChunk)->GetUInt32();
                    if (!rndState)
                        rndState = 1;
                
                    m_workBytes.resize(nextChunk.size());
                    memcpy(&m_workBytes[0], &nextChunk[0], nextChunk.size());
                    i = 0;
                    while(i < nextChunk.size())
                    {
                        uint32_t x = rndState;
                        x ^= x << 13;
                        x ^= x >> 17;
                        x ^= x << 5;
                        rndState = x;
                        nextChunk[i] = m_workBytes[x % nextChunk.size()];
                        i++;
                    }
                }break;
                case 1: //Swap-LR       (e.g. input = 123456   output = 456123)   
                {
                    //while(i < halfSize)
                    //{
                    //    std::swap(nextChunk[i], nextChunk[i + halfSize]);
                    //    i++;
                    //}
                    m_workBytes.resize(halfSize);
                    memcpy(&m_workBytes[0], &nextChunk[0], halfSize);

                    memcpy(&nextChunk[0], &nextChunk[0] + halfSize + sizeIsOdd, halfSize);
                    memcpy(&nextChunk[0] + halfSize + sizeIsOdd, &m_workBytes[0], halfSize);
                    if (sizeIsOdd)
                        nextChunk[halfSize] = m_workBytes[halfSize];

                    //PrintOutSimple("- %d result: %s\n",  (random % 8), toHex(nextChunk).c_str());
                }break;
                case 2: //Reverse       (e.g. input = 123456   output = 654321)  
                {
                    while(i < halfSize)
                    {
                        std::swap(nextChunk[i], nextChunk[size-i-1]);
                        i++;
                    }
                    //PrintOutSimple("- %d result: %s\n",  (random % 8), toHex(nextChunk).c_str());
                }break;
                case 3: //L-Interleave  (e.g. input = 123 456   output = 142536)
                {
                    int left = 0;
                    int right = halfSize + sizeIsOdd;
                    m_workBytes.resize(nextChunk.size());
                    U8* work = &m_workBytes[0];
                    while(left < halfSize)
                    {
                        *work = nextChunk[left++];
                        work++;
                        *work = nextChunk[right++];
                        work++;
                    }
                    if (sizeIsOdd)
                        *work = nextChunk[halfSize];

                    nextChunk = m_workBytes;

                    //PrintOutSimple("- %d result: %s\n",  (random % 8), toHex(nextChunk).c_str());
                }break;
                case 4: //R-Interleave  (e.g. input = 123456   output = 415263)
                {
                    //PrintOutSimple("- %d chunk : %s\n",  (random % 8), toHex(nextChunk).c_str());
                    int left = 0;
                    int right = halfSize + sizeIsOdd;
                    m_workBytes.resize(nextChunk.size());
                    U8* work = &m_workBytes[0];
                    while(left < halfSize)
                    {
                        *work = nextChunk[right++];
                        work++;
                        *work = nextChunk[left++];
                        work++;
                    }
                    if (sizeIsOdd)
                        *work = nextChunk[halfSize];

                    nextChunk = m_workBytes;
                    //PrintOutSimple("- %d result: %s\n",  (random % 8), toHex(nextChunk).c_str());
                }break;//                                                  1         2         3         4         5         6 
                case 5: //L-XOR         (e.g. input = 123456   output = XOR(1,2), XOR(3,4), XOR(5,6), XOR(1,6), XOR(2,5), XOR(3,4)
                {
                    size_t itt = 0;
                    size_t ritt = size-1;
                    m_workBytes = nextChunk;
                    while(i < halfSize)
                    {
                        //first half
                        nextChunk[i] = m_workBytes[itt] ^ m_workBytes[itt + 1];
                        itt += 2;

                        //second half
                        nextChunk[i+halfSize + sizeIsOdd] = m_workBytes[i] ^ m_workBytes[ritt--]; 
                        i++;
                    }
                    if (sizeIsOdd)
                        nextChunk[halfSize] = m_workBytes[nextChunk.size()-1];

                    //PrintOutSimple("- %d result: %s\n",  (random % 8), toHex(nextChunk).c_str());
                }break;
                case 6: //ROL-ladder    (e.g. input = ABCDEF   output = ROL(A, 0), ROL(B, 1), ... , ROL(F, 5)
                {
                    while(i < size)
                    {
                        nextChunk[i] = _rotl8(nextChunk[i], (U8)((size-i) % 8));
                        i++;
                    }
                    //PrintOutSimple("- %d result: %s\n",  (random % 8), toHex(nextChunk).c_str());
                }break;
                case 7: //ROR-ladder    (e.g. input = ABCDEF   output = ROR(A, 0), ROR(B, 1), ... , ROR(F, 5)
                {
                    while(i < size)
                    {
                        nextChunk[i] = _rotr8(nextChunk[i],  (U8)((size-i) % 8));
                        i++;
                    }
                    //PrintOutSimple("- %d result: %s\n",  (random % 8), toHex(nextChunk).c_str());
                }break;
            }

            output.insert(output.end(), nextChunk.begin(), nextChunk.end());
            bytesToAdd -= nextChunk.size();
        }
        
        RH_DEBUG_OUT_STEPS("- LOutput %u size %u \n", m_murmur3->ComputeBytes(output)->GetUInt32(), output.size());
        POLY_ASSERT(sizeExp == output.size());
        Result = output;
    }

    void Compress(const vector<bytes>& inputs, bytes& Result)
    {
        RH_DEBUG_OUT_STEPS("- Compress header %d\n", inputs.size());

#if defined(POLY_DEBUG_RANDOMHASH_STATS)
        U32 tot = GetStats(inputs, 0);
#else
        U32 tot = 0;
#endif
        mersenne_twister_state gen;
        U32 rval;
        U32 seed = Checksum(inputs);
        Reseed(gen, seed, "Compress");

        Result.resize(100);
        for (size_t i = 0; i < 100; i++)
        {
            const bytes& source = inputs[GetNextRnd(gen, "comp1") % inputs.size()];
            rval = GetNextRnd(gen, "COMP2");
            Result[i] = source[rval % source.size()];
        }
        
        RH_DEBUG_OUT_STEPS("- Compress done result %u\n", m_murmur3->ComputeBytes(Result)->GetUInt32());
    } 

    template <class HASH_FUNC>
    void RandomHash_HashRoundInput(bytes& roundInput, bytes& output)
    {
        HASH_FUNC hash;
        output = hash.ComputeBytes(roundInput)->GetBytes();
    }

    //Function RandomHash(blockHeader : ByteArray, Round : Integer) : List of ByteArray
    //Called from [5 to 1]
    void RandomHashItt(const bytes& in_blockHeader, U32 in_round, vector<bytes>& io_results)
    {
        POLY_ASSERT(in_round >= 1 && in_round <= N);

        // [[[[[[[[[[[[[[[[[[ RandomHash_Init Snipet Begin
        RH_PRINT_STEP("RandomHash_Phase_init(%d);                 %s\n", in_round, RH_GET_NEXT_STEP());
        vector<bytes> roundOutputs;
        bytes roundInput;
        mersenne_twister_state gen;
        bool IsPhase3 = false;
        // ]]]]]]]]]]]]]]]]] RandomHash_Phase_init Snipet END

        if (in_round == 1)
        {
            // [[[[[[[[[[[[[[[[[[ RandomHash_start Snipet Begin
            RH_PRINT_STEP("RandomHash_start(%d);                %s                       //DEBUG: in_blockHeader %X\n", in_round, RH_GET_NEXT_STEP(), Checksum(in_blockHeader));
            U32 seed = Checksum(in_blockHeader);      // can hash in_blockHeader first, but not required
            Reseed(gen, seed, "RH_P1"); //merssen-twister
            roundInput = in_blockHeader;
            // ]]]]]]]]]]]]]]]]] RandomHash_start Snipet END
        }
        else
        {
            // [[[[[[[[[[[[[[[[[[ RandomHash_Phase_1 Snipet Begin
            vector<bytes> parenAndNeighbortOutputs;
            
            RH_PRINT_STEP("RandomHash_Phase_1_push(%d);         %s\n", in_round,RH_GET_NEXT_STEP());
            //if (ARound = N) and (Length(ABlockHeader) >= 4) AND (FCachedNonce = GetNonce(ABlockHeader)) and BytesEqual(ABlockHeader, FCachedHeader) then begin
            //  parenAndNeighbortOutputs = cacheOutputs
            //else
            RandomHashItt(in_blockHeader, in_round - 1, parenAndNeighbortOutputs);
            RH_PRINT_STEP("RandomHash_Phase_1_pop(%d);          %s\n", in_round, RH_GET_NEXT_STEP());

            U32 seed = Checksum(parenAndNeighbortOutputs);
            Reseed(gen, seed, "RH_p2");//merssen-twister
            RH_PRINT_STEP("                                                                        //DEBUG:  Phase1 round %d: parenAndNeighbortOutputs checksum %X\n", in_round, seed);

            //roundOutputs.AddMany(parenAndNeighbortOutputs)
            for (auto& i : parenAndNeighbortOutputs)
                roundOutputs.push_back(i);
            
            //vector<bytes> neighborOutputs;
            parenAndNeighbortOutputs.clear();
            // ]]]]]]]]]]]]]]]]] RandomHash_Phase_1 Snipet END
            
            // [[[[[[[[[[[[[[[[[[ RandomHash_Phase_2 Snipet Begin
            U32 newNonce = GetNextRnd(gen, "nonce");
            bytes otherNonceHeader = ChangeNonce(in_blockHeader, newNonce);
            RH_PRINT_STEP("                                                                        //DEBUG:  blockHeader %X (SALTED with %X)\n", Checksum(otherNonceHeader), newNonce);

            RH_PRINT_STEP("RandomHash_Phase_2_push(%d);         %s\n", in_round, RH_GET_NEXT_STEP());
            RandomHashItt(otherNonceHeader, in_round - 1, parenAndNeighbortOutputs);
            RH_PRINT_STEP("RandomHash_Phase_2_pop(%d);          %s\n", in_round, RH_GET_NEXT_STEP());
/*
            // Cache neighbour nonce n-1 calculation if on final round (neighbour will be next nonce)
            if (ARound = N) then begin
              FCachedNonce := LNeighbourNonce;
              FCachedHeader := LNeighbourNonceHeader;
              FCachedOutput := LNeighborOutputs;
            end;
*/

            //roundOutputs.AddMany(parenAndNeighbortOutputs)
            for (auto& i : parenAndNeighbortOutputs)
                roundOutputs.push_back(i);
            
            Compress(roundOutputs, roundInput);
            RH_PRINT_STEP("                                                                        //DEBUG: Compress: roundOutputs %X size %d, roundInput %X size %d\n", Checksum(roundOutputs), roundOutputs.size(),Checksum(roundInput) , roundInput.size());
            // ]]]]]]]]]]]]]]]]]] RandomHash_Phase_2 Snipet END


            // [[[[[[[[[[[[[[[[[[ RandomHash_Phase_3 Snipet Begin
            parenAndNeighbortOutputs.clear();
            IsPhase3 = true;
            RH_PRINT_STEP("RandomHash_Phase_3_push(%d);         %s\n", in_round, RH_GET_NEXT_STEP());
            // ]]]]]]]]]]]]]]]]]] RandomHash_Phase_3 Snipet END
        }   
        
        // [[[[[[[[[[[[[[[[[[ RandomHashItt Snipet Begin
        bytes output;
        U32 rndHash = GetNextRnd(gen, "HashFunc") % 18;
        //let hashFunc = HASH_ALGO[&m_gen() % 18]
        //let output = hashFunc(roundInput)          
        //RH_DEBUG_OUT("RandomHashItt round %d res %d - HashingFunc %d\n", in_round, io_results.size(), rndHash);
        RH_PRINT_STEP("RandomHash(%d);                      %s                       //DEBUG: in_blockHeader %X rndHashFunc %d\n", in_round, RH_GET_NEXT_STEP(),  Checksum(in_blockHeader), rndHash);
        
#ifdef POLY_DEBUG_RANDOMHASH_TOUCH_ALL_ALGOS
        rndHash = (g_rndHashItt++) % 18;
#endif
        RH_STRIDE_STATS( g_check[rndHash] = 1; )

        switch(rndHash)
        {
            case RandomHash1Algos::RH1_SHA2_256     :
            {
                //SHA2_256 hash; output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<SHA2_256>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_SHA2_384     :
            {
                //SHA2_384 hash; output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<SHA2_384>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_SHA2_512     :
            {
                //SHA2_512 hash; output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<SHA2_512>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_SHA3_256     :
            {
                //SHA3_256 hash; output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<SHA3_256>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_SHA3_384     :
            {
                //SHA3_384 hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<SHA3_384>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_SHA3_512     :
            {
                //SHA3_512 hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<SHA3_512>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_RIPEMD160    :
            {
                //RIPEMD160 hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<RIPEMD160>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_RIPEMD256    :
            {
                //RIPEMD256 hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<RIPEMD256>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_RIPEMD320    :
            {
                //RIPEMD320 hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<RIPEMD320>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_Blake2b      :
            {
                blake2b compressor;
                compressor.init(64, NULL, 0);
                output.resize(64);
                compressor.update(roundInput.data(), roundInput.size());
                compressor.finalize(&output[0]);
                //swab256((void*)solution.data(), work.data());
            } break;
            case RandomHash1Algos::RH1_Blake2s      :
            {
	            //unsigned char _ALIGN(64) hash[BLAKE2S_OUTBYTES];
	            blake2s_state blake2_ctx;
                output.resize(BLAKE2S_OUTBYTES);

	            blake2s_init(&blake2_ctx, BLAKE2S_OUTBYTES);
	            blake2s_update(&blake2_ctx, roundInput.data(), roundInput.size());
	            blake2s_final(&blake2_ctx, &output[0], BLAKE2S_OUTBYTES);

	            //memcpy(output, hash, 32);
            } break;
            case RandomHash1Algos::RH1_Tiger2_5_192 :
            {
                //Tiger2_5_192 hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<Tiger2_5_192>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_Snefru_8_256 :
            {
                //Snefru_8_256 hash;
                Snefru hash(8, HashSize256);
                output = hash.ComputeBytes(roundInput)->GetBytes();
            } break;
            case RandomHash1Algos::RH1_Grindahl512  :
            {
                //Grindahl512 hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<Grindahl512>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_Haval_5_256  :
            {
                //Haval_5_256 hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<Haval_5_256>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_MD5          :
            {
                //MD5 hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<MD5>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_RadioGatun32 :
            {
                //RadioGatun32 hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<RadioGatun32>(roundInput, output);
            } break;
            case RandomHash1Algos::RH1_Whirlpool    :
            {
                //WhirlPool hash;                output = hash.ComputeBytes(roundInput)->GetBytes();
                RandomHash_HashRoundInput<WhirlPool>(roundInput, output);
            } break;
        }
        Expand(output, N - in_round, output);
        RH_PRINT_STEP("                                                                        //DEBUG: Expand: RH result %X size %d\n", Checksum(output), output.size());
        roundOutputs.push_back(output);

#if defined(POLY_DEBUG_RANDOMHASH_STATS)
        m_itterationCount++;
        GetStats(roundOutputs, 1);
#endif

        io_results = roundOutputs; 
        RH_PRINT_STEP("RandomHash_end(%d);                  %s                       //DEBUG: roundOutputs %X size %d\n", in_round, RH_GET_NEXT_STEP(), Checksum(roundOutputs), roundOutputs.size());

        // ]]]]]]]]]]]]]]]]]RandomHashItt Snipet END
    }

    void RandomHash(U8* state, const bytes& blockHeader)
    {
        vector<bytes> allOutputs; 
        bytes resComp;
        SHA2_256 hash;

        RH_PRINT_STEP("RandomHash_FirstCall_push(%d);       %s                      //DEBUG: header %s\n", N, RH_GET_NEXT_STEP(), ::toHex(blockHeader).c_str());
#ifdef POLY_DEBUG_RANDOMHASH_TOUCH_ALL_ALGOS
        RH_PRINT_STEP("POLY_DEBUG_RANDOMHASH_TOUCH_ALL_ALGOS            %s\n", RH_GET_NEXT_STEP());
#endif
        //let allOutputs = RandomHash( blockHeader, N)
        RandomHashItt(blockHeader, N, allOutputs);
        RH_PRINT_STEP("RandomHash_FirstCall_pop(%d);        %s\n", N, RH_GET_NEXT_STEP());

        //Result := SHA2_256( Compress( allOutputs ) )
        Compress(allOutputs, resComp);
        RH_PRINT_STEP("                                                                        //DEBUG: CompressLast: allOutputs %X size %d, resComp %X size %d\n", Checksum(allOutputs), allOutputs.size(),Checksum(resComp) , resComp.size());

        HashLibByteArray resSHA = hash.ComputeBytes(resComp)->GetBytes();
        memcpy(state, &resSHA[0], 32);
        RH_PRINT_STEP("                                                                        //DEBUG: Finalize state %s\n", toHex(&resSHA[0], 32).c_str());
        
#if defined(POLY_DEBUG_RANDOMHASH_STATS)
        //RandomHash END with 31 res in 0.528 sec. Max mem=1005016, Longest Res=41024, Max worksize=20480(random), Max Input Cnt=31, Total Itt=31
        U32 maxArrayCount = m_stats[0].m_maxInputsCount;
        if (m_stats[1].m_maxInputsCount > m_stats[0].m_maxInputsCount)
            maxArrayCount = m_stats[1].m_maxInputsCount;
        U32 strideSize = m_stats[0].m_longestResult;
        if (m_stats[1].m_longestResult > strideSize)
            strideSize = m_stats[1].m_longestResult;

        PrintOut(
            "\n#define RH_StrideArrayCount     %d\n"
              "#define RH_StrideSize           %d\n",
            maxArrayCount,
            ((strideSize / 4096) + 1) * 4096);
#endif
    }
}; //RandomHash_REF


void pascalhashV4_Ref(void *state, const void *input)
{
    //U64 start = TimeGetMilliSec();
    bytes header;

    header.clear(); 
    RandomHash_REF rh; 
    header.resize(200);
    memcpy(&header[0], input, 200);
    rh.RandomHash((U8*)state, header);

#ifdef POLY_DEBUG_RANDOMHASH_REFERENCE_TEST_STEPS
    //print output !
    KERNEL_LOG("HASH %llX %llx %llx %llx\n",  ((U64*)state)[0], ((U64*)state)[1], ((U64*)state)[2], ((U64*)state)[3]);
#endif
}

