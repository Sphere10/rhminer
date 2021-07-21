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
#ifdef _MSC_VER
#pragma warning (disable: 4018)
#endif

#include "MinersLib/RandomHash/RandomHash_def.h"
#include "corelib/miniweb.h"

//#include "corelib/int128_o.h"

#define RHMINER_DEBUG_RANDOMHASH_UNITTEST
//#define RHMINER_DEBUG_RANDOMHASH_UNITTEST_CUDA

#if defined(RHMINER_DEBUG_RANDOMHASH_UNITTEST) && !defined(RANDOMHASH_CUDA)



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//RSA

//https://www.mobilefish.com/services/rsa_key_generation/rsa_key_generation.php
//https://www.codeproject.com/Questions/602310/ImplementingplusRSAplusCryptographyplusinplusC-2b
//https://github.com/pBouillon/rsa/blob/master/utils.py

typedef pair<int, int> PublicKey;
typedef pair<int, int> PrivateKey;

struct Keys
{
    PublicKey public_key;
    PrivateKey private_key;
};

///////
#define LIMIT 10000

int log_power(int n, int p, int mod)
{
    int result = 1;
    for (; p; p >>= 1)
    {
        if (p & 1)
            result = (1LL * result * n) % mod;
        n = (1LL * n * n) % mod;
    }
    return result;
}

bool rabin_miller(int n)
{
    bool ok = true;

    for (int i = 1; i <= 5 && ok; i++) {
        int a = rand() + 1;
        int result = log_power(a, n - 1, n);
        ok &= (result == 1);
    }

    return ok;
}

int generate_prime()
{
    int generated = rand() % LIMIT;
    while (!rabin_miller(generated))
        generated = rand() % LIMIT;
    return generated;
}

int gcd(int a, int b)
{
    while (b)
    {
        int r = a % b;
        a = b;
        b = r;
    }
    return a;
}

int generate_coprime(int n)
{
    int generated = rand() % LIMIT;
    while (gcd(n, generated) != 1)
        generated = rand() % LIMIT;
    return generated;
}

pair<int, int> euclid_extended(int a, int b) {
    if(!b) {
        return {1, 0};
    }

    auto result = euclid_extended(b, a % b);
    return { result.second, result.first - (a / b) * result.second };
}

int modular_inverse(int n, int mod)
{
    int inverse = euclid_extended(n, mod).first;
    while(inverse < 0)
        inverse += mod;
    return inverse;
}


Keys generate_keys()
{
    Keys result;

    int p, q;

    p = generate_prime();
    q = generate_prime();

    int n = p * q;
    int phi = (p -1) * (q - 1);
    int e = generate_coprime(phi);

    result.public_key =  make_pair(n, e);

    int d = modular_inverse(e, phi);

    result.private_key = make_pair(n, d);

    return result;
}



inline string RSA_encrypt_array(string message, vector<int>& enc, int pub1, int pub2)
{
    int len = (int)message.length();
    string estr = "U32 estring [] = {";
    enc.resize(len);
    for (int i = 0; i < len; i++)
    {
        enc[i] = log_power(message[i], pub2, pub1);
        estr += FormatString("0x%x, ", enc[i]);
    }

    U32 fcrc = enc[enc.size()-1];
    for (auto c : message)
        fcrc += c;
    enc.push_back(fcrc);
    estr += FormatString("0x%x, ", enc[enc.size()-1]);
    estr += "};";
    return estr;
}

inline string RSA_encrypt_hexstr(string message, vector<int>& enc, int pub1, int pub2)
{
    int len = (int)message.length();
    string estr = "\"";
    enc.resize(len);
    for (int i = 0; i < len; i++)
    {
        enc[i] = log_power(message[i], pub2, pub1);
    }
    U32 fcrc = enc[enc.size()-1];
    for (auto c : message)
        fcrc += c;
    enc.push_back(fcrc);
    estr += toHex((void*)&enc[0], enc.size() * 4).c_str();
    estr += "\"";
    return estr;
}

string RSA_decrypt(vector<int>& enc, int priv1, int priv2)
{
    string dstr = "";
    int dec;
    for (int i = 0; i < enc.size(); i++)
    {
        dec = log_power(enc[i], priv2, priv1);
        dstr += (char)dec;
    }
    dstr += "\0";
    return dstr;
}


void TestRSA2()
{
    srand((U32)time((time_t*)NULL));
    Keys keys;
    vector<int> dec;

    const bool GENERATE_KEY = false;
    if (GENERATE_KEY)
    {
        keys = generate_keys();
        PrintOut("Public key: %d-%d \n", keys.public_key.first, keys.public_key.second);
        PrintOut("Private key: %d-%d \n", keys.private_key.first, keys.private_key.second);
    }
    else
    {
        keys.public_key.first = 11580379;
        keys.public_key.second = 4189;
        keys.private_key.first = 11580379;
        keys.private_key.second = 10249909;
        //hashplaza.org\t3338 523057-58.0.ri        
    }

    string estr;
    string message;
    vector<int> enc;
    strings stringList = { 
        "AbcDef1234567891+:)_/lp[q'a;sproitjvmx.a;p!@#$%^&*()_~?><Z||}{\tz",
        "hashplaza.org\t3336",
        "hashplaza.org\t3337",                            
        "hashplaza.org\t3338",
        "mine.pool.pascalpool.org\t3336",
        "mine.pool.pascalpool.org\t3337",
        "mine.pool.pascalpool.org\t3338",       
        "fastpool.xyz\t10097",
        "fastpool.xyz\t10098",       
        "1300745-41.0.0",
        "1301415-71.0.1",
        "1308045-65.0.2",
        "1312255-33.0.3",
        "Kernel32.dll",
        "ntdll.dll",
        "NtQueryInformationProcess",
        "IsDebuggerPresent",
        "CheckRemoteDebuggerPresent",
    };
    /*
        //AbcDef1234567891+:)_/lp[q'a;sproitjvmx.a;p!@#$%^&*()_~?><Z||}{	z 0x8B015F
        U32 estring [] = {0x600be7, 0x4f8f4f, 0x37c525, 0x8543b9, 0x8a49db, 0x89efe9, 0x1b12f9, 0x5d6c22, 0x123542, 0x71f216, 0x66e0fa, 0x5721f5, 0x275fcb, 0x8120c0, 0x1aa42f, 0x1b12f9, 0x5f37c3, 0x596b0, 0x6df78d, 0x1c177f, 0xa76696, 0x25963a, 0xae9b31, 0x345ac0, 0x76187, 0x86dffb, 0x6c24b9, 0x9866df, 0x21b0e5, 0xae9b31, 0xaff517, 0x2552c3, 0x206787, 0xa9f679, 0xaebc0f, 0x8ebad4, 0x4fef7a, 0x28e7b9, 0x995282, 0x6c24b9, 0x9866df, 0xae9b31, 0x333b5f, 0x993c31, 0x8fbaf9, 0xaccb13, 0x9b979c, 0x3183a7, 0x315347, 0x9cfb4f, 0x7a2f9e, 0x6df78d, 0x1c177f, 0x72a74c, 0x60f91d, 0xa0e468, 0x792b81, 0x5d23f7, 0x56e542, 0x56e542, 0xabc47e, 0x7f1bfc, 0x90ff63, 0x8aede6, 0x8b015f, };
        //hashplaza.org	3336 0x5727FF
        U32 estring [] = {0x3fcde, 0x6c24b9, 0x21b0e5, 0x3fcde, 0xae9b31, 0x25963a, 0x6c24b9, 0x8aede6, 0x6c24b9, 0x995282, 0x2552c3, 0xaff517, 0x41af04, 0x90ff63, 0x123542, 0x123542, 0x123542, 0x5721f5, 0x5727ff, };
        //hashplaza.org	3337 0x2765D6
        U32 estring [] = {0x3fcde, 0x6c24b9, 0x21b0e5, 0x3fcde, 0xae9b31, 0x25963a, 0x6c24b9, 0x8aede6, 0x6c24b9, 0x995282, 0x2552c3, 0xaff517, 0x41af04, 0x90ff63, 0x123542, 0x123542, 0x123542, 0x275fcb, 0x2765d6, };
        //hashplaza.org	3338 0x8126CC
        U32 estring [] = {0x3fcde, 0x6c24b9, 0x21b0e5, 0x3fcde, 0xae9b31, 0x25963a, 0x6c24b9, 0x8aede6, 0x6c24b9, 0x995282, 0x2552c3, 0xaff517, 0x41af04, 0x90ff63, 0x123542, 0x123542, 0x123542, 0x8120c0, 0x8126cc, };
        //mine.pool.pascalpool.org	3336 0x572C30
        U32 estring [] = {0x4fef7a, 0x206787, 0x224b72, 0x8a49db, 0x995282, 0xae9b31, 0x2552c3, 0x2552c3, 0x25963a, 0x995282, 0xae9b31, 0x6c24b9, 0x21b0e5, 0x37c525, 0x6c24b9, 0x25963a, 0xae9b31, 0x2552c3, 0x2552c3, 0x25963a, 0x995282, 0x2552c3, 0xaff517, 0x41af04, 0x90ff63, 0x123542, 0x123542, 0x123542, 0x5721f5, 0x572c30, };
        //mine.pool.pascalpool.org	3337 0x276A07
        U32 estring [] = {0x4fef7a, 0x206787, 0x224b72, 0x8a49db, 0x995282, 0xae9b31, 0x2552c3, 0x2552c3, 0x25963a, 0x995282, 0xae9b31, 0x6c24b9, 0x21b0e5, 0x37c525, 0x6c24b9, 0x25963a, 0xae9b31, 0x2552c3, 0x2552c3, 0x25963a, 0x995282, 0x2552c3, 0xaff517, 0x41af04, 0x90ff63, 0x123542, 0x123542, 0x123542, 0x275fcb, 0x276a07, };
        //mine.pool.pascalpool.org	3338 0x812AFD
        U32 estring [] = {0x4fef7a, 0x206787, 0x224b72, 0x8a49db, 0x995282, 0xae9b31, 0x2552c3, 0x2552c3, 0x25963a, 0x995282, 0xae9b31, 0x6c24b9, 0x21b0e5, 0x37c525, 0x6c24b9, 0x25963a, 0xae9b31, 0x2552c3, 0x2552c3, 0x25963a, 0x995282, 0x2552c3, 0xaff517, 0x41af04, 0x90ff63, 0x123542, 0x123542, 0x123542, 0x8120c0, 0x812afd, };
        //fastpool.xyz	10097 0x2765D6
        U32 estring [] = {0x89efe9, 0x6c24b9, 0x21b0e5, 0xa9f679, 0xae9b31, 0x2552c3, 0x2552c3, 0x25963a, 0x995282, 0x28e7b9, 0x8b8942, 0x8aede6, 0x90ff63, 0x1b12f9, 0xae72d4, 0xae72d4, 0x1aa42f, 0x275fcb, 0x2765d6, };
        //fastpool.xyz	10098 0x8126CC
        U32 estring [] = {0x89efe9, 0x6c24b9, 0x21b0e5, 0xa9f679, 0xae9b31, 0x2552c3, 0x2552c3, 0x25963a, 0x995282, 0x28e7b9, 0x8b8942, 0x8aede6, 0x90ff63, 0x1b12f9, 0xae72d4, 0xae72d4, 0x1aa42f, 0x8120c0, 0x8126cc, };
        //1300745-41.0.0 0xAE7586
        U32 estring [] = {0x1b12f9, 0x123542, 0xae72d4, 0xae72d4, 0x275fcb, 0x71f216, 0x66e0fa, 0x863f0b, 0x71f216, 0x1b12f9, 0x995282, 0xae72d4, 0x995282, 0xae72d4, 0xae7586, };
        //1301415-71.0.1 0x1B15AA
        U32 estring [] = {0x1b12f9, 0x123542, 0xae72d4, 0x1b12f9, 0x71f216, 0x1b12f9, 0x66e0fa, 0x863f0b, 0x275fcb, 0x1b12f9, 0x995282, 0xae72d4, 0x995282, 0x1b12f9, 0x1b15aa, };
        //1308045-65.0.2 0x5D6EDD
        U32 estring [] = {0x1b12f9, 0x123542, 0xae72d4, 0x8120c0, 0xae72d4, 0x71f216, 0x66e0fa, 0x863f0b, 0x5721f5, 0x66e0fa, 0x995282, 0xae72d4, 0x995282, 0x5d6c22, 0x5d6edd, };
        //1312255-33.0.3 0x1237F7
        U32 estring [] = {0x1b12f9, 0x123542, 0x1b12f9, 0x5d6c22, 0x5d6c22, 0x66e0fa, 0x66e0fa, 0x863f0b, 0x123542, 0x123542, 0x995282, 0xae72d4, 0x995282, 0x123542, 0x1237f7, };
        //Kernel32.dll 0x259A6A
        U32 estring [] = {0x7a03ea, 0x8a49db, 0xaff517, 0x224b72, 0x8a49db, 0x25963a, 0x123542, 0x5d6c22, 0x995282, 0x620dcd, 0x25963a, 0x25963a, 0x259a6a, };
        //ntdll.dll 0x2599C2
        U32 estring [] = {0x224b72, 0xa9f679, 0x620dcd, 0x25963a, 0x25963a, 0x995282, 0x620dcd, 0x25963a, 0x25963a, 0x2599c2, };
        //NtQueryInformationProcess 0x21BB22
        U32 estring [] = {0x584498, 0xa9f679, 0x354f61, 0x9bd359, 0x8a49db, 0xaff517, 0x8b8942, 0x209d9, 0x224b72, 0x89efe9, 0x2552c3, 0xaff517, 0x4fef7a, 0x6c24b9, 0xa9f679, 0x206787, 0x2552c3, 0x224b72, 0xac3c16, 0xaff517, 0x2552c3, 0x37c525, 0x8a49db, 0x21b0e5, 0x21b0e5, 0x21bb22, };
        //IsDebuggerPresent 0xA9FD3B
        U32 estring [] = {0x209d9, 0x21b0e5, 0x8543b9, 0x8a49db, 0x4f8f4f, 0x9bd359, 0x41af04, 0x41af04, 0x8a49db, 0xaff517, 0xac3c16, 0xaff517, 0x8a49db, 0x21b0e5, 0x8a49db, 0x224b72, 0xa9f679, 0xa9fd3b, };
        //CheckRemoteDebuggerPresent 0xAA00C9
        U32 estring [] = {0x858cfe, 0x3fcde, 0x8a49db, 0x37c525, 0x8d963e, 0x7bab60, 0x8a49db, 0x4fef7a, 0x2552c3, 0xa9f679, 0x8a49db, 0x8543b9, 0x8a49db, 0x4f8f4f, 0x9bd359, 0x41af04, 0x41af04, 0x8a49db, 0xaff517, 0xac3c16, 0xaff517, 0x8a49db, 0x21b0e5, 0x8a49db, 0x224b72, 0xa9f679, 0xaa00c9, };
        Log   263602351  DEcrypted string : 'AbcDef1234567891+:)_/lp[q'a;sproitjvmx.a;p!@#$%^&*()_~?><Z||}{	z' 
        //AbcDef1234567891+:)_/lp[q'a;sproitjvmx.a;p!@#$%^&*()_~?><Z||}{	z 0x8B015F
        "e70b60004f8f4f0025c53700b9438500db498a00e9ef8900f9121b00226c5d004235120016f27100fae06600f5215700cb5f2700c02081002fa41a00f9121b00c3375f00b09605008df76d007f171c009666a7003a962500319bae00c05a340087610700fbdf8600b9246c00df669800e5b02100319bae0017f5af00c35225008767200079f6a9000fbcae00d4ba8e007aef4f00b9e7280082529900b9246c00df669800319bae005f3b3300313c9900f9ba8f0013cbac009c979b00a7833100475331004ffb9c009e2f7a008df76d007f171c004ca772001df9600068e4a000812b7900f7235d0042e5560042e556007ec4ab00fc1b7f0063ff9000e6ed8a005f018b00"
    */
    
    for (auto s : stringList)
    {
        estr = RSA_encrypt_array(s, enc, 11580379, 4189);
        PrintOut("~//%s 0x%X\n%s\n", s.c_str(), enc[enc.size()-1], estr.c_str());
    }
    

    message = "AbcDef1234567891+:)_/lp[q'a;sproitjvmx.a;p!@#$%^&*()_~?><Z||}{\tz"; //pub = 11580379-4189
    //AbcDef1234567891+:)_/lp[q'a;sproitjvmx.a;p!@#$%^&*()_~?><Z||}{	z 0x8B015F
    vector<int> testEnc = {0x600be7, 0x4f8f4f, 0x37c525, 0x8543b9, 0x8a49db, 0x89efe9, 0x1b12f9, 0x5d6c22, 0x123542, 0x71f216, 0x66e0fa, 0x5721f5, 0x275fcb, 0x8120c0, 0x1aa42f, 0x1b12f9, 0x5f37c3, 0x596b0, 0x6df78d, 0x1c177f, 0xa76696, 0x25963a, 0xae9b31, 0x345ac0, 0x76187, 0x86dffb, 0x6c24b9, 0x9866df, 0x21b0e5, 0xae9b31, 0xaff517, 0x2552c3, 0x206787, 0xa9f679, 0xaebc0f, 0x8ebad4, 0x4fef7a, 0x28e7b9, 0x995282, 0x6c24b9, 0x9866df, 0xae9b31, 0x333b5f, 0x993c31, 0x8fbaf9, 0xaccb13, 0x9b979c, 0x3183a7, 0x315347, 0x9cfb4f, 0x7a2f9e, 0x6df78d, 0x1c177f, 0x72a74c, 0x60f91d, 0xa0e468, 0x792b81, 0x5d23f7, 0x56e542, 0x56e542, 0xabc47e, 0x7f1bfc, 0x90ff63, 0x8aede6, 0x8b015f, };    
    string dstr;
    RH_DECRYPT(testEnc, dstr);

    PrintOut("DEcrypted string : '%s' \n", dstr.c_str());
    RH_ASSERT(strncmp(message.c_str(), dstr.c_str(), dstr.length()) == 0);

    testEnc.clear();
    estr = RSA_encrypt_hexstr(stringList[0], enc, 11580379, 4189);
    PrintOut("~//%s 0x%X\n%s\n", stringList[0].c_str(), enc[enc.size()-1], estr.c_str());

    string testEnvH = "e70b60004f8f4f0025c53700b9438500db498a00e9ef8900f9121b00226c5d004235120016f27100fae06600f5215700cb5f2700c02081002fa41a00f9121b00c3375f00b09605008df76d007f171c009666a7003a962500319bae00c05a340087610700fbdf8600b9246c00df669800e5b02100319bae0017f5af00c35225008767200079f6a9000fbcae00d4ba8e007aef4f00b9e7280082529900b9246c00df669800319bae005f3b3300313c9900f9ba8f0013cbac009c979b00a7833100475331004ffb9c009e2f7a008df76d007f171c004ca772001df9600068e4a000812b7900f7235d0042e5560042e556007ec4ab00fc1b7f0063ff9000e6ed8a005f018b00"; 
    bytes testEncB = fromHex(testEnvH);
    testEnc.resize(testEncB.size() / 4);
    memcpy(&testEnc[0], &testEncB[0], testEncB.size());

    RH_DECRYPT(testEnc, dstr);
    PrintOut("DEcrypted string : '%s' \n", dstr.c_str());
    RH_ASSERT(strncmp(message.c_str(), dstr.c_str(), dstr.length()) == 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
#define DECODE(encoded, decoded) \
{ \
        for (int i = 0; i < encoded.size(); i++) \
        { \
            U32 c = (encoded[i] >> 12) & 0xff; \
            c = ((ROTR8((U8)c, 3 + i))) & 0xff; \
            if (c > 96) \
                c -= 96; \
            else \
                c += 32; \
            decoded += (char)(U8)c; \
        } \
        decoded = ToLower(decoded); \
    }

void TestScrambling()
{
    string MESSAGE = "hashplaza.org\t3338 523057-58.0.rig";
    vector<U32> encoded;
    string decoded;
    string _hr = "{";
    //scramble
    {
        string message = ToUpper(MESSAGE);
        encoded.resize(message.length());
        for (int i = 0; i < message.length(); i++)
        {
            U32 c = message[i];
            if (c > 96)
                c = 32;
            else if (c < 32)
                c = c + 96;
            else
                c = c - 32;
            U8 v8 = ROTL8((U8)c, 3 + i);
            c = ((U32)(v8)) << 12;
            encoded[i] = rand32();
            encoded[i] = encoded[i] & ~0x000FF000;
            encoded[i] |= c;

            _hr += FormatString("0x%x, ", encoded[i]);
        }
        _hr += "};";
    }
    PrintOut("Encoded '%s' : %s\n", MESSAGE.c_str(), _hr.c_str());
    
    DECODE(encoded, decoded);
    PrintOut("Decoded '%s'\n", decoded.c_str());
    RH_ASSERT(strncmp(decoded.c_str(), MESSAGE.c_str(), MESSAGE.length()) == 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

struct HashTestValues
{
    U32 number;
    string name;
    string input;
    string expected;
};


struct TRandomHash2TestCAche
{
    U32 number;
    string name;
    string input;
    string expected;
    vector< std::pair<string, string> > subExpected;
    //vector< std::pair<string, string> > partially;
};


#include "RandomHash_DEV_UNITTEST_algo.test.values.h"

void TestRandomHash2Cache()
{
#pragma message(">~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ RandomHash_DEV_UNITTEST : TEST THAT !!!")
    return;


    RandomHash_State* randomHashArray = 0;
    RandomHash_CreateMany(&randomHashArray, 1);
    RandomHashResult _output2;

    string lastOutput;
    for (auto algo : g_rh2_fast_testvalues)
    {
        if (lastOutput.length())
        {
            string aaa = algo.input.substr(2);
            RHMINER_ASSERT(strcmp(lastOutput.c_str(), aaa.c_str()) == 0);
        }

        bytes testVal;
        if (algo.input.substr(0, 2) == "0x")
            testVal = fromHex(algo.input);
        else
        {
            testVal.resize(algo.input.length());
            memset(&testVal[0], 0, algo.input.length());
            memcpy(&testVal[0], &algo.input[0], algo.input.length());
        }

        string testExpected = algo.expected.substr(2);
        U32 testSize = testVal.size();

        memset(&_output2, 0, sizeof(_output2));
        U8 input2[PascalHeaderSizeV5];
        memcpy(input2, &testVal[0], testSize);
        RHMINER_ASSERT(testSize >= 0);
        
        RandomHash_SetHeader(&randomHashArray[0], &input2[0], testSize, 0);
        U32 noncep = *(U32*)&input2[testSize - 4];
        RandomHash_Search(&randomHashArray[0], _output2, noncep);

        string hexOutput = ToUpper(toHex((void*)_output2.hashes[0], 32));
        PrintOut("\n   %s\n== %s\n", hexOutput.c_str(), testExpected.c_str());
        RHMINER_ASSERT(memcmp(hexOutput.c_str(), testExpected.c_str(), 32) == 0);
    
        lastOutput = hexOutput;

        //Test every outhash value is there
        RH_ASSERT(algo.subExpected.size() == _output2.count - 1);
        for (U32 i = 0; i < algo.subExpected.size(); i++)
        {
            bytes tmpInput = fromHex(algo.subExpected[i].first);
            U32 tmpInputNonce = *(U32*)&tmpInput[tmpInput.size() - 4];

            testVal = fromHex(algo.subExpected[i].second);
            U32 ssize = testVal.size();
            RH_ASSERT(ssize == 32);

            //find it in the list (not same ordered than PAS !!!)
            S32 fndHashe = -1;
            for (int h = 1; h < _output2.count; h++)
            {
                if (memcmp(_output2.hashes[h], &testVal[0], 32) == 0)
                {
                    fndHashe = h;
                    break;
                }                
            }
            RHMINER_ASSERT(fndHashe != -1);
            RHMINER_ASSERT(tmpInputNonce == _output2.nonces[fndHashe]);

            hexOutput = toHex((void*)_output2.hashes[fndHashe], 32);
            PrintOut("\nCache: %s\n    == %s\n", hexOutput.c_str(), toHex(&testVal[0], 32).c_str());
            RHMINER_ASSERT(memcmp(_output2.hashes[fndHashe], &testVal[0], 32) == 0);
        }

        //loop every test value seperately

        for (auto subVal : algo.subExpected)
        {
            testVal = fromHex(subVal.first);
            U32 ssize = testVal.size();
            noncep = *(U32*)&testVal[ssize - 4];
            RandomHash_SetHeader(&randomHashArray[0], &testVal[0], ssize, 0);

            RandomHashResult _output3;
            memset(_output3.hashes[0], 0, sizeof(_output3.hashes[0]));
            RandomHash_Search(&randomHashArray[0], _output3, noncep);
            string hexOutput = ToUpper(toHex((void*)_output3.hashes[0], 32));
            PrintOut("\n       %s\n    == %s\n", hexOutput.c_str(), subVal.second.substr(2).c_str());

            RHMINER_ASSERT(memcmp(hexOutput.c_str(), subVal.second.substr(2).c_str(), 32) == 0);
        }

        //loop every test value seperately
        for (auto subVal : algo.subExpected)
        {
            testVal = fromHex(subVal.first);
            U32 ssize = testVal.size();
            noncep = *(U32*)&testVal[ssize - 4];
            RandomHash_SetHeader(&randomHashArray[0], &testVal[0], ssize, 0);

            RH_ASSERT(ssize <= PascalHeaderSizeV5);
            RH_STRIDE_SET_SIZE(randomHashArray->m_roundInput, ssize);
            memcpy(RH_STRIDE_GET_DATA(randomHashArray->m_roundInput), &testVal[0], ssize);

            RandomHashResult _output3;
            memset(_output3.hashes[0], 0, sizeof(_output3.hashes[0]));
            RandomHash_Search(&randomHashArray[0], _output3, noncep);
            string hexOutput = ToUpper(toHex((void*)_output3.hashes[0], 32));
            PrintOut("\n       %s\n    == %s\n", hexOutput.c_str(), subVal.second.substr(2).c_str());

            RHMINER_ASSERT(memcmp(hexOutput.c_str(), subVal.second.substr(2).c_str(), 32) == 0);
        }

    }
    RandomHash_DestroyMany(randomHashArray, 1);
}

void TestSHA_Intrinsic()
{
    /*
    bytes input_ORI = fromHex("4f550200ca022000bb718b4b00d6f74478c332f5fb310507e55a9ef9b38551f63858e3f7c86dbd00200006f69afae8a6b0735b6acfcc58b7865fc8418897c530211f19140c9f95f24532102700000000000003000300a297fd17506f6c796d696e65722e506f6c796d696e65722e506f6c796d6939303030303030302184d63666eb166619e925cef2a306549bbc4d6f4da3bdf28b4393d5c1856f0ee3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855000000006d68295b00000000");
    bytes input;
    input.resize(input_ORI.size());
    bytes _stride1, _stride2;
    const U32 _BUF_SIZE = 4006;
    _stride1.resize(_BUF_SIZE);
    _stride2.resize(_BUF_SIZE);
    U32* in_stride = (U32*)(void*)&_stride1[0];
    U32* out_stride = (U32*)(void*)&_stride2[0];

    vector<size_t> _in = { input_ORI.size(), input_ORI.size() - 32, input_ORI.size() - 48, input_ORI.size() - 64 };
    //////////////////////////for (int i = 0; i < 4; i++)
    for (int i = 0; i < 1; i++)
    {
        memcpy(&input[0], &input_ORI[0], input_ORI.size());
        *((U32*)&input[input.size() - 4]) = 0x8888AAAA + i;

        memset(in_stride, 0, _in[i]);
        RH_STRIDE_SET_SIZE(in_stride, input.size());
        memcpy(RH_STRIDE_GET_DATA(in_stride), &input[0], input.size());

        //if (testSize != PascalHeaderSize && testSize != PascalHeaderSizeV5)
        RandomHash_SHA2_256(in_stride, out_stride, true);
        DebugOut("Normal RandomHash_SHA2_224 %s \n",toHex((void*)RH_STRIDE_GET_DATA(out_stride), RH_STRIDE_GET_SIZE(out_stride)).c_str());
        RandomHash_SHA2_256(in_stride, out_stride, false);
        DebugOut("Normal RandomHash_SHA2_256 %s \n", toHex((void*)RH_STRIDE_GET_DATA(out_stride), RH_STRIDE_GET_SIZE(out_stride)).c_str());
    }
    */
}

void TestRandomHash2()
{
    //bytes header = fromHex("4f550200ca022000bb718b4b00d6f74478c332f5fb310507e55a9ef9b38551f63858e3f7c86dbd00200006f69afae8a6b0735b6acfcc58b7865fc8418897c530211f19140c9f95f24532102700000000000003000300a297fd17506f6c796d696e65722e506f6c796d696e65722e506f6c796d6939303030303030302184d63666eb166619e925cef2a306549bbc4d6f4da3bdf28b4393d5c1856f0ee3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855000000006d68295b00000000");
    U32 sizes[] = { 17,31,32,33,34,63,64,65,100,117,127,128,129,178,199,200 };
    bytes _stride1, _stride2;
    const U32 _BUF_SIZE = 4006;
    _stride1.resize(_BUF_SIZE);
    _stride2.resize(_BUF_SIZE);
    U32* in_stride = (U32*)(void*)&_stride1[0];
    U32* out_stride = (U32*)(void*)&_stride1[0];
    RandomHash_State* randomHashArray = 0;
    RandomHash_CreateMany(&randomHashArray, 1);

    for (auto algo : g_testvalues)
    {
        bytes testVal;
        if (algo.input.substr(0, 2) == "0x")
            testVal = fromHex(algo.input);
        else
        {
            testVal.resize(algo.input.length());
            memset(&testVal[0], 0, algo.input.length());
            memcpy(&testVal[0], &algo.input[0], algo.input.length());
        }

        string testExpected = algo.expected.substr(2);
        U32 testSize = testVal.size();

        if (algo.name == "RandomHash2")
        {
            if (testSize >= 0 && testSize <= PascalHeaderSizeV5)
            {
                RandomHashResult _output2;
                U8 input2[PascalHeaderSizeV5];
                memcpy(input2, &testVal[0], testSize);
                RHMINER_ASSERT(testSize >= PascalHeaderSize);

                RandomHash_SetHeader(&randomHashArray[0], &input2[0], testSize, 0);
                U32 noncep = *(U32*)&input2[testSize - 4];
                RandomHash_Search(&randomHashArray[0], _output2, noncep);
                string hexOutput = ToUpper(toHex((void*)_output2.hashes[0], 32));
                PrintOut("\n   %s\n== %s\n", hexOutput.c_str(), testExpected.c_str());
                RHMINER_ASSERT(memcmp(hexOutput.c_str(), testExpected.c_str(), 32) == 0);
            }
        }
        else
        {
            memset(in_stride, 0, _BUF_SIZE);
            memset(out_stride, 0, _BUF_SIZE);
            RH_STRIDE_RESET(in_stride);
            RH_STRIDE_RESET(out_stride);

            RH_STRIDE_SET_SIZE(in_stride, testSize);
            memcpy(RH_STRIDE_GET_DATA(in_stride), &testVal[0], testSize);

            if (algo.name == "Ghost")
            {
                RandomHash_Ghost(in_stride, out_stride);
            }
            else if (algo.name == "GHOST3411_2012_256")
            {
                RandomHash_Ghost3411(in_stride, out_stride, 256);
            }
            else if (algo.name == "GHOST3411_2012_512")
            {
                RandomHash_Ghost3411(in_stride, out_stride, 512);
            }
            else if (algo.name == "MD2")
            {
                if (testSize == 100 || testSize == 32)
                    RandomHash_MD2(in_stride, out_stride);
                else
                    continue;
            }
            else if (algo.name == "MD4")
            {
                RandomHash_MD4(in_stride, out_stride);
            }
            else if (algo.name == "MD5")
            {
                RandomHash_MD5(in_stride, out_stride);
            }
            else if (algo.name == "Panama")
            {
                RandomHash_Panama(in_stride, out_stride);
            }
            else if (algo.name == "HAS160")
            {
                RandomHash_HAS160(in_stride, out_stride);
            }
            else if (algo.name == "SHA0")
            {
                RandomHash_SHA0(in_stride, out_stride);
            }
            else if (algo.name == "SHA1")
            {
                RandomHash_SHA1(in_stride, out_stride);
            }
            else if (algo.name == "Blake2B_512")
            {
                RandomHash_blake2b(in_stride, out_stride, 512);
            }
            else if (algo.name == "Blake2B_256")
            {
                RandomHash_blake2b(in_stride, out_stride, 256);
            }
            else if (algo.name == "Blake2B_160")
            {
                RandomHash_blake2b(in_stride, out_stride, 160);
            }
            else if (algo.name == "Blake2B_384")
            {
                RandomHash_blake2b(in_stride, out_stride, 384);
            }
            else if (algo.name == "Blake2S_128")
            {
                RandomHash_blake2s(in_stride, out_stride, 128);
            }
            else if (algo.name == "Blake2S_160")
            {
                RandomHash_blake2s(in_stride, out_stride, 160);
            }
            else if (algo.name == "Blake2S_224")
            {
                RandomHash_blake2s(in_stride, out_stride, 224);
            }
            else if (algo.name == "Blake2S_256")
            {
                RandomHash_blake2s(in_stride, out_stride, 256);
            }
            else if (algo.name == "Grindahl256")
            {
                RandomHash_Grindahl256(in_stride, out_stride);
            }
            else if (algo.name == "Grindahl512")
            {
                RandomHash_Grindahl512(in_stride, out_stride);
            }
            else if (algo.name == "Haval_3_128")
            {
                RandomHash_Haval3(in_stride, out_stride, 128);
            }
            else if (algo.name == "Haval_3_160")
            {
                RandomHash_Haval3(in_stride, out_stride, 160);
            }
            else if (algo.name == "Haval_3_192")
            {
                RandomHash_Haval3(in_stride, out_stride, 192);
            }
            else if (algo.name == "Haval_3_224")
            {
                RandomHash_Haval3(in_stride, out_stride, 224);
            }
            else if (algo.name == "Haval_3_256")
            {
                RandomHash_Haval3(in_stride, out_stride, 256);
            }
            else if (algo.name == "Haval_4_128")
            {
                RandomHash_Haval4(in_stride, out_stride, 128);
            }
            else if (algo.name == "Haval_4_160")
            {
                RandomHash_Haval4(in_stride, out_stride, 160);
            }
            else if (algo.name == "Haval_4_192")
            {
                RandomHash_Haval4(in_stride, out_stride, 192);
            }
            else if (algo.name == "Haval_4_224")
            {
                RandomHash_Haval4(in_stride, out_stride, 224);
            }
            else if (algo.name == "Haval_4_256")
            {
                RandomHash_Haval4(in_stride, out_stride, 256);
            }
            else if (algo.name == "Haval_5_128")
            {
                RandomHash_Haval_5_256(in_stride, out_stride, 128);
            }
            else if (algo.name == "Haval_5_160")
            {
                RandomHash_Haval_5_256(in_stride, out_stride, 160);
            }
            else if (algo.name == "Haval_5_192")
            {
                RandomHash_Haval_5_256(in_stride, out_stride, 192);
            }
            else if (algo.name == "Haval_5_224")
            {
                RandomHash_Haval_5_256(in_stride, out_stride, 224);
            }
            else if (algo.name == "Haval_5_256")
            {
                RandomHash_Haval_5_256(in_stride, out_stride, 256);
            }
            else if (algo.name == "Keccak_224")
            {
                _RandomHash_SHA3_512(in_stride, out_stride, 28, false);
            }
            else if (algo.name == "Keccak_256")
            {
                _RandomHash_SHA3_512(in_stride, out_stride, 32, false);
            }
            else if (algo.name == "Keccak_288")
            {
                _RandomHash_SHA3_512(in_stride, out_stride, 36, false);
            }
            else if (algo.name == "Keccak_384")
            {
                _RandomHash_SHA3_512(in_stride, out_stride, 48, false);
            }
            else if (algo.name == "Keccak_512")
            {
                _RandomHash_SHA3_512(in_stride, out_stride, 64, false);
            }
            else if (algo.name == "RadioGatun32")
            {
                RandomHash_RadioGatun32(in_stride, out_stride);
            }
            else if (algo.name == "RIPEMD")
            {
                RandomHash_RIPEMD(in_stride, out_stride);
            }
            else if (algo.name == "RIPEMD128")
            {
                RandomHash_RIPEMD128(in_stride, out_stride);
            }
            else if (algo.name == "RIPEMD160")
            {
                RandomHash_RIPEMD160(in_stride, out_stride);
            }
            else if (algo.name == "RIPEMD256")
            {
                RandomHash_RIPEMD256(in_stride, out_stride);
            }
            else if (algo.name == "RIPEMD320")
            {
                RandomHash_RIPEMD320(in_stride, out_stride);
            }
            else if (algo.name == "SHA2_224")
            {
                if (testSize != PascalHeaderSize && testSize != PascalHeaderSizeV5)
                    continue;
                RandomHash_SHA2_256(in_stride, out_stride, true);
            }
            else if (algo.name == "SHA2_256")
            {
                if (testSize != PascalHeaderSize && testSize != PascalHeaderSizeV5)
                    continue;
                RandomHash_SHA2_256(in_stride, out_stride, false);
            }
            else if (algo.name == "SHA2_384")
            {
                RandomHash_SHA2_512(in_stride, out_stride, SHA2_512_MODE_384);
            }
            else if (algo.name == "SHA2_512")
            {
                RandomHash_SHA2_512(in_stride, out_stride, SHA2_512_MODE_512);
            }
            else if (algo.name == "SHA2_512_224")
            {
                RandomHash_SHA2_512(in_stride, out_stride, SHA2_512_MODE_512_224);
            }
            else if (algo.name == "SHA2_512_256")
            {
                RandomHash_SHA2_512(in_stride, out_stride, SHA2_512_MODE_512_256);
            }
            else if (algo.name == "SHA3_224")
            {
                RandomHash_SHA3_224(in_stride, out_stride);
            }
            else if (algo.name == "SHA3_256")
            {
                RandomHash_SHA3_256(in_stride, out_stride);
            }
            else if (algo.name == "SHA3_384")
            {
                RandomHash_SHA3_384(in_stride, out_stride);
            }
            else if (algo.name == "SHA3_512")
            {
                RandomHash_SHA3_512(in_stride, out_stride);
            }
            else if (algo.name == "Snefru_8_128")
            {
                RandomHash_Snefru_8_256(in_stride, out_stride, 16);
            }
            else if (algo.name == "Snefru_8_256")
            {
                RandomHash_Snefru_8_256(in_stride, out_stride, 32);
            }
            else if (algo.name == "Tiger_3_128")
            {
                RandomHash_Tiger(in_stride, out_stride, 3, 128, false);
            }
            else if (algo.name == "Tiger_3_160")
            {
                RandomHash_Tiger(in_stride, out_stride, 3, 160, false);
            }
            else if (algo.name == "Tiger_3_192")
            {
                RandomHash_Tiger(in_stride, out_stride, 3, 192, false);
            }
            else if (algo.name == "Tiger_4_128")
            {
                RandomHash_Tiger(in_stride, out_stride, 4, 128, false);
            }
            else if (algo.name == "Tiger_4_160")
            {
                RandomHash_Tiger(in_stride, out_stride, 4, 160, false);
            }
            else if (algo.name == "Tiger_4_192")
            {
                RandomHash_Tiger(in_stride, out_stride, 4, 192, false);
            }
            else if (algo.name == "Tiger_5_128")
            {
                RandomHash_Tiger(in_stride, out_stride, 5, 128, false);
            }
            else if (algo.name == "Tiger_5_160")
            {
                RandomHash_Tiger(in_stride, out_stride, 5, 160, false);
            }
            else if (algo.name == "Tiger_5_192")
            {
                RandomHash_Tiger(in_stride, out_stride, 5, 192, false);
            }
            else if (algo.name == "Tiger2_3_128")
            {
                RandomHash_Tiger(in_stride, out_stride, 3, 128, true);
            }
            else if (algo.name == "Tiger2_3_160")
            {
                RandomHash_Tiger(in_stride, out_stride, 3, 160, true);
            }
            else if (algo.name == "Tiger2_3_192")
            {
                RandomHash_Tiger(in_stride, out_stride, 3, 192, true);
            }
            else if (algo.name == "Tiger2_4_128")
            {
                RandomHash_Tiger(in_stride, out_stride, 4, 128, true);
            }
            else if (algo.name == "Tiger2_4_160")
            {
                RandomHash_Tiger(in_stride, out_stride, 4, 160, true);
            }
            else if (algo.name == "Tiger2_4_192")
            {
                RandomHash_Tiger(in_stride, out_stride, 4, 192, true);
            }
            else if (algo.name == "Tiger2_5_128")
            {
                RandomHash_Tiger(in_stride, out_stride, 5, 128, true);
            }
            else if (algo.name == "Tiger2_5_160")
            {
                RandomHash_Tiger(in_stride, out_stride, 5, 160, true);
            }
            else if (algo.name == "Tiger2_5_192")
            {
                RandomHash_Tiger(in_stride, out_stride, 5, 192, true);
            }
            else if (algo.name == "WhirlPool")
            {
                //PAS implementation is incorrect according to www.fileformat.info
                RandomHash_WhirlPool(in_stride, out_stride);
            }
            else
            {
                RHMINER_ASSERT(false);
            }

            PrintOut("Testing %s with %s\n", algo.name.c_str(), algo.input.c_str());
            string testRes = toHex((void*)RH_STRIDE_GET_DATA(out_stride), RH_STRIDE_GET_SIZE(out_stride));
            string expected = algo.expected.substr(2);
            if (__stricmp(testRes.c_str(), expected.c_str()) != 0)
            {
                PrintOut("TEST Fail with input %s\n", algo.input.c_str());
                PrintOut("VALUES : %s\n %s ==\n %s\n", algo.name.c_str(), ToUpper(testRes).c_str(), expected.c_str());
                RHMINER_ASSERT(0);
            }
        }
    }
    RandomHash_DestroyMany(randomHashArray, 1);
    /////////////////////////////////////////////////////////
    //Dump Stats
#ifdef RH_MAKE_STATS
    extern void PrintStats();
    PrintStats();
#endif
}

void TestRandomHash2Resume()
{
    extern void setThreadName(char const* _n);

    const U32 ThreadCount = 4;
    const U32 cRounds = 2000;
    //const U32 cRounds = 1000;

    RandomHash_State* randomHashArray = 0;
    RandomHash_CreateMany(&randomHashArray, ThreadCount);
    RandomHashResult _output2[ThreadCount];


    bytes input = fromHex(g_rh2_fast_testvalues[0].input);

    //set thread init vals
    for (U32 t = 0; t < ThreadCount; t++)
    {
        RandomHash_SetHeader(&randomHashArray[t], &input[0], input.size(), 0);
    }

    std::vector<std::thread> threads(ThreadCount);
    U32 gid=0;
    for(int i = 0; i < ThreadCount; i++) 
    {
        threads[i] = std::thread([&]
        {
            U32 _gid = AtomicIncrement(gid);
            char tname[64];
            snprintf(tname, 64, "Cpu%d", (int)_gid-1);
            setThreadName(tname);            
            RH_SetThreadPriority(RH_ThreadPrio_High);
            CpuSleep(20);

            U32 hash = (_gid - 1) * 100000;
            PrintOut("[%d] Mining %d round for thread \n", _gid-1, cRounds);
            for (int _i = 0; _i < cRounds; _i++)
            {
                RandomHash_Search(&randomHashArray[_gid-1], _output2[_gid-1], hash);
                hash += _output2[_gid-1].count;
                if ((_i % 500) == 0)
                    PrintOut("[%d] rounds %d hash %u\n", _gid-1, _i, hash);
            }
        }
        );
        CpuSleep(100);
    }

    //wait for all
    for(std::thread & thread : threads) 
        thread.join();

    RandomHash_DestroyMany(randomHashArray, ThreadCount);
}

#ifdef _WIN32_WINNT
#include "MinersLib/RandomHash/RandomHash_DEV_UNITTEST_SHA2_256_SSE.h"
#endif

void TestRandomHash2RoundsSeq()
{
    bytes input_ORI = fromHex(g_testvalues[1].input);
    bytes input;
    input.resize(input_ORI.size());

    const U32 cRounds = 40;
    RandomHash_State* randomHashArray = 0;
    RandomHash_CreateMany(&randomHashArray, 1);
    RandomHashResult _output2;
    struct TestRes 
    { 
        U32 gid;
        U32 cnt;
        vector<std::pair<U32, string>> nonces; 
    };
    TestRes testResult [] = {
        {2956630144, 3, {{0xb03a9880, "70b861adb518b1c9247b28dc9c353914c1303cc8700218a1eda262ac05995b05"},{0x81a312b2, "70b861adb518b1c9247b28dc9c353914c1303cc8700218a1eda262ac05995b05"},{0x888bb9cb, "70b861adb518b1c9247b28dc9c353914c1303cc8700218a1eda262ac05995b05"},}},
        {2956730144, 10, {{0xb03c1f20, "fc12ba9daf0976366c7247e42f34bfbf4bb0e703ac1c34f47ded6d973e3b12e9"},{0xbb5752d0, "fc12ba9daf0976366c7247e42f34bfbf4bb0e703ac1c34f47ded6d973e3b12e9"},{0xb375fd42, "fc12ba9daf0976366c7247e42f34bfbf4bb0e703ac1c34f47ded6d973e3b12e9"},{0x6a76ab9e, "fc12ba9daf0976366c7247e42f34bfbf4bb0e703ac1c34f47ded6d973e3b12e9"},{0x9de66188, "fc12ba9daf0976366c7247e42f34bfbf4bb0e703ac1c34f47ded6d973e3b12e9"},{0x2d4651a5, "fc12ba9daf0976366c7247e42f34bfbf4bb0e703ac1c34f47ded6d973e3b12e9"},{0x7badd8c4, "fc12ba9daf0976366c7247e42f34bfbf4bb0e703ac1c34f47ded6d973e3b12e9"},{0x5d70a80, "fc12ba9daf0976366c7247e42f34bfbf4bb0e703ac1c34f47ded6d973e3b12e9"},{0x750384d4, "fc12ba9daf0976366c7247e42f34bfbf4bb0e703ac1c34f47ded6d973e3b12e9"},{0xe2a49ff0, "fc12ba9daf0976366c7247e42f34bfbf4bb0e703ac1c34f47ded6d973e3b12e9"},}},
        {2956830144, 9, {{0x9c029955, "3177c35da29dca8478000ec143057c1585dc11afb2c8ae645e8c81b49f38ea4e"},{0x9d2433e1, "3177c35da29dca8478000ec143057c1585dc11afb2c8ae645e8c81b49f38ea4e"},{0xd649391a, "3177c35da29dca8478000ec143057c1585dc11afb2c8ae645e8c81b49f38ea4e"},{0x5629d8ae, "3177c35da29dca8478000ec143057c1585dc11afb2c8ae645e8c81b49f38ea4e"},{0x7ca70e5b, "3177c35da29dca8478000ec143057c1585dc11afb2c8ae645e8c81b49f38ea4e"},{0xdf094cac, "3177c35da29dca8478000ec143057c1585dc11afb2c8ae645e8c81b49f38ea4e"},{0xcee93895, "3177c35da29dca8478000ec143057c1585dc11afb2c8ae645e8c81b49f38ea4e"},{0xf805c05a, "3177c35da29dca8478000ec143057c1585dc11afb2c8ae645e8c81b49f38ea4e"},{0x7f243ff1, "3177c35da29dca8478000ec143057c1585dc11afb2c8ae645e8c81b49f38ea4e"},}},
        {2956930144, 3, {{0xd060467d, "596dccf32ec5c0428fa9050ea50f3920869a524bd99c8f19981e6f6a9a0a5e60"},{0xfce67778, "596dccf32ec5c0428fa9050ea50f3920869a524bd99c8f19981e6f6a9a0a5e60"},{0xece60323, "596dccf32ec5c0428fa9050ea50f3920869a524bd99c8f19981e6f6a9a0a5e60"},}},
        {2957030144, 5, {{0x7cb2fa0f, "d634ae9d5c57fc131fb9891b9cace8d582e838483fd283913d7cab5dbabeefd1"},{0x52b8a97e, "d634ae9d5c57fc131fb9891b9cace8d582e838483fd283913d7cab5dbabeefd1"},{0xa2f45b59, "d634ae9d5c57fc131fb9891b9cace8d582e838483fd283913d7cab5dbabeefd1"},{0x19b3c52f, "d634ae9d5c57fc131fb9891b9cace8d582e838483fd283913d7cab5dbabeefd1"},{0x897191ec, "d634ae9d5c57fc131fb9891b9cace8d582e838483fd283913d7cab5dbabeefd1"},}},
        {2957130144, 7, {{0xdcee1d71, "081d45af3f1cab704958bc38e8a35ba733c7d3acf4ebb6afa49e27d0dd3d2063"},{0x981daec9, "081d45af3f1cab704958bc38e8a35ba733c7d3acf4ebb6afa49e27d0dd3d2063"},{0xbde23301, "081d45af3f1cab704958bc38e8a35ba733c7d3acf4ebb6afa49e27d0dd3d2063"},{0x18f5dd7, "081d45af3f1cab704958bc38e8a35ba733c7d3acf4ebb6afa49e27d0dd3d2063"},{0xa85f628b, "081d45af3f1cab704958bc38e8a35ba733c7d3acf4ebb6afa49e27d0dd3d2063"},{0xc771f86, "081d45af3f1cab704958bc38e8a35ba733c7d3acf4ebb6afa49e27d0dd3d2063"},{0xaee90d59, "081d45af3f1cab704958bc38e8a35ba733c7d3acf4ebb6afa49e27d0dd3d2063"},}},
        {2957230144, 5, {{0x5ef4f385, "463265f8f2ade328449f0c8ae435f667ecf64f0816a8c2b54c09f961fce1ceab"},{0xa69f36b8, "463265f8f2ade328449f0c8ae435f667ecf64f0816a8c2b54c09f961fce1ceab"},{0xa0246277, "463265f8f2ade328449f0c8ae435f667ecf64f0816a8c2b54c09f961fce1ceab"},{0xbbc51fe9, "463265f8f2ade328449f0c8ae435f667ecf64f0816a8c2b54c09f961fce1ceab"},{0xfce06e0c, "463265f8f2ade328449f0c8ae435f667ecf64f0816a8c2b54c09f961fce1ceab"},}},
        {2957330144, 4, {{0xd964ceb2, "607909c82988b61c9fc96e38e0a605d2f2f4580792b26e697d3501149e1a4854"},{0x7ef9af49, "607909c82988b61c9fc96e38e0a605d2f2f4580792b26e697d3501149e1a4854"},{0xa0e90646, "607909c82988b61c9fc96e38e0a605d2f2f4580792b26e697d3501149e1a4854"},{0x70ee1a09, "607909c82988b61c9fc96e38e0a605d2f2f4580792b26e697d3501149e1a4854"},}},
        {2957430144, 5, {{0x9d9564dd, "a7c1e8de9c4ad214ef22da4bcbfb605beb426ea9cbb3f790c8aaf8028a538274"},{0x1305956c, "a7c1e8de9c4ad214ef22da4bcbfb605beb426ea9cbb3f790c8aaf8028a538274"},{0x65b30f34, "a7c1e8de9c4ad214ef22da4bcbfb605beb426ea9cbb3f790c8aaf8028a538274"},{0xc8b1c67a, "a7c1e8de9c4ad214ef22da4bcbfb605beb426ea9cbb3f790c8aaf8028a538274"},{0x2d204143, "a7c1e8de9c4ad214ef22da4bcbfb605beb426ea9cbb3f790c8aaf8028a538274"},}},
        {2957530144, 2, {{0x42d3c50a, "38efe10ea993b918e07e744f2cf7b9f7cfddf81a87af0bda8217f8715cd96e26"},{0xf702d93f, "38efe10ea993b918e07e744f2cf7b9f7cfddf81a87af0bda8217f8715cd96e26"},}},
        {2957630144, 12, {{0xb049dac0, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0xfea48e50, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0xcf5301b1, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0x19c82d6b, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0xfcf843c0, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0xb7551473, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0x10ea5c40, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0x2929132d, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0x26578722, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0xa5cde46d, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0x8bf2fb6c, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},{0xa7149042, "bfca814021efd35d503dce6c9d49fe72955cf0d87e1c83bf6d536ac86aa6fd7d"},}},
        {2957730144, 2, {{0x194b469, "00d611b961a57989c89b96629a2c786f3bfe05461de35548ba2d7629e89275b4"},{0x38e177f2, "00d611b961a57989c89b96629a2c786f3bfe05461de35548ba2d7629e89275b4"},}},
        {2957830144, 4, {{0xfffcf0d1, "d5c49b99c725925df132e3c5e64d28bddc5ed57244cda3dd73b6beecf37c9bdb"},{0x6a9f8cfb, "d5c49b99c725925df132e3c5e64d28bddc5ed57244cda3dd73b6beecf37c9bdb"},{0xa824b5ba, "d5c49b99c725925df132e3c5e64d28bddc5ed57244cda3dd73b6beecf37c9bdb"},{0x65e80c11, "d5c49b99c725925df132e3c5e64d28bddc5ed57244cda3dd73b6beecf37c9bdb"},}},
        {2957930144, 11, {{0xc62e3e6c, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},{0x7a2ffc08, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},{0x710e310b, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},{0xab8d1063, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},{0x2d87680b, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},{0x3d4ece01, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},{0x88ae9e1a, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},{0x3630a7e2, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},{0xceea0ae1, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},{0x12f598ab, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},{0x83f61fab, "0f78e5e14b6653d6a86ec01e7184cde16af321b1f5975a7c4da8de4527fb4198"},}},
        {2958030144, 7, {{0xf94f76a6, "625100e49642ef3fcc69b6071eee969dc05e97700f9ccadda0f7372bf4ae3942"},{0xf16d107a, "625100e49642ef3fcc69b6071eee969dc05e97700f9ccadda0f7372bf4ae3942"},{0x2437b75f, "625100e49642ef3fcc69b6071eee969dc05e97700f9ccadda0f7372bf4ae3942"},{0x37f090ef, "625100e49642ef3fcc69b6071eee969dc05e97700f9ccadda0f7372bf4ae3942"},{0xf61f156e, "625100e49642ef3fcc69b6071eee969dc05e97700f9ccadda0f7372bf4ae3942"},{0x452a2529, "625100e49642ef3fcc69b6071eee969dc05e97700f9ccadda0f7372bf4ae3942"},{0x195485a, "625100e49642ef3fcc69b6071eee969dc05e97700f9ccadda0f7372bf4ae3942"},}},
        {2958130144, 10, {{0x37df6213, "ef5ccdf6fb36385c8cbdc16ef6c2ed9d3ca1e2cb66a67564612bc8538486b9fe"},{0x6a4a7524, "ef5ccdf6fb36385c8cbdc16ef6c2ed9d3ca1e2cb66a67564612bc8538486b9fe"},{0x69e7df0e, "ef5ccdf6fb36385c8cbdc16ef6c2ed9d3ca1e2cb66a67564612bc8538486b9fe"},{0x850c23f4, "ef5ccdf6fb36385c8cbdc16ef6c2ed9d3ca1e2cb66a67564612bc8538486b9fe"},{0x267adf15, "ef5ccdf6fb36385c8cbdc16ef6c2ed9d3ca1e2cb66a67564612bc8538486b9fe"},{0xa3435a7f, "ef5ccdf6fb36385c8cbdc16ef6c2ed9d3ca1e2cb66a67564612bc8538486b9fe"},{0xe709721, "ef5ccdf6fb36385c8cbdc16ef6c2ed9d3ca1e2cb66a67564612bc8538486b9fe"},{0x504eaa11, "ef5ccdf6fb36385c8cbdc16ef6c2ed9d3ca1e2cb66a67564612bc8538486b9fe"},{0x7ae4074a, "ef5ccdf6fb36385c8cbdc16ef6c2ed9d3ca1e2cb66a67564612bc8538486b9fe"},{0x59be64c7, "ef5ccdf6fb36385c8cbdc16ef6c2ed9d3ca1e2cb66a67564612bc8538486b9fe"},}},
        {2958230144, 1, {{0xef9cf33c, "56c463a9be6e39a3c3f2a1f5f1104c35dd5dc164f0e9de5040776d7d91f797f3"},}},
        {2958330144, 8, {{0xaae8259f, "a88784a49a97f889b84e64604b8e00753bab484e2ca1ca87cd67d72aeff92a47"},{0x278c4d9, "a88784a49a97f889b84e64604b8e00753bab484e2ca1ca87cd67d72aeff92a47"},{0x5f6359cc, "a88784a49a97f889b84e64604b8e00753bab484e2ca1ca87cd67d72aeff92a47"},{0x8771ef44, "a88784a49a97f889b84e64604b8e00753bab484e2ca1ca87cd67d72aeff92a47"},{0xc7686879, "a88784a49a97f889b84e64604b8e00753bab484e2ca1ca87cd67d72aeff92a47"},{0x16cf0fa8, "a88784a49a97f889b84e64604b8e00753bab484e2ca1ca87cd67d72aeff92a47"},{0xac76b4db, "a88784a49a97f889b84e64604b8e00753bab484e2ca1ca87cd67d72aeff92a47"},{0xa4933912, "a88784a49a97f889b84e64604b8e00753bab484e2ca1ca87cd67d72aeff92a47"},}},
        {2958430144, 11, {{0x4b0aeafa, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},{0xe8268e57, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},{0xee1481e2, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},{0x23aa7672, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},{0xf3393c74, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},{0xe88b2139, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},{0x7c3dcc0a, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},{0x59a8ea62, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},{0xc72d36aa, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},{0xaf1d7c82, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},{0x460b6285, "c103ce07dd0acbb05b9f399fbf3e3114295d2380f09ad815d68eb13010785e12"},}},
        {2958530144, 2, {{0x4138a8cd, "81df01397a364507893aa53eb7228c74639624aaa7ef424f879808b64ccbe3db"},{0x9004da9c, "81df01397a364507893aa53eb7228c74639624aaa7ef424f879808b64ccbe3db"},}},

    };
    

    //set thread init vals
    memcpy(&input[0], &input_ORI[0], input_ORI.size());
    RandomHash_SetHeader(&randomHashArray[0], &input[0], input.size(), 0);

    U32 gid=0x493234;
    for (int i = 0; i < cRounds; i++)
    {
        U32 _gid = AtomicIncrement(gid); 
        U32 hash = (_gid - 1) * 100000;
        
        PrintOut("Round %d with %u  ---------------------------------------\n", i, gid);
        memset(&_output2, 0, sizeof(_output2));
        RandomHash_Search(&randomHashArray[0], _output2, hash);
         
        PrintOut("~{%u, %d, {",hash, _output2.count );
        for (int z = 0; z < _output2.count; z++)
            PrintOut("~{0x%x, \"%s\"},", _output2.nonces[z], toHex((void*)_output2.hashes[0], 32).c_str());
        PrintOut("~}},\n");
        
        if (i <  20 )
        {
            RHMINER_ASSERT(testResult[i].gid == hash && testResult[i].cnt == _output2.count);
            for (int z = 0; z < _output2.count; z++)
            {
                RHMINER_ASSERT(testResult[i].nonces[z].first == _output2.nonces[z]);
                RHMINER_ASSERT(strcmp(testResult[i].nonces[z].second.c_str(), toHex((void*)_output2.hashes[0], 32).c_str()) == 0);
            }
        }
        
    }


    {
        bytes input_ORI = fromHex(g_testvalues[1].input);
        bytes input;
        input.resize(input_ORI.size());
        

        bytes _in[4], _out1[4], _out2[4], _out3[4];
        U32* in_stride[4];
        U32* out_stride_base[4];
        U32* out_stride2[4];
        U32* out_stride3[4];
        for (int i = 0; i < 4; i++)
        {
            memcpy(&input[0], &input_ORI[0], input_ORI.size());
            *((U32*)&input[input.size() - 4]) = 0x8888AAAA+i;
            
            _in[i].resize(1024);
            _out1[i].resize(_in[i].size());
            _out2[i].resize(_in[i].size());
            _out3[i].resize(_in[i].size());

            in_stride[i] = (U32*)(void*)&(_in[i][0]);
            out_stride_base[i] =(U32*)(void*) &(_out1[i][0]);
            out_stride2[i] = (U32*)(void*)&(_out2[i][0]);
            out_stride3[i] = (U32*)(void*)&(_out3[i][0]);

            memset(in_stride[i], 0, _in[i].size());
            memset(out_stride_base[i], 0, _in[i].size());
            memset(out_stride2[i], 0, _in[i].size());
            memset(out_stride3[i], 0, _in[i].size());

            RH_STRIDE_RESET(in_stride[i]);
            RH_STRIDE_RESET(out_stride_base[i]);

            memset(in_stride[i], 0, _in[i].size());
            RH_STRIDE_SET_SIZE(in_stride[i], input.size());
            memcpy(RH_STRIDE_GET_DATA(in_stride[i]), &input[0], input.size());

            RandomHash_SHA2_256(in_stride[i], out_stride_base[i], false);
        }

#ifdef RH2_ENABLE_PREFLIGHT_CACHE
        //test part1 + part2
        {
            SHA2_256_SavedState state[4];

            for (int i = 0; i < 4; i++)
            {
                memcpy(&input[0], &input_ORI[0], input_ORI.size());
                *((U32*)&input[input.size() - 4]) = 0x8888AAAA+i;
                state[i].endCut = 8;
                RandomHash_SHA2_256_Part1((U32*)(void*)&input[0], input.size(), state[i]);

                RH_STRIDE_RESET(out_stride2[i]);

                memset(in_stride[i], 0, _in[i].size());
                RH_STRIDE_SET_SIZE(in_stride[i], input.size());
                memcpy(RH_STRIDE_GET_DATA(in_stride[i]), &input[0], input.size());

                RandomHash_SHA2_256_Part2(in_stride[i], state[i], out_stride2[i]);

                RHMINER_ASSERT(memcmp(RH_STRIDE_GET_DATA(out_stride2[i]), RH_STRIDE_GET_DATA(out_stride_base[i]), 32) == 0);
            }
        }
#endif

#if defined(_WIN32_WINNT) && !defined(RHMINER_NO_SIMD)
        //test ori SHA256_4x
        {
            bytes input_ORI = fromHex("c0a5db28ef94fe2302cca71e4731fa4b6b0246dc31e392f81a57a8417f09f2d9e1d2436bd3f2505e1d13f6ea8000000000000000000000000000000000000760");
            bytes input;
            input.resize(input_ORI.size());

            for (int i = 0; i < 4; i++)
            {
                memcpy(&input[0], &input_ORI[0], input_ORI.size());
                *((U32*)&input[input.size() - 4]) = 0x8888AAAA+i;

                memset(in_stride[i], 0, _in[i].size());
                RH_STRIDE_SET_SIZE(in_stride[i], input.size());
                memcpy(RH_STRIDE_GET_DATA(in_stride[i]), &input[0], input.size());

                RandomHash_SHA2_256(in_stride[i], out_stride3[i], false);
            }

            for (int i = 0; i < 4; i++)
            {
                memcpy(&input[0], &input_ORI[0], input_ORI.size());
                *((U32*)&input[input.size() - 4]) = 0x8888AAAA+i;

                RH_STRIDE_RESET(out_stride2[i]);

                memset(in_stride[i], 0, _in[i].size());
                RH_STRIDE_SET_SIZE(in_stride[i], input.size());
                memcpy(RH_STRIDE_GET_DATA(in_stride[i]), &input[0], input.size());
            }

            //last block
            U32 nullBlock[] = { 0x00000080, 0x00000000, 0x00000000, 0x00000000,
                                0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                0x00000000, 0x00000000, 0x00000000, 0x00020000 };
            sha256sse_4X(
                (U32*)RH_STRIDE_GET_DATA(in_stride[0]),
                (U32*)RH_STRIDE_GET_DATA(in_stride[1]),
                (U32*)RH_STRIDE_GET_DATA(in_stride[2]),
                (U32*)RH_STRIDE_GET_DATA(in_stride[3]),
                RH_STRIDE_GET_DATA8(out_stride2[0]),
                RH_STRIDE_GET_DATA8(out_stride2[1]),
                RH_STRIDE_GET_DATA8(out_stride2[2]),
                RH_STRIDE_GET_DATA8(out_stride2[3]),
                nullBlock);

            for (int i = 0; i < 4; i++)
            {
                RHMINER_ASSERT(memcmp(RH_STRIDE_GET_DATA(out_stride2[i]), RH_STRIDE_GET_DATA(out_stride3[i]), 32) == 0);
            }
        }


#ifdef RH2_ENABLE_PREFLIGHT_4X
        //test part1 + part2_SSE_4x
        {
            SHA2_256_SavedState state[4];
            memcpy(&input[0], &input_ORI[0], input_ORI.size());
            *((U32*)&input[input.size() - 4]) = 0x8888AAAA+0;

            state[0].endCut = 8;
            RandomHash_SHA2_256_Part1((U32*)(void*)&input[0], input.size(), state[0]);

            RH_STRIDE_RESET(out_stride2[0]);
            RH_STRIDE_RESET(out_stride2[1]);
            RH_STRIDE_RESET(out_stride2[2]);
            RH_STRIDE_RESET(out_stride2[3]);

            memset(in_stride[0], 0, _in[0].size());
            RH_STRIDE_SET_SIZE(in_stride[0], input.size());
            memcpy(RH_STRIDE_GET_DATA(in_stride[0]), &input[0], input.size());

            RandomHash_SHA2_256_Part2_SSE_4x(in_stride[0], state[0], 
                0x8888AAAA + 1, 0x8888AAAA + 2, 0x8888AAAA + 3,
                out_stride2[0],out_stride2[1],out_stride2[2],out_stride2[3]);

            for(int i=0; i < 4; i++)
            {
                RHMINER_ASSERT(memcmp(RH_STRIDE_GET_DATA(out_stride2[i]), RH_STRIDE_GET_DATA(out_stride_base[i]), 32) == 0);
            }
        }
#endif

        /*{
            SHA2_256_SavedState state[4];

            for (int i = 0; i < 4; i++)
            {
                memcpy(&input[0], input_ORI[0], input_ORI.size());
                *((U32*)&input[input.size() - 4]) = 0x8888AAAA+i;
                state[i].endCut = 8;
                RandomHash_SHA2_256_Part1(&input[0], input.size(), state[i]);

                RH_STRIDE_RESET(out_stride2[i]);
                RH_STRIDE_SET_SIZE(in_stride[i], input.size());
                memcpy(RH_STRIDE_GET_DATA(in_stride[i]), &input[0], input.size());
                RandomHash_SHA2_256_Part2_SSE_1x(in_stride[i], state[i], out_stride2[i]);

                RHMINER_ASSERT(memcmp(RH_STRIDE_GET_DATA(out_stride2[i]), RH_STRIDE_GET_DATA(out_stride_base[i]), 32) == 0);
            }
        }
        */
#endif //#ifdef _WIN32_WINNT
    }
    RandomHash_DestroyMany(randomHashArray, 1);
}

void Test_merssen_twister_rand()
{
    {
        // Merssen 4 U8 numbers using 2 U64
        U32 seed = 0x82749999;
        mersenne_twister_state slow;
        merssen_twister_seed(seed, &slow);
        U32 mt4_sequence = 0;
        U32 _TEST_VALUES[4];
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

            U32 fval = 5;
            while (fval < MERSENNE_TWISTER_PERIOD)
                pval = 0x6c078965 * (pval ^ pval >> 30) + fval++;

            //future
            fval = 0x6c078965 * (pval ^ pval >> 30) + MERSENNE_TWISTER_PERIOD;
            // 0 ---
            pval = M32((U32)mt10);
            pval |= L31(mt10 >> 32);
            pval = fval ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);

            pval ^= pval >> 11;
            pval ^= pval << 7 & 0x9d2c5680;
            pval ^= pval << 15 & 0xefc60000;
            pval ^= pval >> 18;

            //store
            _TEST_VALUES[0] = pval;
            mt4_sequence |= (pval % 8);
            mt4_sequence <<= 8;

            // 1 ---
            fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD + 1;
            pval = M32(mt10 >> 32);
            pval |= L31((U32)mt32);
            pval = fval ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);

            pval ^= pval >> 11;
            pval ^= pval << 7 & 0x9d2c5680;
            pval ^= pval << 15 & 0xefc60000;
            pval ^= pval >> 18;

            //store
            _TEST_VALUES[1] = pval;
            mt4_sequence |= (pval % 8);
            mt4_sequence <<= 8;

            // 3 ---
            fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD + 2;
            pval = M32((U32)mt32);
            pval |= L31(mt32 >> 32);
            pval = fval ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);

            pval ^= pval >> 11;
            pval ^= pval << 7 & 0x9d2c5680;
            pval ^= pval << 15 & 0xefc60000;
            pval ^= pval >> 18;

            //store
            _TEST_VALUES[2] = pval;
            mt4_sequence |= (pval % 8);
            mt4_sequence <<= 8;

            // 4 ---
            fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD + 3;
            pval = M32(mt32 >> 32);
            pval |= L31(mt5);
            pval = fval ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);

            pval ^= pval >> 11;
            pval ^= pval << 7 & 0x9d2c5680;
            pval ^= pval << 15 & 0xefc60000;
            pval ^= pval >> 18;

            //store
            _TEST_VALUES[3] = pval;
            mt4_sequence |= (pval % 8);
            mt4_sequence = RH_swap_u32(mt4_sequence);
        }

        U32 sval = merssen_twister_rand(&slow);
        RH_ASSERT((sval % 8) == (U8)mt4_sequence);
        RH_ASSERT(sval == _TEST_VALUES[0]);
        mt4_sequence >>= 8;

        sval = merssen_twister_rand(&slow);
        RH_ASSERT((sval % 8) == (U8)mt4_sequence);
        RH_ASSERT(sval == _TEST_VALUES[1]);
        mt4_sequence >>= 8;

        sval = merssen_twister_rand(&slow);
        RH_ASSERT((sval % 8) == (U8)mt4_sequence);
        RH_ASSERT(sval == _TEST_VALUES[2]);
        mt4_sequence >>= 8;

        sval = merssen_twister_rand(&slow);
        RH_ASSERT((sval % 8) == (U8)mt4_sequence);
        RH_ASSERT(sval == _TEST_VALUES[3]);
    }

    #ifndef RHMINER_NO_SIMD
    {
        // Merssen 4 numbers using mm128i
        U32 seed = 0x82749999;
        mersenne_twister_state slow;
        merssen_twister_seed(seed, &slow);
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

        for (int i = 0; i < 4; i++)
        {
            U32 r = _mm_extract_epi32_M(r1, 0);
            r1 = _mm_bsrli_si128(r1, 4);

            U32 sval = merssen_twister_rand(&slow);
            RH_ASSERT(sval == r);
        }
    }
    #endif //RHMINER_NO_SIMD
    
    {
        U32 seed = 0x82749999;
        //inplace 4
        //-------------------
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

        mersenne_twister_state slow;
        mersenne_twister_state fast_max;
        merssen_twister_seed(0x82749999, &slow);
        for (int i = 0; i < 4; i++)
        {
            U32 sval = merssen_twister_rand(&slow);
            //----------------------------------
            switch (i)
            {
                case 0: 
                    // 0 ---
                    fval = 0x6c078965 * (pval ^ pval >> 30) + MERSENNE_TWISTER_PERIOD;
                    pval = M32((U32)mt10);
                    pval |= L31(mt10>>32);
                    break;
                case 1: 
                    // 1 ---
                    fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD+1;
                    pval = M32(mt10>>32);
                    pval |= L31((U32)mt32);
                    break;
                case 2: 
                    fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD+2;
                    pval = M32((U32)mt32);
                    pval |= L31(mt32>>32);
                    break;
                case 3: 
                    fval = 0x6c078965 * (fval ^ fval >> 30) + MERSENNE_TWISTER_PERIOD+3;
                    pval = M32(mt32>>32);
                    pval |= L31(mt5);
                    break;
            }
            pval = fval ^ (pval >> 1) ^ (((int32_t(pval) << 31) >> 31) & MERSENNE_TWISTER_MAGIC);

            pval ^= pval >> 11; 
            pval ^= pval << 7 & 0x9d2c5680; 
            pval ^= pval << 15 & 0xefc60000; 
            pval ^= pval >> 18;
            
            //----------------------------------
            RHMINER_ASSERT(sval == pval);
        }

    }

    {
        mersenne_twister_state slow;
        mersenne_twister_state fast_max;
        merssen_twister_seed(0x82749999, &slow);
        merssen_twister_seed_fast_partial(0x82749999, &fast_max, 4);
        for (int i = 0; i < 4; i++)
        {
            U32 sval = merssen_twister_rand(&slow);
            U32 fval = merssen_twister_rand_fast_partial_4(&fast_max);
            RHMINER_ASSERT(sval == fval);
        }
    }

    {
        mersenne_twister_state slow;
        mersenne_twister_state fast_max;
        U32 seed = 0xe6b666f6; //0x82749999
        merssen_twister_seed(seed, &slow);
        merssen_twister_seed_fast_partial(seed, &fast_max, 12);
        for (int i = 0; i < 12; i++)
        {
            U32 sval = merssen_twister_rand(&slow);
            U32 fval = merssen_twister_rand_fast_partial_12(&fast_max);
            RHMINER_ASSERT(sval == fval);
        }
    }

    {
        mersenne_twister_state slow;
        mersenne_twister_state fast_max;
        merssen_twister_seed(0x82749999, &slow);
        merssen_twister_seed_fast_partial(0x82749999, &fast_max, 204);
        for (int i = 0; i < 204; i++)
        {
            U32 sval = merssen_twister_rand(&slow);
            U32 fval = merssen_twister_rand_fast_partial(&fast_max, 204);
            RHMINER_ASSERT(sval == fval);
        }
    }

    {
        mersenne_twister_state slow;
        mersenne_twister_state fast;
        merssen_twister_seed(0x82749999, &slow);
        merssen_twister_seed_fast(0x82749999, &fast);
        for (int _i = 0; _i < 32; _i++)
        {
            U32 nextReset = 1024 + (rand32() % 1024);
            for (int i = 0; i < 1024 * 1024; i++)
            {
                if (i == nextReset)
                {
                    U32 seed = merssen_twister_rand(&slow);

                    merssen_twister_seed(seed, &slow);
                    merssen_twister_seed_fast(seed, &fast);
                    nextReset = 1024 + (rand32() % 1024);
                }
                U32 sval = merssen_twister_rand(&slow);
                U32 fval = merssen_twister_rand_fast(&fast);
                RHMINER_ASSERT(sval == fval);
            }
        }
    }
        
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TestTransfos()
{
    U32 size;
    U8 _workBytes[256];
    U8  nextChunk[256];
    bytes test26{ '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '*', '-'};
    U32 sizes[] = { 27, 24, 7 };
    string res[8][3] = {
        /*0*/{"37434a4f4447504d39464a4a324d313550434630504d4e46354f32","324e4a304a304d4c4236384e394b383548304e4141354230","36363232353135"},
        /*1*/{"45464748494a4b4c4d4e4f50514431323334353637383930414243","434445464748494a4b4c4d4e313233343536373839304142","35363734313233"},
        /*2*/{"51504f4e4d4c4b4a49484746454443424130393837363534333231","4e4d4c4b4a49484746454443424130393837363534333231","37363534333231"},
        /*3*/{"31453246334734483549364a374b384c394d304e414f4250435144","3143324433453446354736483749384a394b304c414d424e","31353236333734"},
        /*4*/{"453146324733483449354a364b374c384d394e304f415042514344","43314432453346344735483649374a384b394c304d414e42","35313632373334"},
        /*5*/{"0307030f090307030f0307031f5160627c7a787a7c727078060406","0307030f090307030f0307037f7f7f7f7f7f7f7f7f750501","03070337060406"},
        /*6*/{"89c866349a8de683c9c08242a111a8643a21924aa513a9e47a41a2","3119cc8653b1dc70391850483422158c47245249b462359c","988c6643a9d86e"},
        /*7*/{"268c99346ad8b983270ca04286112a64e812a44a96316ae4e914a8","3164cca153c6cd1c396005123488512347902552b4895327","62c89943a68d9b"},
    };

    {
        U32 sizes[] = { 2, 7,8,9, 13,14,15,16,17,18 };
        for(auto s : sizes)
        {
            U8* uptr = nextChunk;
            RH_Accum_8();
            for(U32 x = 0; x < s; x++)
                RH_Accum_8_Add(test26[x], uptr);
            
            RH_Accum_8_Finish(uptr);
            RHMINER_ASSERT(memcmp(&test26[0], nextChunk, s) == 0);
        }
    }
    /*
    {
        U32 sizes[] = { 7,8,9,1,13,8,15,16,17,18 };
        for(auto s : sizes)
        {
            RH_Accum_8(nextChunk);
            for(int x = 0; x < s; x++)
            {
                RH_Accum_8_Add_4(test26[x],  acc8_idx, acc8_buf, acc8_ptr);
            }
            
            RH_Accum_8_Finish();
            RHMINER_ASSERT(memcmp(&test26[0], nextChunk, s) == 0);
        }        
    }
    */

    //transfo4
    //U8 t4res[130];
    //U8 t4res_sse[130];
    const int T4_TestSize = 128;

    //for (int s = 8; s <= T4_TestSize; s += 2)
    //{
    //    for (int i = 0; i <= T4_TestSize; i++)
    //        _workBytes[i] = rand32() % 256;

    //    memset(t4res, 0, sizeof(t4res));
    //    memset(t4res_sse, 0, sizeof(t4res));
    //    Transfo4_2(t4res, s, _workBytes);
    //    Transfo4_2_128_SSE4(t4res_sse, s, _workBytes);
    //    RHMINER_ASSERT(memcmp(&t4res_sse[0], &t4res[0], s) == 0);
    //}

    U8* nextChunkPtr = nextChunk + 3;
    U8* workBytes = _workBytes + 3;
    for(int i=0; i < 8; i++)
    {
        //DebugOut("~[%d] ", i);
        for(int j=0; j < 3; j++)
        {
            size = sizes[j];
            bytes backTest26 = test26;
            memcpy((void*)nextChunkPtr,(void*)&test26[0], size);
            nextChunkPtr[size] = 0;
            bytes texp = fromHex(res[i][j]); texp.push_back(0);
            switch(i)
            {
            case 0: 
                Transfo0_2(nextChunkPtr, size,(U8*)(void*)&test26[0]);                 
                break;
            case 1: 
                if ((size % 2) == 0)
                    Transfo1_2(nextChunkPtr, size, (U8*)(void*)&test26[0]);
                else
                    continue;
                break;
            case 2: 
                    Transfo2_2(nextChunkPtr, size, (U8*)(void*)&test26[0]);
                break;
            case 3: 
                if ((size % 2) == 0)
                    Transfo3_2(nextChunkPtr, size,(U8*)(void*)&test26[0]);                 
                else
                    continue;
                break;
            case 4: 
                if ((size % 2) == 0)
                    Transfo4_2(nextChunkPtr, size, (U8*)(void*)&test26[0]);
                else
                    continue;
                break;
            case 5: 
                if ((size % 2) == 0)
                    Transfo5_2(nextChunkPtr, size, (U8*)(void*)&test26[0]);
                else
                    continue;
                break;
            case 6: 
                Transfo6_2(nextChunkPtr, size,(U8*)(void*)&test26[0]);                 
                break;
            case 7: 
                //Transfo7(nextChunkPtr, size);                 
                Transfo7_2(nextChunkPtr, size,(U8*)(void*)&test26[0]);                 
                break;
            }
            //DebugOut("~\nRES %s \n", nextChunkPtr);
            //DebugOut("~EXP %s \n", (char*)&texp[0]);
            DebugOut("~ALGO %d size %d \nRES %s \n", i, size, toHex((void*)nextChunkPtr, size).c_str());
            DebugOut("~EXP %s \n", toHex((void*)&texp[0], size).c_str());
            RHMINER_ASSERT(memcmp(&backTest26[0], &test26[0], backTest26.size()) == 0);
            RHMINER_ASSERT(memcmp(toHex((void*)nextChunkPtr, size).c_str(), res[i][j].c_str(), size<<1) == 0 );
        }
        DebugOut("~\n");
    }
}
#endif

#if defined(RHMINER_DEBUG_RANDOMHASH_UNITTEST) && !defined(RANDOMHASH_CUDA)
void TestMur3()
{
    bytes test26{ '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P'};

    /*
    __m128i t128;
#ifdef _WIN32_WINNT
    t128.m128i_u64[0] = 4050765991979987505;
    t128.m128i_u64[1] = 5063528411713056825;
#else
    t128[0] = 4050765991979987505;
    t128[1] = 5063528411713056825;
#endif
    t128.m128i_u8[15] = 'F';
    t128.m128i_u8[14] = 'E';
    t128.m128i_u8[13] = 'D';
    t128.m128i_u8[12] = 'C';
    t128.m128i_u8[11] = 'B';
    t128.m128i_u8[10] = 'A';
    t128.m128i_u8[9] = '0';
    t128.m128i_u8[8] = '9';
    t128.m128i_u8[7] = '8';
    t128.m128i_u8[6] = '7';
    t128.m128i_u8[5] = '6';
    t128.m128i_u8[4] = '5';
    t128.m128i_u8[3] = '4';
    t128.m128i_u8[2] = '3';
    t128.m128i_u8[1] = '2';
    t128.m128i_u8[0] = '1';*/
    

    U32 accumSlow;
    U32 accum8;
    U32 accum1;
    U32 accumFast;
    const int sizes[] = { 7, 2, 6,7,4,1,8,7,1,1,4,4,4,7,7,7,3,7,4,8,1 };
    //const int sizes[] = { 2, 8, 6};
    U8 test[512];
    int testSize = 0;
    
    // TEST accum 8 -------------------------------------------------------------
    {
        //idx != 0
        MurmurHash3_x86_32_State state;
        MurmurHash3_x86_32_Init(0, &state);
        for(auto n : sizes)
        {
            MurmurHash3_x86_32_Update(&test26[0], n, &state); 
            memcpy(test + testSize, &test26[0], n);
            testSize += n;
        }
        
        accumSlow = MurmurHash3_x86_32_Finalize(&state);
        accumFast = MurmurHash3_x86_32_Fast(test, testSize);
        RHMINER_ASSERT(accumFast == accumSlow);
    }
    {
        //idx != 0
        U64 t64 = (*(U64*)&test26[0]);
        MurmurHash3_x86_32_State state;
        MurmurHash3_x86_32_Init(0, &state);
        for(auto n : sizes)
        {
            MurmurHash3_x86_32_Update_8(t64, n, &state); 
            memcpy(test + testSize, &test26[0], n);
            testSize += n;
        }
        accum8 = MurmurHash3_x86_32_Finalize(&state);
    }
    PrintOut("Test 8 : Murmur3 normal = %u  sse2 = %u\n", accumSlow, accum8);
    RHMINER_ASSERT(accum8 == accumSlow);
    {
        //idx != 0
        U64 t64 = (*(U64*)&test26[0]);
        MurmurHash3_x86_32_State state;
        MurmurHash3_x86_32_Init(0, &state);
        RH_MUR3_BACKUP_STATE(&state);
        for(auto n : sizes)
        {
            INPLACE_M_MurmurHash3_x86_32_Update_8(t64, n); 
            memcpy(test + testSize, &test26[0], n);
            testSize += n;
        }
        RH_MUR3_RESTORE_STATE(&state);
        accum8 = MurmurHash3_x86_32_Finalize(&state);
    }
    PrintOut("Test 8 Macro : Murmur3 normal = %u  sse2 = %u\n", accumSlow, accum8);
    RHMINER_ASSERT(accum8 == accumSlow);

    // TEST accum 16 -------------------------------------------------------------
    /*
    {
        MurmurHash3_x86_32_State state;
        MurmurHash3_x86_32_Init(0, &state);
        MurmurHash3_x86_32_Update(&test26[0], 15, &state); 
        MurmurHash3_x86_32_Update(&test26[0], 6, &state);  
        MurmurHash3_x86_32_Update(&test26[0], 7, &state);  
        MurmurHash3_x86_32_Update(&test26[0], 4, &state);  
        MurmurHash3_x86_32_Update(&test26[0], 15, &state); 
        MurmurHash3_x86_32_Update(&test26[0], 16, &state); 
        for(int i=0; i <32; i++)
        {
            MurmurHash3_x86_32_Update(&test26[0], i%16, &state);  
            for(int i=0; i <7; i++)
                MurmurHash3_x86_32_Update(&test26[0], 8, &state); 
            for(int i=0; i <7; i++)
                MurmurHash3_x86_32_Update(&test26[0], 16, &state);
            for(int i=0; i <12; i++)
                MurmurHash3_x86_32_Update(&test26[0], 1, &state); 
        }
        accumSlow = MurmurHash3_x86_32_Finalize(&state);
    }
    
    {
        //idx != 0
        MurmurHash3_x86_32_State state;
        MurmurHash3_x86_32_Init(0, &state);
        MurmurHash3_x86_32_Update_16(t128, 15, &state); 
        MurmurHash3_x86_32_Update_16(t128, 6, &state);  
        MurmurHash3_x86_32_Update_16(t128, 7, &state);  
        MurmurHash3_x86_32_Update_16(t128, 4, &state);  
        MurmurHash3_x86_32_Update_16(t128, 15, &state); 
        MurmurHash3_x86_32_Update_16(t128, 16, &state); 
        for(int i=0; i <32; i++)
        {
            MurmurHash3_x86_32_Update_16(t128, i%16, &state);  
            for(int i=0; i <7; i++)
                MurmurHash3_x86_32_Update_16(t128, 8, &state); 
            for(int i=0; i <7; i++)
                MurmurHash3_x86_32_Update_16(t128, 16, &state);
            for(int i=0; i <12; i++)
                MurmurHash3_x86_32_Update_16(t128, 1, &state); 
        }
        accum16 = MurmurHash3_x86_32_Finalize(&state);
    }

    PrintOut("Test 16 : Murmur3 normal = %u  sse2 = %u\n", accumSlow, accum16);
    if (accum16 != accumSlow)
        PrintOut("UNITTEST ERROR **** accum16 != accumSlow \n");    
    {
        //idx != 0
        MurmurHash3_x86_32_State state;
        MurmurHash3_x86_32_Init(0, &state);
        RH_MUR3_BACKUP_STATE(&state);

        INPLACE_M_MurmurHash3_x86_32_Update_16(t128, 15); 
        INPLACE_M_MurmurHash3_x86_32_Update_16(t128, 6);  
        INPLACE_M_MurmurHash3_x86_32_Update_16(t128, 7);  
        INPLACE_M_MurmurHash3_x86_32_Update_16(t128, 4);  
        INPLACE_M_MurmurHash3_x86_32_Update_16(t128, 15); 
        INPLACE_M_MurmurHash3_x86_32_Update_16(t128, 16); 
        for(int i=0; i <32; i++)
        {
            INPLACE_M_MurmurHash3_x86_32_Update_16(t128, i%16);  
            for(int i=0; i <7; i++)
                INPLACE_M_MurmurHash3_x86_32_Update_16(t128, 8); 
            for(int i=0; i <7; i++)
                INPLACE_M_MurmurHash3_x86_32_Update_16(t128, 16);
            for(int i=0; i <12; i++)
                INPLACE_M_MurmurHash3_x86_32_Update_16(t128, 1); 
        }
        RH_MUR3_RESTORE_STATE(&state);
        accum16 = MurmurHash3_x86_32_Finalize(&state);
    }

    PrintOut("Test 16 MACRO : Murmur3 normal = %u  sse2 = %u\n", accumSlow, accum8);
    if (accum16 != accumSlow)
        PrintOut("UNITTEST ERROR ****  - accum16 != accumSlow\n");    
*/

    // TEST accum 1 -------------------------------------------------------------
    {
        MurmurHash3_x86_32_State state;
        MurmurHash3_x86_32_Init(0, &state);
        MurmurHash3_x86_32_Update(&test26[0] + 15, 1, &state);
        MurmurHash3_x86_32_Update(&test26[0] + 6, 1, &state);
        MurmurHash3_x86_32_Update(&test26[0] + 7, 1, &state);
        MurmurHash3_x86_32_Update(&test26[0] + 4, 1, &state);
        MurmurHash3_x86_32_Update(&test26[0] + 15, 1, &state);
        MurmurHash3_x86_32_Update(&test26[0] + 16, 1, &state);

        for (int i = 0; i < 32; i++)
        {
            MurmurHash3_x86_32_Update(&test26[0] + (i % 16), 1, &state);
            for (int i = 0; i < 7; i++)
                MurmurHash3_x86_32_Update(&test26[0] + 8, 1, &state);
            for (int i = 0; i < 7; i++)
                MurmurHash3_x86_32_Update(&test26[0] + 16, 1, &state);
            for (int i = 0; i < 12; i++)
                MurmurHash3_x86_32_Update(&test26[0] + 1, 1, &state);
        }
        accumSlow = MurmurHash3_x86_32_Finalize(&state);
    }
    {
        MurmurHash3_x86_32_State state;
        MurmurHash3_x86_32_Init(0, &state);
        RH_MUR3_BACKUP_STATE(&state);

        INPLACE_M_MurmurHash3_x86_32_Update_1(test26[15]);
        INPLACE_M_MurmurHash3_x86_32_Update_1(test26[6]);
        INPLACE_M_MurmurHash3_x86_32_Update_1(test26[7]);
        INPLACE_M_MurmurHash3_x86_32_Update_1(test26[4]);
        INPLACE_M_MurmurHash3_x86_32_Update_1(test26[15]);
        INPLACE_M_MurmurHash3_x86_32_Update_1(test26[16]);
        for (int i = 0; i < 32; i++)
        {
            INPLACE_M_MurmurHash3_x86_32_Update_1(test26[(i % 16)]);
            for (int i = 0; i < 7; i++)
                INPLACE_M_MurmurHash3_x86_32_Update_1(test26[8]);
            for (int i = 0; i < 7; i++)
                INPLACE_M_MurmurHash3_x86_32_Update_1(test26[16]);
            for (int i = 0; i < 12; i++)
                INPLACE_M_MurmurHash3_x86_32_Update_1(test26[1]);
        }
        
        RH_MUR3_RESTORE_STATE(&state);
        accum1 = MurmurHash3_x86_32_Finalize(&state);
    }
    PrintOut("Test 1 MACRO : Murmur3 normal = %u  sse2 = %u\n", accumSlow, accum8);
    RHMINER_ASSERT(accum1 == accumSlow);
    
}
 /*
void Transfo0_128(U8* nextChunk, U32 size)
{
    U32 rndState = MurmurHash3_x86_32_Fast(nextChunk,size);
    if (!rndState)
        rndState = 1;
    RH_ASSERT(size <= 128);
    //load in mmx reg
    __m128i r0,r1,r2,r3,r4,r5,r6,r7;
    switch(size/16)
    {
            case 8:
            case 7: r0 = RH_MM_LOAD128 ((__m128i *)(nextChunk+0*sizeof(__m128i)));
                    r1 = RH_MM_LOAD128 ((__m128i *)(nextChunk+1*sizeof(__m128i)));
                    r2 = RH_MM_LOAD128 ((__m128i *)(nextChunk+2*sizeof(__m128i)));
                    r3 = RH_MM_LOAD128 ((__m128i *)(nextChunk+3*sizeof(__m128i)));
                    r4 = RH_MM_LOAD128 ((__m128i *)(nextChunk+4*sizeof(__m128i)));
                    r5 = RH_MM_LOAD128 ((__m128i *)(nextChunk+5*sizeof(__m128i)));
                    r6 = RH_MM_LOAD128 ((__m128i *)(nextChunk+6*sizeof(__m128i)));
                    r7 = RH_MM_LOAD128 ((__m128i *)(nextChunk+7*sizeof(__m128i)));
                    break;
            case 6: r0 = RH_MM_LOAD128 ((__m128i *)(nextChunk+0*sizeof(__m128i)));
                    r1 = RH_MM_LOAD128 ((__m128i *)(nextChunk+1*sizeof(__m128i)));
                    r2 = RH_MM_LOAD128 ((__m128i *)(nextChunk+2*sizeof(__m128i)));
                    r3 = RH_MM_LOAD128 ((__m128i *)(nextChunk+3*sizeof(__m128i)));
                    r4 = RH_MM_LOAD128 ((__m128i *)(nextChunk+4*sizeof(__m128i)));
                    r5 = RH_MM_LOAD128 ((__m128i *)(nextChunk+5*sizeof(__m128i)));
                    r6 = RH_MM_LOAD128 ((__m128i *)(nextChunk+6*sizeof(__m128i)));
                    break;
            case 5: r0 = RH_MM_LOAD128 ((__m128i *)(nextChunk+0*sizeof(__m128i)));
                    r1 = RH_MM_LOAD128 ((__m128i *)(nextChunk+1*sizeof(__m128i)));
                    r2 = RH_MM_LOAD128 ((__m128i *)(nextChunk+2*sizeof(__m128i)));
                    r3 = RH_MM_LOAD128 ((__m128i *)(nextChunk+3*sizeof(__m128i)));
                    r4 = RH_MM_LOAD128 ((__m128i *)(nextChunk+4*sizeof(__m128i)));
                    r5 = RH_MM_LOAD128 ((__m128i *)(nextChunk+5*sizeof(__m128i)));
                    break;
            case 4: r0 = RH_MM_LOAD128 ((__m128i *)(nextChunk+0*sizeof(__m128i)));
                    r1 = RH_MM_LOAD128 ((__m128i *)(nextChunk+1*sizeof(__m128i)));
                    r2 = RH_MM_LOAD128 ((__m128i *)(nextChunk+2*sizeof(__m128i)));
                    r3 = RH_MM_LOAD128 ((__m128i *)(nextChunk+3*sizeof(__m128i)));
                    r4 = RH_MM_LOAD128 ((__m128i *)(nextChunk+4*sizeof(__m128i)));
                    break;
            case 3: r0 = RH_MM_LOAD128 ((__m128i *)(nextChunk+0*sizeof(__m128i)));
                    r1 = RH_MM_LOAD128 ((__m128i *)(nextChunk+1*sizeof(__m128i)));
                    r2 = RH_MM_LOAD128 ((__m128i *)(nextChunk+2*sizeof(__m128i)));
                    r3 = RH_MM_LOAD128 ((__m128i *)(nextChunk+3*sizeof(__m128i)));
                    break;
            case 2: r0 = RH_MM_LOAD128 ((__m128i *)(nextChunk+0*sizeof(__m128i)));
                    r1 = RH_MM_LOAD128 ((__m128i *)(nextChunk+1*sizeof(__m128i)));
                    r2 = RH_MM_LOAD128 ((__m128i *)(nextChunk+2*sizeof(__m128i)));
                    break;
            case 1: r0 = RH_MM_LOAD128 ((__m128i *)(nextChunk+0*sizeof(__m128i)));
                    r1 = RH_MM_LOAD128 ((__m128i *)(nextChunk+1*sizeof(__m128i)));
                    break;
            case 0: r0 = RH_MM_LOAD128 ((__m128i *)(nextChunk+0*sizeof(__m128i)));
                    break;
            default: RHMINER_ASSERT(false);
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

        //RH_M3_GET_BYTE
 
        U32 d;
        #define RH_GB128(chunk128, n)                                         \
            {                                                               \
                d = ((n) & 0x7)*8;                                          \
                switch((n)>>2)                                              \
                {                                                           \
                    case 0:b = _mm_extract_epi32_M(chunk128, 0)>>d; break;  \
                    case 1:b = _mm_extract_epi32_M(chunk128, 1)>>d; break;  \
                    case 2:b = _mm_extract_epi32_M(chunk128, 2)>>d; break;  \
                    case 3:b = _mm_extract_epi32_M(chunk128, 3)>>d; break;  \
                    default:                                                \
                        RHMINER_ASSERT(false);                              \
                };                                                          \
            }

        U8 b;
        U32 val = x % size;
        U32 reg = val / 16;
        U32 n = val % 16;
        switch(reg)
        {
            case 7: RH_GB128(r7, n)  break;
            case 6: RH_GB128(r6, n)  break;
            case 5: RH_GB128(r5, n)  break;
            case 4: RH_GB128(r4, n)  break;
            case 3: RH_GB128(r3, n)  break;
            case 2: RH_GB128(r2, n)  break;
            case 1: RH_GB128(r1, n)  break;
            case 0: RH_GB128(r0, n)  break;
            default: RHMINER_ASSERT(false);
        }
        
        *head = b;
        head++;
    }
}
*/

void Transfo0(U8* workBytes, U8* nextChunk, U32 size)
{
    U32 rndState = MurmurHash3_x86_32_Fast(nextChunk,size);
    if (!rndState)
        rndState = 1;

    memcpy(&workBytes[0], nextChunk, size);
    U8* head = nextChunk;
    U8* end = head + size;
    string _s;
    while(head < end)
    {
        uint32_t x = rndState;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        rndState = x;
        *head = workBytes[x % size];
        head++;
    }
}
/*
void TestTrx0()
{
    U8 workBytes[256];
    RH_ALIGN(32) U8 nextChunk1[256];
    RH_ALIGN(32) U8 nextChunk[256];

    //test platform alignment
    RHMINER_ASSERT((size_t(&nextChunk1[0]) % 32) == 0);

    mersenne_twister_state mtss;
    merssen_twister_seed(0x77665544, &mtss);
    for( int size = 1; size <= 128; size++)
    {
        for (int t = 0; t < 1024; t++)
        {
            memset(nextChunk, 0, 256);
            memset(nextChunk1, 0, 256);
            for (U32 i = 0; i < size; i++)
            {
                nextChunk[i] = merssen_twister_rand(&mtss) % 255;
                nextChunk1[i] = nextChunk[i];
            }

            Transfo0(workBytes, nextChunk1, size);
            Transfo0_128(nextChunk, size);
            RHMINER_ASSERT(memcmp(nextChunk, nextChunk1, size) == 0);
        }
    }
}
*/



void RunUnitTests(U32 coinID)
{   
    extern string g_selectedMiningCoin;
    
    //if (g_selectedMiningCoin == RH_COIN_NAME_PASC)
    {
        g_selectedMiningCoin = RH_COIN_NAME_PASC;
        PrintOut("Running basic Unit Tests for PASC\n");
        TestSHA_Intrinsic();
        Test_merssen_twister_rand();
        TestRandomHash2();
        TestRandomHash2RoundsSeq();
        TestRandomHash2Cache();
        TestRandomHash2Resume();
        TestRSA2();
        TestScrambling();
    }

    //if (g_selectedMiningCoin == RH_COIN_NAME_VNET)
    {
        g_selectedMiningCoin = RH_COIN_NAME_VNET;
        PrintOut("Running basic Unit Tests for VNET\n");
    }
    
    PrintOut("Unit Tests DONE \n");
    exit(0);
}




#endif //#if defined(RHMINER_DEBUG_RANDOMHASH_UNITTEST) 



//--------------------------------------------------------------------------------------------------------
#if defined(RHMINER_DEBUG_RANDOMHASH_UNITTEST_CUDA) && defined(RANDOMHASH_CUDA)
#include "cudalib/cuda_helper.h"

CUDA_DECL_HOST
inline void CUDA_SYM(TestMidsize)()
{
    mersenne_twister_state mtss;
    _CM(merssen_twister_seed)(0x27374757, &mtss);
    const char* result[] = { "519063c65dc5e7e0ad204c18bdcf492fde1a01ac885eab371c7aa4a5cf648497",
                             "be2c575e152d71273cec1dbdffb56346b5374112965baa2c7e8b921e10fa4d8c",
                             "d156b6b555eff3d87a4f7ea774e56136911f83198d8102fddac73973dc3d12ba",
                             "469514779990e0b4faa4a8c16052c13aa60fc8cdf66e6963a8998314947352f4",
                             "6229d20f8cf2a50c8804e697f2248fff388b76efda7e60e47e3a5ca6bbb43ca2",
                             "975e00aa4fd875e4b891f7f42ce91cd87bf0176387855ba26269fcd8fa1bc526",
                             "770ac7bc5725b8675e3843e148247880bccf702399953487d699d864a1bf42f9",
                             "e6f09702455018a08f7cd06222e0913fefc5d3e0b544f05acc9435a09be1f09d" };

    const U8 resultBin[8][32] = {{0x51,0x90,0x63,0xc6,0x5d,0xc5,0xe7,0xe0,0xad,0x20,0x4c,0x18,0xbd,0xcf,0x49,0x2f,0xde,0x1a,0x01,0xac,0x88,0x5e,0xab,0x37,0x1c,0x7a,0xa4,0xa5,0xcf,0x64,0x84,0x97},
                                {0xbe,0x2c,0x57,0x5e,0x15,0x2d,0x71,0x27,0x3c,0xec,0x1d,0xbd,0xff,0xb5,0x63,0x46,0xb5,0x37,0x41,0x12,0x96,0x5b,0xaa,0x2c,0x7e,0x8b,0x92,0x1e,0x10,0xfa,0x4d,0x8c},
                                {0xd1,0x56,0xb6,0xb5,0x55,0xef,0xf3,0xd8,0x7a,0x4f,0x7e,0xa7,0x74,0xe5,0x61,0x36,0x91,0x1f,0x83,0x19,0x8d,0x81,0x02,0xfd,0xda,0xc7,0x39,0x73,0xdc,0x3d,0x12,0xba},
                                {0x46,0x95,0x14,0x77,0x99,0x90,0xe0,0xb4,0xfa,0xa4,0xa8,0xc1,0x60,0x52,0xc1,0x3a,0xa6,0x0f,0xc8,0xcd,0xf6,0x6e,0x69,0x63,0xa8,0x99,0x83,0x14,0x94,0x73,0x52,0xf4},
                                {0x62,0x29,0xd2,0x0f,0x8c,0xf2,0xa5,0x0c,0x88,0x04,0xe6,0x97,0xf2,0x24,0x8f,0xff,0x38,0x8b,0x76,0xef,0xda,0x7e,0x60,0xe4,0x7e,0x3a,0x5c,0xa6,0xbb,0xb4,0x3c,0xa2},
                                {0x97,0x5e,0x00,0xaa,0x4f,0xd8,0x75,0xe4,0xb8,0x91,0xf7,0xf4,0x2c,0xe9,0x1c,0xd8,0x7b,0xf0,0x17,0x63,0x87,0x85,0x5b,0xa2,0x62,0x69,0xfc,0xd8,0xfa,0x1b,0xc5,0x26},
                                {0x77,0x0a,0xc7,0xbc,0x57,0x25,0xb8,0x67,0x5e,0x38,0x43,0xe1,0x48,0x24,0x78,0x80,0xbc,0xcf,0x70,0x23,0x99,0x95,0x34,0x87,0xd6,0x99,0xd8,0x64,0xa1,0xbf,0x42,0xf9},
                                {0xe6,0xf0,0x97,0x02,0x45,0x50,0x18,0xa0,0x8f,0x7c,0xd0,0x62,0x22,0xe0,0x91,0x3f,0xef,0xc5,0xd3,0xe0,0xb5,0x44,0xf0,0x5a,0xcc,0x94,0x35,0xa0,0x9b,0xe1,0xf0,0x9d}};
    U8 in2[] = { 0x90, 0x01, 0x00, 0x00, 0xCA, 0x02, 0x20, 0x00, 0xBB, 0x71, 0x8B, 0x4B, 0x00, 0xD6, 0xF7, 0x44, 0x78, 0xC3, 0x32, 0xF5, 0xFB, 0x31, 0x05, 0x07, 0xE5, 0x5A, 0x9E, 0xF9, 0xB3, 0x85, 0x51, 0xF6, 0x38, 0x58, 0xE3, 0xF7, 0xC8, 0x6D, 0xBD, 0x00, 0x20, 0x00, 0x06, 0xF6, 0x9A, 0xFA, 0xE8, 0xA6, 0xB0, 0x73, 0x5B, 0x6A, 0xCF, 0xCC, 0x58, 0xB7, 0x86, 0x5F, 0xC8, 0x41, 0x88, 0x97, 0xC5, 0x30, 0x21, 0x1F, 0x19, 0x14, 0x0C, 0x9F, 0x95, 0xF2, 0x45, 0x32, 0x40, 0x42, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x08, 0x61, 0x73, 0x64, 0x66, 0x2E, 0x50, 0x6F, 0x6C, 0x79, 0x6D, 0x69, 0x6E, 0x65, 0x72, 0x31, 0x50, 0x6F, 0x6C, 0x79, 0x6D, 0x69, 0x6E, 0x65, 0x72, 0x31, 0x50, 0x62, 0x37, 0x38, 0x61, 0x31, 0x35, 0x33, 0x66, 0x1A, 0x8F, 0x7B, 0xAA, 0xC0, 0x8A, 0xF9, 0x18, 0xB3, 0x63, 0xC5, 0x7F, 0xB5, 0x49, 0x48, 0xBF, 0xA1, 0xC8, 0xEB, 0xB6, 0xF8, 0xDF, 0x88, 0xD5, 0x3C, 0xE0, 0x96, 0x83, 0x95, 0x16, 0xC6, 0x63, 0xE3, 0xB0, 0xC4, 0x42, 0x98, 0xFC, 0x1C, 0x14, 0x9A, 0xFB, 0xF4, 0xC8, 0x99, 0x6F, 0xB9, 0x24, 0x27, 0xAE, 0x41, 0xE4, 0x64, 0x9B, 0x93, 0x4C, 0xA4, 0x95, 0x99, 0x1B, 0x78, 0x52, 0xB8, 0x55, 0x00, 0x00, 0x00, 0x00, 0x5A, 0xCB, 0x92, 0x5B, 0xE7, 0x2C, 0x2D, 0x31 };

    RandomHash_State* randomHashArray = g_threadsData;

    U32 count = 0;
    U32 crc = 0;
    U8 output2[256];
    U8* _output2;
    _CM(RandomHash_Alloc)((void**)&_output2, 256);
    U8 input2[PascalHeaderSize];
    memcpy(input2, &in2[0], PascalHeaderSize);
    
    //extern __host__ void cuda_randomhash_init(uint32_t* input, U32 nonce2);
    //cuda_randomhash_init((uint32_t*)&input2[0], 0x20b7436d);
    _CM(RandomHash_SetHeader)(&randomHashArray[0], &input2[0], 0x20b7436d);
    _CM(RandomHash_SetHeader)(&randomHashArray[1], &input2[0], 0x20b7436d); 
    _CM(RandomHash_SetHeader)(&randomHashArray[2], &input2[0], 0x20b7436d);
    _CM(RandomHash_SetHeader)(&randomHashArray[3], &input2[0], 0x20b7436d);
    U32 noncep = *(U32*)&input2[PascalHeaderSize - 4];

    int threadsPerBlock = 2;
    int blocks = 2;
    for(int i=0; i < 8; i++)
    {
        //RandomHash_Search(&randomHashArray[0], _output2, noncep);
        {
            RandomHash_State* allStates = g_threadsData;
            CUDA_SYM(RandomHash_Init)<<<threadsPerBlock, blocks>>>(allStates, (uint8_t*)_output2, noncep);
//CUDA_SAFE_CALL(cudaDeviceSynchronize());
            RH_CALL_ALL_KERNEL_BLOCKS
CUDA_SAFE_CALL(cudaDeviceSynchronize());
            CUDA_SYM(RandomHash_Finalize)<<<threadsPerBlock, blocks>>>(allStates, (uint8_t*)_output2);
            CUDA_SAFE_CALL(cudaDeviceSynchronize());
        }

        if (count < 8)
        {
            cudaMemcpy(output2, cuGetMemberPtr(g_threadsData, RH_GET_MEMBER_POS(RandomHash_State, m_workBytes)), 32, cudaMemcpyDeviceToHost);
            //printf("\n #%d %s\n == %s\n", count, toHex((void*)output2, 32).c_str(), result[count]);
            printf("\n #%d ", count);
            _CM(RandomHash_PrintHex)((void*)output2, 32, false, false);
            printf("\n                     == %s\n",result[count]);

            RHMINER_ASSERT(memcmp( (void*)output2, resultBin[count], 32 ) == 0);
        }

        count++;
        crc += *(U32*)(_output2 + 3);
    }
   printf("DONE Performance test : CNT %d c %u\n ",count, crc);

    //exit(0);
}

#ifdef RHMINER_DEBUG_RANDOMHASH_UNITTEST_CUDA

__host__ void cuda_randomhash_init(uint32_t* input, U32 nonce2)
{
    //init nonce to ZERO, each thread uses gid as the nonce
//#ifdef RHMINER_DEBUG_RANDOMHASH_UNITTEST_CUDA
    input[PascalHeaderNoncePosV4(PascalHeaderSize) / 4] = 0;
//#endif      

    for(int i=0; i < g_threadsDataSize; i++)
    {
        CUDA_SYM(RandomHash_SetHeader)(&g_threadsData[i], (U8*)input, nonce2);
    }
}

__host__ void cuda_randomhash_search(uint32_t blocks, uint32_t threadsPerBlock, cudaStream_t stream, uint32_t* input, uint32_t* output, U32 startNonce)
{  
    //Runtime change of total thrads NOT IMPLEMENTED YET
    //RHMINER_ASSERT((threadsPerBlock*blocks == g_threadsDataSize));    
    //RHMINER_DEBUG_RANDOMHASH_TIMINGS_BEGIN();
    RandomHash_State* allStates = g_threadsData;

#if 1
    //test Murmur3 accums
    //CUDA_SYM(TestMidsizeKernel)<<<1, 1>>>();
    _CM(TestMidsize)();
    
#endif

    RHMINER_ASSERT(threadsPerBlock*blocks >= 4);
    //to match the CPU thest that uses 4 threads
    blocks = 2;
    threadsPerBlock = 2;

    int c = 0;
    mersenne_twister_state rnd;
    _CM(merssen_twister_seed)(0xF923A401, &rnd);
    //_CM(RandomHash_SetHeaderSize)(PascalHeaderSize);
    do
    {
        for (int i = 0; i < PascalHeaderSize / 4; i++)
            input[i] = merssen_twister_rand(&rnd);
        
        startNonce = 0;
        cudaDeviceSynchronize();
        cuda_randomhash_init(input, 0);

        CUDA_SYM(RandomHash_Init)<<<threadsPerBlock, blocks>>>(allStates, (uint8_t*)output, startNonce);
        RH_CALL_ALL_KERNEL_BLOCKS
        CUDA_SYM(RandomHash_Finalize)<<<threadsPerBlock, blocks>>>(allStates, (uint8_t*)output);

        printf("--------------- Test %d\n", c);
        CUDA_SAFE_CALL(cudaDeviceSynchronize());
        //U32 i = (int)err+1; while (i < 0xFFFFFFF) i += (int)err; printf("%d\n", i); \
        //basic test to see if something broke ! 
        static bool onlyonce = true;
        if (onlyonce)
        {
            onlyonce = false;
            //print output !
            U64 hashBuf[32/8];
            for(int i=0; i < 4; i++)
            {
                #ifndef RHMINER_DEBUG_DEV
                #error Add RHMINER_DEBUG_DEV ...
                #endif
                cudaMemcpy((void*)&hashBuf, g_threadsData[i].m_workBytes, 32, cudaMemcpyDeviceToHost);
                KERNEL_LOG("Thread %d HASH %llX %llx %llx %llx\n",  i, hashBuf[0], hashBuf[1], hashBuf[2], hashBuf[3]);
            }            
            // 50d757f17f6bd69dcb5668c3466ff6d7047a3aea76efcaa7028ef8d2a95b398233f6fe7623a47d2eebf4fc9b44fc55b3f063a27dc7e3d337c3ff07e90d7e9e440605b1aa92e24de4ff7603ec18cdec57edbb548cba5056eb2a9e9d28f54259a91fa9a5a0eacd55fa7c7f065a4aef162ef00f5dcd91b34bbee8ba8a551c17bbc8385c981ec604b04a094f4b1a15edb9d149eb9f41e48f60bb45696b53caf5130835a11c599923774749211b6a4f0d6d5b8342a98773e789cc27beebc10cb3077a8c79fd8800000000
            // 50d757f17f6bd69dcb5668c3466ff6d7047a3aea76efcaa7028ef8d2a95b398233f6fe7623a47d2eebf4fc9b44fc55b3f063a27dc7e3d337c3ff07e90d7e9e440605b1aa92e24de4ff7603ec18cdec57edbb548cba5056eb2a9e9d28f54259a91fa9a5a0eacd55fa7c7f065a4aef162ef00f5dcd91b34bbee8ba8a551c17bbc8385c981ec604b04a094f4b1a15edb9d149eb9f41e48f60bb45696b53caf5130835a11c599923774749211b6a4f0d6d5b8342a98773e789cc27beebc10cb3077a8c79fd8801000000
            // 50d757f17f6bd69dcb5668c3466ff6d7047a3aea76efcaa7028ef8d2a95b398233f6fe7623a47d2eebf4fc9b44fc55b3f063a27dc7e3d337c3ff07e90d7e9e440605b1aa92e24de4ff7603ec18cdec57edbb548cba5056eb2a9e9d28f54259a91fa9a5a0eacd55fa7c7f065a4aef162ef00f5dcd91b34bbee8ba8a551c17bbc8385c981ec604b04a094f4b1a15edb9d149eb9f41e48f60bb45696b53caf5130835a11c599923774749211b6a4f0d6d5b8342a98773e789cc27beebc10cb3077a8c79fd8802000000
            // 50d757f17f6bd69dcb5668c3466ff6d7047a3aea76efcaa7028ef8d2a95b398233f6fe7623a47d2eebf4fc9b44fc55b3f063a27dc7e3d337c3ff07e90d7e9e440605b1aa92e24de4ff7603ec18cdec57edbb548cba5056eb2a9e9d28f54259a91fa9a5a0eacd55fa7c7f065a4aef162ef00f5dcd91b34bbee8ba8a551c17bbc8385c981ec604b04a094f4b1a15edb9d149eb9f41e48f60bb45696b53caf5130835a11c599923774749211b6a4f0d6d5b8342a98773e789cc27beebc10cb3077a8c79fd8803000000
            //Thread 0 HASH 37D5065986C97CCA a240eafa25b495e2 3896869a83c8af09 69be038ade18efd6
            //Thread 1 HASH 9FA3A500885990DE 7792a125969c8c03 c6555359a70ba4aa f98b316cb60b73a9
            //Thread 2 HASH 5F7EED3E9220F07E e5a9194579d25040 675b0aab966c140 a0997cab49864be0
            //Thread 3 HASH 4420B9301971AA96 40f1acbe509d07d4 eac856e54cd8fe38 6a7367216b423c2c
            //NOTE: M=10K*5 numbers
            RHMINER_ASSERT(hashBuf[0] == 0x4420B9301971AA96 &&
                        hashBuf[1] == 0x40f1acbe509d07d4 &&
                        hashBuf[2] == 0xeac856e54cd8fe38 &&
                        hashBuf[3] == 0x6a7367216b423c2c);
        }
        c++;
    } while (c <= 10);

    //So timings take the *c into account
    threadsPerBlock *= c;
    //RHMINER_DEBUG_RANDOMHASH_TIMINGS_END(threadsPerBlock*blocks);
}
#endif //RHMINER_DEBUG_RANDOMHASH_UNITTEST_CUDA


#endif //#if defined(RHMINER_DEBUG_RANDOMHASH_UNITTEST) && !defined(RANDOMHASH_CUDA)
