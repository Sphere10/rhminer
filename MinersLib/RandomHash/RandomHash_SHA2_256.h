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

#include "RandomHash_core.h"
#ifndef RHMINER_NO_SIMD
    #include <immintrin.h>
#endif

#define SHA2_256_BLOCK_SIZE 64

void SHA2_256_RoundFunction(uint32_t* data, uint32_t* state)
{
    uint32_t A, B, C, D, E, F, G, H, T, T2;

    //scratch buffer 64 uint32
    RH_ALIGN(128) uint32_t beData[64];
    {
        /*RH_swap_u32(data[0]) =  RH_swap_u32(data[0]);
        RH_swap_u32(data[1]) =  RH_swap_u32(data[1]);
        RH_swap_u32(data[2]) =  RH_swap_u32(data[2]);
        RH_swap_u32(data[3]) =  RH_swap_u32(data[3]);
        RH_swap_u32(data[4]) =  RH_swap_u32(data[4]);
        RH_swap_u32(data[5]) =  RH_swap_u32(data[5]);
        RH_swap_u32(data[6]) =  RH_swap_u32(data[6]);
        RH_swap_u32(data[7]) =  RH_swap_u32(data[7]);
        RH_swap_u32(data[8]) =  RH_swap_u32(data[8]);
        RH_swap_u32(data[9]) =  RH_swap_u32(data[9]);
        RH_swap_u32(data[10]) = RH_swap_u32(data[10]);
        RH_swap_u32(data[11]) = RH_swap_u32(data[11]);
        RH_swap_u32(data[12]) = RH_swap_u32(data[12]);
        RH_swap_u32(data[13]) = RH_swap_u32(data[13]);
        RH_swap_u32(data[14]) = RH_swap_u32(data[14]);
        RH_swap_u32(data[15]) = RH_swap_u32(data[15]);*/
    }

    A = state[0];
    B = state[1];
    C = state[2];
    D = state[3];
    E = state[4];
    F = state[5];
    G = state[6];
    H = state[7];


    // Step 1

    T = RH_swap_u32(data[14]);
    T2 = RH_swap_u32(data[1]);
    beData[16] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + RH_swap_u32(data[9]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[0]);

    T = RH_swap_u32(data[15]);
    T2 = RH_swap_u32(data[2]);
    beData[17] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + RH_swap_u32(data[10]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[1]);

    T = beData[16];
    T2 = RH_swap_u32(data[3]);
    beData[18] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + RH_swap_u32(data[11]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[2]);

    T = beData[17];
    T2 = RH_swap_u32(data[4]);
    beData[19] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + RH_swap_u32(data[12]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[3]);

    T = beData[18];
    T2 = RH_swap_u32(data[5]);
    beData[20] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + RH_swap_u32(data[13]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[4]);

    T = beData[19];
    T2 = RH_swap_u32(data[6]);
    beData[21] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + RH_swap_u32(data[14]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[5]);

    T = beData[20];
    T2 = RH_swap_u32(data[7]);
    beData[22] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + RH_swap_u32(data[15]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[6]);

    T = beData[21];
    T2 = RH_swap_u32(data[8]);
    beData[23] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[16] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[7]);

    T = beData[22];
    T2 = RH_swap_u32(data[9]);
    beData[24] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[17] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[8]);

    T = beData[23];
    T2 = RH_swap_u32(data[10]);
    beData[25] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[18] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[9]);

    T = beData[24];
    T2 = RH_swap_u32(data[11]);
    beData[26] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[19] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[10]);

    T = beData[25];
    T2 = RH_swap_u32(data[12]);
    beData[27] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[20] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[11]);

    T = beData[26];
    T2 = RH_swap_u32(data[13]);
    beData[28] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[21] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[12]);

    T = beData[27];
    T2 = RH_swap_u32(data[14]);
    beData[29] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[22] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[13]);

    T = beData[28];
    T2 = RH_swap_u32(data[15]);
    beData[30] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[23] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[14]);

    T = beData[29];
    T2 = beData[16];
    beData[31] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[24] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[15]);

    T = beData[30];
    T2 = beData[17];
    beData[32] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[25] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[16];

    T = beData[31];
    T2 = beData[18];
    beData[33] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[26] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[17];

    T = beData[32];
    T2 = beData[19];
    beData[34] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[27] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[18];

    T = beData[33];
    T2 = beData[20];
    beData[35] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[28] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[19];

    T = beData[34];
    T2 = beData[21];
    beData[36] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[29] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[20];

    T = beData[35];
    T2 = beData[22];
    beData[37] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[30] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[21];

    T = beData[36];
    T2 = beData[23];
    beData[38] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[31] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[22];

    T = beData[37];
    T2 = beData[24];
    beData[39] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[32] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[23];

    T = beData[38];
    T2 = beData[25];
    beData[40] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[33] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[24];

    T = beData[39];
    T2 = beData[26];
    beData[41] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[34] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[25];

    T = beData[40];
    T2 = beData[27];
    beData[42] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[35] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[26];

    T = beData[41];
    T2 = beData[28];
    beData[43] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[36] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[27];

    T = beData[42];
    T2 = beData[29];
    beData[44] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[37] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[28];

    T = beData[43];
    T2 = beData[30];
    beData[45] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[38] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[29];

    T = beData[44];
    T2 = beData[31];
    beData[46] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[39] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[30];

    T = beData[45];
    T2 = beData[32];
    beData[47] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[40] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[31];

    T = beData[46];
    T2 = beData[33];
    beData[48] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[41] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[32];

    T = beData[47];
    T2 = beData[34];
    beData[49] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[42] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[33];

    T = beData[48];
    T2 = beData[35];
    beData[50] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[43] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[34];

    T = beData[49];
    T2 = beData[36];
    beData[51] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[44] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[35];

    T = beData[50];
    T2 = beData[37];
    beData[52] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[45] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[36];

    T = beData[51];
    T2 = beData[38];
    beData[53] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[46] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[37];

    T = beData[52];
    T2 = beData[39];
    beData[54] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[47] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[38];

    T = beData[53];
    T2 = beData[40];
    beData[55] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[48] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[39];

    T = beData[54];
    T2 = beData[41];
    beData[56] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[49] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[40];

    T = beData[55];
    T2 = beData[42];
    beData[57] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[50] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[41];

    T = beData[56];
    T2 = beData[43];
    beData[58] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[51] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[42];

    T = beData[57];
    T2 = beData[44];
    beData[59] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[52] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[43];

    T = beData[58];
    T2 = beData[45];
    beData[60] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[53] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[44];

    T = beData[59];
    T2 = beData[46];
    beData[61] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[54] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[45];

    T = beData[60];
    T2 = beData[47];
    beData[62] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[55] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[46];

    T = beData[61];
    T2 = beData[48];
    beData[63] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[56] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[47];

    // Step 2

    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x428A2F98 + RH_swap_u32(data[0]);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x71374491 + RH_swap_u32(data[1]);
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB5C0FBCF + RH_swap_u32(data[2]);
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xE9B5DBA5 + RH_swap_u32(data[3]);
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x3956C25B + RH_swap_u32(data[4]);
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x59F111F1 + RH_swap_u32(data[5]);
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x923F82A4 + RH_swap_u32(data[6]);
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xAB1C5ED5 + RH_swap_u32(data[7]);
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xD807AA98 + RH_swap_u32(data[8]);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x12835B01 + RH_swap_u32(data[9]);
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x243185BE + RH_swap_u32(data[10]);
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x550C7DC3 + RH_swap_u32(data[11]);
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x72BE5D74 + RH_swap_u32(data[12]);
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x80DEB1FE + RH_swap_u32(data[13]);
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x9BDC06A7 + RH_swap_u32(data[14]);
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC19BF174 + RH_swap_u32(data[15]);
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xE49B69C1 + beData[16];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xEFBE4786 + beData[17];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x0FC19DC6 + beData[18];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x240CA1CC + beData[19];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x2DE92C6F + beData[20];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4A7484AA + beData[21];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5CB0A9DC + beData[22];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x76F988DA + beData[23];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x983E5152 + beData[24];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA831C66D + beData[25];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB00327C8 + beData[26];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xBF597FC7 + beData[27];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xC6E00BF3 + beData[28];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD5A79147 + beData[29];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x06CA6351 + beData[30];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x14292967 + beData[31];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x27B70A85 + beData[32];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x2E1B2138 + beData[33];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x4D2C6DFC + beData[34];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x53380D13 + beData[35];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x650A7354 + beData[36];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x766A0ABB + beData[37];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x81C2C92E + beData[38];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x92722C85 + beData[39];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xA2BFE8A1 + beData[40];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA81A664B + beData[41];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xC24B8B70 + beData[42];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xC76C51A3 + beData[43];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xD192E819 + beData[44];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD6990624 + beData[45];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xF40E3585 + beData[46];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x106AA070 + beData[47];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x19A4C116 + beData[48];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x1E376C08 + beData[49];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x2748774C + beData[50];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x34B0BCB5 + beData[51];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x391C0CB3 + beData[52];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4ED8AA4A + beData[53];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5B9CCA4F + beData[54];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x682E6FF3 + beData[55];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x748F82EE + beData[56];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x78A5636F + beData[57];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x84C87814 + beData[58];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x8CC70208 + beData[59];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x90BEFFFA + beData[60];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xA4506CEB + beData[61];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xBEF9A3F7 + beData[62];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC67178F2 + beData[63];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;

    state[0] += A;
    state[1] += B;
    state[2] += C;
    state[3] += D;
    state[4] += E;
    state[5] += F;
    state[6] += G;
    state[7] += H;
}


// NOTE Slower with SHA2_256_RoundFunction_Last28, SHA2_256_RoundFunction_Last20 and SHA2_256_RoundFunction_Last32 togetter

//32 : 53C16FE5 90C854DD 9EC9CAD4 37DDB06C EF068213 944F631E C8E91059 BD297E02       80        0        0        0        0        0        0    10000  
void SHA2_256_RoundFunction_Last32(uint32_t* data, uint32_t* state)
{
    uint32_t A, B, C, D, E, F, G, H, T, T2;

    //scratch buffer 64 uint32
    RH_ALIGN(128) uint32_t beData[64];
/*
    RHMINER_ASSERT(0x80 == data[8]); // 0x80000000
    RHMINER_ASSERT(0 == data[9]);
    RHMINER_ASSERT(0 == data[10]);
    RHMINER_ASSERT(0 == data[11]);
    RHMINER_ASSERT(0 == data[12]);
    RHMINER_ASSERT(0 == data[13]);
    RHMINER_ASSERT(0 == data[14]);
    RHMINER_ASSERT(0x10000 == data[15]); //0x00000100
*/

    A = state[0];
    B = state[1];
    C = state[2];
    D = state[3];
    E = state[4];
    F = state[5];
    G = state[6];
    H = state[7];


    // Step 1

    //T = 0;
    T2 = RH_swap_u32(data[1]);
    beData[16] = ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[0]);

    //T = 0x00000100;
    T2 = RH_swap_u32(data[2]);
    beData[17] = ((ROTR32(0x00000100, 17)) ^ (ROTR32(0x00000100, 19)) ^ (0x00000100 >> 10))  + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[1]);

    T = beData[16];
    T2 = RH_swap_u32(data[3]);
    beData[18] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10))  + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[2]);

    T = beData[17];
    T2 = RH_swap_u32(data[4]);
    beData[19] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10))  + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[3]);

    T = beData[18];
    T2 = RH_swap_u32(data[5]);
    beData[20] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10))  + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[4]);

    T = beData[19];
    T2 = RH_swap_u32(data[6]);
    beData[21] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10))  + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[5]);

    T = beData[20];
    T2 = RH_swap_u32(data[7]);
    beData[22] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x00000100 + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[6]);

    T = beData[21];
    //T2 = 0x80000000;
    beData[23] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[16] + ((ROTR32(0x80000000, 7)) ^ (ROTR32(0x80000000, 18)) ^ (0x80000000 >> 3)) + RH_swap_u32(data[7]);

    T = beData[22];
    //T2 = 0;
    beData[24] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[17] + 0x80000000;

    T = beData[23];
    //T2 = 0;
    beData[25] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[18] ;

    T = beData[24];
    //T2 = 0;
    beData[26] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[19] ;

    T = beData[25];
    //T2 = 0;
    beData[27] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[20] ;

    T = beData[26];
    //T2 = 0;
    beData[28] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[21] ;

    T = beData[27];
    //T2 = 0;
    beData[29] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[22] ;

    T = beData[28];
    //T2 = 0x00000100;
    beData[30] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[23] + ((ROTR32(0x00000100, 7)) ^ (ROTR32(0x00000100, 18)) ^ (0x00000100 >> 3)) ;

    T = beData[29];
    T2 = beData[16];
    beData[31] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[24] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + 0x00000100;

    T = beData[30];
    T2 = beData[17];
    beData[32] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[25] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[16];

    T = beData[31];
    T2 = beData[18];
    beData[33] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[26] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[17];

    T = beData[32];
    T2 = beData[19];
    beData[34] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[27] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[18];

    T = beData[33];
    T2 = beData[20];
    beData[35] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[28] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[19];

    T = beData[34];
    T2 = beData[21];
    beData[36] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[29] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[20];

    T = beData[35];
    T2 = beData[22];
    beData[37] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[30] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[21];

    T = beData[36];
    T2 = beData[23];
    beData[38] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[31] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[22];

    T = beData[37];
    T2 = beData[24];
    beData[39] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[32] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[23];

    T = beData[38];
    T2 = beData[25];
    beData[40] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[33] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[24];

    T = beData[39];
    T2 = beData[26];
    beData[41] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[34] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[25];

    T = beData[40];
    T2 = beData[27];
    beData[42] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[35] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[26];

    T = beData[41];
    T2 = beData[28];
    beData[43] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[36] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[27];

    T = beData[42];
    T2 = beData[29];
    beData[44] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[37] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[28];

    T = beData[43];
    T2 = beData[30];
    beData[45] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[38] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[29];

    T = beData[44];
    T2 = beData[31];
    beData[46] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[39] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[30];

    T = beData[45];
    T2 = beData[32];
    beData[47] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[40] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[31];

    T = beData[46];
    T2 = beData[33];
    beData[48] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[41] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[32];

    T = beData[47];
    T2 = beData[34];
    beData[49] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[42] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[33];

    T = beData[48];
    T2 = beData[35];
    beData[50] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[43] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[34];

    T = beData[49];
    T2 = beData[36];
    beData[51] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[44] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[35];

    T = beData[50];
    T2 = beData[37];
    beData[52] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[45] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[36];

    T = beData[51];
    T2 = beData[38];
    beData[53] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[46] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[37];

    T = beData[52];
    T2 = beData[39];
    beData[54] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[47] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[38];

    T = beData[53];
    T2 = beData[40];
    beData[55] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[48] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[39];

    T = beData[54];
    T2 = beData[41];
    beData[56] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[49] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[40];

    T = beData[55];
    T2 = beData[42];
    beData[57] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[50] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[41];

    T = beData[56];
    T2 = beData[43];
    beData[58] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[51] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[42];

    T = beData[57];
    T2 = beData[44];
    beData[59] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[52] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[43];

    T = beData[58];
    T2 = beData[45];
    beData[60] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[53] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[44];

    T = beData[59];
    T2 = beData[46];
    beData[61] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[54] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[45];

    T = beData[60];
    T2 = beData[47];
    beData[62] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[55] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[46];

    T = beData[61];
    T2 = beData[48];
    beData[63] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[56] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[47];

    // Step 2

    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x428A2F98 + RH_swap_u32(data[0]);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x71374491 + RH_swap_u32(data[1]);
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB5C0FBCF + RH_swap_u32(data[2]);
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xE9B5DBA5 + RH_swap_u32(data[3]);
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x3956C25B + RH_swap_u32(data[4]);
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x59F111F1 + RH_swap_u32(data[5]);
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x923F82A4 + RH_swap_u32(data[6]);
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xAB1C5ED5 + RH_swap_u32(data[7]);
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xD807AA98 + 0x80000000;
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x12835B01 ;
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x243185BE ;
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x550C7DC3 ;
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x72BE5D74 ;
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x80DEB1FE ;
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x9BDC06A7 ;
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC19BF174 + 0x00000100;
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xE49B69C1 + beData[16];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xEFBE4786 + beData[17];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x0FC19DC6 + beData[18];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x240CA1CC + beData[19];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x2DE92C6F + beData[20];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4A7484AA + beData[21];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5CB0A9DC + beData[22];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x76F988DA + beData[23];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x983E5152 + beData[24];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA831C66D + beData[25];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB00327C8 + beData[26];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xBF597FC7 + beData[27];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xC6E00BF3 + beData[28];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD5A79147 + beData[29];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x06CA6351 + beData[30];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x14292967 + beData[31];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x27B70A85 + beData[32];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x2E1B2138 + beData[33];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x4D2C6DFC + beData[34];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x53380D13 + beData[35];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x650A7354 + beData[36];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x766A0ABB + beData[37];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x81C2C92E + beData[38];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x92722C85 + beData[39];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xA2BFE8A1 + beData[40];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA81A664B + beData[41];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xC24B8B70 + beData[42];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xC76C51A3 + beData[43];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xD192E819 + beData[44];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD6990624 + beData[45];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xF40E3585 + beData[46];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x106AA070 + beData[47];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x19A4C116 + beData[48];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x1E376C08 + beData[49];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x2748774C + beData[50];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x34B0BCB5 + beData[51];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x391C0CB3 + beData[52];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4ED8AA4A + beData[53];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5B9CCA4F + beData[54];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x682E6FF3 + beData[55];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x748F82EE + beData[56];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x78A5636F + beData[57];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x84C87814 + beData[58];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x8CC70208 + beData[59];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x90BEFFFA + beData[60];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xA4506CEB + beData[61];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xBEF9A3F7 + beData[62];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC67178F2 + beData[63];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;

    state[0] += A;
    state[1] += B;
    state[2] += C;
    state[3] += D;
    state[4] += E;
    state[5] += F;
    state[6] += G;
    state[7] += H;
}

#if 0

//28 : 7E912666 2E38AC2E 5CF931A8 D7AE04C4 AC68451A 2CAC67EB 719A0B00 2C005CA8 91AE193D       80        0        0        0        0        0 20030000  
void SHA2_256_RoundFunction_Last28(uint32_t* data, uint32_t* state)
{
    uint32_t A, B, C, D, E, F, G, H, T, T2;

    //scratch buffer 64 uint32
    RH_ALIGN(128) uint32_t beData[64];

    RHMINER_ASSERT(0x80 == data[9]); // 0x80000000
    RHMINER_ASSERT(0 == data[10]);
    RHMINER_ASSERT(0 == data[11]);
    RHMINER_ASSERT(0 == data[12]);
    RHMINER_ASSERT(0 == data[13]);
    RHMINER_ASSERT(0 == data[14]);
    RHMINER_ASSERT(0x20030000 == data[15]); //0x00000320

    A = state[0];
    B = state[1];
    C = state[2];
    D = state[3];
    E = state[4];
    F = state[5];
    G = state[6];
    H = state[7];


    // Step 1

    //T = 0;
    T2 = RH_swap_u32(data[1]);
    beData[16] =  0x80000000 + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[0]);

    //T = 0x00000320;
    T2 = RH_swap_u32(data[2]);
    beData[17] = ((ROTR32(0x00000320, 17)) ^ (ROTR32(0x00000320, 19)) ^ (0x00000320 >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[1]);

    T = beData[16];
    T2 = RH_swap_u32(data[3]);
    beData[18] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[2]);

    T = beData[17];
    T2 = RH_swap_u32(data[4]);
    beData[19] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[3]);

    T = beData[18];
    T2 = RH_swap_u32(data[5]);
    beData[20] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[4]);

    T = beData[19];
    T2 = RH_swap_u32(data[6]);
    beData[21] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[5]);

    T = beData[20];
    T2 = RH_swap_u32(data[7]);
    beData[22] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x00000320 + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[6]);

    T = beData[21];
    T2 = RH_swap_u32(data[8]);
    beData[23] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[16] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[7]);

    T = beData[22];
    //T2 = 0x80000000;
    beData[24] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[17] + ((ROTR32(0x80000000, 7)) ^ (ROTR32(0x80000000, 18)) ^ (0x80000000 >> 3)) + RH_swap_u32(data[8]);

    T = beData[23];
    //T2 = 0;
    beData[25] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[18] + 0x80000000;

    T = beData[24];
    //T2 = 0;
    beData[26] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[19];

    T = beData[25];
    //T2 = 0;
    beData[27] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[20];

    T = beData[26];
    //T2 = 0;
    beData[28] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[21];

    T = beData[27];
    //T2 = 0;
    beData[29] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[22];

    T = beData[28];
    //T2 = 0x00000320;
    beData[30] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[23] + ((ROTR32(0x00000320, 7)) ^ (ROTR32(0x00000320, 18)) ^ (0x00000320 >> 3)) ;

    T = beData[29];
    T2 = beData[16];
    beData[31] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[24] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + 0x00000320;

    T = beData[30];
    T2 = beData[17];
    beData[32] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[25] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[16];

    T = beData[31];
    T2 = beData[18];
    beData[33] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[26] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[17];

    T = beData[32];
    T2 = beData[19];
    beData[34] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[27] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[18];

    T = beData[33];
    T2 = beData[20];
    beData[35] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[28] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[19];

    T = beData[34];
    T2 = beData[21];
    beData[36] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[29] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[20];

    T = beData[35];
    T2 = beData[22];
    beData[37] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[30] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[21];

    T = beData[36];
    T2 = beData[23];
    beData[38] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[31] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[22];

    T = beData[37];
    T2 = beData[24];
    beData[39] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[32] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[23];

    T = beData[38];
    T2 = beData[25];
    beData[40] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[33] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[24];

    T = beData[39];
    T2 = beData[26];
    beData[41] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[34] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[25];

    T = beData[40];
    T2 = beData[27];
    beData[42] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[35] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[26];

    T = beData[41];
    T2 = beData[28];
    beData[43] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[36] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[27];

    T = beData[42];
    T2 = beData[29];
    beData[44] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[37] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[28];

    T = beData[43];
    T2 = beData[30];
    beData[45] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[38] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[29];

    T = beData[44];
    T2 = beData[31];
    beData[46] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[39] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[30];

    T = beData[45];
    T2 = beData[32];
    beData[47] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[40] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[31];

    T = beData[46];
    T2 = beData[33];
    beData[48] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[41] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[32];

    T = beData[47];
    T2 = beData[34];
    beData[49] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[42] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[33];

    T = beData[48];
    T2 = beData[35];
    beData[50] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[43] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[34];

    T = beData[49];
    T2 = beData[36];
    beData[51] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[44] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[35];

    T = beData[50];
    T2 = beData[37];
    beData[52] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[45] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[36];

    T = beData[51];
    T2 = beData[38];
    beData[53] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[46] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[37];

    T = beData[52];
    T2 = beData[39];
    beData[54] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[47] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[38];

    T = beData[53];
    T2 = beData[40];
    beData[55] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[48] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[39];

    T = beData[54];
    T2 = beData[41];
    beData[56] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[49] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[40];

    T = beData[55];
    T2 = beData[42];
    beData[57] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[50] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[41];

    T = beData[56];
    T2 = beData[43];
    beData[58] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[51] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[42];

    T = beData[57];
    T2 = beData[44];
    beData[59] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[52] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[43];

    T = beData[58];
    T2 = beData[45];
    beData[60] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[53] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[44];

    T = beData[59];
    T2 = beData[46];
    beData[61] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[54] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[45];

    T = beData[60];
    T2 = beData[47];
    beData[62] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[55] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[46];

    T = beData[61];
    T2 = beData[48];
    beData[63] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[56] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[47];

    // Step 2

    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x428A2F98 + RH_swap_u32(data[0]);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x71374491 + RH_swap_u32(data[1]);
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB5C0FBCF + RH_swap_u32(data[2]);
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xE9B5DBA5 + RH_swap_u32(data[3]);
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x3956C25B + RH_swap_u32(data[4]);
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x59F111F1 + RH_swap_u32(data[5]);
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x923F82A4 + RH_swap_u32(data[6]);
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xAB1C5ED5 + RH_swap_u32(data[7]);
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xD807AA98 + RH_swap_u32(data[8]);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x12835B01 + 0x80000000;
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x243185BE ;
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x550C7DC3 ;
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x72BE5D74 ;
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x80DEB1FE ;
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x9BDC06A7 ;
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC19BF174 + 0x00000320;
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xE49B69C1 + beData[16];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xEFBE4786 + beData[17];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x0FC19DC6 + beData[18];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x240CA1CC + beData[19];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x2DE92C6F + beData[20];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4A7484AA + beData[21];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5CB0A9DC + beData[22];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x76F988DA + beData[23];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x983E5152 + beData[24];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA831C66D + beData[25];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB00327C8 + beData[26];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xBF597FC7 + beData[27];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xC6E00BF3 + beData[28];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD5A79147 + beData[29];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x06CA6351 + beData[30];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x14292967 + beData[31];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x27B70A85 + beData[32];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x2E1B2138 + beData[33];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x4D2C6DFC + beData[34];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x53380D13 + beData[35];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x650A7354 + beData[36];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x766A0ABB + beData[37];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x81C2C92E + beData[38];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x92722C85 + beData[39];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xA2BFE8A1 + beData[40];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA81A664B + beData[41];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xC24B8B70 + beData[42];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xC76C51A3 + beData[43];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xD192E819 + beData[44];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD6990624 + beData[45];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xF40E3585 + beData[46];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x106AA070 + beData[47];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x19A4C116 + beData[48];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x1E376C08 + beData[49];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x2748774C + beData[50];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x34B0BCB5 + beData[51];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x391C0CB3 + beData[52];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4ED8AA4A + beData[53];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5B9CCA4F + beData[54];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x682E6FF3 + beData[55];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x748F82EE + beData[56];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x78A5636F + beData[57];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x84C87814 + beData[58];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x8CC70208 + beData[59];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x90BEFFFA + beData[60];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xA4506CEB + beData[61];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xBEF9A3F7 + beData[62];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC67178F2 + beData[63];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;

    state[0] += A;
    state[1] += B;
    state[2] += C;
    state[3] += D;
    state[4] += E;
    state[5] += F;
    state[6] += G;
    state[7] += H;
}


#endif //0

//#if 0
//imbeded in Part2
//20 : 9A8EA877 87B50DB7 E8833640 19BB35B7 8322ED2F EEB36563 69ADB4A6  C03CEB9 9471E32D 598D0C3E 76874604       80        0        0        0 60070000  
void SHA2_256_RoundFunction_Last20(uint32_t* data, uint32_t* state)
{
    //add 6% on xeon
    uint32_t A, B, C, D, E, F, G, H, T, T2;

    //scratch buffer 64 uint32
    RH_ALIGN(128) uint32_t beData[64];
    

    RHMINER_ASSERT(0x80 == data[11]); // 0x80000000
    RHMINER_ASSERT(0 == data[12]);
    RHMINER_ASSERT(0 == data[13]);
    RHMINER_ASSERT(0 == data[14]);
    RHMINER_ASSERT(0x60070000 == data[15]); //0x00000760

    A = state[0];
    B = state[1];
    C = state[2];
    D = state[3];
    E = state[4];
    F = state[5];
    G = state[6];
    H = state[7];


    // Step 1

    //T = 0;
    T2 = RH_swap_u32(data[1]);
    beData[16] =  RH_swap_u32(data[9]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[0]);

    //T = 0x00000760;
    T2 = RH_swap_u32(data[2]);
    beData[17] = ((ROTR32(0x00000760, 17)) ^ (ROTR32(0x00000760, 19)) ^ (0x00000760 >> 10)) + RH_swap_u32(data[10]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[1]);

    T = beData[16];
    T2 = RH_swap_u32(data[3]);
    beData[18] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x80000000 + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[2]);

    T = beData[17];
    T2 = RH_swap_u32(data[4]);
    beData[19] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[3]);

    T = beData[18];
    T2 = RH_swap_u32(data[5]);
    beData[20] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[4]);

    T = beData[19];
    T2 = RH_swap_u32(data[6]);
    beData[21] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[5]);

    T = beData[20];
    T2 = RH_swap_u32(data[7]);
    beData[22] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x00000760 + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[6]);

    T = beData[21];
    T2 = RH_swap_u32(data[8]);
    beData[23] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[16] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[7]);

    T = beData[22];
    T2 = RH_swap_u32(data[9]);
    beData[24] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[17] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[8]);

    T = beData[23];
    T2 = RH_swap_u32(data[10]);
    beData[25] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[18] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[9]);

    T = beData[24];
    //T2 = 0x80000000;
    beData[26] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[19] + ((ROTR32(0x80000000, 7)) ^ (ROTR32(0x80000000, 18)) ^ (0x80000000 >> 3)) + RH_swap_u32(data[10]);

    T = beData[25];
    //T2 = 0;
    beData[27] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[20] + 0x80000000;

    T = beData[26];
    //T2 = 0;
    beData[28] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[21];

    T = beData[27];
    //T2 = 0;
    beData[29] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[22];

    T = beData[28];
    //T2 = 0x00000760;
    beData[30] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[23] + ((ROTR32(0x00000760, 7)) ^ (ROTR32(0x00000760, 18)) ^ (0x00000760 >> 3));

    T = beData[29];
    T2 = beData[16];
    beData[31] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[24] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + 0x00000760;

    T = beData[30];
    T2 = beData[17];
    beData[32] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[25] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[16];

    T = beData[31];
    T2 = beData[18];
    beData[33] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[26] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[17];

    T = beData[32];
    T2 = beData[19];
    beData[34] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[27] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[18];

    T = beData[33];
    T2 = beData[20];
    beData[35] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[28] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[19];

    T = beData[34];
    T2 = beData[21];
    beData[36] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[29] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[20];

    T = beData[35];
    T2 = beData[22];
    beData[37] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[30] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[21];

    T = beData[36];
    T2 = beData[23];
    beData[38] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[31] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[22];

    T = beData[37];
    T2 = beData[24];
    beData[39] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[32] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[23];

    T = beData[38];
    T2 = beData[25];
    beData[40] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[33] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[24];

    T = beData[39];
    T2 = beData[26];
    beData[41] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[34] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[25];

    T = beData[40];
    T2 = beData[27];
    beData[42] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[35] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[26];

    T = beData[41];
    T2 = beData[28];
    beData[43] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[36] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[27];

    T = beData[42];
    T2 = beData[29];
    beData[44] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[37] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[28];

    T = beData[43];
    T2 = beData[30];
    beData[45] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[38] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[29];

    T = beData[44];
    T2 = beData[31];
    beData[46] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[39] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[30];

    T = beData[45];
    T2 = beData[32];
    beData[47] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[40] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[31];

    T = beData[46];
    T2 = beData[33];
    beData[48] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[41] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[32];

    T = beData[47];
    T2 = beData[34];
    beData[49] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[42] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[33];

    T = beData[48];
    T2 = beData[35];
    beData[50] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[43] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[34];

    T = beData[49];
    T2 = beData[36];
    beData[51] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[44] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[35];

    T = beData[50];
    T2 = beData[37];
    beData[52] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[45] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[36];

    T = beData[51];
    T2 = beData[38];
    beData[53] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[46] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[37];

    T = beData[52];
    T2 = beData[39];
    beData[54] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[47] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[38];

    T = beData[53];
    T2 = beData[40];
    beData[55] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[48] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[39];

    T = beData[54];
    T2 = beData[41];
    beData[56] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[49] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[40];

    T = beData[55];
    T2 = beData[42];
    beData[57] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[50] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[41];

    T = beData[56];
    T2 = beData[43];
    beData[58] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[51] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[42];

    T = beData[57];
    T2 = beData[44];
    beData[59] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[52] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[43];

    T = beData[58];
    T2 = beData[45];
    beData[60] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[53] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[44];

    T = beData[59];
    T2 = beData[46];
    beData[61] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[54] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[45];

    T = beData[60];
    T2 = beData[47];
    beData[62] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[55] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[46];

    T = beData[61];
    T2 = beData[48];
    beData[63] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[56] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[47];

    // Step 2

    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x428A2F98 + RH_swap_u32(data[0]);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x71374491 + RH_swap_u32(data[1]);
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB5C0FBCF + RH_swap_u32(data[2]);
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xE9B5DBA5 + RH_swap_u32(data[3]);
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x3956C25B + RH_swap_u32(data[4]);
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x59F111F1 + RH_swap_u32(data[5]);
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x923F82A4 + RH_swap_u32(data[6]);
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xAB1C5ED5 + RH_swap_u32(data[7]);
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xD807AA98 + RH_swap_u32(data[8]);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x12835B01 + RH_swap_u32(data[9]);
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x243185BE + RH_swap_u32(data[10]);
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x550C7DC3 + 0x80000000;
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x72BE5D74 ;
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x80DEB1FE ;
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x9BDC06A7 ;
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC19BF174 + 0x00000760;
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xE49B69C1 + beData[16];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xEFBE4786 + beData[17];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x0FC19DC6 + beData[18];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x240CA1CC + beData[19];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x2DE92C6F + beData[20];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4A7484AA + beData[21];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5CB0A9DC + beData[22];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x76F988DA + beData[23];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x983E5152 + beData[24];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA831C66D + beData[25];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB00327C8 + beData[26];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xBF597FC7 + beData[27];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xC6E00BF3 + beData[28];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD5A79147 + beData[29];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x06CA6351 + beData[30];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x14292967 + beData[31];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x27B70A85 + beData[32];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x2E1B2138 + beData[33];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x4D2C6DFC + beData[34];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x53380D13 + beData[35];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x650A7354 + beData[36];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x766A0ABB + beData[37];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x81C2C92E + beData[38];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x92722C85 + beData[39];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xA2BFE8A1 + beData[40];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA81A664B + beData[41];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xC24B8B70 + beData[42];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xC76C51A3 + beData[43];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xD192E819 + beData[44];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD6990624 + beData[45];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xF40E3585 + beData[46];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x106AA070 + beData[47];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x19A4C116 + beData[48];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x1E376C08 + beData[49];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x2748774C + beData[50];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x34B0BCB5 + beData[51];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x391C0CB3 + beData[52];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4ED8AA4A + beData[53];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5B9CCA4F + beData[54];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x682E6FF3 + beData[55];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x748F82EE + beData[56];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x78A5636F + beData[57];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x84C87814 + beData[58];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x8CC70208 + beData[59];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x90BEFFFA + beData[60];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xA4506CEB + beData[61];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xBEF9A3F7 + beData[62];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC67178F2 + beData[63];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;

    state[0] += A;
    state[1] += B;
    state[2] += C;
    state[3] += D;
    state[4] += E;
    state[5] += F;
    state[6] += G;
    state[7] += H;
}
//#endif //0


#if 0 
Barely add 1%
//20 : 9A8EA877 87B50DB7 E8833640 19BB35B7 8322ED2F EEB36563 69ADB4A6  C03CEB9 9471E32D 598D0C3E 76874604       80        0        0        0 60070000  
void SHA2_256_RoundFunction_3_Zero(uint32_t* data, uint32_t* state)
{
    uint32_t A, B, C, D, E, F, G, H, T, T2;

    //scratch buffer 64 uint32
    RH_ALIGN(128) uint32_t beData[64];
    

    //RHMINER_ASSERT(0x80 == data[11]); // RH_swap_u32(data[11])
    RHMINER_ASSERT(0 == data[12]);
    RHMINER_ASSERT(0 == data[13]);
    RHMINER_ASSERT(0 == data[14]);
    //RHMINER_ASSERT(0x60070000 == data[15]); //RH_swap_u32(data[15])

    A = state[0];
    B = state[1];
    C = state[2];
    D = state[3];
    E = state[4];
    F = state[5];
    G = state[6];
    H = state[7];


    // Step 1

    //T = 0;
    T2 = RH_swap_u32(data[1]);
    beData[16] =  RH_swap_u32(data[9]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[0]);

    //T = RH_swap_u32(data[15]);
    T2 = RH_swap_u32(data[2]);
    beData[17] = ((ROTR32(RH_swap_u32(data[15]), 17)) ^ (ROTR32(RH_swap_u32(data[15]), 19)) ^ (RH_swap_u32(data[15]) >> 10)) + RH_swap_u32(data[10]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[1]);

    T = beData[16];
    T2 = RH_swap_u32(data[3]);
    beData[18] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + RH_swap_u32(data[11]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[2]);

    T = beData[17];
    T2 = RH_swap_u32(data[4]);
    beData[19] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[3]);

    T = beData[18];
    T2 = RH_swap_u32(data[5]);
    beData[20] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[4]);

    T = beData[19];
    T2 = RH_swap_u32(data[6]);
    beData[21] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[5]);

    T = beData[20];
    T2 = RH_swap_u32(data[7]);
    beData[22] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + RH_swap_u32(data[15]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[6]);

    T = beData[21];
    T2 = RH_swap_u32(data[8]);
    beData[23] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[16] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[7]);

    T = beData[22];
    T2 = RH_swap_u32(data[9]);
    beData[24] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[17] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[8]);

    T = beData[23];
    T2 = RH_swap_u32(data[10]);
    beData[25] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[18] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[9]);

    T = beData[24];
    //T2 = RH_swap_u32(data[11]);
    beData[26] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[19] + ((ROTR32(RH_swap_u32(data[11]), 7)) ^ (ROTR32(RH_swap_u32(data[11]), 18)) ^ (RH_swap_u32(data[11]) >> 3)) + RH_swap_u32(data[10]);

    T = beData[25];
    //T2 = 0;
    beData[27] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[20] + RH_swap_u32(data[11]);

    T = beData[26];
    //T2 = 0;
    beData[28] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[21];

    T = beData[27];
    //T2 = 0;
    beData[29] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[22];

    T = beData[28];
    //T2 = RH_swap_u32(data[15]);
    beData[30] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[23] + ((ROTR32(RH_swap_u32(data[15]), 7)) ^ (ROTR32(RH_swap_u32(data[15]), 18)) ^ (RH_swap_u32(data[15]) >> 3));

    T = beData[29];
    T2 = beData[16];
    beData[31] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[24] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[15]);

    T = beData[30];
    T2 = beData[17];
    beData[32] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[25] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[16];

    T = beData[31];
    T2 = beData[18];
    beData[33] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[26] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[17];

    T = beData[32];
    T2 = beData[19];
    beData[34] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[27] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[18];

    T = beData[33];
    T2 = beData[20];
    beData[35] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[28] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[19];

    T = beData[34];
    T2 = beData[21];
    beData[36] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[29] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[20];

    T = beData[35];
    T2 = beData[22];
    beData[37] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[30] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[21];

    T = beData[36];
    T2 = beData[23];
    beData[38] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[31] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[22];

    T = beData[37];
    T2 = beData[24];
    beData[39] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[32] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[23];

    T = beData[38];
    T2 = beData[25];
    beData[40] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[33] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[24];

    T = beData[39];
    T2 = beData[26];
    beData[41] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[34] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[25];

    T = beData[40];
    T2 = beData[27];
    beData[42] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[35] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[26];

    T = beData[41];
    T2 = beData[28];
    beData[43] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[36] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[27];

    T = beData[42];
    T2 = beData[29];
    beData[44] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[37] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[28];

    T = beData[43];
    T2 = beData[30];
    beData[45] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[38] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[29];

    T = beData[44];
    T2 = beData[31];
    beData[46] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[39] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[30];

    T = beData[45];
    T2 = beData[32];
    beData[47] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[40] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[31];

    T = beData[46];
    T2 = beData[33];
    beData[48] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[41] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[32];

    T = beData[47];
    T2 = beData[34];
    beData[49] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[42] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[33];

    T = beData[48];
    T2 = beData[35];
    beData[50] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[43] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[34];

    T = beData[49];
    T2 = beData[36];
    beData[51] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[44] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[35];

    T = beData[50];
    T2 = beData[37];
    beData[52] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[45] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[36];

    T = beData[51];
    T2 = beData[38];
    beData[53] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[46] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[37];

    T = beData[52];
    T2 = beData[39];
    beData[54] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[47] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[38];

    T = beData[53];
    T2 = beData[40];
    beData[55] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[48] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[39];

    T = beData[54];
    T2 = beData[41];
    beData[56] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[49] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[40];

    T = beData[55];
    T2 = beData[42];
    beData[57] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[50] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[41];

    T = beData[56];
    T2 = beData[43];
    beData[58] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[51] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[42];

    T = beData[57];
    T2 = beData[44];
    beData[59] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[52] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[43];

    T = beData[58];
    T2 = beData[45];
    beData[60] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[53] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[44];

    T = beData[59];
    T2 = beData[46];
    beData[61] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[54] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[45];

    T = beData[60];
    T2 = beData[47];
    beData[62] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[55] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[46];

    T = beData[61];
    T2 = beData[48];
    beData[63] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[56] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[47];

    // Step 2

    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x428A2F98 + RH_swap_u32(data[0]);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x71374491 + RH_swap_u32(data[1]);
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB5C0FBCF + RH_swap_u32(data[2]);
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xE9B5DBA5 + RH_swap_u32(data[3]);
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x3956C25B + RH_swap_u32(data[4]);
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x59F111F1 + RH_swap_u32(data[5]);
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x923F82A4 + RH_swap_u32(data[6]);
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xAB1C5ED5 + RH_swap_u32(data[7]);
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xD807AA98 + RH_swap_u32(data[8]);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x12835B01 + RH_swap_u32(data[9]);
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x243185BE + RH_swap_u32(data[10]);
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x550C7DC3 + RH_swap_u32(data[11]);
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x72BE5D74 ;
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x80DEB1FE ;
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x9BDC06A7 ;
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC19BF174 + RH_swap_u32(data[15]);
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xE49B69C1 + beData[16];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xEFBE4786 + beData[17];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x0FC19DC6 + beData[18];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x240CA1CC + beData[19];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x2DE92C6F + beData[20];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4A7484AA + beData[21];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5CB0A9DC + beData[22];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x76F988DA + beData[23];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x983E5152 + beData[24];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA831C66D + beData[25];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB00327C8 + beData[26];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xBF597FC7 + beData[27];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xC6E00BF3 + beData[28];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD5A79147 + beData[29];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x06CA6351 + beData[30];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x14292967 + beData[31];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x27B70A85 + beData[32];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x2E1B2138 + beData[33];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x4D2C6DFC + beData[34];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x53380D13 + beData[35];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x650A7354 + beData[36];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x766A0ABB + beData[37];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x81C2C92E + beData[38];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x92722C85 + beData[39];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xA2BFE8A1 + beData[40];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA81A664B + beData[41];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xC24B8B70 + beData[42];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xC76C51A3 + beData[43];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xD192E819 + beData[44];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD6990624 + beData[45];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xF40E3585 + beData[46];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x106AA070 + beData[47];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x19A4C116 + beData[48];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x1E376C08 + beData[49];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x2748774C + beData[50];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x34B0BCB5 + beData[51];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x391C0CB3 + beData[52];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4ED8AA4A + beData[53];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5B9CCA4F + beData[54];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x682E6FF3 + beData[55];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x748F82EE + beData[56];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x78A5636F + beData[57];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x84C87814 + beData[58];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x8CC70208 + beData[59];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x90BEFFFA + beData[60];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xA4506CEB + beData[61];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xBEF9A3F7 + beData[62];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC67178F2 + beData[63];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;

    state[0] += A;
    state[1] += B;
    state[2] += C;
    state[3] += D;
    state[4] += E;
    state[5] += F;
    state[6] += G;
    state[7] += H;
}
#endif

#if 0
Only add 2% on windows !!!

void SHA2_256_RoundFunction_Fix15(uint32_t* data, uint32_t* state)
{
    uint32_t A, B, C, D, E, F, G, H, T, T2;

    //scratch buffer 64 uint32
    RH_ALIGN(128) uint32_t beData[64];
    {
        /*RH_swap_u32(0) =  RH_swap_u32(0);
        RH_swap_u32(0) =  RH_swap_u32(0);
        RH_swap_u32(0x02cca71e) =  RH_swap_u32(0x02cca71e);
        RH_swap_u32(0x4731fa4b) =  RH_swap_u32(0x4731fa4b);
        RH_swap_u32(0x6b0246dc) =  RH_swap_u32(0x6b0246dc);
        RH_swap_u32(0x31e392f8) =  RH_swap_u32(0x31e392f8);
        RH_swap_u32(0x1a57a841) =  RH_swap_u32(0x1a57a841);
        RH_swap_u32(0x7f09f2d9) =  RH_swap_u32(0x7f09f2d9);
        RH_swap_u32(0xe1d2436b) =  RH_swap_u32(0xe1d2436b);
        RH_swap_u32(0xd3f2505e) =  RH_swap_u32(0xd3f2505e);
        RH_swap_u32(data[10]) = RH_swap_u32(data[10]);
        (0x80000000) = (0x80000000);
        RH_swap_u32(00000000) = RH_swap_u32(00000000);
        RH_swap_u32(00000000) = RH_swap_u32(00000000);
        RH_swap_u32(00000000) = RH_swap_u32(00000000);
        RH_swap_u32(0x00000760) = RH_swap_u32(0x00000760);*/
    }
    //  0        1          2                                                                     10    
    //00000000 00000000 02cca71e 4731fa4b 6b0246dc 31e392f8 1a57a841 7f09f2d9 e1d2436b d3f2505e 6ddaab05 80000000 00000000 00000000 00000000 0x00000760")
    //00000000 00000000 02cca71e 4731fa4b 6b0246dc 31e392f8 1a57a841 7f09f2d9 e1d2436b d3f2505e 6944155c 80000000 00000000 00000000 00000000 0x00000760")

    A = state[0];
    B = state[1];
    C = state[2];
    D = state[3];
    E = state[4];
    F = state[5];
    G = state[6];
    H = state[7];

    // Step 1

    //T = RH_swap_u32(00000000);
    //T2 = RH_swap_u32(0);
    beData[16] = (0xd3f2505e);

    //T = RH_swap_u32(0x00000760);
    //T2 = RH_swap_u32(0x02cca71e);
    beData[17] = ((ROTR32(0x60070000, 17)) ^ (ROTR32(0x60070000, 19)) ^ (0x60070000 >> 10)) + RH_swap_u32(data[10]) + ((ROTR32(0x1EA7CC02, 7)) ^ (ROTR32(0x1EA7CC02, 18)) ^ (0x1EA7CC02 >> 3));

    //T = beData[16];
    //T2 = RH_swap_u32(0x4731fa4b);
    beData[18] = ((ROTR32((0xd3f2505e), 17)) ^ (ROTR32((0xd3f2505e), 19)) ^ ((0xd3f2505e) >> 10)) + (0x80000000) + ((ROTR32(0x4BFA3147, 7)) ^ (ROTR32(0x4BFA3147, 18)) ^ (0x4BFA3147 >> 3)) + 0x1EA7CC02;

    T = beData[17];
    //T2 = RH_swap_u32(0x6b0246dc);
    beData[19] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(0xDC46026B, 7)) ^ (ROTR32(0xDC46026B, 18)) ^ (0xDC46026B >> 3)) + (0x4731fa4b);

    T = beData[18];
    //T2 = RH_swap_u32(0x31e392f8);
    beData[20] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(0xF892E331, 7)) ^ (ROTR32(0xF892E331, 18)) ^ (0xF892E331 >> 3)) + (0x6b0246dc);

    T = beData[19];
    //T2 = RH_swap_u32(0x1a57a841);
    beData[21] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(0x41A8571A, 7)) ^ (ROTR32(0x41A8571A, 18)) ^ (0x41A8571A >> 3)) + (0x31e392f8);

    T = beData[20];
    //T2 = RH_swap_u32(0x7f09f2d9);
    beData[22] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + (0x00000760) + ((ROTR32(0xD9F2097F, 7)) ^ (ROTR32(0xD9F2097F, 18)) ^ (0xD9F2097F >> 3)) + (0x1a57a841);

    T = beData[21];
    //T2 = RH_swap_u32(0xe1d2436b);
    beData[23] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) +(0xd3f2505e) + ((ROTR32(0x6B43D2E1, 7)) ^ (ROTR32(0x6B43D2E1, 18)) ^ (0x6B43D2E1 >> 3)) + (0x7f09f2d9);

    T = beData[22];
    //T2 = RH_swap_u32(0xd3f2505e);
    beData[24] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[17] + ((ROTR32(0x5E50F2D3, 7)) ^ (ROTR32(0x5E50F2D3, 18)) ^ (0x5E50F2D3 >> 3)) + (0xe1d2436b);

    T = beData[23];
    T2 = RH_swap_u32(data[10]);
    beData[25] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[18] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + (0xd3f2505e);

    T = beData[24];
    T2 = (0x80000000);
    beData[26] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[19] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[10]);

    T = beData[25];
    //T2 = RH_swap_u32(00000000);
    beData[27] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[20] + (0x80000000);

    T = beData[26];
    //T2 = RH_swap_u32(00000000);
    beData[28] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[21];

    T = beData[27];
    //T2 = RH_swap_u32(00000000);
    beData[29] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[22];

    T = beData[28];
    //T2 = RH_swap_u32(0x00000760);
    beData[30] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[23] + ((ROTR32(0x60070000, 7)) ^ (ROTR32(0x60070000, 18)) ^ (0x60070000 >> 3)) ;

    T = beData[29];
    //T2 = beData[16];
    beData[31] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[24] + ((ROTR32((0xd3f2505e), 7)) ^ (ROTR32((0xd3f2505e), 18)) ^ ((0xd3f2505e) >> 3)) + (0x00000760);

    T = beData[30];
    T2 = beData[17];
    beData[32] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[25] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) +(0xd3f2505e);

    T = beData[31];
    T2 = beData[18];
    beData[33] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[26] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[17];

    T = beData[32];
    T2 = beData[19];
    beData[34] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[27] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[18];

    T = beData[33];
    T2 = beData[20];
    beData[35] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[28] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[19];

    T = beData[34];
    T2 = beData[21];
    beData[36] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[29] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[20];

    T = beData[35];
    T2 = beData[22];
    beData[37] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[30] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[21];

    T = beData[36];
    T2 = beData[23];
    beData[38] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[31] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[22];

    T = beData[37];
    T2 = beData[24];
    beData[39] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[32] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[23];

    T = beData[38];
    T2 = beData[25];
    beData[40] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[33] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[24];

    T = beData[39];
    T2 = beData[26];
    beData[41] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[34] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[25];

    T = beData[40];
    T2 = beData[27];
    beData[42] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[35] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[26];

    T = beData[41];
    T2 = beData[28];
    beData[43] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[36] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[27];

    T = beData[42];
    T2 = beData[29];
    beData[44] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[37] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[28];

    T = beData[43];
    T2 = beData[30];
    beData[45] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[38] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[29];

    T = beData[44];
    T2 = beData[31];
    beData[46] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[39] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[30];

    T = beData[45];
    T2 = beData[32];
    beData[47] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[40] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[31];

    T = beData[46];
    T2 = beData[33];
    beData[48] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[41] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[32];

    T = beData[47];
    T2 = beData[34];
    beData[49] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[42] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[33];

    T = beData[48];
    T2 = beData[35];
    beData[50] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[43] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[34];

    T = beData[49];
    T2 = beData[36];
    beData[51] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[44] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[35];

    T = beData[50];
    T2 = beData[37];
    beData[52] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[45] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[36];

    T = beData[51];
    T2 = beData[38];
    beData[53] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[46] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[37];

    T = beData[52];
    T2 = beData[39];
    beData[54] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[47] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[38];

    T = beData[53];
    T2 = beData[40];
    beData[55] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[48] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[39];

    T = beData[54];
    T2 = beData[41];
    beData[56] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[49] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[40];

    T = beData[55];
    T2 = beData[42];
    beData[57] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[50] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[41];

    T = beData[56];
    T2 = beData[43];
    beData[58] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[51] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[42];

    T = beData[57];
    T2 = beData[44];
    beData[59] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[52] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[43];

    T = beData[58];
    T2 = beData[45];
    beData[60] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[53] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[44];

    T = beData[59];
    T2 = beData[46];
    beData[61] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[54] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[45];

    T = beData[60];
    T2 = beData[47];
    beData[62] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[55] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[46];

    T = beData[61];
    T2 = beData[48];
    beData[63] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[56] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[47];

    // Step 2

    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x428A2F98;
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x71374491;
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB5C0FBCF + 0x1EA7CC02;
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xE9B5DBA5 + (0x4731fa4b);
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x3956C25B + (0x6b0246dc);
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x59F111F1 + (0x31e392f8);
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x923F82A4 + (0x1a57a841);
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xAB1C5ED5 + (0x7f09f2d9);
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xD807AA98 + (0xe1d2436b);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x12835B01 + (0xd3f2505e);
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x243185BE + RH_swap_u32(data[10]);
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x550C7DC3 + (0x80000000);
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x72BE5D74;
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x80DEB1FE;
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x9BDC06A7;
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC19BF174 + (0x00000760);
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xE49B69C1 +(0xd3f2505e);
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xEFBE4786 + beData[17];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x0FC19DC6 + beData[18];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x240CA1CC + beData[19];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x2DE92C6F + beData[20];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4A7484AA + beData[21];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5CB0A9DC + beData[22];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x76F988DA + beData[23];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x983E5152 + beData[24];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA831C66D + beData[25];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB00327C8 + beData[26];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xBF597FC7 + beData[27];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xC6E00BF3 + beData[28];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD5A79147 + beData[29];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x06CA6351 + beData[30];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x14292967 + beData[31];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x27B70A85 + beData[32];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x2E1B2138 + beData[33];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x4D2C6DFC + beData[34];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x53380D13 + beData[35];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x650A7354 + beData[36];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x766A0ABB + beData[37];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x81C2C92E + beData[38];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x92722C85 + beData[39];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xA2BFE8A1 + beData[40];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA81A664B + beData[41];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xC24B8B70 + beData[42];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xC76C51A3 + beData[43];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xD192E819 + beData[44];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD6990624 + beData[45];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xF40E3585 + beData[46];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x106AA070 + beData[47];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x19A4C116 + beData[48];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x1E376C08 + beData[49];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x2748774C + beData[50];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x34B0BCB5 + beData[51];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x391C0CB3 + beData[52];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4ED8AA4A + beData[53];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5B9CCA4F + beData[54];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x682E6FF3 + beData[55];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;
    T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x748F82EE + beData[56];
    T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
    H = T + T2;
    D = D + T;
    T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x78A5636F + beData[57];
    T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
    G = T + T2;
    C = C + T;
    T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x84C87814 + beData[58];
    T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
    F = T + T2;
    B = B + T;
    T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x8CC70208 + beData[59];
    T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
    E = T + T2;
    A = A + T;
    T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x90BEFFFA + beData[60];
    T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
    D = T + T2;
    H = H + T;
    T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xA4506CEB + beData[61];
    T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
    C = T + T2;
    G = G + T;
    T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xBEF9A3F7 + beData[62];
    T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
    B = T + T2;
    F = F + T;
    T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC67178F2 + beData[63];
    T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
    A = T + T2;
    E = E + T;

    state[0] += A;
    state[1] += B;
    state[2] += C;
    state[3] += D;
    state[4] += E;
    state[5] += F;
    state[6] += G;
    state[7] += H;
}

#endif //0


void RandomHash_SHA2_256_32(RH_StridePtr roundInput, RH_StridePtr output, bool is224 = false, bool strideOutput = true)
{
    // NOTE: ONLY Size 32 and 100.

    // optimized algo
    RH_ALIGN(128) uint32_t state[8];
    if (is224)
    {
        state[0] = 0xC1059ED8;
        state[1] = 0x367CD507;
        state[2] = 0x3070DD17;
        state[3] = 0xF70E5939;
        state[4] = 0xFFC00B31;
        state[5] = 0x68581511;
        state[6] = 0x64F98FA7;
        state[7] = 0xBEFA4FA4;
    }
    else
    {
        state[0] = 0x6A09E667;
        state[1] = 0xBB67AE85;
        state[2] = 0x3C6EF372;
        state[3] = 0xA54FF53A;
        state[4] = 0x510E527F;
        state[5] = 0x9B05688C;
        state[6] = 0x1F83D9AB;
        state[7] = 0x5BE0CD19;
    };

    SHA2_256_RoundFunction_Last32(RH_STRIDE_GET_DATA(roundInput), state);

    //get the hash result IN BE
    uint32_t* dataPtr;
    if (strideOutput)
    {
        dataPtr = RH_STRIDE_GET_DATA(output);
        RH_STRIDE_SET_SIZE(output, is224 ? 28 : 32);
    }
    else
    {
        dataPtr = (U32*)output;
        RH_ASSERT(is224 == false); //224 not supported in direct mode yet
    }
    copy8_op(dataPtr, state, ReverseBytesUInt32);
}

void RandomHash_SHA2_256(RH_StridePtr roundInput, RH_StridePtr output, bool is224 = false, bool strideOutput = true)
{
    // NOTE: ONLY Size 32 and 100.

    // optimized algo
    RH_ALIGN(128) uint32_t state[8];
    if (is224)
    {
        state[0] = 0xC1059ED8;
        state[1] = 0x367CD507;
        state[2] = 0x3070DD17;
        state[3] = 0xF70E5939;
        state[4] = 0xFFC00B31;
        state[5] = 0x68581511;
        state[6] = 0x64F98FA7;
        state[7] = 0xBEFA4FA4;
    }
    else
    {
        state[0] = 0x6A09E667;
        state[1] = 0xBB67AE85;
        state[2] = 0x3C6EF372;
        state[3] = 0xA54FF53A;
        state[4] = 0x510E527F;
        state[5] = 0x9B05688C;
        state[6] = 0x1F83D9AB;
        state[7] = 0x5BE0CD19;
    };

    {
        int32_t len = (int32_t)RH_STRIDE_GET_SIZE(roundInput);
        uint32_t blockCount = len / SHA2_256_BLOCK_SIZE;
        uint32_t *dataPtr = RH_STRIDE_GET_DATA(roundInput);
        uint64_t bits = len * 8;
        while (blockCount > 0)
        {
            SHA2_256_RoundFunction(dataPtr, state);
            len -= SHA2_256_BLOCK_SIZE;
            dataPtr += SHA2_256_BLOCK_SIZE / 4;
            blockCount--;
        }
        
        {
            int32_t padindex;
            RH_ALIGN(128) uint8_t pad[/*72*/80];
            if (len < 56)
                padindex = 56 - len;
            else
                padindex = 120 - len;

            RH_memzero_of16(pad, sizeof(pad));

            pad[0] = 0x80;
            bits = ReverseBytesUInt64(bits);
            ReadUInt64AsBytesLE(bits, pad + padindex);
            
            padindex = padindex + 8;
            memcpy(((uint8_t*)dataPtr) + len, pad, padindex);
            RH_ASSERT(padindex <= 72);
            RH_ASSERT(((padindex + len) % SHA2_256_BLOCK_SIZE) == 0);

            //RHMINER_ASSERT(padindex == 32 || padindex == 28);
            //SHA2_256_RoundFunction_3_Zero(dataPtr, state);
            SHA2_256_RoundFunction/*_Last28*/(dataPtr, state);


            /*
            rh2 does not make an extra block
            padindex -= SHA2_256_BLOCK_SIZE;
            if (padindex > 0)
                SHA2_256_RoundFunction(dataPtr + (SHA2_256_BLOCK_SIZE / 4), state);
            RHMINER_ASSERT(padindex > -SHA2_256_BLOCK_SIZE);
            */
        }
    }



    //get the hash result IN BE
    uint32_t* dataPtr;
    if (strideOutput)
    {
        dataPtr = RH_STRIDE_GET_DATA(output);
        RH_STRIDE_SET_SIZE(output, is224 ? 28 : 32);
    }
    else
    {
        dataPtr = (U32*)output;
        RH_ASSERT(is224 == false); //224 not supported in direct mode yet
    }
    copy8_op(dataPtr, state, ReverseBytesUInt32);
}

#ifdef RH2_ENABLE_PREFLIGHT_CACHE

// output contain state[8]
void RandomHash_SHA2_256_Part1(U32* input, U32 size, SHA2_256_SavedState& stateObj)
{
    stateObj.state[0] = 0x6A09E667;
    stateObj.state[1] = 0xBB67AE85;
    stateObj.state[2] = 0x3C6EF372;
    stateObj.state[3] = 0xA54FF53A;
    stateObj.state[4] = 0x510E527F;
    stateObj.state[5] = 0x9B05688C;
    stateObj.state[6] = 0x1F83D9AB;
    stateObj.state[7] = 0x5BE0CD19;

    RH_ASSERT(stateObj.endCut < 1024);

    stateObj.bits = size * 8;
    stateObj.len = size - stateObj.endCut;
    uint32_t blockCount = stateObj.len / SHA2_256_BLOCK_SIZE;
    uint32_t* dataPtr = input;
    stateObj.nextCut = blockCount * SHA2_256_BLOCK_SIZE;
    while (blockCount > 0)
    {
        SHA2_256_RoundFunction(dataPtr, stateObj.state);
        stateObj.len -= SHA2_256_BLOCK_SIZE;
        dataPtr += SHA2_256_BLOCK_SIZE / 4;
        blockCount--;
    }
}

#if 0 
For Goldmont architecture
untested
void RandomHash_SHA2_256_Part2_SHA_1x(RH_StridePtr roundInput, const SHA2_256_SavedState& stateObj, RH_StridePtr output)
{
    RH_ALIGN(128) uint32_t state[8];
    U32* data = (U32*)(RH_STRIDE_GET_DATA8(roundInput) + stateObj.nextCut);
    RH_STRIDE_SET_SIZE(output, 32);


    state[0] = stateObj.state[0];
    state[1] = stateObj.state[1];
    state[2] = stateObj.state[2];
    state[3] = stateObj.state[3];
    state[4] = stateObj.state[4];
    state[5] = stateObj.state[5];
    state[6] = stateObj.state[6];
    state[7] = stateObj.state[7];
    
    RH_ASSERT(stateObj.len == 36)

    //SHA2_256_RoundFunction_Last20(data, state);
    {
        __m128i STATE0, STATE1;
        __m128i MSG, TMP;
        __m128i MSG0, MSG1, MSG2, MSG3;
        __m128i ABEF_SAVE, CDGH_SAVE;
        const __m128i MASK = _mm_set_epi64x(0x0c0d0e0f08090a0bULL, 0x0405060700010203ULL);

        /* Load initial values */
        TMP = _mm_loadu_si128((const __m128i*) &state[0]);
        STATE1 = _mm_loadu_si128((const __m128i*) &state[4]);


        TMP = _mm_shuffle_epi32(TMP, 0xB1);          /* CDAB */
        STATE1 = _mm_shuffle_epi32(STATE1, 0x1B);    /* EFGH */
        STATE0 = _mm_alignr_epi8(TMP, STATE1, 8);    /* ABEF */
        STATE1 = _mm_blend_epi16(STATE1, TMP, 0xF0); /* CDGH */

        
        /* Save current state */
        ABEF_SAVE = STATE0;
        CDGH_SAVE = STATE1;

        /* Rounds 0-3 */
        MSG = _mm_loadu_si128((const __m128i*) (data+0));
        MSG0 = _mm_shuffle_epi8(MSG, MASK);
        MSG = _mm_add_epi32(MSG0, _mm_set_epi64x(0xE9B5DBA5B5C0FBCFULL, 0x71374491428A2F98ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);

        /* Rounds 4-7 */
        MSG1 = _mm_loadu_si128((const __m128i*) (data+(16/4)));
        MSG1 = _mm_shuffle_epi8(MSG1, MASK);
        MSG = _mm_add_epi32(MSG1, _mm_set_epi64x(0xAB1C5ED5923F82A4ULL, 0x59F111F13956C25BULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG0 = _mm_sha256msg1_epu32(MSG0, MSG1);

        /* Rounds 8-11 */
        MSG2 = _mm_loadu_si128((const __m128i*) (data+(32/4)));
        MSG2 = _mm_shuffle_epi8(MSG2, MASK);
        MSG = _mm_add_epi32(MSG2, _mm_set_epi64x(0x550C7DC3243185BEULL, 0x12835B01D807AA98ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG1 = _mm_sha256msg1_epu32(MSG1, MSG2);

        /* Rounds 12-15 */
        MSG3 = _mm_loadu_si128((const __m128i*) (data+(48/4)));
        MSG3 = _mm_shuffle_epi8(MSG3, MASK);
        MSG = _mm_add_epi32(MSG3, _mm_set_epi64x(0xC19BF1749BDC06A7ULL, 0x80DEB1FE72BE5D74ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG3, MSG2, 4);
        MSG0 = _mm_add_epi32(MSG0, TMP);
        MSG0 = _mm_sha256msg2_epu32(MSG0, MSG3);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG2 = _mm_sha256msg1_epu32(MSG2, MSG3);

        /* Rounds 16-19 */
        MSG = _mm_add_epi32(MSG0, _mm_set_epi64x(0x240CA1CC0FC19DC6ULL, 0xEFBE4786E49B69C1ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG0, MSG3, 4);
        MSG1 = _mm_add_epi32(MSG1, TMP);
        MSG1 = _mm_sha256msg2_epu32(MSG1, MSG0);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG3 = _mm_sha256msg1_epu32(MSG3, MSG0);

        /* Rounds 20-23 */
        MSG = _mm_add_epi32(MSG1, _mm_set_epi64x(0x76F988DA5CB0A9DCULL, 0x4A7484AA2DE92C6FULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG1, MSG0, 4);
        MSG2 = _mm_add_epi32(MSG2, TMP);
        MSG2 = _mm_sha256msg2_epu32(MSG2, MSG1);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG0 = _mm_sha256msg1_epu32(MSG0, MSG1);

        /* Rounds 24-27 */
        MSG = _mm_add_epi32(MSG2, _mm_set_epi64x(0xBF597FC7B00327C8ULL, 0xA831C66D983E5152ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG2, MSG1, 4);
        MSG3 = _mm_add_epi32(MSG3, TMP);
        MSG3 = _mm_sha256msg2_epu32(MSG3, MSG2);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG1 = _mm_sha256msg1_epu32(MSG1, MSG2);

        /* Rounds 28-31 */
        MSG = _mm_add_epi32(MSG3, _mm_set_epi64x(0x1429296706CA6351ULL,  0xD5A79147C6E00BF3ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG3, MSG2, 4);
        MSG0 = _mm_add_epi32(MSG0, TMP);
        MSG0 = _mm_sha256msg2_epu32(MSG0, MSG3);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG2 = _mm_sha256msg1_epu32(MSG2, MSG3);

        /* Rounds 32-35 */
        MSG = _mm_add_epi32(MSG0, _mm_set_epi64x(0x53380D134D2C6DFCULL, 0x2E1B213827B70A85ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG0, MSG3, 4);
        MSG1 = _mm_add_epi32(MSG1, TMP);
        MSG1 = _mm_sha256msg2_epu32(MSG1, MSG0);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG3 = _mm_sha256msg1_epu32(MSG3, MSG0);

        /* Rounds 36-39 */
        MSG = _mm_add_epi32(MSG1, _mm_set_epi64x(0x92722C8581C2C92EULL, 0x766A0ABB650A7354ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG1, MSG0, 4);
        MSG2 = _mm_add_epi32(MSG2, TMP);
        MSG2 = _mm_sha256msg2_epu32(MSG2, MSG1);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG0 = _mm_sha256msg1_epu32(MSG0, MSG1);

        /* Rounds 40-43 */
        MSG = _mm_add_epi32(MSG2, _mm_set_epi64x(0xC76C51A3C24B8B70ULL, 0xA81A664BA2BFE8A1ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG2, MSG1, 4);
        MSG3 = _mm_add_epi32(MSG3, TMP);
        MSG3 = _mm_sha256msg2_epu32(MSG3, MSG2);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG1 = _mm_sha256msg1_epu32(MSG1, MSG2);

        /* Rounds 44-47 */
        MSG = _mm_add_epi32(MSG3, _mm_set_epi64x(0x106AA070F40E3585ULL, 0xD6990624D192E819ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG3, MSG2, 4);
        MSG0 = _mm_add_epi32(MSG0, TMP);
        MSG0 = _mm_sha256msg2_epu32(MSG0, MSG3);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG2 = _mm_sha256msg1_epu32(MSG2, MSG3);

        /* Rounds 48-51 */
        MSG = _mm_add_epi32(MSG0, _mm_set_epi64x(0x34B0BCB52748774CULL, 0x1E376C0819A4C116ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG0, MSG3, 4);
        MSG1 = _mm_add_epi32(MSG1, TMP);
        MSG1 = _mm_sha256msg2_epu32(MSG1, MSG0);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);
        MSG3 = _mm_sha256msg1_epu32(MSG3, MSG0);

        /* Rounds 52-55 */
        MSG = _mm_add_epi32(MSG1, _mm_set_epi64x(0x682E6FF35B9CCA4FULL, 0x4ED8AA4A391C0CB3ULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG1, MSG0, 4);
        MSG2 = _mm_add_epi32(MSG2, TMP);
        MSG2 = _mm_sha256msg2_epu32(MSG2, MSG1);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);

        /* Rounds 56-59 */
        MSG = _mm_add_epi32(MSG2, _mm_set_epi64x(0x8CC7020884C87814ULL, 0x78A5636F748F82EEULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        TMP = _mm_alignr_epi8(MSG2, MSG1, 4);
        MSG3 = _mm_add_epi32(MSG3, TMP);
        MSG3 = _mm_sha256msg2_epu32(MSG3, MSG2);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);

        /* Rounds 60-63 */
        MSG = _mm_add_epi32(MSG3, _mm_set_epi64x(0xC67178F2BEF9A3F7ULL, 0xA4506CEB90BEFFFAULL));
        STATE1 = _mm_sha256rnds2_epu32(STATE1, STATE0, MSG);
        MSG = _mm_shuffle_epi32(MSG, 0x0E);
        STATE0 = _mm_sha256rnds2_epu32(STATE0, STATE1, MSG);

        /* Combine state  */
        STATE0 = _mm_add_epi32(STATE0, ABEF_SAVE);
        STATE1 = _mm_add_epi32(STATE1, CDGH_SAVE);

        TMP = _mm_shuffle_epi32(STATE0, 0x1B);       /* FEBA */
        STATE1 = _mm_shuffle_epi32(STATE1, 0xB1);    /* DCHG */
        STATE0 = _mm_blend_epi16(TMP, STATE1, 0xF0); /* DCBA */
        STATE1 = _mm_alignr_epi8(STATE1, TMP, 8);    /* ABEF */

        /* Save state */
        _mm_storeu_si128((__m128i*) &state[0], STATE0);
        _mm_storeu_si128((__m128i*) &state[4], STATE1);
    }

        
    //get the hash result IN BE
    data = RH_STRIDE_GET_DATA(output);    

    copy8_op(data, state, ReverseBytesUInt32);
}
#endif //Goldmont, Intel Ice Lake, Rocket Lake

#ifndef RHMINER_NO_SIMD
void RandomHash_SHA2_256_Part2_SSE_4x(RH_StridePtr roundInput, const SHA2_256_SavedState& stateObj, 
    U32 nonce2, U32 nonce3, U32 nonce4,
    RH_StridePtr output1, RH_StridePtr output2, RH_StridePtr output3, RH_StridePtr output4)
{
    //RH_ALIGN(128) uint32_t state[8];
    U32* data = (U32*)(RH_STRIDE_GET_DATA8(roundInput) + stateObj.nextCut);
    __m128i a, b, c, d, e, f, g, h;


    //data[10] is the nonce
/*
    RHMINER_ASSERT(0x80 == data[11]); // 0x80000000
    RHMINER_ASSERT(0 == data[12]);
    RHMINER_ASSERT(0 == data[13]);
    RHMINER_ASSERT(0 == data[14]);
    RHMINER_ASSERT(0x60070000 == data[15]); //0x00000760
*/
    RH_ASSERT(stateObj.len == 36)

    //SHA2_256_RoundFunction_Last20(data, state);
    /*
    __m128i s[8];
    s[0] = _mm_set1_epi32(stateObj.state[0]);
    s[1] = _mm_set1_epi32(stateObj.state[1]);
    s[2] = _mm_set1_epi32(stateObj.state[2]);
    s[3] = _mm_set1_epi32(stateObj.state[3]);
    s[4] = _mm_set1_epi32(stateObj.state[4]);
    s[5] = _mm_set1_epi32(stateObj.state[5]);
    s[6] = _mm_set1_epi32(stateObj.state[6]);
    s[7] = _mm_set1_epi32(stateObj.state[7]);
    */


    //_sha256sse::Initialize(s);
    //_sha256sse::Transform(s,i0,i1,i2,i3);
    {
        __m128i w0, w1, w2, w3, w4, w5, w6, w7;
        __m128i w8, w9, w10, w11, w12, w13, w14, w15;
        __m128i T1, T2;
        /*
        a = _mm_load_si128(s + 0);
        b = _mm_load_si128(s + 1);
        c = _mm_load_si128(s + 2);
        d = _mm_load_si128(s + 3);
        e = _mm_load_si128(s + 4);
        f = _mm_load_si128(s + 5);
        g = _mm_load_si128(s + 6);
        h = _mm_load_si128(s + 7);
        */
        a = _mm_set1_epi32(stateObj.state[0]);
        b = _mm_set1_epi32(stateObj.state[1]);
        c = _mm_set1_epi32(stateObj.state[2]);
        d = _mm_set1_epi32(stateObj.state[3]);
        e = _mm_set1_epi32(stateObj.state[4]);
        f = _mm_set1_epi32(stateObj.state[5]);
        g = _mm_set1_epi32(stateObj.state[6]);
        h = _mm_set1_epi32(stateObj.state[7]);

        //cut /4 + const last 4
        w0 = _mm_set1_epi32(RH_swap_u32(data[0]));
        w1 = _mm_set1_epi32(RH_swap_u32(data[1]));
        w2 = _mm_set1_epi32(RH_swap_u32(data[2]));
        w3 = _mm_set1_epi32(RH_swap_u32(data[3]));
        w4 = _mm_set1_epi32(RH_swap_u32(data[4]));
             
        w5 = _mm_set1_epi32(RH_swap_u32(data[5]));
        w6 = _mm_set1_epi32(RH_swap_u32(data[6]));
        w7 = _mm_set1_epi32(RH_swap_u32(data[7]));
        w8 = _mm_set1_epi32(RH_swap_u32(data[8]));
        w9 = _mm_set1_epi32(RH_swap_u32(data[9]));

        w10 = _mm_set_epi32(RH_swap_u32(data[10]), RH_swap_u32(nonce2), RH_swap_u32(nonce3), RH_swap_u32(nonce4));


        w11 = RH_SSE_CONST(0x80000000);
        //w12 = _mm_setzero_si128();
        //w13 = _mm_setzero_si128();
        //w14 = _mm_setzero_si128();
        w15 = RH_SSE_CONST(0x00000760);
        

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(h, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 6), _mm_slli_epi32((e), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 11), _mm_slli_epi32((e), 32 - 11)), _mm_or_si128(_mm_srli_epi32((e), 25), _mm_slli_epi32((e), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(e, f), _mm_andnot_si128(e, g))), _mm_add_epi32(RH_SSE_CONST(0x428A2F98), w0));
            d = _mm_add_epi32(d, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 2), _mm_slli_epi32((a), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 13), _mm_slli_epi32((a), 32 - 13)), _mm_or_si128(_mm_srli_epi32((a), 22), _mm_slli_epi32((a), 32 - 22))))), _mm_or_si128(_mm_and_si128(a, b), _mm_and_si128(c, _mm_or_si128(a, b))));
            h = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(g, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 6), _mm_slli_epi32((d), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 11), _mm_slli_epi32((d), 32 - 11)), _mm_or_si128(_mm_srli_epi32((d), 25), _mm_slli_epi32((d), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(d, e), _mm_andnot_si128(d, f))), _mm_add_epi32(RH_SSE_CONST(0x71374491), w1));
            c = _mm_add_epi32(c, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 2), _mm_slli_epi32((h), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 13), _mm_slli_epi32((h), 32 - 13)), _mm_or_si128(_mm_srli_epi32((h), 22), _mm_slli_epi32((h), 32 - 22))))), _mm_or_si128(_mm_and_si128(h, a), _mm_and_si128(b, _mm_or_si128(h, a))));
            g = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(f, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 6), _mm_slli_epi32((c), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 11), _mm_slli_epi32((c), 32 - 11)), _mm_or_si128(_mm_srli_epi32((c), 25), _mm_slli_epi32((c), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(c, d), _mm_andnot_si128(c, e))), _mm_add_epi32(RH_SSE_CONST(0xB5C0FBCF), w2));
            b = _mm_add_epi32(b, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 2), _mm_slli_epi32((g), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 13), _mm_slli_epi32((g), 32 - 13)), _mm_or_si128(_mm_srli_epi32((g), 22), _mm_slli_epi32((g), 32 - 22))))), _mm_or_si128(_mm_and_si128(g, h), _mm_and_si128(a, _mm_or_si128(g, h))));
            f = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(e, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 6), _mm_slli_epi32((b), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 11), _mm_slli_epi32((b), 32 - 11)), _mm_or_si128(_mm_srli_epi32((b), 25), _mm_slli_epi32((b), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(b, c), _mm_andnot_si128(b, d))), _mm_add_epi32(RH_SSE_CONST(0xE9B5DBA5), w3));
            a = _mm_add_epi32(a, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 2), _mm_slli_epi32((f), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 13), _mm_slli_epi32((f), 32 - 13)), _mm_or_si128(_mm_srli_epi32((f), 22), _mm_slli_epi32((f), 32 - 22))))), _mm_or_si128(_mm_and_si128(f, g), _mm_and_si128(h, _mm_or_si128(f, g))));
            e = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(d, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 6), _mm_slli_epi32((a), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 11), _mm_slli_epi32((a), 32 - 11)), _mm_or_si128(_mm_srli_epi32((a), 25), _mm_slli_epi32((a), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(a, b), _mm_andnot_si128(a, c))), _mm_add_epi32(RH_SSE_CONST(0x3956C25B), w4));
            h = _mm_add_epi32(h, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 2), _mm_slli_epi32((e), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 13), _mm_slli_epi32((e), 32 - 13)), _mm_or_si128(_mm_srli_epi32((e), 22), _mm_slli_epi32((e), 32 - 22))))), _mm_or_si128(_mm_and_si128(e, f), _mm_and_si128(g, _mm_or_si128(e, f))));
            d = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(c, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 6), _mm_slli_epi32((h), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 11), _mm_slli_epi32((h), 32 - 11)), _mm_or_si128(_mm_srli_epi32((h), 25), _mm_slli_epi32((h), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(h, a), _mm_andnot_si128(h, b))), _mm_add_epi32(RH_SSE_CONST(0x59F111F1), w5));
            g = _mm_add_epi32(g, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 2), _mm_slli_epi32((d), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 13), _mm_slli_epi32((d), 32 - 13)), _mm_or_si128(_mm_srli_epi32((d), 22), _mm_slli_epi32((d), 32 - 22))))), _mm_or_si128(_mm_and_si128(d, e), _mm_and_si128(f, _mm_or_si128(d, e))));
            c = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(b, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 6), _mm_slli_epi32((g), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 11), _mm_slli_epi32((g), 32 - 11)), _mm_or_si128(_mm_srli_epi32((g), 25), _mm_slli_epi32((g), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(g, h), _mm_andnot_si128(g, a))), _mm_add_epi32(RH_SSE_CONST(0x923F82A4), w6));
            f = _mm_add_epi32(f, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 2), _mm_slli_epi32((c), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 13), _mm_slli_epi32((c), 32 - 13)), _mm_or_si128(_mm_srli_epi32((c), 22), _mm_slli_epi32((c), 32 - 22))))), _mm_or_si128(_mm_and_si128(c, d), _mm_and_si128(e, _mm_or_si128(c, d))));
            b = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(a, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 6), _mm_slli_epi32((f), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 11), _mm_slli_epi32((f), 32 - 11)), _mm_or_si128(_mm_srli_epi32((f), 25), _mm_slli_epi32((f), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(f, g), _mm_andnot_si128(f, h))), _mm_add_epi32(RH_SSE_CONST(0xAB1C5ED5), w7));
            e = _mm_add_epi32(e, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 2), _mm_slli_epi32((b), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 13), _mm_slli_epi32((b), 32 - 13)), _mm_or_si128(_mm_srli_epi32((b), 22), _mm_slli_epi32((b), 32 - 22))))), _mm_or_si128(_mm_and_si128(b, c), _mm_and_si128(d, _mm_or_si128(b, c))));
            a = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(h, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 6), _mm_slli_epi32((e), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 11), _mm_slli_epi32((e), 32 - 11)), _mm_or_si128(_mm_srli_epi32((e), 25), _mm_slli_epi32((e), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(e, f), _mm_andnot_si128(e, g))), _mm_add_epi32(RH_SSE_CONST(0xD807AA98), w8));
            d = _mm_add_epi32(d, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 2), _mm_slli_epi32((a), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 13), _mm_slli_epi32((a), 32 - 13)), _mm_or_si128(_mm_srli_epi32((a), 22), _mm_slli_epi32((a), 32 - 22))))), _mm_or_si128(_mm_and_si128(a, b), _mm_and_si128(c, _mm_or_si128(a, b))));
            h = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(g, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 6), _mm_slli_epi32((d), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 11), _mm_slli_epi32((d), 32 - 11)), _mm_or_si128(_mm_srli_epi32((d), 25), _mm_slli_epi32((d), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(d, e), _mm_andnot_si128(d, f))), _mm_add_epi32(RH_SSE_CONST(0x12835B01), w9));
            c = _mm_add_epi32(c, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 2), _mm_slli_epi32((h), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 13), _mm_slli_epi32((h), 32 - 13)), _mm_or_si128(_mm_srli_epi32((h), 22), _mm_slli_epi32((h), 32 - 22))))), _mm_or_si128(_mm_and_si128(h, a), _mm_and_si128(b, _mm_or_si128(h, a))));
            g = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(f, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 6), _mm_slli_epi32((c), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 11), _mm_slli_epi32((c), 32 - 11)), _mm_or_si128(_mm_srli_epi32((c), 25), _mm_slli_epi32((c), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(c, d), _mm_andnot_si128(c, e))), _mm_add_epi32(RH_SSE_CONST(0x243185BE), w10));
            b = _mm_add_epi32(b, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 2), _mm_slli_epi32((g), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 13), _mm_slli_epi32((g), 32 - 13)), _mm_or_si128(_mm_srli_epi32((g), 22), _mm_slli_epi32((g), 32 - 22))))), _mm_or_si128(_mm_and_si128(g, h), _mm_and_si128(a, _mm_or_si128(g, h))));
            f = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(e, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 6), _mm_slli_epi32((b), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 11), _mm_slli_epi32((b), 32 - 11)), _mm_or_si128(_mm_srli_epi32((b), 25), _mm_slli_epi32((b), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(b, c), _mm_andnot_si128(b, d))), _mm_add_epi32(RH_SSE_CONST(0x550C7DC3), w11));
            a = _mm_add_epi32(a, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 2), _mm_slli_epi32((f), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 13), _mm_slli_epi32((f), 32 - 13)), _mm_or_si128(_mm_srli_epi32((f), 22), _mm_slli_epi32((f), 32 - 22))))), _mm_or_si128(_mm_and_si128(f, g), _mm_and_si128(h, _mm_or_si128(f, g))));
            e = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(d, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 6), _mm_slli_epi32((a), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 11), _mm_slli_epi32((a), 32 - 11)), _mm_or_si128(_mm_srli_epi32((a), 25), _mm_slli_epi32((a), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(a, b), _mm_andnot_si128(a, c))), _mm_add_epi32(RH_SSE_CONST(0x72BE5D74), _mm_setzero_si128()));
            h = _mm_add_epi32(h, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 2), _mm_slli_epi32((e), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 13), _mm_slli_epi32((e), 32 - 13)), _mm_or_si128(_mm_srli_epi32((e), 22), _mm_slli_epi32((e), 32 - 22))))), _mm_or_si128(_mm_and_si128(e, f), _mm_and_si128(g, _mm_or_si128(e, f))));
            d = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(c, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 6), _mm_slli_epi32((h), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 11), _mm_slli_epi32((h), 32 - 11)), _mm_or_si128(_mm_srli_epi32((h), 25), _mm_slli_epi32((h), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(h, a), _mm_andnot_si128(h, b))), _mm_add_epi32(RH_SSE_CONST(0x80DEB1FE), _mm_setzero_si128()));
            g = _mm_add_epi32(g, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 2), _mm_slli_epi32((d), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 13), _mm_slli_epi32((d), 32 - 13)), _mm_or_si128(_mm_srli_epi32((d), 22), _mm_slli_epi32((d), 32 - 22))))), _mm_or_si128(_mm_and_si128(d, e), _mm_and_si128(f, _mm_or_si128(d, e))));
            c = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(b, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 6), _mm_slli_epi32((g), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 11), _mm_slli_epi32((g), 32 - 11)), _mm_or_si128(_mm_srli_epi32((g), 25), _mm_slli_epi32((g), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(g, h), _mm_andnot_si128(g, a))), _mm_add_epi32(RH_SSE_CONST(0x9BDC06A7), _mm_setzero_si128()));
            f = _mm_add_epi32(f, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 2), _mm_slli_epi32((c), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 13), _mm_slli_epi32((c), 32 - 13)), _mm_or_si128(_mm_srli_epi32((c), 22), _mm_slli_epi32((c), 32 - 22))))), _mm_or_si128(_mm_and_si128(c, d), _mm_and_si128(e, _mm_or_si128(c, d))));
            b = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(a, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 6), _mm_slli_epi32((f), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 11), _mm_slli_epi32((f), 32 - 11)), _mm_or_si128(_mm_srli_epi32((f), 25), _mm_slli_epi32((f), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(f, g), _mm_andnot_si128(f, h))), _mm_add_epi32(RH_SSE_CONST(0xC19BF174), w15));
            e = _mm_add_epi32(e, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 2), _mm_slli_epi32((b), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 13), _mm_slli_epi32((b), 32 - 13)), _mm_or_si128(_mm_srli_epi32((b), 22), _mm_slli_epi32((b), 32 - 22))))), _mm_or_si128(_mm_and_si128(b, c), _mm_and_si128(d, _mm_or_si128(b, c))));
            a = _mm_add_epi32(T1, T2);
        };


        {
            w0 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((_mm_setzero_si128()), 17), _mm_slli_epi32((_mm_setzero_si128()), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((_mm_setzero_si128()), 19), _mm_slli_epi32((_mm_setzero_si128()), 32 - 19)), _mm_srli_epi32((_mm_setzero_si128()), 10)))), w9), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 7), _mm_slli_epi32((w1), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 18), _mm_slli_epi32((w1), 32 - 18)), _mm_srli_epi32((w1), 3)))), w0));
            w1 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 17), _mm_slli_epi32((w15), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 19), _mm_slli_epi32((w15), 32 - 19)), _mm_srli_epi32((w15), 10)))), w10), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 7), _mm_slli_epi32((w2), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 18), _mm_slli_epi32((w2), 32 - 18)), _mm_srli_epi32((w2), 3)))), w1));
            w2 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 17), _mm_slli_epi32((w0), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 19), _mm_slli_epi32((w0), 32 - 19)), _mm_srli_epi32((w0), 10)))), w11), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 7), _mm_slli_epi32((w3), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 18), _mm_slli_epi32((w3), 32 - 18)), _mm_srli_epi32((w3), 3)))), w2));
            w3 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 17), _mm_slli_epi32((w1), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 19), _mm_slli_epi32((w1), 32 - 19)), _mm_srli_epi32((w1), 10)))), _mm_setzero_si128()), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 7), _mm_slli_epi32((w4), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 18), _mm_slli_epi32((w4), 32 - 18)), _mm_srli_epi32((w4), 3)))), w3));
            w4 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 17), _mm_slli_epi32((w2), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 19), _mm_slli_epi32((w2), 32 - 19)), _mm_srli_epi32((w2), 10)))), _mm_setzero_si128()), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 7), _mm_slli_epi32((w5), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 18), _mm_slli_epi32((w5), 32 - 18)), _mm_srli_epi32((w5), 3)))), w4));
            w5 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 17), _mm_slli_epi32((w3), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 19), _mm_slli_epi32((w3), 32 - 19)), _mm_srli_epi32((w3), 10)))), _mm_setzero_si128()), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 7), _mm_slli_epi32((w6), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 18), _mm_slli_epi32((w6), 32 - 18)), _mm_srli_epi32((w6), 3)))), w5));
            w6 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 17), _mm_slli_epi32((w4), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 19), _mm_slli_epi32((w4), 32 - 19)), _mm_srli_epi32((w4), 10)))), w15), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 7), _mm_slli_epi32((w7), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 18), _mm_slli_epi32((w7), 32 - 18)), _mm_srli_epi32((w7), 3)))), w6));
            w7 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 17), _mm_slli_epi32((w5), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 19), _mm_slli_epi32((w5), 32 - 19)), _mm_srli_epi32((w5), 10)))), w0), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 7), _mm_slli_epi32((w8), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 18), _mm_slli_epi32((w8), 32 - 18)), _mm_srli_epi32((w8), 3)))), w7));
            w8 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 17), _mm_slli_epi32((w6), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 19), _mm_slli_epi32((w6), 32 - 19)), _mm_srli_epi32((w6), 10)))), w1), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 7), _mm_slli_epi32((w9), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 18), _mm_slli_epi32((w9), 32 - 18)), _mm_srli_epi32((w9), 3)))), w8));
            w9 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 17), _mm_slli_epi32((w7), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 19), _mm_slli_epi32((w7), 32 - 19)), _mm_srli_epi32((w7), 10)))), w2), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 7), _mm_slli_epi32((w10), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 18), _mm_slli_epi32((w10), 32 - 18)), _mm_srli_epi32((w10), 3)))), w9));
            w10 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 17), _mm_slli_epi32((w8), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 19), _mm_slli_epi32((w8), 32 - 19)), _mm_srli_epi32((w8), 10)))), w3), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 7), _mm_slli_epi32((w11), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 18), _mm_slli_epi32((w11), 32 - 18)), _mm_srli_epi32((w11), 3)))), w10));
            w11 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 17), _mm_slli_epi32((w9), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 19), _mm_slli_epi32((w9), 32 - 19)), _mm_srli_epi32((w9), 10)))), w4), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((_mm_setzero_si128()), 7), _mm_slli_epi32((_mm_setzero_si128()), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((_mm_setzero_si128()), 18), _mm_slli_epi32((_mm_setzero_si128()), 32 - 18)), _mm_srli_epi32((_mm_setzero_si128()), 3)))), w11));
            w12 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 17), _mm_slli_epi32((w10), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 19), _mm_slli_epi32((w10), 32 - 19)), _mm_srli_epi32((w10), 10)))), w5), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((_mm_setzero_si128()), 7), _mm_slli_epi32((_mm_setzero_si128()), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((_mm_setzero_si128()), 18), _mm_slli_epi32((_mm_setzero_si128()), 32 - 18)), _mm_srli_epi32((_mm_setzero_si128()), 3)))), _mm_setzero_si128()));
            w13 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 17), _mm_slli_epi32((w11), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 19), _mm_slli_epi32((w11), 32 - 19)), _mm_srli_epi32((w11), 10)))), w6), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((_mm_setzero_si128()), 7), _mm_slli_epi32((_mm_setzero_si128()), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((_mm_setzero_si128()), 18), _mm_slli_epi32((_mm_setzero_si128()), 32 - 18)), _mm_srli_epi32((_mm_setzero_si128()), 3)))), _mm_setzero_si128()));
            w14 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w12), 17), _mm_slli_epi32((w12), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w12), 19), _mm_slli_epi32((w12), 32 - 19)), _mm_srli_epi32((w12), 10)))), w7), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 7), _mm_slli_epi32((w15), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 18), _mm_slli_epi32((w15), 32 - 18)), _mm_srli_epi32((w15), 3)))), _mm_setzero_si128()));
            w15 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w13), 17), _mm_slli_epi32((w13), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w13), 19), _mm_slli_epi32((w13), 32 - 19)), _mm_srli_epi32((w13), 10)))), w8), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 7), _mm_slli_epi32((w0), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 18), _mm_slli_epi32((w0), 32 - 18)), _mm_srli_epi32((w0), 3)))), w15));
        }

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(h, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 6), _mm_slli_epi32((e), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 11), _mm_slli_epi32((e), 32 - 11)), _mm_or_si128(_mm_srli_epi32((e), 25), _mm_slli_epi32((e), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(e, f), _mm_andnot_si128(e, g))), _mm_add_epi32(RH_SSE_CONST(0xE49B69C1), w0));
            d = _mm_add_epi32(d, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 2), _mm_slli_epi32((a), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 13), _mm_slli_epi32((a), 32 - 13)), _mm_or_si128(_mm_srli_epi32((a), 22), _mm_slli_epi32((a), 32 - 22))))), _mm_or_si128(_mm_and_si128(a, b), _mm_and_si128(c, _mm_or_si128(a, b))));
            h = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(g, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 6), _mm_slli_epi32((d), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 11), _mm_slli_epi32((d), 32 - 11)), _mm_or_si128(_mm_srli_epi32((d), 25), _mm_slli_epi32((d), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(d, e), _mm_andnot_si128(d, f))), _mm_add_epi32(RH_SSE_CONST(0xEFBE4786), w1));
            c = _mm_add_epi32(c, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 2), _mm_slli_epi32((h), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 13), _mm_slli_epi32((h), 32 - 13)), _mm_or_si128(_mm_srli_epi32((h), 22), _mm_slli_epi32((h), 32 - 22))))), _mm_or_si128(_mm_and_si128(h, a), _mm_and_si128(b, _mm_or_si128(h, a))));
            g = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(f, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 6), _mm_slli_epi32((c), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 11), _mm_slli_epi32((c), 32 - 11)), _mm_or_si128(_mm_srli_epi32((c), 25), _mm_slli_epi32((c), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(c, d), _mm_andnot_si128(c, e))), _mm_add_epi32(RH_SSE_CONST(0x0FC19DC6), w2));
            b = _mm_add_epi32(b, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 2), _mm_slli_epi32((g), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 13), _mm_slli_epi32((g), 32 - 13)), _mm_or_si128(_mm_srli_epi32((g), 22), _mm_slli_epi32((g), 32 - 22))))), _mm_or_si128(_mm_and_si128(g, h), _mm_and_si128(a, _mm_or_si128(g, h))));
            f = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(e, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 6), _mm_slli_epi32((b), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 11), _mm_slli_epi32((b), 32 - 11)), _mm_or_si128(_mm_srli_epi32((b), 25), _mm_slli_epi32((b), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(b, c), _mm_andnot_si128(b, d))), _mm_add_epi32(RH_SSE_CONST(0x240CA1CC), w3));
            a = _mm_add_epi32(a, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 2), _mm_slli_epi32((f), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 13), _mm_slli_epi32((f), 32 - 13)), _mm_or_si128(_mm_srli_epi32((f), 22), _mm_slli_epi32((f), 32 - 22))))), _mm_or_si128(_mm_and_si128(f, g), _mm_and_si128(h, _mm_or_si128(f, g))));
            e = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(d, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 6), _mm_slli_epi32((a), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 11), _mm_slli_epi32((a), 32 - 11)), _mm_or_si128(_mm_srli_epi32((a), 25), _mm_slli_epi32((a), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(a, b), _mm_andnot_si128(a, c))), _mm_add_epi32(RH_SSE_CONST(0x2DE92C6F), w4));
            h = _mm_add_epi32(h, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 2), _mm_slli_epi32((e), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 13), _mm_slli_epi32((e), 32 - 13)), _mm_or_si128(_mm_srli_epi32((e), 22), _mm_slli_epi32((e), 32 - 22))))), _mm_or_si128(_mm_and_si128(e, f), _mm_and_si128(g, _mm_or_si128(e, f))));
            d = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(c, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 6), _mm_slli_epi32((h), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 11), _mm_slli_epi32((h), 32 - 11)), _mm_or_si128(_mm_srli_epi32((h), 25), _mm_slli_epi32((h), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(h, a), _mm_andnot_si128(h, b))), _mm_add_epi32(RH_SSE_CONST(0x4A7484AA), w5));
            g = _mm_add_epi32(g, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 2), _mm_slli_epi32((d), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 13), _mm_slli_epi32((d), 32 - 13)), _mm_or_si128(_mm_srli_epi32((d), 22), _mm_slli_epi32((d), 32 - 22))))), _mm_or_si128(_mm_and_si128(d, e), _mm_and_si128(f, _mm_or_si128(d, e))));
            c = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(b, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 6), _mm_slli_epi32((g), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 11), _mm_slli_epi32((g), 32 - 11)), _mm_or_si128(_mm_srli_epi32((g), 25), _mm_slli_epi32((g), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(g, h), _mm_andnot_si128(g, a))), _mm_add_epi32(RH_SSE_CONST(0x5CB0A9DC), w6));
            f = _mm_add_epi32(f, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 2), _mm_slli_epi32((c), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 13), _mm_slli_epi32((c), 32 - 13)), _mm_or_si128(_mm_srli_epi32((c), 22), _mm_slli_epi32((c), 32 - 22))))), _mm_or_si128(_mm_and_si128(c, d), _mm_and_si128(e, _mm_or_si128(c, d))));
            b = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(a, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 6), _mm_slli_epi32((f), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 11), _mm_slli_epi32((f), 32 - 11)), _mm_or_si128(_mm_srli_epi32((f), 25), _mm_slli_epi32((f), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(f, g), _mm_andnot_si128(f, h))), _mm_add_epi32(RH_SSE_CONST(0x76F988DA), w7));
            e = _mm_add_epi32(e, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 2), _mm_slli_epi32((b), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 13), _mm_slli_epi32((b), 32 - 13)), _mm_or_si128(_mm_srli_epi32((b), 22), _mm_slli_epi32((b), 32 - 22))))), _mm_or_si128(_mm_and_si128(b, c), _mm_and_si128(d, _mm_or_si128(b, c))));
            a = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(h, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 6), _mm_slli_epi32((e), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 11), _mm_slli_epi32((e), 32 - 11)), _mm_or_si128(_mm_srli_epi32((e), 25), _mm_slli_epi32((e), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(e, f), _mm_andnot_si128(e, g))), _mm_add_epi32(RH_SSE_CONST(0x983E5152), w8));
            d = _mm_add_epi32(d, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 2), _mm_slli_epi32((a), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 13), _mm_slli_epi32((a), 32 - 13)), _mm_or_si128(_mm_srli_epi32((a), 22), _mm_slli_epi32((a), 32 - 22))))), _mm_or_si128(_mm_and_si128(a, b), _mm_and_si128(c, _mm_or_si128(a, b))));
            h = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(g, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 6), _mm_slli_epi32((d), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 11), _mm_slli_epi32((d), 32 - 11)), _mm_or_si128(_mm_srli_epi32((d), 25), _mm_slli_epi32((d), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(d, e), _mm_andnot_si128(d, f))), _mm_add_epi32(RH_SSE_CONST(0xA831C66D), w9));
            c = _mm_add_epi32(c, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 2), _mm_slli_epi32((h), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 13), _mm_slli_epi32((h), 32 - 13)), _mm_or_si128(_mm_srli_epi32((h), 22), _mm_slli_epi32((h), 32 - 22))))), _mm_or_si128(_mm_and_si128(h, a), _mm_and_si128(b, _mm_or_si128(h, a))));
            g = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(f, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 6), _mm_slli_epi32((c), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 11), _mm_slli_epi32((c), 32 - 11)), _mm_or_si128(_mm_srli_epi32((c), 25), _mm_slli_epi32((c), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(c, d), _mm_andnot_si128(c, e))), _mm_add_epi32(RH_SSE_CONST(0xB00327C8), w10));
            b = _mm_add_epi32(b, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 2), _mm_slli_epi32((g), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 13), _mm_slli_epi32((g), 32 - 13)), _mm_or_si128(_mm_srli_epi32((g), 22), _mm_slli_epi32((g), 32 - 22))))), _mm_or_si128(_mm_and_si128(g, h), _mm_and_si128(a, _mm_or_si128(g, h))));
            f = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(e, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 6), _mm_slli_epi32((b), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 11), _mm_slli_epi32((b), 32 - 11)), _mm_or_si128(_mm_srli_epi32((b), 25), _mm_slli_epi32((b), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(b, c), _mm_andnot_si128(b, d))), _mm_add_epi32(RH_SSE_CONST(0xBF597FC7), w11));
            a = _mm_add_epi32(a, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 2), _mm_slli_epi32((f), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 13), _mm_slli_epi32((f), 32 - 13)), _mm_or_si128(_mm_srli_epi32((f), 22), _mm_slli_epi32((f), 32 - 22))))), _mm_or_si128(_mm_and_si128(f, g), _mm_and_si128(h, _mm_or_si128(f, g))));
            e = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(d, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 6), _mm_slli_epi32((a), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 11), _mm_slli_epi32((a), 32 - 11)), _mm_or_si128(_mm_srli_epi32((a), 25), _mm_slli_epi32((a), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(a, b), _mm_andnot_si128(a, c))), _mm_add_epi32(RH_SSE_CONST(0xC6E00BF3), w12));
            h = _mm_add_epi32(h, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 2), _mm_slli_epi32((e), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 13), _mm_slli_epi32((e), 32 - 13)), _mm_or_si128(_mm_srli_epi32((e), 22), _mm_slli_epi32((e), 32 - 22))))), _mm_or_si128(_mm_and_si128(e, f), _mm_and_si128(g, _mm_or_si128(e, f))));
            d = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(c, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 6), _mm_slli_epi32((h), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 11), _mm_slli_epi32((h), 32 - 11)), _mm_or_si128(_mm_srli_epi32((h), 25), _mm_slli_epi32((h), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(h, a), _mm_andnot_si128(h, b))), _mm_add_epi32(RH_SSE_CONST(0xD5A79147), w13));
            g = _mm_add_epi32(g, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 2), _mm_slli_epi32((d), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 13), _mm_slli_epi32((d), 32 - 13)), _mm_or_si128(_mm_srli_epi32((d), 22), _mm_slli_epi32((d), 32 - 22))))), _mm_or_si128(_mm_and_si128(d, e), _mm_and_si128(f, _mm_or_si128(d, e))));
            c = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(b, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 6), _mm_slli_epi32((g), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 11), _mm_slli_epi32((g), 32 - 11)), _mm_or_si128(_mm_srli_epi32((g), 25), _mm_slli_epi32((g), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(g, h), _mm_andnot_si128(g, a))), _mm_add_epi32(RH_SSE_CONST(0x06CA6351), w14));
            f = _mm_add_epi32(f, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 2), _mm_slli_epi32((c), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 13), _mm_slli_epi32((c), 32 - 13)), _mm_or_si128(_mm_srli_epi32((c), 22), _mm_slli_epi32((c), 32 - 22))))), _mm_or_si128(_mm_and_si128(c, d), _mm_and_si128(e, _mm_or_si128(c, d))));
            b = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(a, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 6), _mm_slli_epi32((f), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 11), _mm_slli_epi32((f), 32 - 11)), _mm_or_si128(_mm_srli_epi32((f), 25), _mm_slli_epi32((f), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(f, g), _mm_andnot_si128(f, h))), _mm_add_epi32(RH_SSE_CONST(0x14292967), w15));
            e = _mm_add_epi32(e, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 2), _mm_slli_epi32((b), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 13), _mm_slli_epi32((b), 32 - 13)), _mm_or_si128(_mm_srli_epi32((b), 22), _mm_slli_epi32((b), 32 - 22))))), _mm_or_si128(_mm_and_si128(b, c), _mm_and_si128(d, _mm_or_si128(b, c))));
            a = _mm_add_epi32(T1, T2);
        };


        {
            w0 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w14), 17), _mm_slli_epi32((w14), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w14), 19), _mm_slli_epi32((w14), 32 - 19)), _mm_srli_epi32((w14), 10)))), w9), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 7), _mm_slli_epi32((w1), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 18), _mm_slli_epi32((w1), 32 - 18)), _mm_srli_epi32((w1), 3)))), w0));
            w1 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 17), _mm_slli_epi32((w15), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 19), _mm_slli_epi32((w15), 32 - 19)), _mm_srli_epi32((w15), 10)))), w10), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 7), _mm_slli_epi32((w2), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 18), _mm_slli_epi32((w2), 32 - 18)), _mm_srli_epi32((w2), 3)))), w1));
            w2 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 17), _mm_slli_epi32((w0), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 19), _mm_slli_epi32((w0), 32 - 19)), _mm_srli_epi32((w0), 10)))), w11), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 7), _mm_slli_epi32((w3), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 18), _mm_slli_epi32((w3), 32 - 18)), _mm_srli_epi32((w3), 3)))), w2));
            w3 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 17), _mm_slli_epi32((w1), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 19), _mm_slli_epi32((w1), 32 - 19)), _mm_srli_epi32((w1), 10)))), w12), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 7), _mm_slli_epi32((w4), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 18), _mm_slli_epi32((w4), 32 - 18)), _mm_srli_epi32((w4), 3)))), w3));
            w4 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 17), _mm_slli_epi32((w2), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 19), _mm_slli_epi32((w2), 32 - 19)), _mm_srli_epi32((w2), 10)))), w13), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 7), _mm_slli_epi32((w5), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 18), _mm_slli_epi32((w5), 32 - 18)), _mm_srli_epi32((w5), 3)))), w4));
            w5 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 17), _mm_slli_epi32((w3), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 19), _mm_slli_epi32((w3), 32 - 19)), _mm_srli_epi32((w3), 10)))), w14), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 7), _mm_slli_epi32((w6), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 18), _mm_slli_epi32((w6), 32 - 18)), _mm_srli_epi32((w6), 3)))), w5));
            w6 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 17), _mm_slli_epi32((w4), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 19), _mm_slli_epi32((w4), 32 - 19)), _mm_srli_epi32((w4), 10)))), w15), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 7), _mm_slli_epi32((w7), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 18), _mm_slli_epi32((w7), 32 - 18)), _mm_srli_epi32((w7), 3)))), w6));
            w7 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 17), _mm_slli_epi32((w5), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 19), _mm_slli_epi32((w5), 32 - 19)), _mm_srli_epi32((w5), 10)))), w0), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 7), _mm_slli_epi32((w8), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 18), _mm_slli_epi32((w8), 32 - 18)), _mm_srli_epi32((w8), 3)))), w7));
            w8 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 17), _mm_slli_epi32((w6), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 19), _mm_slli_epi32((w6), 32 - 19)), _mm_srli_epi32((w6), 10)))), w1), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 7), _mm_slli_epi32((w9), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 18), _mm_slli_epi32((w9), 32 - 18)), _mm_srli_epi32((w9), 3)))), w8));
            w9 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 17), _mm_slli_epi32((w7), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 19), _mm_slli_epi32((w7), 32 - 19)), _mm_srli_epi32((w7), 10)))), w2), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 7), _mm_slli_epi32((w10), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 18), _mm_slli_epi32((w10), 32 - 18)), _mm_srli_epi32((w10), 3)))), w9));
            w10 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 17), _mm_slli_epi32((w8), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 19), _mm_slli_epi32((w8), 32 - 19)), _mm_srli_epi32((w8), 10)))), w3), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 7), _mm_slli_epi32((w11), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 18), _mm_slli_epi32((w11), 32 - 18)), _mm_srli_epi32((w11), 3)))), w10));
            w11 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 17), _mm_slli_epi32((w9), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 19), _mm_slli_epi32((w9), 32 - 19)), _mm_srli_epi32((w9), 10)))), w4), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w12), 7), _mm_slli_epi32((w12), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w12), 18), _mm_slli_epi32((w12), 32 - 18)), _mm_srli_epi32((w12), 3)))), w11));
            w12 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 17), _mm_slli_epi32((w10), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 19), _mm_slli_epi32((w10), 32 - 19)), _mm_srli_epi32((w10), 10)))), w5), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w13), 7), _mm_slli_epi32((w13), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w13), 18), _mm_slli_epi32((w13), 32 - 18)), _mm_srli_epi32((w13), 3)))), w12));
            w13 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 17), _mm_slli_epi32((w11), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 19), _mm_slli_epi32((w11), 32 - 19)), _mm_srli_epi32((w11), 10)))), w6), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w14), 7), _mm_slli_epi32((w14), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w14), 18), _mm_slli_epi32((w14), 32 - 18)), _mm_srli_epi32((w14), 3)))), w13));
            w14 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w12), 17), _mm_slli_epi32((w12), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w12), 19), _mm_slli_epi32((w12), 32 - 19)), _mm_srli_epi32((w12), 10)))), w7), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 7), _mm_slli_epi32((w15), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 18), _mm_slli_epi32((w15), 32 - 18)), _mm_srli_epi32((w15), 3)))), w14));
            w15 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w13), 17), _mm_slli_epi32((w13), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w13), 19), _mm_slli_epi32((w13), 32 - 19)), _mm_srli_epi32((w13), 10)))), w8), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 7), _mm_slli_epi32((w0), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 18), _mm_slli_epi32((w0), 32 - 18)), _mm_srli_epi32((w0), 3)))), w15));
        }

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(h, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 6), _mm_slli_epi32((e), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 11), _mm_slli_epi32((e), 32 - 11)), _mm_or_si128(_mm_srli_epi32((e), 25), _mm_slli_epi32((e), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(e, f), _mm_andnot_si128(e, g))), _mm_add_epi32(RH_SSE_CONST(0x27B70A85), w0));
            d = _mm_add_epi32(d, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 2), _mm_slli_epi32((a), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 13), _mm_slli_epi32((a), 32 - 13)), _mm_or_si128(_mm_srli_epi32((a), 22), _mm_slli_epi32((a), 32 - 22))))), _mm_or_si128(_mm_and_si128(a, b), _mm_and_si128(c, _mm_or_si128(a, b))));
            h = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(g, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 6), _mm_slli_epi32((d), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 11), _mm_slli_epi32((d), 32 - 11)), _mm_or_si128(_mm_srli_epi32((d), 25), _mm_slli_epi32((d), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(d, e), _mm_andnot_si128(d, f))), _mm_add_epi32(RH_SSE_CONST(0x2E1B2138), w1));
            c = _mm_add_epi32(c, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 2), _mm_slli_epi32((h), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 13), _mm_slli_epi32((h), 32 - 13)), _mm_or_si128(_mm_srli_epi32((h), 22), _mm_slli_epi32((h), 32 - 22))))), _mm_or_si128(_mm_and_si128(h, a), _mm_and_si128(b, _mm_or_si128(h, a))));
            g = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(f, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 6), _mm_slli_epi32((c), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 11), _mm_slli_epi32((c), 32 - 11)), _mm_or_si128(_mm_srli_epi32((c), 25), _mm_slli_epi32((c), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(c, d), _mm_andnot_si128(c, e))), _mm_add_epi32(RH_SSE_CONST(0x4D2C6DFC), w2));
            b = _mm_add_epi32(b, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 2), _mm_slli_epi32((g), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 13), _mm_slli_epi32((g), 32 - 13)), _mm_or_si128(_mm_srli_epi32((g), 22), _mm_slli_epi32((g), 32 - 22))))), _mm_or_si128(_mm_and_si128(g, h), _mm_and_si128(a, _mm_or_si128(g, h))));
            f = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(e, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 6), _mm_slli_epi32((b), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 11), _mm_slli_epi32((b), 32 - 11)), _mm_or_si128(_mm_srli_epi32((b), 25), _mm_slli_epi32((b), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(b, c), _mm_andnot_si128(b, d))), _mm_add_epi32(RH_SSE_CONST(0x53380D13), w3));
            a = _mm_add_epi32(a, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 2), _mm_slli_epi32((f), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 13), _mm_slli_epi32((f), 32 - 13)), _mm_or_si128(_mm_srli_epi32((f), 22), _mm_slli_epi32((f), 32 - 22))))), _mm_or_si128(_mm_and_si128(f, g), _mm_and_si128(h, _mm_or_si128(f, g))));
            e = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(d, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 6), _mm_slli_epi32((a), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 11), _mm_slli_epi32((a), 32 - 11)), _mm_or_si128(_mm_srli_epi32((a), 25), _mm_slli_epi32((a), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(a, b), _mm_andnot_si128(a, c))), _mm_add_epi32(RH_SSE_CONST(0x650A7354), w4));
            h = _mm_add_epi32(h, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 2), _mm_slli_epi32((e), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 13), _mm_slli_epi32((e), 32 - 13)), _mm_or_si128(_mm_srli_epi32((e), 22), _mm_slli_epi32((e), 32 - 22))))), _mm_or_si128(_mm_and_si128(e, f), _mm_and_si128(g, _mm_or_si128(e, f))));
            d = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(c, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 6), _mm_slli_epi32((h), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 11), _mm_slli_epi32((h), 32 - 11)), _mm_or_si128(_mm_srli_epi32((h), 25), _mm_slli_epi32((h), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(h, a), _mm_andnot_si128(h, b))), _mm_add_epi32(RH_SSE_CONST(0x766A0ABB), w5));
            g = _mm_add_epi32(g, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 2), _mm_slli_epi32((d), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 13), _mm_slli_epi32((d), 32 - 13)), _mm_or_si128(_mm_srli_epi32((d), 22), _mm_slli_epi32((d), 32 - 22))))), _mm_or_si128(_mm_and_si128(d, e), _mm_and_si128(f, _mm_or_si128(d, e))));
            c = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(b, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 6), _mm_slli_epi32((g), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 11), _mm_slli_epi32((g), 32 - 11)), _mm_or_si128(_mm_srli_epi32((g), 25), _mm_slli_epi32((g), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(g, h), _mm_andnot_si128(g, a))), _mm_add_epi32(RH_SSE_CONST(0x81C2C92E), w6));
            f = _mm_add_epi32(f, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 2), _mm_slli_epi32((c), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 13), _mm_slli_epi32((c), 32 - 13)), _mm_or_si128(_mm_srli_epi32((c), 22), _mm_slli_epi32((c), 32 - 22))))), _mm_or_si128(_mm_and_si128(c, d), _mm_and_si128(e, _mm_or_si128(c, d))));
            b = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(a, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 6), _mm_slli_epi32((f), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 11), _mm_slli_epi32((f), 32 - 11)), _mm_or_si128(_mm_srli_epi32((f), 25), _mm_slli_epi32((f), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(f, g), _mm_andnot_si128(f, h))), _mm_add_epi32(RH_SSE_CONST(0x92722C85), w7));
            e = _mm_add_epi32(e, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 2), _mm_slli_epi32((b), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 13), _mm_slli_epi32((b), 32 - 13)), _mm_or_si128(_mm_srli_epi32((b), 22), _mm_slli_epi32((b), 32 - 22))))), _mm_or_si128(_mm_and_si128(b, c), _mm_and_si128(d, _mm_or_si128(b, c))));
            a = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(h, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 6), _mm_slli_epi32((e), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 11), _mm_slli_epi32((e), 32 - 11)), _mm_or_si128(_mm_srli_epi32((e), 25), _mm_slli_epi32((e), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(e, f), _mm_andnot_si128(e, g))), _mm_add_epi32(RH_SSE_CONST(0xA2BFE8A1), w8));
            d = _mm_add_epi32(d, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 2), _mm_slli_epi32((a), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 13), _mm_slli_epi32((a), 32 - 13)), _mm_or_si128(_mm_srli_epi32((a), 22), _mm_slli_epi32((a), 32 - 22))))), _mm_or_si128(_mm_and_si128(a, b), _mm_and_si128(c, _mm_or_si128(a, b))));
            h = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(g, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 6), _mm_slli_epi32((d), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 11), _mm_slli_epi32((d), 32 - 11)), _mm_or_si128(_mm_srli_epi32((d), 25), _mm_slli_epi32((d), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(d, e), _mm_andnot_si128(d, f))), _mm_add_epi32(RH_SSE_CONST(0xA81A664B), w9));
            c = _mm_add_epi32(c, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 2), _mm_slli_epi32((h), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 13), _mm_slli_epi32((h), 32 - 13)), _mm_or_si128(_mm_srli_epi32((h), 22), _mm_slli_epi32((h), 32 - 22))))), _mm_or_si128(_mm_and_si128(h, a), _mm_and_si128(b, _mm_or_si128(h, a))));
            g = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(f, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 6), _mm_slli_epi32((c), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 11), _mm_slli_epi32((c), 32 - 11)), _mm_or_si128(_mm_srli_epi32((c), 25), _mm_slli_epi32((c), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(c, d), _mm_andnot_si128(c, e))), _mm_add_epi32(RH_SSE_CONST(0xC24B8B70), w10));
            b = _mm_add_epi32(b, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 2), _mm_slli_epi32((g), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 13), _mm_slli_epi32((g), 32 - 13)), _mm_or_si128(_mm_srli_epi32((g), 22), _mm_slli_epi32((g), 32 - 22))))), _mm_or_si128(_mm_and_si128(g, h), _mm_and_si128(a, _mm_or_si128(g, h))));
            f = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(e, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 6), _mm_slli_epi32((b), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 11), _mm_slli_epi32((b), 32 - 11)), _mm_or_si128(_mm_srli_epi32((b), 25), _mm_slli_epi32((b), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(b, c), _mm_andnot_si128(b, d))), _mm_add_epi32(RH_SSE_CONST(0xC76C51A3), w11));
            a = _mm_add_epi32(a, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 2), _mm_slli_epi32((f), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 13), _mm_slli_epi32((f), 32 - 13)), _mm_or_si128(_mm_srli_epi32((f), 22), _mm_slli_epi32((f), 32 - 22))))), _mm_or_si128(_mm_and_si128(f, g), _mm_and_si128(h, _mm_or_si128(f, g))));
            e = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(d, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 6), _mm_slli_epi32((a), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 11), _mm_slli_epi32((a), 32 - 11)), _mm_or_si128(_mm_srli_epi32((a), 25), _mm_slli_epi32((a), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(a, b), _mm_andnot_si128(a, c))), _mm_add_epi32(RH_SSE_CONST(0xD192E819), w12));
            h = _mm_add_epi32(h, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 2), _mm_slli_epi32((e), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 13), _mm_slli_epi32((e), 32 - 13)), _mm_or_si128(_mm_srli_epi32((e), 22), _mm_slli_epi32((e), 32 - 22))))), _mm_or_si128(_mm_and_si128(e, f), _mm_and_si128(g, _mm_or_si128(e, f))));
            d = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(c, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 6), _mm_slli_epi32((h), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 11), _mm_slli_epi32((h), 32 - 11)), _mm_or_si128(_mm_srli_epi32((h), 25), _mm_slli_epi32((h), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(h, a), _mm_andnot_si128(h, b))), _mm_add_epi32(RH_SSE_CONST(0xD6990624), w13));
            g = _mm_add_epi32(g, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 2), _mm_slli_epi32((d), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 13), _mm_slli_epi32((d), 32 - 13)), _mm_or_si128(_mm_srli_epi32((d), 22), _mm_slli_epi32((d), 32 - 22))))), _mm_or_si128(_mm_and_si128(d, e), _mm_and_si128(f, _mm_or_si128(d, e))));
            c = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(b, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 6), _mm_slli_epi32((g), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 11), _mm_slli_epi32((g), 32 - 11)), _mm_or_si128(_mm_srli_epi32((g), 25), _mm_slli_epi32((g), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(g, h), _mm_andnot_si128(g, a))), _mm_add_epi32(RH_SSE_CONST(0xF40E3585), w14));
            f = _mm_add_epi32(f, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 2), _mm_slli_epi32((c), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 13), _mm_slli_epi32((c), 32 - 13)), _mm_or_si128(_mm_srli_epi32((c), 22), _mm_slli_epi32((c), 32 - 22))))), _mm_or_si128(_mm_and_si128(c, d), _mm_and_si128(e, _mm_or_si128(c, d))));
            b = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(a, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 6), _mm_slli_epi32((f), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 11), _mm_slli_epi32((f), 32 - 11)), _mm_or_si128(_mm_srli_epi32((f), 25), _mm_slli_epi32((f), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(f, g), _mm_andnot_si128(f, h))), _mm_add_epi32(RH_SSE_CONST(0x106AA070), w15));
            e = _mm_add_epi32(e, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 2), _mm_slli_epi32((b), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 13), _mm_slli_epi32((b), 32 - 13)), _mm_or_si128(_mm_srli_epi32((b), 22), _mm_slli_epi32((b), 32 - 22))))), _mm_or_si128(_mm_and_si128(b, c), _mm_and_si128(d, _mm_or_si128(b, c))));
            a = _mm_add_epi32(T1, T2);
        };


        {
            w0 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w14), 17), _mm_slli_epi32((w14), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w14), 19), _mm_slli_epi32((w14), 32 - 19)), _mm_srli_epi32((w14), 10)))), w9), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 7), _mm_slli_epi32((w1), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 18), _mm_slli_epi32((w1), 32 - 18)), _mm_srli_epi32((w1), 3)))), w0));
            w1 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 17), _mm_slli_epi32((w15), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 19), _mm_slli_epi32((w15), 32 - 19)), _mm_srli_epi32((w15), 10)))), w10), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 7), _mm_slli_epi32((w2), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 18), _mm_slli_epi32((w2), 32 - 18)), _mm_srli_epi32((w2), 3)))), w1));
            w2 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 17), _mm_slli_epi32((w0), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 19), _mm_slli_epi32((w0), 32 - 19)), _mm_srli_epi32((w0), 10)))), w11), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 7), _mm_slli_epi32((w3), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 18), _mm_slli_epi32((w3), 32 - 18)), _mm_srli_epi32((w3), 3)))), w2));
            w3 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 17), _mm_slli_epi32((w1), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w1), 19), _mm_slli_epi32((w1), 32 - 19)), _mm_srli_epi32((w1), 10)))), w12), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 7), _mm_slli_epi32((w4), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 18), _mm_slli_epi32((w4), 32 - 18)), _mm_srli_epi32((w4), 3)))), w3));
            w4 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 17), _mm_slli_epi32((w2), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w2), 19), _mm_slli_epi32((w2), 32 - 19)), _mm_srli_epi32((w2), 10)))), w13), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 7), _mm_slli_epi32((w5), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 18), _mm_slli_epi32((w5), 32 - 18)), _mm_srli_epi32((w5), 3)))), w4));
            w5 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 17), _mm_slli_epi32((w3), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w3), 19), _mm_slli_epi32((w3), 32 - 19)), _mm_srli_epi32((w3), 10)))), w14), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 7), _mm_slli_epi32((w6), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 18), _mm_slli_epi32((w6), 32 - 18)), _mm_srli_epi32((w6), 3)))), w5));
            w6 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 17), _mm_slli_epi32((w4), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w4), 19), _mm_slli_epi32((w4), 32 - 19)), _mm_srli_epi32((w4), 10)))), w15), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 7), _mm_slli_epi32((w7), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 18), _mm_slli_epi32((w7), 32 - 18)), _mm_srli_epi32((w7), 3)))), w6));
            w7 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 17), _mm_slli_epi32((w5), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w5), 19), _mm_slli_epi32((w5), 32 - 19)), _mm_srli_epi32((w5), 10)))), w0), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 7), _mm_slli_epi32((w8), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 18), _mm_slli_epi32((w8), 32 - 18)), _mm_srli_epi32((w8), 3)))), w7));
            w8 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 17), _mm_slli_epi32((w6), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w6), 19), _mm_slli_epi32((w6), 32 - 19)), _mm_srli_epi32((w6), 10)))), w1), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 7), _mm_slli_epi32((w9), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 18), _mm_slli_epi32((w9), 32 - 18)), _mm_srli_epi32((w9), 3)))), w8));
            w9 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 17), _mm_slli_epi32((w7), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w7), 19), _mm_slli_epi32((w7), 32 - 19)), _mm_srli_epi32((w7), 10)))), w2), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 7), _mm_slli_epi32((w10), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 18), _mm_slli_epi32((w10), 32 - 18)), _mm_srli_epi32((w10), 3)))), w9));
            w10 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 17), _mm_slli_epi32((w8), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w8), 19), _mm_slli_epi32((w8), 32 - 19)), _mm_srli_epi32((w8), 10)))), w3), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 7), _mm_slli_epi32((w11), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 18), _mm_slli_epi32((w11), 32 - 18)), _mm_srli_epi32((w11), 3)))), w10));
            w11 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 17), _mm_slli_epi32((w9), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w9), 19), _mm_slli_epi32((w9), 32 - 19)), _mm_srli_epi32((w9), 10)))), w4), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w12), 7), _mm_slli_epi32((w12), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w12), 18), _mm_slli_epi32((w12), 32 - 18)), _mm_srli_epi32((w12), 3)))), w11));
            w12 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 17), _mm_slli_epi32((w10), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w10), 19), _mm_slli_epi32((w10), 32 - 19)), _mm_srli_epi32((w10), 10)))), w5), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w13), 7), _mm_slli_epi32((w13), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w13), 18), _mm_slli_epi32((w13), 32 - 18)), _mm_srli_epi32((w13), 3)))), w12));
            w13 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 17), _mm_slli_epi32((w11), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w11), 19), _mm_slli_epi32((w11), 32 - 19)), _mm_srli_epi32((w11), 10)))), w6), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w14), 7), _mm_slli_epi32((w14), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w14), 18), _mm_slli_epi32((w14), 32 - 18)), _mm_srli_epi32((w14), 3)))), w13));
            w14 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w12), 17), _mm_slli_epi32((w12), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w12), 19), _mm_slli_epi32((w12), 32 - 19)), _mm_srli_epi32((w12), 10)))), w7), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 7), _mm_slli_epi32((w15), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w15), 18), _mm_slli_epi32((w15), 32 - 18)), _mm_srli_epi32((w15), 3)))), w14));
            w15 = _mm_add_epi32(_mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w13), 17), _mm_slli_epi32((w13), 32 - 17)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w13), 19), _mm_slli_epi32((w13), 32 - 19)), _mm_srli_epi32((w13), 10)))), w8), _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 7), _mm_slli_epi32((w0), 32 - 7)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((w0), 18), _mm_slli_epi32((w0), 32 - 18)), _mm_srli_epi32((w0), 3)))), w15));
        }

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(h, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 6), _mm_slli_epi32((e), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 11), _mm_slli_epi32((e), 32 - 11)), _mm_or_si128(_mm_srli_epi32((e), 25), _mm_slli_epi32((e), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(e, f), _mm_andnot_si128(e, g))), _mm_add_epi32(RH_SSE_CONST(0x19A4C116), w0));
            d = _mm_add_epi32(d, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 2), _mm_slli_epi32((a), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 13), _mm_slli_epi32((a), 32 - 13)), _mm_or_si128(_mm_srli_epi32((a), 22), _mm_slli_epi32((a), 32 - 22))))), _mm_or_si128(_mm_and_si128(a, b), _mm_and_si128(c, _mm_or_si128(a, b))));
            h = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(g, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 6), _mm_slli_epi32((d), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 11), _mm_slli_epi32((d), 32 - 11)), _mm_or_si128(_mm_srli_epi32((d), 25), _mm_slli_epi32((d), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(d, e), _mm_andnot_si128(d, f))), _mm_add_epi32(RH_SSE_CONST(0x1E376C08), w1));
            c = _mm_add_epi32(c, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 2), _mm_slli_epi32((h), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 13), _mm_slli_epi32((h), 32 - 13)), _mm_or_si128(_mm_srli_epi32((h), 22), _mm_slli_epi32((h), 32 - 22))))), _mm_or_si128(_mm_and_si128(h, a), _mm_and_si128(b, _mm_or_si128(h, a))));
            g = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(f, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 6), _mm_slli_epi32((c), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 11), _mm_slli_epi32((c), 32 - 11)), _mm_or_si128(_mm_srli_epi32((c), 25), _mm_slli_epi32((c), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(c, d), _mm_andnot_si128(c, e))), _mm_add_epi32(RH_SSE_CONST(0x2748774C), w2));
            b = _mm_add_epi32(b, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 2), _mm_slli_epi32((g), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 13), _mm_slli_epi32((g), 32 - 13)), _mm_or_si128(_mm_srli_epi32((g), 22), _mm_slli_epi32((g), 32 - 22))))), _mm_or_si128(_mm_and_si128(g, h), _mm_and_si128(a, _mm_or_si128(g, h))));
            f = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(e, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 6), _mm_slli_epi32((b), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 11), _mm_slli_epi32((b), 32 - 11)), _mm_or_si128(_mm_srli_epi32((b), 25), _mm_slli_epi32((b), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(b, c), _mm_andnot_si128(b, d))), _mm_add_epi32(RH_SSE_CONST(0x34B0BCB5), w3));
            a = _mm_add_epi32(a, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 2), _mm_slli_epi32((f), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 13), _mm_slli_epi32((f), 32 - 13)), _mm_or_si128(_mm_srli_epi32((f), 22), _mm_slli_epi32((f), 32 - 22))))), _mm_or_si128(_mm_and_si128(f, g), _mm_and_si128(h, _mm_or_si128(f, g))));
            e = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(d, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 6), _mm_slli_epi32((a), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 11), _mm_slli_epi32((a), 32 - 11)), _mm_or_si128(_mm_srli_epi32((a), 25), _mm_slli_epi32((a), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(a, b), _mm_andnot_si128(a, c))), _mm_add_epi32(RH_SSE_CONST(0x391C0CB3), w4));
            h = _mm_add_epi32(h, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 2), _mm_slli_epi32((e), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 13), _mm_slli_epi32((e), 32 - 13)), _mm_or_si128(_mm_srli_epi32((e), 22), _mm_slli_epi32((e), 32 - 22))))), _mm_or_si128(_mm_and_si128(e, f), _mm_and_si128(g, _mm_or_si128(e, f))));
            d = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(c, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 6), _mm_slli_epi32((h), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 11), _mm_slli_epi32((h), 32 - 11)), _mm_or_si128(_mm_srli_epi32((h), 25), _mm_slli_epi32((h), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(h, a), _mm_andnot_si128(h, b))), _mm_add_epi32(RH_SSE_CONST(0x4ED8AA4A), w5));
            g = _mm_add_epi32(g, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 2), _mm_slli_epi32((d), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 13), _mm_slli_epi32((d), 32 - 13)), _mm_or_si128(_mm_srli_epi32((d), 22), _mm_slli_epi32((d), 32 - 22))))), _mm_or_si128(_mm_and_si128(d, e), _mm_and_si128(f, _mm_or_si128(d, e))));
            c = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(b, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 6), _mm_slli_epi32((g), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 11), _mm_slli_epi32((g), 32 - 11)), _mm_or_si128(_mm_srli_epi32((g), 25), _mm_slli_epi32((g), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(g, h), _mm_andnot_si128(g, a))), _mm_add_epi32(RH_SSE_CONST(0x5B9CCA4F), w6));
            f = _mm_add_epi32(f, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 2), _mm_slli_epi32((c), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 13), _mm_slli_epi32((c), 32 - 13)), _mm_or_si128(_mm_srli_epi32((c), 22), _mm_slli_epi32((c), 32 - 22))))), _mm_or_si128(_mm_and_si128(c, d), _mm_and_si128(e, _mm_or_si128(c, d))));
            b = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(a, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 6), _mm_slli_epi32((f), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 11), _mm_slli_epi32((f), 32 - 11)), _mm_or_si128(_mm_srli_epi32((f), 25), _mm_slli_epi32((f), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(f, g), _mm_andnot_si128(f, h))), _mm_add_epi32(RH_SSE_CONST(0x682E6FF3), w7));
            e = _mm_add_epi32(e, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 2), _mm_slli_epi32((b), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 13), _mm_slli_epi32((b), 32 - 13)), _mm_or_si128(_mm_srli_epi32((b), 22), _mm_slli_epi32((b), 32 - 22))))), _mm_or_si128(_mm_and_si128(b, c), _mm_and_si128(d, _mm_or_si128(b, c))));
            a = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(h, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 6), _mm_slli_epi32((e), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 11), _mm_slli_epi32((e), 32 - 11)), _mm_or_si128(_mm_srli_epi32((e), 25), _mm_slli_epi32((e), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(e, f), _mm_andnot_si128(e, g))), _mm_add_epi32(RH_SSE_CONST(0x748F82EE), w8));
            d = _mm_add_epi32(d, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 2), _mm_slli_epi32((a), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 13), _mm_slli_epi32((a), 32 - 13)), _mm_or_si128(_mm_srli_epi32((a), 22), _mm_slli_epi32((a), 32 - 22))))), _mm_or_si128(_mm_and_si128(a, b), _mm_and_si128(c, _mm_or_si128(a, b))));
            h = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(g, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 6), _mm_slli_epi32((d), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 11), _mm_slli_epi32((d), 32 - 11)), _mm_or_si128(_mm_srli_epi32((d), 25), _mm_slli_epi32((d), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(d, e), _mm_andnot_si128(d, f))), _mm_add_epi32(RH_SSE_CONST(0x78A5636F), w9));
            c = _mm_add_epi32(c, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 2), _mm_slli_epi32((h), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 13), _mm_slli_epi32((h), 32 - 13)), _mm_or_si128(_mm_srli_epi32((h), 22), _mm_slli_epi32((h), 32 - 22))))), _mm_or_si128(_mm_and_si128(h, a), _mm_and_si128(b, _mm_or_si128(h, a))));
            g = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(f, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 6), _mm_slli_epi32((c), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 11), _mm_slli_epi32((c), 32 - 11)), _mm_or_si128(_mm_srli_epi32((c), 25), _mm_slli_epi32((c), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(c, d), _mm_andnot_si128(c, e))), _mm_add_epi32(RH_SSE_CONST(0x84C87814), w10));
            b = _mm_add_epi32(b, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 2), _mm_slli_epi32((g), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 13), _mm_slli_epi32((g), 32 - 13)), _mm_or_si128(_mm_srli_epi32((g), 22), _mm_slli_epi32((g), 32 - 22))))), _mm_or_si128(_mm_and_si128(g, h), _mm_and_si128(a, _mm_or_si128(g, h))));
            f = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(e, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 6), _mm_slli_epi32((b), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 11), _mm_slli_epi32((b), 32 - 11)), _mm_or_si128(_mm_srli_epi32((b), 25), _mm_slli_epi32((b), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(b, c), _mm_andnot_si128(b, d))), _mm_add_epi32(RH_SSE_CONST(0x8CC70208), w11));
            a = _mm_add_epi32(a, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 2), _mm_slli_epi32((f), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 13), _mm_slli_epi32((f), 32 - 13)), _mm_or_si128(_mm_srli_epi32((f), 22), _mm_slli_epi32((f), 32 - 22))))), _mm_or_si128(_mm_and_si128(f, g), _mm_and_si128(h, _mm_or_si128(f, g))));
            e = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(d, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 6), _mm_slli_epi32((a), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((a), 11), _mm_slli_epi32((a), 32 - 11)), _mm_or_si128(_mm_srli_epi32((a), 25), _mm_slli_epi32((a), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(a, b), _mm_andnot_si128(a, c))), _mm_add_epi32(RH_SSE_CONST(0x90BEFFFA), w12));
            h = _mm_add_epi32(h, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 2), _mm_slli_epi32((e), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((e), 13), _mm_slli_epi32((e), 32 - 13)), _mm_or_si128(_mm_srli_epi32((e), 22), _mm_slli_epi32((e), 32 - 22))))), _mm_or_si128(_mm_and_si128(e, f), _mm_and_si128(g, _mm_or_si128(e, f))));
            d = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(c, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 6), _mm_slli_epi32((h), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((h), 11), _mm_slli_epi32((h), 32 - 11)), _mm_or_si128(_mm_srli_epi32((h), 25), _mm_slli_epi32((h), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(h, a), _mm_andnot_si128(h, b))), _mm_add_epi32(RH_SSE_CONST(0xA4506CEB), w13));
            g = _mm_add_epi32(g, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 2), _mm_slli_epi32((d), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((d), 13), _mm_slli_epi32((d), 32 - 13)), _mm_or_si128(_mm_srli_epi32((d), 22), _mm_slli_epi32((d), 32 - 22))))), _mm_or_si128(_mm_and_si128(d, e), _mm_and_si128(f, _mm_or_si128(d, e))));
            c = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(b, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 6), _mm_slli_epi32((g), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((g), 11), _mm_slli_epi32((g), 32 - 11)), _mm_or_si128(_mm_srli_epi32((g), 25), _mm_slli_epi32((g), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(g, h), _mm_andnot_si128(g, a))), _mm_add_epi32(RH_SSE_CONST(0xBEF9A3F7), w14));
            f = _mm_add_epi32(f, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 2), _mm_slli_epi32((c), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((c), 13), _mm_slli_epi32((c), 32 - 13)), _mm_or_si128(_mm_srli_epi32((c), 22), _mm_slli_epi32((c), 32 - 22))))), _mm_or_si128(_mm_and_si128(c, d), _mm_and_si128(e, _mm_or_si128(c, d))));
            b = _mm_add_epi32(T1, T2);
        };

        {
            T1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(a, (_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 6), _mm_slli_epi32((f), 32 - 6)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((f), 11), _mm_slli_epi32((f), 32 - 11)), _mm_or_si128(_mm_srli_epi32((f), 25), _mm_slli_epi32((f), 32 - 25)))))), _mm_xor_si128(_mm_and_si128(f, g), _mm_andnot_si128(f, h))), _mm_add_epi32(RH_SSE_CONST(0xC67178F2), w15));
            e = _mm_add_epi32(e, T1);
            T2 = _mm_add_epi32((_mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 2), _mm_slli_epi32((b), 32 - 2)), _mm_xor_si128(_mm_or_si128(_mm_srli_epi32((b), 13), _mm_slli_epi32((b), 32 - 13)), _mm_or_si128(_mm_srli_epi32((b), 22), _mm_slli_epi32((b), 32 - 22))))), _mm_or_si128(_mm_and_si128(b, c), _mm_and_si128(d, _mm_or_si128(b, c))));
            a = _mm_add_epi32(T1, T2);
        };


#if 0        
        RH_SHSSE_Round(a, b, c, d, e, f, g, h, 0x428A2F98, w0);
        RH_SHSSE_Round(h, a, b, c, d, e, f, g, 0x71374491, w1);
        RH_SHSSE_Round(g, h, a, b, c, d, e, f, 0xB5C0FBCF, w2);
        RH_SHSSE_Round(f, g, h, a, b, c, d, e, 0xE9B5DBA5, w3);
        RH_SHSSE_Round(e, f, g, h, a, b, c, d, 0x3956C25B, w4);
        RH_SHSSE_Round(d, e, f, g, h, a, b, c, 0x59F111F1, w5);
        RH_SHSSE_Round(c, d, e, f, g, h, a, b, 0x923F82A4, w6);
        RH_SHSSE_Round(b, c, d, e, f, g, h, a, 0xAB1C5ED5, w7);
        RH_SHSSE_Round(a, b, c, d, e, f, g, h, 0xD807AA98, w8);
        RH_SHSSE_Round(h, a, b, c, d, e, f, g, 0x12835B01, w9);
        RH_SHSSE_Round(g, h, a, b, c, d, e, f, 0x243185BE, w10);
        RH_SHSSE_Round(f, g, h, a, b, c, d, e, 0x550C7DC3, w11);
        RH_SHSSE_Round(e, f, g, h, a, b, c, d, 0x72BE5D74, w12);
        RH_SHSSE_Round(d, e, f, g, h, a, b, c, 0x80DEB1FE, w13);
        RH_SHSSE_Round(c, d, e, f, g, h, a, b, 0x9BDC06A7, w14);
        RH_SHSSE_Round(b, c, d, e, f, g, h, a, 0xC19BF174, w15);

        RH_SHSSE_WMIX()

            RH_SHSSE_Round(a, b, c, d, e, f, g, h, 0xE49B69C1, w0);
        RH_SHSSE_Round(h, a, b, c, d, e, f, g, 0xEFBE4786, w1);
        RH_SHSSE_Round(g, h, a, b, c, d, e, f, 0x0FC19DC6, w2);
        RH_SHSSE_Round(f, g, h, a, b, c, d, e, 0x240CA1CC, w3);
        RH_SHSSE_Round(e, f, g, h, a, b, c, d, 0x2DE92C6F, w4);
        RH_SHSSE_Round(d, e, f, g, h, a, b, c, 0x4A7484AA, w5);
        RH_SHSSE_Round(c, d, e, f, g, h, a, b, 0x5CB0A9DC, w6);
        RH_SHSSE_Round(b, c, d, e, f, g, h, a, 0x76F988DA, w7);
        RH_SHSSE_Round(a, b, c, d, e, f, g, h, 0x983E5152, w8);
        RH_SHSSE_Round(h, a, b, c, d, e, f, g, 0xA831C66D, w9);
        RH_SHSSE_Round(g, h, a, b, c, d, e, f, 0xB00327C8, w10);
        RH_SHSSE_Round(f, g, h, a, b, c, d, e, 0xBF597FC7, w11);
        RH_SHSSE_Round(e, f, g, h, a, b, c, d, 0xC6E00BF3, w12);
        RH_SHSSE_Round(d, e, f, g, h, a, b, c, 0xD5A79147, w13);
        RH_SHSSE_Round(c, d, e, f, g, h, a, b, 0x06CA6351, w14);
        RH_SHSSE_Round(b, c, d, e, f, g, h, a, 0x14292967, w15);

        RH_SHSSE_WMIX()

            RH_SHSSE_Round(a, b, c, d, e, f, g, h, 0x27B70A85, w0);
        RH_SHSSE_Round(h, a, b, c, d, e, f, g, 0x2E1B2138, w1);
        RH_SHSSE_Round(g, h, a, b, c, d, e, f, 0x4D2C6DFC, w2);
        RH_SHSSE_Round(f, g, h, a, b, c, d, e, 0x53380D13, w3);
        RH_SHSSE_Round(e, f, g, h, a, b, c, d, 0x650A7354, w4);
        RH_SHSSE_Round(d, e, f, g, h, a, b, c, 0x766A0ABB, w5);
        RH_SHSSE_Round(c, d, e, f, g, h, a, b, 0x81C2C92E, w6);
        RH_SHSSE_Round(b, c, d, e, f, g, h, a, 0x92722C85, w7);
        RH_SHSSE_Round(a, b, c, d, e, f, g, h, 0xA2BFE8A1, w8);
        RH_SHSSE_Round(h, a, b, c, d, e, f, g, 0xA81A664B, w9);
        RH_SHSSE_Round(g, h, a, b, c, d, e, f, 0xC24B8B70, w10);
        RH_SHSSE_Round(f, g, h, a, b, c, d, e, 0xC76C51A3, w11);
        RH_SHSSE_Round(e, f, g, h, a, b, c, d, 0xD192E819, w12);
        RH_SHSSE_Round(d, e, f, g, h, a, b, c, 0xD6990624, w13);
        RH_SHSSE_Round(c, d, e, f, g, h, a, b, 0xF40E3585, w14);
        RH_SHSSE_Round(b, c, d, e, f, g, h, a, 0x106AA070, w15);

        RH_SHSSE_WMIX()

            RH_SHSSE_Round(a, b, c, d, e, f, g, h, 0x19A4C116, w0);
        RH_SHSSE_Round(h, a, b, c, d, e, f, g, 0x1E376C08, w1);
        RH_SHSSE_Round(g, h, a, b, c, d, e, f, 0x2748774C, w2);
        RH_SHSSE_Round(f, g, h, a, b, c, d, e, 0x34B0BCB5, w3);
        RH_SHSSE_Round(e, f, g, h, a, b, c, d, 0x391C0CB3, w4);
        RH_SHSSE_Round(d, e, f, g, h, a, b, c, 0x4ED8AA4A, w5);
        RH_SHSSE_Round(c, d, e, f, g, h, a, b, 0x5B9CCA4F, w6);
        RH_SHSSE_Round(b, c, d, e, f, g, h, a, 0x682E6FF3, w7);
        RH_SHSSE_Round(a, b, c, d, e, f, g, h, 0x748F82EE, w8);
        RH_SHSSE_Round(h, a, b, c, d, e, f, g, 0x78A5636F, w9);
        RH_SHSSE_Round(g, h, a, b, c, d, e, f, 0x84C87814, w10);
        RH_SHSSE_Round(f, g, h, a, b, c, d, e, 0x8CC70208, w11);
        RH_SHSSE_Round(e, f, g, h, a, b, c, d, 0x90BEFFFA, w12);
        RH_SHSSE_Round(d, e, f, g, h, a, b, c, 0xA4506CEB, w13);
        RH_SHSSE_Round(c, d, e, f, g, h, a, b, 0xBEF9A3F7, w14);
        RH_SHSSE_Round(b, c, d, e, f, g, h, a, 0xC67178F2, w15);
#endif //0

        /*
                s[0] = _mm_add_epi32(a, s[0]);
                s[1] = _mm_add_epi32(b, s[1]);
                s[2] = _mm_add_epi32(c, s[2]);
                s[3] = _mm_add_epi32(d, s[3]);
                s[4] = _mm_add_epi32(e, s[4]);
                s[5] = _mm_add_epi32(f, s[5]);
                s[6] = _mm_add_epi32(g, s[6]);
                s[7] = _mm_add_epi32(h, s[7]);
        */

        T1 = _mm_set1_epi32(stateObj.state[0]);
        a = _mm_add_epi32(a, T1);
        T1 = _mm_set1_epi32(stateObj.state[1]);
        b = _mm_add_epi32(b, T1);
        T1 = _mm_set1_epi32(stateObj.state[2]);
        c = _mm_add_epi32(c, T1);
        T1 = _mm_set1_epi32(stateObj.state[3]);
        d = _mm_add_epi32(d, T1);
        T1 = _mm_set1_epi32(stateObj.state[4]);
        e = _mm_add_epi32(e, T1);
        T1 = _mm_set1_epi32(stateObj.state[5]);
        f = _mm_add_epi32(f, T1);
        T1 = _mm_set1_epi32(stateObj.state[6]);
        g = _mm_add_epi32(g, T1);
        T1 = _mm_set1_epi32(stateObj.state[7]);
        h = _mm_add_epi32(h, T1);
        T1 = _mm_set1_epi32(stateObj.state[8]);

    }

    // Unpack
/*
    __m128i u0 = _mm_unpacklo_epi32(s[0], s[1]);   // S2_1 S2_0 S3_1 S3_0
    __m128i u1 = _mm_unpackhi_epi32(s[0], s[1]);   // S0_1 S0_0 S1_1 S1_0

    __m128i u2 = _mm_unpacklo_epi32(s[2], s[3]);   // S2_3 S2_2 S3_3 S3_2
    __m128i u3 = _mm_unpackhi_epi32(s[2], s[3]);   // S0_3 S0_2 S1_3 S1_2
*/


    __m128i u0 = _mm_unpacklo_epi32(a, b);   // S2_1 S2_0 S3_1 S3_0
    __m128i u1 = _mm_unpackhi_epi32(a, b);   // S0_1 S0_0 S1_1 S1_0

    __m128i u2 = _mm_unpacklo_epi32(c, d);   // S2_3 S2_2 S3_3 S3_2
    __m128i u3 = _mm_unpackhi_epi32(c, d);   // S0_3 S0_2 S1_3 S1_2

    __m128i _d3 = _mm_unpacklo_epi32(u0, u2);      // S3_3 S3_1 S3_2 S3_0
    __m128i _d2 = _mm_unpackhi_epi32(u0, u2);      // S2_3 S2_1 S2_2 S2_0
    __m128i _d1 = _mm_unpacklo_epi32(u1, u3);      // S1_3 S1_1 S1_2 S1_0
    __m128i _d0 = _mm_unpackhi_epi32(u1, u3);      // S0_3 S0_1 S0_2 S0_0

    RH_STRIDE_SET_SIZE(output1, 32);
    RH_STRIDE_SET_SIZE(output2, 32);
    RH_STRIDE_SET_SIZE(output3, 32);
    RH_STRIDE_SET_SIZE(output4, 32);
    output1 = RH_STRIDE_GET_DATA(output1);
    output2 = RH_STRIDE_GET_DATA(output2);
    output3 = RH_STRIDE_GET_DATA(output3);
    output4 = RH_STRIDE_GET_DATA(output4);


    //linux's gcc to dumb to compile _mm_shuffle_epi8 with -mssse3 !!!
#ifndef RH2_DISABLE_SHUFFLE_EPI8
    __m128i mask = _mm_set_epi8(12, 13, 14, 15, /**/ 4, 5, 6, 7,  /**/ 8, 9, 10, 11,  /**/ 0, 1, 2, 3 );
    _d0 = _mm_shuffle_epi8(_d0, mask);
    _d1 = _mm_shuffle_epi8(_d1, mask);
    _d2 = _mm_shuffle_epi8(_d2, mask);
    _d3 = _mm_shuffle_epi8(_d3, mask);
#endif

    _mm_store_si128((__m128i *)output1, _d0);
    _mm_store_si128((__m128i *)output2, _d1);
    _mm_store_si128((__m128i *)output3, _d2);
    _mm_store_si128((__m128i *)output4, _d3);


    u0 = _mm_unpacklo_epi32(e, f);
    u1 = _mm_unpackhi_epi32(e, f);

    u2 = _mm_unpacklo_epi32(g, h);
    u3 = _mm_unpackhi_epi32(g, h);

    _d3 = _mm_unpacklo_epi32(u0, u2);
    _d2 = _mm_unpackhi_epi32(u0, u2);
    _d1 = _mm_unpacklo_epi32(u1, u3);
    _d0 = _mm_unpackhi_epi32(u1, u3);

#ifndef RH2_DISABLE_SHUFFLE_EPI8
    _d0 = _mm_shuffle_epi8(_d0, mask);
    _d1 = _mm_shuffle_epi8(_d1, mask);
    _d2 = _mm_shuffle_epi8(_d2, mask);
    _d3 = _mm_shuffle_epi8(_d3, mask);
#endif

    _mm_store_si128((__m128i *)(output1 + 4), _d0);
    _mm_store_si128((__m128i *)(output2 + 4), _d1);
    _mm_store_si128((__m128i *)(output3 + 4), _d2);
    _mm_store_si128((__m128i *)(output4 + 4), _d3);

    //linux's gcc to dumb to compile _mm_shuffle_epi8 with -mssse3 !!
#ifdef RH2_DISABLE_SHUFFLE_EPI8

    #define RH_SWAP_TEMP(a, b) \
    { \
        U32 t = RH_swap_u32(a); \
        a = RH_swap_u32(b); \
        b = t; \
    } \

    #define RH_SWAP_BLOCK_TEMP(OUTPUT) \
        OUTPUT[0] = RH_swap_u32(OUTPUT[0]); \
        RH_SWAP_TEMP(OUTPUT[2], OUTPUT[1]); \
        OUTPUT[3] = RH_swap_u32(OUTPUT[3]); \
        OUTPUT[4] = RH_swap_u32(OUTPUT[4]); \
        RH_SWAP_TEMP(OUTPUT[6], OUTPUT[5]); \
        OUTPUT[7] = RH_swap_u32(OUTPUT[7]); \

    RH_SWAP_BLOCK_TEMP(output1);
    RH_SWAP_BLOCK_TEMP(output2);
    RH_SWAP_BLOCK_TEMP(output3);
    RH_SWAP_BLOCK_TEMP(output4);

#endif

}

#endif //#ifndef RHMINER_NO_SIMD

//#define RandomHash_SHA2_256_Part2 RandomHash_SHA2_256_Part2_SSE_4x
void RandomHash_SHA2_256_Part2(RH_StridePtr roundInput, const SHA2_256_SavedState& stateObj, RH_StridePtr output)
{
    uint32_t* out32 = RH_STRIDE_GET_DATA(output);
    RH_STRIDE_SET_SIZE(output, 32);
    uint32_t *data = (uint32_t *)(RH_STRIDE_GET_DATA8(roundInput) + stateObj.nextCut);

//{{{ TEMP TEMP TEMP
#ifdef RHMINER_DEBUG_RANDOMHASH_UNITTEST
    //ONLY FOR UNITTEST
    if (stateObj.len  == 36)
#else
    RH_ASSERT(stateObj.len  == 36)
#endif
//}}} TEMP TEMP TEMP

    {
        //SHA2_256_RoundFunction_Last20(data, state);
        uint32_t A, B, C, D, E, F, G, H, T, T2;

        //scratch buffer 64 uint32
        RH_ALIGN(128) uint32_t beData[64];


        /*
        RHMINER_ASSERT(0x80 == data[11]); // 0x80000000
        RHMINER_ASSERT(0 == data[12]);
        RHMINER_ASSERT(0 == data[13]);
        RHMINER_ASSERT(0 == data[14]);
        RHMINER_ASSERT(0x60070000 == data[15]); //0x00000760
        */


        A = stateObj.state[0];
        B = stateObj.state[1];
        C = stateObj.state[2];
        D = stateObj.state[3];
        E = stateObj.state[4];
        F = stateObj.state[5];
        G = stateObj.state[6];
        H = stateObj.state[7];

#if 0 //Collpse 1..9 + part2
#define  _CACHE(X, code) data[X]
        {
            // Step 1
            //T = 0;
            //T2 = RH_swap_u32(data[1]);
            //0x16161616 = RH_swap_u32(data[9]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[0]);

            //T = 0x00000760;
            //T2 = RH_swap_u32(data[2]);
            beData[17] = ((ROTR32(0x00000760, 17)) ^ (ROTR32(0x00000760, 19)) ^ (0x00000760 >> 10)) + RH_swap_u32(data[10]) + _CACHE(1, ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[1]));

            //T = 0x16161616;
            //T2 = RH_swap_u32(data[3]);
            //0x18181818 = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x80000000 + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[2]);

            T = beData[17];
            //T2 = RH_swap_u32(data[4]);
            beData[19] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + _CACHE(3, ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[3]));

            //T = 0x18181818;
            //T2 = RH_swap_u32(data[5]);
            //0x20202020 = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[4]);

            T = beData[19];
            //T2 = RH_swap_u32(data[6]);
            beData[21] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + _CACHE(5, ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[5]));

            //T = 0x20202020;
            //T2 = RH_swap_u32(data[7]);
            //0x22222222 = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x00000760 + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[6]);

            T = beData[21];
            //T2 = RH_swap_u32(data[8]);
            beData[23] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x16161616 + _CACHE(7, ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[7]));

            T = 0x22222222;
            //T2 = RH_swap_u32(data[9]);
            beData[24] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[17] + _CACHE(8, ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[8]));

            T = beData[23];
            T2 = RH_swap_u32(data[10]);
            beData[25] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x18181818 + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[9]);

            T = beData[24];
            //T2 = 0x80000000;
            beData[26] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[19] + ((ROTR32(0x80000000, 7)) ^ (ROTR32(0x80000000, 18)) ^ (0x80000000 >> 3)) + RH_swap_u32(data[10]);

            T = beData[25];
            //T2 = 0;
            beData[27] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x20202020 + 0x80000000;

            T = beData[26];
            //T2 = 0;
            beData[28] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[21];

            T = beData[27];
            //T2 = 0;
            beData[29] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x22222222;

            T = beData[28];
            //T2 = 0x00000760;
            beData[30] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[23] + ((ROTR32(0x00000760, 7)) ^ (ROTR32(0x00000760, 18)) ^ (0x00000760 >> 3));

            T = beData[29];
            T2 = 0x16161616;
            beData[31] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[24] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + 0x00000760;

            T = beData[30];
            T2 = beData[17];
            beData[32] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[25] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + 0x16161616;

            T = beData[31];
            T2 = 0x18181818;
            beData[33] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[26] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[17];

            T = beData[32];
            T2 = beData[19];
            beData[34] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[27] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + 0x18181818;

            T = beData[33];
            T2 = 0x20202020;
            beData[35] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[28] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[19];

            T = beData[34];
            T2 = beData[21];
            beData[36] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[29] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + 0x20202020;

            T = beData[35];
            T2 = 0x22222222;
            beData[37] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[30] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[21];

            T = beData[36];
            T2 = beData[23];
            beData[38] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[31] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + 0x22222222;

            T = beData[37];
            T2 = beData[24];
            beData[39] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[32] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[23];

            T = beData[38];
            T2 = beData[25];
            beData[40] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[33] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[24];

            T = beData[39];
            T2 = beData[26];
            beData[41] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[34] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[25];

            T = beData[40];
            T2 = beData[27];
            beData[42] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[35] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[26];

            T = beData[41];
            T2 = beData[28];
            beData[43] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[36] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[27];

            T = beData[42];
            T2 = beData[29];
            beData[44] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[37] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[28];

            T = beData[43];
            T2 = beData[30];
            beData[45] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[38] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[29];

            T = beData[44];
            T2 = beData[31];
            beData[46] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[39] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[30];

            T = beData[45];
            T2 = beData[32];
            beData[47] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[40] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[31];

            T = beData[46];
            T2 = beData[33];
            beData[48] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[41] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[32];

            T = beData[47];
            T2 = beData[34];
            beData[49] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[42] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[33];

            T = beData[48];
            T2 = beData[35];
            beData[50] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[43] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[34];

            T = beData[49];
            T2 = beData[36];
            beData[51] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[44] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[35];

            T = beData[50];
            T2 = beData[37];
            beData[52] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[45] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[36];

            T = beData[51];
            T2 = beData[38];
            beData[53] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[46] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[37];

            T = beData[52];
            T2 = beData[39];
            beData[54] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[47] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[38];

            T = beData[53];
            T2 = beData[40];
            beData[55] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[48] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[39];

            T = beData[54];
            T2 = beData[41];
            beData[56] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[49] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[40];

            T = beData[55];
            T2 = beData[42];
            beData[57] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[50] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[41];

            T = beData[56];
            T2 = beData[43];
            beData[58] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[51] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[42];

            T = beData[57];
            T2 = beData[44];
            beData[59] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[52] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[43];

            T = beData[58];
            T2 = beData[45];
            beData[60] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[53] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[44];

            T = beData[59];
            T2 = beData[46];
            beData[61] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[54] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[45];

            T = beData[60];
            T2 = beData[47];
            beData[62] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[55] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[46];

            T = beData[61];
            T2 = beData[48];
            beData[63] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[56] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[47];

            // Step 2
            /*
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x428A2F98 + RH_swap_u32(data[0]);
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x71374491 + RH_swap_u32(data[1]);
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB5C0FBCF + RH_swap_u32(data[2]);
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xE9B5DBA5 + RH_swap_u32(data[3]);
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x3956C25B + RH_swap_u32(data[4]);
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x59F111F1 + RH_swap_u32(data[5]);
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x923F82A4 + RH_swap_u32(data[6]);
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xAB1C5ED5 + RH_swap_u32(data[7]);
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xD807AA98 + RH_swap_u32(data[8]);
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x12835B01 + RH_swap_u32(data[9]);
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            */

            T = beData[1];
            B = beData[2];
            F = beData[3];
            T = beData[4];
            A = beData[5];
            E = beData[6];
            T = beData[7];
            H = beData[8];
            D = beData[9];
            T = beData[10];
            G = beData[11];
            C = beData[12];

            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x243185BE + RH_swap_u32(data[10]);
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x550C7DC3 + 0x80000000;
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x72BE5D74;
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x80DEB1FE;
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x9BDC06A7;
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC19BF174 + 0x00000760;
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xE49B69C1 + 0x16161616;
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xEFBE4786 + beData[17];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x0FC19DC6 + 0x18181818;
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x240CA1CC + beData[19];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x2DE92C6F + 0x20202020;
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4A7484AA + beData[21];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5CB0A9DC + 0x22222222;
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x76F988DA + beData[23];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x983E5152 + beData[24];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA831C66D + beData[25];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB00327C8 + beData[26];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xBF597FC7 + beData[27];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xC6E00BF3 + beData[28];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD5A79147 + beData[29];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x06CA6351 + beData[30];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x14292967 + beData[31];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x27B70A85 + beData[32];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x2E1B2138 + beData[33];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x4D2C6DFC + beData[34];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x53380D13 + beData[35];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x650A7354 + beData[36];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x766A0ABB + beData[37];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x81C2C92E + beData[38];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x92722C85 + beData[39];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xA2BFE8A1 + beData[40];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA81A664B + beData[41];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xC24B8B70 + beData[42];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xC76C51A3 + beData[43];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xD192E819 + beData[44];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD6990624 + beData[45];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xF40E3585 + beData[46];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x106AA070 + beData[47];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x19A4C116 + beData[48];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x1E376C08 + beData[49];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x2748774C + beData[50];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x34B0BCB5 + beData[51];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x391C0CB3 + beData[52];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4ED8AA4A + beData[53];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5B9CCA4F + beData[54];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x682E6FF3 + beData[55];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x748F82EE + beData[56];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x78A5636F + beData[57];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x84C87814 + beData[58];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x8CC70208 + beData[59];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x90BEFFFA + beData[60];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xA4506CEB + beData[61];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xBEF9A3F7 + beData[62];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC67178F2 + beData[63];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
        }
#endif //colaps 1..9+part2

#if 1
        {
            //T = 0;
            T2 = RH_swap_u32(data[1]);
            beData[16] = RH_swap_u32(data[9]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[0]);

            //T = 0x00000760;
            T2 = RH_swap_u32(data[2]);
            beData[17] = ((ROTR32(0x00000760, 17)) ^ (ROTR32(0x00000760, 19)) ^ (0x00000760 >> 10)) + RH_swap_u32(data[10]) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[1]);

            T = beData[16];
            T2 = RH_swap_u32(data[3]);
            beData[18] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x80000000 + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[2]);

            T = beData[17];
            T2 = RH_swap_u32(data[4]);
            beData[19] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[3]);

            T = beData[18];
            T2 = RH_swap_u32(data[5]);
            beData[20] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[4]);

            T = beData[19];
            T2 = RH_swap_u32(data[6]);
            beData[21] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[5]);

            T = beData[20];
            T2 = RH_swap_u32(data[7]);
            beData[22] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + 0x00000760 + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[6]);

            T = beData[21];
            T2 = RH_swap_u32(data[8]);
            beData[23] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[16] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[7]);

            T = beData[22];
            T2 = RH_swap_u32(data[9]);
            beData[24] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[17] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[8]);

            T = beData[23];
            T2 = RH_swap_u32(data[10]);
            beData[25] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[18] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + RH_swap_u32(data[9]);

            T = beData[24];
            //T2 = 0x80000000;
            beData[26] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[19] + ((ROTR32(0x80000000, 7)) ^ (ROTR32(0x80000000, 18)) ^ (0x80000000 >> 3)) + RH_swap_u32(data[10]);

            T = beData[25];
            //T2 = 0;
            beData[27] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[20] + 0x80000000;

            T = beData[26];
            //T2 = 0;
            beData[28] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[21];

            T = beData[27];
            //T2 = 0;
            beData[29] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[22];

            T = beData[28];
            //T2 = 0x00000760;
            beData[30] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[23] + ((ROTR32(0x00000760, 7)) ^ (ROTR32(0x00000760, 18)) ^ (0x00000760 >> 3));

            T = beData[29];
            T2 = beData[16];
            beData[31] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[24] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + 0x00000760;

            T = beData[30];
            T2 = beData[17];
            beData[32] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[25] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[16];

            T = beData[31];
            T2 = beData[18];
            beData[33] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[26] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[17];

            T = beData[32];
            T2 = beData[19];
            beData[34] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[27] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[18];

            T = beData[33];
            T2 = beData[20];
            beData[35] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[28] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[19];

            T = beData[34];
            T2 = beData[21];
            beData[36] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[29] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[20];

            T = beData[35];
            T2 = beData[22];
            beData[37] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[30] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[21];

            T = beData[36];
            T2 = beData[23];
            beData[38] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[31] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[22];

            T = beData[37];
            T2 = beData[24];
            beData[39] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[32] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[23];

            T = beData[38];
            T2 = beData[25];
            beData[40] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[33] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[24];

            T = beData[39];
            T2 = beData[26];
            beData[41] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[34] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[25];

            T = beData[40];
            T2 = beData[27];
            beData[42] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[35] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[26];

            T = beData[41];
            T2 = beData[28];
            beData[43] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[36] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[27];

            T = beData[42];
            T2 = beData[29];
            beData[44] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[37] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[28];

            T = beData[43];
            T2 = beData[30];
            beData[45] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[38] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[29];

            T = beData[44];
            T2 = beData[31];
            beData[46] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[39] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[30];

            T = beData[45];
            T2 = beData[32];
            beData[47] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[40] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[31];

            T = beData[46];
            T2 = beData[33];
            beData[48] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[41] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[32];

            T = beData[47];
            T2 = beData[34];
            beData[49] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[42] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[33];

            T = beData[48];
            T2 = beData[35];
            beData[50] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[43] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[34];

            T = beData[49];
            T2 = beData[36];
            beData[51] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[44] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[35];

            T = beData[50];
            T2 = beData[37];
            beData[52] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[45] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[36];

            T = beData[51];
            T2 = beData[38];
            beData[53] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[46] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[37];

            T = beData[52];
            T2 = beData[39];
            beData[54] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[47] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[38];

            T = beData[53];
            T2 = beData[40];
            beData[55] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[48] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[39];

            T = beData[54];
            T2 = beData[41];
            beData[56] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[49] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[40];

            T = beData[55];
            T2 = beData[42];
            beData[57] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[50] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[41];

            T = beData[56];
            T2 = beData[43];
            beData[58] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[51] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[42];

            T = beData[57];
            T2 = beData[44];
            beData[59] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[52] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[43];

            T = beData[58];
            T2 = beData[45];
            beData[60] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[53] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[44];

            T = beData[59];
            T2 = beData[46];
            beData[61] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[54] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[45];

            T = beData[60];
            T2 = beData[47];
            beData[62] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[55] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[46];

            T = beData[61];
            T2 = beData[48];
            beData[63] = ((ROTR32(T, 17)) ^ (ROTR32(T, 19)) ^ (T >> 10)) + beData[56] + ((ROTR32(T2, 7)) ^ (ROTR32(T2, 18)) ^ (T2 >> 3)) + beData[47];

            // Step 2

            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x428A2F98 + RH_swap_u32(data[0]);
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x71374491 + RH_swap_u32(data[1]);
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB5C0FBCF + RH_swap_u32(data[2]);
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xE9B5DBA5 + RH_swap_u32(data[3]);
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x3956C25B + RH_swap_u32(data[4]);
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x59F111F1 + RH_swap_u32(data[5]);
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x923F82A4 + RH_swap_u32(data[6]);
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xAB1C5ED5 + RH_swap_u32(data[7]);
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xD807AA98 + RH_swap_u32(data[8]);
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x12835B01 + RH_swap_u32(data[9]);
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x243185BE + RH_swap_u32(data[10]);
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x550C7DC3 + 0x80000000;
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x72BE5D74;
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x80DEB1FE;
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x9BDC06A7;
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC19BF174 + 0x00000760;
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xE49B69C1 + beData[16];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xEFBE4786 + beData[17];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x0FC19DC6 + beData[18];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x240CA1CC + beData[19];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x2DE92C6F + beData[20];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4A7484AA + beData[21];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5CB0A9DC + beData[22];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x76F988DA + beData[23];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x983E5152 + beData[24];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA831C66D + beData[25];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xB00327C8 + beData[26];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xBF597FC7 + beData[27];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xC6E00BF3 + beData[28];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD5A79147 + beData[29];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x06CA6351 + beData[30];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x14292967 + beData[31];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x27B70A85 + beData[32];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x2E1B2138 + beData[33];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x4D2C6DFC + beData[34];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x53380D13 + beData[35];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x650A7354 + beData[36];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x766A0ABB + beData[37];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x81C2C92E + beData[38];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x92722C85 + beData[39];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0xA2BFE8A1 + beData[40];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0xA81A664B + beData[41];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0xC24B8B70 + beData[42];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0xC76C51A3 + beData[43];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0xD192E819 + beData[44];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xD6990624 + beData[45];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xF40E3585 + beData[46];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x106AA070 + beData[47];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x19A4C116 + beData[48];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x1E376C08 + beData[49];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x2748774C + beData[50];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x34B0BCB5 + beData[51];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x391C0CB3 + beData[52];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0x4ED8AA4A + beData[53];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0x5B9CCA4F + beData[54];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0x682E6FF3 + beData[55];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
            T = H + ((ROTR32(E, 6)) ^ (ROTR32(E, 11)) ^ (ROTR32(E, 25))) + ((E & F) ^ (~E & G)) + 0x748F82EE + beData[56];
            T2 = ((ROTR32(A, 2)) ^ (ROTR32(A, 13)) ^ ((A >> 22) ^ (A << 10))) + ((A & B) ^ (A & C) ^ (B & C));
            H = T + T2;
            D = D + T;
            T = G + ((ROTR32(D, 6)) ^ (ROTR32(D, 11)) ^ (ROTR32(D, 25))) + ((D & E) ^ (~D & F)) + 0x78A5636F + beData[57];
            T2 = ((ROTR32(H, 2)) ^ (ROTR32(H, 13)) ^ ((H >> 22) ^ (H << 10))) + ((H & A) ^ (H & B) ^ (A & B));
            G = T + T2;
            C = C + T;
            T = F + ((ROTR32(C, 6)) ^ (ROTR32(C, 11)) ^ (ROTR32(C, 25))) + ((C & D) ^ (~C & E)) + 0x84C87814 + beData[58];
            T2 = ((ROTR32(G, 2)) ^ (ROTR32(G, 13)) ^ ((G >> 22) ^ (G << 10))) + ((G & H) ^ (G & A) ^ (H & A));
            F = T + T2;
            B = B + T;
            T = E + ((ROTR32(B, 6)) ^ (ROTR32(B, 11)) ^ (ROTR32(B, 25))) + ((B & C) ^ (~B & D)) + 0x8CC70208 + beData[59];
            T2 = ((ROTR32(F, 2)) ^ (ROTR32(F, 13)) ^ ((F >> 22) ^ (F << 10))) + ((F & G) ^ (F & H) ^ (G & H));
            E = T + T2;
            A = A + T;
            T = D + ((ROTR32(A, 6)) ^ (ROTR32(A, 11)) ^ (ROTR32(A, 25))) + ((A & B) ^ (~A & C)) + 0x90BEFFFA + beData[60];
            T2 = ((ROTR32(E, 2)) ^ (ROTR32(E, 13)) ^ ((E >> 22) ^ (E << 10))) + ((E & F) ^ (E & G) ^ (F & G));
            D = T + T2;
            H = H + T;
            T = C + ((ROTR32(H, 6)) ^ (ROTR32(H, 11)) ^ (ROTR32(H, 25))) + ((H & A) ^ (~H & B)) + 0xA4506CEB + beData[61];
            T2 = ((ROTR32(D, 2)) ^ (ROTR32(D, 13)) ^ ((D >> 22) ^ (D << 10))) + ((D & E) ^ (D & F) ^ (E & F));
            C = T + T2;
            G = G + T;
            T = B + ((ROTR32(G, 6)) ^ (ROTR32(G, 11)) ^ (ROTR32(G, 25))) + ((G & H) ^ (~G & A)) + 0xBEF9A3F7 + beData[62];
            T2 = ((ROTR32(C, 2)) ^ (ROTR32(C, 13)) ^ ((C >> 22) ^ (C << 10))) + ((C & D) ^ (C & E) ^ (D & E));
            B = T + T2;
            F = F + T;
            T = A + ((ROTR32(F, 6)) ^ (ROTR32(F, 11)) ^ (ROTR32(F, 25))) + ((F & G) ^ (~F & H)) + 0xC67178F2 + beData[63];
            T2 = ((ROTR32(B, 2)) ^ (ROTR32(B, 13)) ^ ((B >> 22) ^ (B << 10))) + ((B & C) ^ (B & D) ^ (C & D));
            A = T + T2;
            E = E + T;
        }
#endif //no-collaps

            out32[0] = RH_swap_u32(stateObj.state[0] + A);
            out32[1] = RH_swap_u32(stateObj.state[1] + B);
            out32[2] = RH_swap_u32(stateObj.state[2] + C);
            out32[3] = RH_swap_u32(stateObj.state[3] + D);
            out32[4] = RH_swap_u32(stateObj.state[4] + E);
            out32[5] = RH_swap_u32(stateObj.state[5] + F);
            out32[6] = RH_swap_u32(stateObj.state[6] + G);
            out32[7] = RH_swap_u32(stateObj.state[7] + H);
        }
//{{{ TEMP TEMP TEMP
#ifdef RHMINER_DEBUG_RANDOMHASH_UNITTEST
    else
    {
        RH_ALIGN(128) uint8_t pad[/*72*/80];
        RH_ALIGN(128) uint32_t state[8];
        state[0] = stateObj.state[0];
        state[1] = stateObj.state[1];
        state[2] = stateObj.state[2];
        state[3] = stateObj.state[3];
        state[4] = stateObj.state[4];
        state[5] = stateObj.state[5];
        state[6] = stateObj.state[6];
        state[7] = stateObj.state[7];

        U32 len = stateObj.len + stateObj.endCut; //len 8 -> header 200, len 44 -> header 236
        RH_ASSERT(len < 56);
        int32_t padindex;
        padindex = 56 - len;

        RH_memzero_of16(pad, sizeof(pad));

        pad[0] = 0x80;
        uint64_t bits = ReverseBytesUInt64(stateObj.bits); 
        ReadUInt64AsBytesLE(bits, pad + padindex);

        padindex = padindex + 8;
        memcpy(((uint8_t*)data) + len, pad, padindex);

        SHA2_256_RoundFunction(data, state);

        //get the hash result IN BE
        data = RH_STRIDE_GET_DATA(output);
        RH_STRIDE_SET_SIZE(output, 32);

        copy8_op(data, state, ReverseBytesUInt32);
    }
#endif
//}}} TEMP TEMP TEMP
}


#endif