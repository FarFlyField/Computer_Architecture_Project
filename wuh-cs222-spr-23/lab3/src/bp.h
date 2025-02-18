/*
 * ARM pipeline timing simulator
 *
 * CMSC 22200
 * 
 * Contributors:
 * Wu,Haoran (wuh)
 * Wang,Shaoge (shaogew)
 * 
 */

#include <inttypes.h>

#ifndef _BP_H_
#define _BP_H_

#define NT 0
#define T 1

#define SNT 0
#define WNT 1
#define WT 2
#define ST 3

typedef struct Gshare 
{
    //8-bit global branch history register (GHR). The most recent branch
    //is stored in the least-significant-bit of the GHR and a value of ‘1’ denotes a taken branch
    uint8_t GHR;
    //The predictor
    //XORs the GHR with 
    //bits [9:2] of the PC and uses this 8 bit value to index 
    //into a 256-entry pattern history
    //table (PHT). 
    uint8_t PHT[256];

} gshare_t;

typedef struct BTB 
{
    //(i) an address tag, which is the full 64 bits of the fetch stage PC;
    uint64_t tag;

    //(ii) a valid bit (1 means the entry is valid, 0 means the entry is not valid); 
    uint8_t valid;

    //(iii) a bit indicating whether this branch is conditional 
    //(1 means the branch is conditional, 0 means the branch is unconditional)
    uint8_t condb;

    //(iv) the target of the branch, which is 64 bits, with the low two bits always equal to 2'b00.
    uint64_t target;
} BTB_t;

typedef struct bp 
{ 
    gshare_t Gshare;

    //The branch target buffer (BTB) contains 1024 entries and is indexed by bits
    //[11:2] of the PC.
    BTB_t BTB[1024];
} bp_t;

extern bp_t BP;

uint64_t getbits_64(uint64_t i, int hi, int lo);
void setbit_8(uint8_t *i, int b);

void bp_init();

void bp_predict(uint64_t *predPC, uint8_t* missed);
void bp_update(uint64_t PC, uint64_t bPC, uint8_t condb, uint8_t btaken);

#endif
