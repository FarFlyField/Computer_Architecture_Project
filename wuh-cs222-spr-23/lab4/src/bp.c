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

#include "pipe.h"
#include "shell.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "bp.h"

//will init in pipe.c

// //the two functions below are borrowed from pipe.c
// // creates bitmask of specified len
// uint32_t mask_bp(int len) {
// 	return ~(~0x0<<len);
// }
// // gets inclusive bits from hi to lo, but for 64 bits. 
// uint64_t getbits_64(uint64_t i, int hi, int lo) {
// 	return (i>>lo) & mask_bp(hi-lo+1);
// }

void bp_init()
{
    //init Gshare, set everything to 0
    BP.Gshare.GHR=0;
    for (int i=0; i<256; i++){
        BP.Gshare.PHT[i]=0;
    }
    
    //init BTB, set everything to 0
    for (int i=0; i<1024; i++){
        BP.BTB[i].tag=0;
        BP.BTB[i].valid=0;
        BP.BTB[i].condb=0;
        BP.BTB[i].target=0;
    }
}

void bp_predict(uint64_t *predPC, uint8_t* missed)
{
    uint64_t PC = CURRENT_STATE.PC;

    //At every fetch cycle, the predictor indexes into both the BTB and the PHT.
    //Therefore, do the indexing first
    uint64_t BTB_index = getbits_64(PC, 11, 2);
    uint64_t PHT_index = (getbits_64(PC, 9, 2)^(BP.Gshare.GHR)) & 0xFF;

    //If the predictor misses in the BTB (i.e., address tag != PC or valid bit == 0), 
    //then the next PC is predicted as PC+4.
    if (((BP.BTB[BTB_index].valid) && BP.BTB[BTB_index].tag == PC)){
        //this is when we hit
        *missed = 0;
        //then the next PC is predicted as the target supplied by the BTB entry when
        //either of the following two conditions are met: 
        //(i) the BTB entry indicates that the branch is unconditional
        //(ii) the gshare predictor indicates that the branch should be taken
        if ((BP.BTB[BTB_index].condb==0) || (BP.Gshare.PHT[PHT_index]>=WT)){
            PC = BP.BTB[BTB_index].target;
            //printf("Predicted taken. Predicted PC:0x%lx\n", PC);   // DB
        } else {
            PC += 4;
            //printf("Predicted not taken. Predicted PC:0x%lx\n",PC);
        }
    } else {
        // Otherwise, the next PC is predicted as PC+4.
        PC += 4;
        *missed = 1;
        //printf("Missed in BTB. Predicted PC:0x%lx\n", PC); // DB
    }

    *predPC = CURRENT_STATE.PC = PC;
    //printf("PREDICTED PC:0x%lx\n", CURRENT_STATE.PC);
}

void bp_update(uint64_t PC, uint64_t bPC, uint8_t condb, uint8_t btaken)
{
    /* Update BTB */
    /* Update gshare directional predictor */
    /* Update global history register */
    /* Predict next PC */
    //not changing PC here

    //Unconditional branches do not update the PHT or the GHR, 
    //but only the BTB (setting the unconditional
    //bit in the corresponding entry).
    //Therefore, only update "PHT or the GHR" when it's conditional branch
    if (condb){
        //(i) updating the PHT, which is indexed using the current value of the GHR and PC,
        uint64_t PHT_to_XOR = getbits_64(PC, 9, 2);
        //XOR to get the index
        uint8_t PHT_index = (PHT_to_XOR^BP.Gshare.GHR) & 0xFF;
        uint8_t state_now = BP.Gshare.PHT[PHT_index];
        if (btaken) {
            if((++state_now)>3){
                state_now = 3;
            }
        }else{
            if((--state_now)<0){
                state_now = 0;
            }
        }
        //now I have upper bound the state_now by 0,1,2,3
        //finished updating the PHT inside
        BP.Gshare.PHT[PHT_index] = state_now;

        //(ii) updating the GHR (note that, the update of GHR must happen after the 
        //update to PHT to make sure we update the right entry)
        BP.Gshare.GHR = (BP.Gshare.GHR<<1)|(btaken ? 1:0);
        //finished updating Gshare
    }

    //but then whether condional or not, we always update BTB. 
    //(iii) updating the BTB. Unconditional branches do not update the PHT or the GHR, 
    //but only the BTB (setting the unconditional
    //bit in the corresponding entry). 
    uint64_t BTB_index = getbits_64(PC, 11, 2);
    BP.BTB[BTB_index].tag = PC;
    BP.BTB[BTB_index].valid = 1;
    BP.BTB[BTB_index].condb = condb;
    BP.BTB[BTB_index].target = bPC;
    //finished indexing BTB

    //When you need to add a new PC/branch target to BTB, but the BTB entry
    //was already taken by another PC with the same lower address bits, you should always replace the entry with
    //the new PC.
    //Therefore I can update BTB no matter what. 
    
    //5 8 10 100, run small.s
}
