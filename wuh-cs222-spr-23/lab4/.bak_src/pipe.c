/*
 * CMSC 22200
 *
 * ARM pipeline timing simulator
 * 
 * Contributors:
 * Wu,Haoran (wuh)
 * Wang,Shaoge (shaogew)
 * 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "pipe.h"
#include "shell.h"
#include "bp.h"
#include "cache.h"

/* global pipeline state */
CPU_State CURRENT_STATE;
pipereg_IFID IFID;
pipereg_IDEX IDEX;
pipereg_EXM EXM;
pipereg_EXM EXM_bak; // for dcache stall
pipereg_MWB MWB;

uint8_t CYCLE_INIT_DC_STALL;    // dc stall status at beginning of cycle
int DCMISS_ICMISS_DIFF;    // num of cycles by which a dcache miss
                            // occurs later than a pending icache miss

//make the BP here
bp_t BP;

// caches
cache_t *icache;
cache_t *dcache;

// create 32b bitmask of specified len
uint32_t mask(int len) {
    return ~(~0x0<<len);
}

// 64b version of mask
uint64_t mask_64(int len) {
    return ~(~0x0<<len);
}

// get bit with specified idx
uint8_t getbit(uint32_t i, int idx) {
    return (i>>idx) & mask(1);
}

// get inclusive bits from hi to lo
uint32_t getbits(uint32_t i, int hi, int lo) {
    return (i>>lo) & mask(hi-lo+1);
}

// 64b version of getbits
uint64_t getbits_64(uint64_t i, int hi, int lo) {
    return (i>>lo) & mask(hi-lo+1);
}

// set bit 
void setbit(uint32_t *i, int b) {
    *i |= (0x1<<b);
}

// set N and Z flags
void flagset(int64_t ALU_res, uint8_t *newN, uint8_t *newZ) {
    *newN = ALU_res>>63 ? 0x1 : 0x0;
    *newZ = ALU_res ? 0x0 : 0x1;
}

uint8_t flagtest(uint8_t cond, uint8_t N, uint8_t Z) {
    uint8_t C = 0x0;
    uint8_t V = 0x0;
    if (cond==0x0)      return Z;   // eq
    else if (cond==0x1) return !Z;  // ne
    else if (cond==0xa) return N==V;    // ge
    else if (cond==0xb) return N!=V;    // lt
    else if (cond==0xc) return (!Z)&&(N==V);    // gt
    else if (cond==0xd) return Z||(N!=V);   // le
}

// adder representation
int64_t add(int64_t data1, int64_t data2) {
    return data1+data2;
}

// Mux representation
int64_t mux(int sel, int64_t data1, int64_t data0) {
    return sel? data1 : data0;
}

// ALU representation
int64_t ALU(uint8_t ALU_sig, int64_t data1, int64_t data2) 
{
    int64_t res;
    if (ALU_sig==AND)   res = data1&data2;
    else if (ALU_sig==OR)   res = data1|data2;
    else if (ALU_sig==ADD)  res = data1+data2;
    else if (ALU_sig==MUL)  res = data1*data2;
    else if (ALU_sig==XOR)  res = data1^data2;
    else if (ALU_sig==SUB)  res = data1-data2;
    else if (ALU_sig==TST0) res = !data2;   
    else if (ALU_sig==MOV)  res = data2;
    else if (ALU_sig==LSL)  res = data1<<data2;
    else if (ALU_sig==LSR)  res = data1>>data2;
    return res;
}

// sign extension unit representation
int64_t signext(int imm, int len) {
    int64_t sign = (int64_t)imm;
    sign >>= len-1;
    sign = (~sign)+1;
    return (sign<<len) | (int64_t)imm;
}

void pipe_init() {
    memset(&CURRENT_STATE, 0, sizeof(CPU_State));
    CURRENT_STATE.PC = 0x00400000;
    DO_IF = 0x1;
    DO_ID = 0x0;
    DO_EX = 0x0;
    DO_MEM = 0x0;
    DO_WB = 0x0;
    IC_STALL = 0x0;
    IC_STALL_CYCLE = 0;
    DC_STALL = 0x0;
    DC_STALL_CYCLE = 0;
	DC_WRITE = 0x0;
    
    bp_init();
    
    icache = cache_new(6,4,5);  // 2^6=64 sets, 4-way, 2^5=32B blocks
    dcache = cache_new(8,8,5);  // 2^8=256 sets, 8-way, 2^5=32B blocks
    assert(icache && dcache);
}

void pipe_cycle()
{
    printf("Cycle %d\n",stat_cycles+1);
    
    CYCLE_INIT_DC_STALL = DC_STALL;
    // printf("pipe:begin cycle IC_STALL:%d\n",IC_STALL);
    // printf("pipe:begin cycle IC_STALL_CYCLE:%d\n",IC_STALL_CYCLE);
    // printf("pipe:begin cycle DC_STALL:%d\n",DC_STALL);
    // printf("pipe:begin cycle DC_STALL_CYCLE:%d\n",DC_STALL_CYCLE);
    // printf("pipe:begin cycle DC_RUN_CYCLE:%d\n",DC_RUN_CYCLE);

    if (IC_STALL_CYCLE==10) {
        DO_IF = 0x1;
    }
    if (DC_STALL_CYCLE==2) {    // implement dcache stall
        DO_WB = 0x0;
        DO_MEM = 0x0;
        DO_EX = 0x0;
        DO_ID = 0x0;
        DO_IF = 0x0;
    }
	if (DC_STALL_CYCLE==10) {DO_MEM = 0x1;}


    printf("        pipe:apparent WB instr:0x%x\n",MWB.instr);
    if (DO_WB) {
        printf("%s\n\n","        do WB"); // DB
        pipe_stage_wb();
    } 

    printf("        pipe:apparent MEM instr:0x%x\n",EXM.instr);
    if (DO_MEM) {        
        printf("%s\n\n","        do MEM"); // DB
        pipe_stage_mem();
    }

    printf("        pipe:apparent EX instr:0x%x\n",IDEX.instr);
    if (DO_EX) {        
        printf("%s\n\n","        do EX"); // DB
        pipe_stage_execute();
    }

    printf("        pipe:apparent ID instr:0x%x\n",IFID.instr);
    if (DO_ID) {        
        printf("%s\n\n","        do ID"); // DB
        pipe_stage_decode();
    }
    
    // printf("pipe:pre-IF IC_STALL:%d\n",IC_STALL);
    // printf("pipe:pre-IF IC_STALL_CYCLE:%d\n",IC_STALL_CYCLE);
    // printf("pipe:pre-IF DC_STALL:%d\n",DC_STALL);
    // printf("pipe:pre-IF DC_STALL_CYCLE:%d\n",DC_STALL_CYCLE);
    // printf("pipe:pre-IF cycle DC_RUN_CYCLE:%d\n",DC_RUN_CYCLE);

    if (DO_IF) {        
        printf("%s\n\n","do IF"); // DB
        pipe_stage_fetch();
    }

    if (IC_STALL) 	{IC_STALL_CYCLE++;} 
	else 			{IC_RUN_CYCLE++;}
	if (DC_STALL) 	{DC_STALL_CYCLE++;}
	else 			{DC_RUN_CYCLE++;}

    // printf("pipe:end cycle IC_STALL:%d\n",IC_STALL);
    // printf("pipe:end cycle IC_STALL_CYCLE:%d\n",IC_STALL_CYCLE);
    // printf("pipe:end cycle DC_STALL:%d\n",DC_STALL);
    // printf("pipe:end cycle DC_STALL_CYCLE:%d\n",DC_STALL_CYCLE);
    // printf("pipe:end cycle DC_RUN_CYCLE:%d\n",DC_RUN_CYCLE);
    
    printf("Next PC to fetch:0x%lx\n",CURRENT_STATE.PC);
    printf("%s\n","***************************");

}

void pipe_stage_wb() {
    //printf("WB instr:0x%x\n",MWB.instr);       // DB
    // select data to potentially write back
    int64_t mem_res = mux(getbit(MWB.control,MEMTOREG),
                MWB.mem_data,
                MWB.ALU_res);
    
    if (getbit(MWB.control,MEM_R) && !DO_EX && IDEX.control) {
        printf("%s","load-use unstall\n");
        DO_EX = 0x1;
        DO_ID = 0x1;
        DO_IF = 0x1;
    }

    if (DC_RUN_CYCLE==2) {
        if (DCMISS_ICMISS_DIFF<2) {DO_MEM = 0x1;}
        if (DCMISS_ICMISS_DIFF<1) {DO_EX = 0x1;}
        DO_ID = 0x1;
        DO_IF = 0x1;
    } 

    // forwarding
    if ((MWB.rdest!=31) && getbit(MWB.control,REG_W)) {
        if (MWB.rdest==IDEX.rsrc1) 
            IDEX.rsrc1_val = mem_res;
        if (MWB.rdest==IDEX.rsrc2)
            IDEX.rsrc2_val = mem_res;
    }
    IDEX.N = MWB.newN;
    IDEX.Z = MWB.newZ;

    // write back to regfile, flags
    CURRENT_STATE.FLAG_N = MWB.newN;
    CURRENT_STATE.FLAG_Z = MWB.newZ;
    if (getbit(MWB.control,REG_W))
        CURRENT_STATE.REGS[MWB.rdest] = mem_res;
    
    CURRENT_STATE.REGS[31] ^= CURRENT_STATE.REGS[31];
    if (getbit(MWB.control,HLT))
        RUN_BIT = 0x0;
    
    ++stat_inst_retire;
    //print("The following got retired this time %d \n"+);
    DO_WB = 0x0; // avoid same instr WB twice in case of stall

    if (getbit(MWB.control,HLT)) {
        cache_destroy(icache);
        cache_destroy(dcache);
    }
}

void pipe_stage_mem() {
    printf("pipe:DC_RUN_CYCLE:%d\n",DC_RUN_CYCLE);
    printf("pipe:EXM_bak.ALU_res:0x%lx\n",EXM_bak.ALU_res);
    printf("pipe:EXM_bak.instr:0x%x\n",EXM_bak.instr);
    uint64_t addr, rdata, wdata;

    uint8_t rdest = EXM.rdest;
    int64_t ALU_res = EXM.ALU_res;
    printf("MEM accepted ALU_res (potentially mem acc addr):0x%lx\n",ALU_res);
    int64_t rsrc2_val = EXM.rsrc2_val;
    uint8_t newN = EXM.newN;
    uint8_t newZ = EXM.newZ;
    uint32_t control = EXM.control;
    uint64_t instr = EXM.instr;

    if (DC_RUN_CYCLE<=1) {
        // restore pre-stall EXM during stall and 1st cycle after stall (do MEM only)
        rdest = EXM_bak.rdest;
        ALU_res = EXM_bak.ALU_res;
        rsrc2_val = EXM_bak.rsrc2_val;
        newN = EXM_bak.newN;
        newZ = EXM_bak.newZ;
        control = EXM_bak.control;
        instr = EXM_bak.instr;
    }

    uint8_t is_condb = getbit(EXM.control,CONDB);
    uint8_t is_uncondb = getbit(EXM.control,UNCONDB);

    uint8_t is_ld = getbit(control,MEM_R);
    uint8_t is_st = getbit(control,MEM_W);
    uint8_t size;
    uint32_t memword;


    //printf("pipe:MEM instr:0x%x\n",EXM.instr);       // DB
    // forwarding 
    // shadows WB forwarding due to later position in simulated cycle
    if ((EXM.rdest!=31) && getbit(EXM.control,REG_W)) {
        if (EXM.rdest==IDEX.rsrc1) 
            IDEX.rsrc1_val = EXM.ALU_res;
        if (EXM.rdest==IDEX.rsrc2)
            IDEX.rsrc2_val = EXM.ALU_res;
    }
        IDEX.N = EXM.newN;
        IDEX.Z = EXM.newZ;
    
    // unstall if branch  
	if (is_condb||is_uncondb) {
		uint8_t btaken = EXM.btaken;
    	//printf("%s","branch unstall\n");
        if ((!getbit(EXM.control,BTBHIT) && !(is_condb&&!btaken)) 
        || 
        (EXM.predPC!=EXM.bPC)){
            IDEX.control = 0x0;
            EXM.control = 0x0;
            DO_IF = 0x1;    /* enable branch target fetch
                  in same cycle */  
            DO_ID = 0x0;
            DO_EX = 0x0;
        } else {
            DO_IF = 0x1;    // enable branch+8 fetch
            DO_ID = 0x1;    // enable branch+4 fetch
            DO_EX = 0x1;
        }
    }
    if (is_ld||is_st) {
        int dcache_hit;
	    if (!DC_STALL) {
            addr = ALU_res;	
		    dcache_hit = cache_update(dcache,addr,ACTN_POSS_STALL);
        	printf("pipe:mem:dcache_hit:%d\n",dcache_hit);   // DB
		    if (!dcache_hit) {	                   // MISS
			    printf("%s\n","      pipe:mem:dcache miss");
                printf("      pipe:mem:IC_STALL_CYCLE:%d\n",IC_STALL_CYCLE);
                DCMISS_ICMISS_DIFF = IC_STALL_CYCLE-1;
			    DC_STALL = 0x1;
				DC_STALL_CYCLE = 1;
				DC_RUN_CYCLE = 0;
                // backup EXM
                EXM_bak.rdest = rdest;
                EXM_bak.ALU_res = ALU_res;
                EXM_bak.rsrc2_val = rsrc2_val;
                EXM_bak.newN = newN;
                EXM_bak.newZ = newZ;
                EXM_bak.control = control;
                printf("pipe:mem:instr to backup:0x%lx\n",instr);
                EXM_bak.instr = instr;
		    } else {		                        // HIT
                printf("pipe:dcache hit mem acc addr:0x%lx\n",addr);
			 	memword = cread_32(addr,dcache);
			    if (is_ld) {
				    if ((rdest==IDEX.rsrc1) ||
				    ((!getbit(IDEX.control,MEM_R))&&(rdest==IDEX.rsrc2))) {
				    // necessary stall;
				    // older load 20:16 unused as rsrc2, unaffected 
					    printf("%s","load-use\n");          // DB
					    printf("EXM.rdest:X%d,IDEX.rsrc1:X%d,IDEX.rsrc2:X%d\n",
					    rdest,IDEX.rsrc1,IDEX.rsrc2);   //DB
					    DO_EX = 0x0;
					    DO_ID = 0x0;
					    DO_IF = 0x0;
				    }
                    if (size==0x3) {        // ldur
                        rdata = cread_32(addr+4,dcache);
                        rdata <<= 32;
                        rdata |= memword;
                    } else if (size==0x0) { // ldurb
                        rdata = memword & mask(8);
                    } else if (size==0x1) { // ldurh
                        rdata = memword & mask(16);
                    } 
                } else if (is_st) {
                    printf("pipe:st rsrc2_val:0x%lx\n",rsrc2_val); // DB
                    printf("pipe:st mem dest addr:0x%lx\n",addr); // DB
                    wdata = rsrc2_val; 
                    if (size==0x3) {        // stur
                        cwrite_32(addr,(uint32_t)(wdata),dcache);
                        cwrite_32(addr+4,(uint32_t)(wdata>>32),dcache);
                        mem_write_32(addr,(uint32_t)wdata); // write thru
                        mem_write_32(addr+4,(uint32_t)(wdata>>32));
                    } else if (size==0x0) { // sturb
                        wdata &= mask_64(8);
                        memword &= ~mask(8);
                        memword |= (uint32_t)wdata;
                        cwrite_32(addr,memword,dcache);
                        mem_write_32(addr,memword);
                    } else if (size==0x1) { // sturh
                        wdata &= mask_64(16);
                        memword &= ~mask(16);
                        memword |= (uint32_t)wdata;
                        cwrite_32(addr,memword,dcache);
                        mem_write_32(addr,memword);
                    }
					DC_WRITE = 0x1;
                }
                DO_MEM = 0x0;   // avoid same instr MEM twice
                DO_WB = 0x1;    // enable same instr WB
		    }
	    // during stall, do nothing
	    } else if (DC_STALL_CYCLE!=10){    ;
        } else { // cache ready
            addr = EXM_bak.ALU_res;
            printf("pipe:last stall cycle recovered mem acc addr:0x%lx\n",addr);    // DB
		    cache_update(dcache,addr,ACTN_UNSTALL);
		    DC_STALL = 0x0;
		    DC_STALL_CYCLE = 0;
	    }
    } else { // non-mem
        DO_MEM = 0x0;   // avoid same instr MEM twice
        DO_WB = 0x1;    // enable same instr WB
    }

    printf("    pipe:DC_STALL_CYCLE in MEM:%d\n",DC_STALL_CYCLE);
    printf("    pipe: MEM local \"instr\":0x%lx\n",instr);
    if (DC_STALL_CYCLE!=10) {
        MWB.mem_data = rdata;
        MWB.control = control & mask(8);
        MWB.rdest = rdest;
        MWB.ALU_res = ALU_res;
        MWB.newN = newN;
        MWB.newZ = newZ;
        MWB.instr = instr;          // DB
    }
}

void pipe_stage_execute() {
    uint8_t newN, newZ;
    uint8_t setflgs = getbit(IDEX.control,SETFLAGS);
    
    //printf("EX instr:0x%x\n",IDEX.instr);       // DB

    if (getbit(IDEX.control,HLT)) {
        DO_IF = 0x0;
        DO_ID = 0x0;
    }
    
    // ALU computation
    int64_t ALU_res;
    int64_t data1 = mux(getbit(IDEX.control,ALUSRC1),
                    IDEX.instrPC,
                    IDEX.rsrc1_val);
    int64_t data2 = mux(getbit(IDEX.control,ALUSRC2),
                    IDEX.imm,
                    IDEX.rsrc2_val);
    ALU_res = ALU(IDEX.ALU_sig,data1,data2);
    if (getbit(IDEX.control,CBTYPE))    // CBNZ
        ALU_res = !ALU_res;
    if (setflgs) flagset(ALU_res,&newN,&newZ);


    // branch stall & address computation 
    uint64_t bPC;
    uint8_t bres, btaken;
    uint8_t hit = getbit(IDEX.control,BTBHIT);
    uint8_t is_condb = getbit(IDEX.control,CONDB);
    uint8_t is_uncondb = getbit(IDEX.control,UNCONDB);
    uint8_t is_b = is_condb || is_uncondb;
    if (is_b) {
        //printf("%s","branch stall\n");                //DB 
	// branch target resolution
        bPC = mux(getbit(IDEX.control,PCSRC),
                    IDEX.rsrc1_val, 
                    (uint64_t)add(IDEX.instrPC,IDEX.imm));
    }
    // cond branch resolution
    if (getbit(IDEX.control,CONDB)) {
        bres = mux(getbit(IDEX.control,BRESSRC),
                ALU_res,
                flagtest(IDEX.bcond,IDEX.N,IDEX.Z));
    }

    // change PC if branch taken
    btaken = (is_uncondb || (is_condb&&bres));
    uint64_t save_pc = CURRENT_STATE.PC;
    uint8_t not_taken = !btaken;
    if (btaken) {
        printf("  Resolved bPC:%lx\n",bPC);           // DB
        printf("  C_S.PC:      %lx\n",CURRENT_STATE.PC);       // DB
        printf("  IFID.instrPC:%lx\n",IFID.instrPC);        // DB
        printf("  IDEX.instrPC:%lx\n",IDEX.instrPC);        // DB
        CURRENT_STATE.PC = bPC;
        int tell_me = 0;
        if ((bPC==IFID.instrPC) && hit){
            CURRENT_STATE.PC = save_pc;
            tell_me = 1;
        }

        if ((bPC==IFID.instrPC) &&
            (IDEX.instrPC+4==CURRENT_STATE.PC)) {   
            printf("%s","pos1\n");  // DB
            DO_IF = 0x0;
        }
        if (tell_me){
            DO_IF = 0x1;
        }
    } else {    // not taken; "bPC" effectively current instrPC+4
        bPC = add(IDEX.instrPC,4);  
    }
    uint8_t into_pos2 = 0;
    if (is_b&&(bPC!=IDEX.predPC)) { // mispredicted
        printf("%s","pos2\n");
        DO_ID = 0x0;
        DO_IF = 0x0;    // stall younger instrs in same cycle
        into_pos2 = 1;
    }
    if(hit && into_pos2 && not_taken){
        CURRENT_STATE.PC = IDEX.instrPC+4; //try
    }

    // BP update
    if (is_b) {
        bp_update(IDEX.instrPC,bPC,is_condb,btaken);
    }
	
	// TODO compare
	if (IC_STALL) {
        printf("%s","branch resln during icache stall\n"); // DB
        printf("CURRENT_STATE.PC:0x%lx\n",CURRENT_STATE.PC);
        printf("IFID.instrPC:0x%lx\n",IFID.instrPC);
		if ((get_sidx(CURRENT_STATE.PC,icache) ^
				get_sidx(IFID.instrPC,icache)) || 
			(get_tag(CURRENT_STATE.PC,icache) ^
			 	get_tag(IFID.instrPC,icache))) { 
			// branch taken, target PC not in same block as 
			// pending fetch PC
            printf("%s\n","PC invalid");    // DB
		}
	}	

	//After a branch is fully resoved in the EX stage, 
	//the pipeline is flushed under any of the following conditions:
	//The instruction is a branch, but the predicted target destination does not match the actual target.
	//The instruction is a branch, but it was not recognized as a branch (i.e., BTB miss).

    EXM.ALU_res = ALU_res;
    EXM.instrPC = IDEX.instrPC;
    EXM.predPC = IDEX.predPC;
    EXM.bres = bres;
    EXM.btaken = btaken;
    EXM.bPC = bPC;
    EXM.rdest = IDEX.rdest;
    EXM.rsrc2_val = IDEX.rsrc2_val;
    EXM.newN = mux(setflgs,newN,IDEX.N);
    EXM.newZ = mux(setflgs,newZ,IDEX.Z);
    EXM.control = IDEX.control & mask(16);
    DO_EX = 0x0;
    DO_MEM = 0x1;

    EXM.instr = IDEX.instr;         // DB
}

void pipe_stage_decode() {
    uint32_t instr = IFID.instr;
    uint32_t control = 0x0;
    uint8_t rsrc1 = getbits(instr,9,5);
    uint8_t rdest = getbits(instr,4,0);
    uint8_t rsrc2;
    uint8_t op0 = (uint8_t)getbits(instr,28,25);
    uint8_t op,opc,S;
    uint8_t op1,op2,op3,op4,op5;

    //printf("ID instr:0x%x\n",instr);
    
    if (!IFID.missed) {
        setbit(&control,BTBHIT);
    }

    if ((op0>>1)==0x4) {        // data proc imm
        setbit(&control,ALUSRC2);
        setbit(&control,REG_W);
        
        op0 = getbits(instr,25,23);
        if ((op0>>1)==0x1) {    // add/sub imm
            op = getbit(instr,30);
            IDEX.ALU_sig = op ? SUB : ADD;
            S = getbit(instr,29);
            if (S) setbit(&control,SETFLAGS);
            IDEX.imm = getbits(instr,21,10);
        } else if (op0==0x5) {          // mov
            IDEX.ALU_sig = MOV;
            IDEX.imm = getbits(instr,20,5);
        } else if (op0==0x6) {          // bitfield (lsl/r)
            int8_t imms = getbits(instr,15,10);
            int8_t immr = getbits(instr,21,16);
            if (imms==0x3f) {
                IDEX.ALU_sig = LSR;
                IDEX.imm = immr;
            } else if (imms+1==immr) {
                IDEX.ALU_sig = LSL;
                IDEX.imm = (~imms)&mask(6);
            }
        }

    } else if ((op0>>1)==0x5) {// branch,etc.
        op0 = getbits(instr,31,29);
        op1 = getbits(instr,25,22);
        if (op0==0x6) { 
            if (op1<0x4) {          // excgen (hlt)
                setbit(&control,HLT);
            } else if (op1>=0x8) {      // br
                setbit(&control,UNCONDB);
                setbit(&control,PCSRC); // Rn
            }
        } else {    // PC+offset
            if ((op0==0x2)&&(op1<0x8)) {    // b.cond
                setbit(&control,CONDB);
                IDEX.bcond = getbits(instr,3,0);
                IDEX.imm = signext(getbits(instr,23,5),19)<<2;
            } else if ((op0&mask(2))==0x0) {// b 
                setbit(&control,UNCONDB);
                IDEX.imm = signext(getbits(instr,25,0),26)<<2;
            } else if (((op0&mask(2))==0x1)&&(op1<0x8)) {   // cb
                setbit(&control,CONDB);
                setbit(&control,BRESSRC);// ALU==0?
                setbit(&control,REG2LOC);
                op = getbit(instr,24);
                if (op)         //CBNZ
                    setbit(&control,CBTYPE); 
                IDEX.ALU_sig = TST0;
                IDEX.imm = signext(getbits(instr,23,5),19)<<2;
            }
        }
    } else if (getbit(op0,0)==0x0&&getbit(op0,2)==0x1) {    // ld/st
        IDEX.imm = getbits(instr,20,12);
        setbit(&control,ALUSRC2);
        IDEX.ALU_sig = ADD;

        op1 = getbits(instr,29,28);
        op3 = getbits(instr,24,23);
        op4 = getbits(instr,21,16);
        op5 = getbits(instr,11,10);
        if ((op1==0x3)&&(op3<0x2)&&
                    (op4<0x8)&&(op5==0x0)) {    // ld/stur

            opc = getbits(instr,23,22);
            if (opc==0x0) {         // st 
                setbit(&control,MEM_W);
                setbit(&control,REG2LOC);
            } else if (opc==0x1) {  // ld
                setbit(&control,MEM_R);
                setbit(&control,MEMTOREG);
                setbit(&control,REG_W);
            }

            // size bits
            if (getbit(instr,30)) setbit(&control,MEMSIZE1);
            if (getbit(instr,31)) setbit(&control,MEMSIZE2);
        }
    } else if ((op0&mask(3))==0x5) {// data proc reg
        setbit(&control,REG_W);
        op0 = getbit(instr,30);
        op1 = getbit(instr,28);
        op2 = getbits(instr,24,21);
        op3 = getbit(instr,11);
        if (op1==0x0) {
            if (op2<0x8) {              // logical sr
                opc = getbits(instr,30,29);
                if (opc==0x1) IDEX.ALU_sig = OR;
                else if (opc==0x2) IDEX.ALU_sig = XOR;
                else IDEX.ALU_sig = AND;
                if (opc==0x3) setbit(&control,SETFLAGS);
                
            } else if (getbit(op2,3)==0x1) {    // add/sub er
                op = getbit(instr,30);
                IDEX.ALU_sig = op ? SUB : ADD;
                S = getbit(instr,29);
                if (S) setbit(&control,SETFLAGS);
                IDEX.imm = getbits(instr,21,10);
            }
        } else if ((op1==0x1)&&(op2>=0x8)) {    // 3-src (madd->mul)
            IDEX.ALU_sig = MUL;
        }
    }

    IDEX.rsrc1 = rsrc1;
    IDEX.rsrc1_val = CURRENT_STATE.REGS[rsrc1];
    rsrc2 = mux(getbit(control,REG2LOC),
                rdest,
                getbits(instr,20,16));
    IDEX.rsrc2 = rsrc2;
    IDEX.rsrc2_val = CURRENT_STATE.REGS[rsrc2];
    IDEX.rdest = rdest;
    IDEX.instrPC = IFID.instrPC;
    IDEX.predPC = IFID.predPC;
    IDEX.N = IFID.N;
    IDEX.Z = IFID.Z;
    IDEX.control = control;
    DO_ID = 0x0;
    DO_EX = 0x1;

    //pass the bPC's need (missed)
    IDEX.missed = IFID.missed;

    IDEX.instr = instr;     // DB
}

void pipe_stage_fetch() {
	int icache_hit;
	printf("pipe:DC_STALL in IF:%d\n",DC_STALL);
	icache_hit = cache_update(icache,CURRENT_STATE.PC,ACTN_POSS_STALL);
    printf("        pipe:icache_hit:%d\n",icache_hit);   // DB
    if (!icache_hit) {	// miss
		if (!IC_STALL) {
			IC_STALL = 0x1;
			IC_STALL_CYCLE = 1;
			IC_RUN_CYCLE = 0;
		}
		IFID.instrPC = CURRENT_STATE.PC;
	} else if (!CYCLE_INIT_DC_STALL) {	
        // hit & dcache not in stall before current cycle
        // should fetch
        //printf("%s\n","        pipe:icache hit");
		uint32_t instr = cread_32(CURRENT_STATE.PC,icache);
		IFID.instr = instr;
		IFID.instrPC = CURRENT_STATE.PC;
		IFID.N = CURRENT_STATE.FLAG_N;
		IFID.Z = CURRENT_STATE.FLAG_Z;

		uint8_t missed = 0;
		uint64_t predPC;
			
		printf("        pipe:fetched IF instr:0x%x\n",instr);       // DB
			
		//predict next PC & update CURRENT_STATE.PC
		bp_predict(&predPC,&missed);
			
		//put the missed information here
		IFID.missed = missed;
			
    	IFID.predPC = predPC;
		++stat_inst_fetch;
		DO_ID = 0x1; // enable decoding of fetched instr
	}
	if (IC_STALL_CYCLE==10) { // earlier cache miss now resolved
		cache_update(icache,CURRENT_STATE.PC,ACTN_UNSTALL);
		IC_STALL = 0x0;
		IC_STALL_CYCLE = 0;
    }
}

