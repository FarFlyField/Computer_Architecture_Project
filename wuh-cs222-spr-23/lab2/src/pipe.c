/*
 * Contributors:
 * Wang,Shaoge (shaogew)
 * Wu,Haoran (wuh)
 *
 */

/*
 * CMSC 22200
 *
 * ARM pipeline timing simulator
 */

#include "pipe.h"
#include "shell.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>


/* global pipeline state */
CPU_State CURRENT_STATE;
pipereg_IFID IFID;
pipereg_IDEX IDEX;
pipereg_EXM EXM;
pipereg_MWB MWB;

// creates bitmask of specified len
uint32_t mask(int len) {
	return ~(~0x0<<len);
}

// get bit with specified idx
uint8_t getbit(uint32_t i, int idx) {
	return (i>>idx) & mask(1);
}

// gets inclusive bits from hi to lo
uint32_t getbits(uint32_t i, int hi, int lo) {
	return (i>>lo) & mask(hi-lo+1);
}

// set bit 
void setbit(uint32_t *i, int b) {
	*i |= (0x1<<b);
}


// set N and Z flags
void flagset(int64_t alu_res, uint8_t *newN, uint8_t *newZ) {
	*newN = alu_res>>63 ? 0x1 : 0x0;
	*newZ = alu_res ? 0x0 : 0x1;
}

uint8_t flagtest(uint8_t cond, uint8_t N, uint8_t Z) {
	uint8_t C = 0x0;
	uint8_t V = 0x0;
	if (cond==0x0) 		return Z;	// eq
	else if (cond==0x1)	return !Z;	// ne
	else if (cond==0xa)	return N==V;	// ge
	else if (cond==0xb)	return N!=V;	// lt
	else if (cond==0xc)	return (!Z)&&(N==V);	// gt
	else if (cond==0xd)	return Z||(N!=V);	// le
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
int64_t alu(uint8_t alu_sig, int64_t data1, int64_t data2) 
{
	int64_t res;
	if (alu_sig==AND)	res = data1&data2;
	else if (alu_sig==OR)	res = data1|data2;
	else if (alu_sig==ADD)	res = data1+data2;
	else if (alu_sig==MUL)	res = data1*data2;
	else if (alu_sig==XOR)	res = data1^data2;
	else if (alu_sig==SUB)	res = data1-data2;
	else if (alu_sig==TST0)	res = !data2;	
	else if (alu_sig==MOV)	res = data2;
	else if (alu_sig==LSL)	res = data1<<data2;
	else if (alu_sig==LSR)	res = data1>>data2;
	return res;
}

// sign extension unit representation
int64_t signext(int imm, int len) {
	int64_t sign = (int64_t)imm;
	sign >>= len-1;
	sign = (~sign)+1;
	return (sign<<len) | (int64_t)imm;
}

void fwdset(uint8_t *FWDSIG, uint8_t x_rsrc, 
			uint8_t m_rdest, uint8_t m_regw,
			uint8_t w_rdest, uint8_t w_regw) {
		*FWDSIG = 0x0;
		if ((x_rsrc==w_rdest) &&
			(w_rdest!=31) &&
			w_regw)
			*FWDSIG = 0x1;
		if ((x_rsrc==m_rdest) &&
			(m_rdest!=31) &&
			m_regw)
			*FWDSIG = 0x2;
}

void pipe_init() {
	memset(&CURRENT_STATE, 0, sizeof(CPU_State));
	CURRENT_STATE.PC = 0x00400000;
	DO_IF = 0x1;
	DO_ID = 0x0;
	DO_EX = 0x0;
	DO_MEM = 0x0;
	DO_WB = 0x0;
}

void pipe_cycle()
{
	if (DO_WB) pipe_stage_wb(); 
	if (DO_MEM) pipe_stage_mem();
	if (DO_EX) pipe_stage_execute();
	if (DO_ID) pipe_stage_decode();
	if (DO_IF) pipe_stage_fetch();
}

void pipe_stage_wb() {
	// select data to potentially write back
	MEM_RES = mux(getbit(MWB.control,MEMTOREG),
				MWB.mem_data,
				MWB.alu_res);
	
	if (getbit(MWB.control,MEM_R) && !DO_EX && IDEX.control) {
		printf("%s","load-use unstall\n");
		DO_EX = 0x1;
		DO_ID = 0x1;
		DO_IF = 0x1;
	}

	// forwarding
	if ((MWB.rdest!=31) && getbit(MWB.control,REG_W)) {
		if (MWB.rdest==IDEX.rsrc1) 
			IDEX.rsrc1_val = MEM_RES;
		if (MWB.rdest==IDEX.rsrc2)
			IDEX.rsrc2_val = MEM_RES;
	}
	IDEX.N = MWB.newN;
	IDEX.Z = MWB.newZ;

	// write back to regfile, flags
	CURRENT_STATE.FLAG_N = MWB.newN;
	CURRENT_STATE.FLAG_Z = MWB.newZ;
	if (getbit(MWB.control,REG_W))
		CURRENT_STATE.REGS[MWB.rdest] = MEM_RES;
	
	CURRENT_STATE.REGS[31] ^= CURRENT_STATE.REGS[31];
	if (getbit(MWB.control,HLT))
		RUN_BIT = 0x0;
	
	++stat_inst_retire;
	//print("The following got retired this time %d \n"+);
	DO_WB = 0x0; // avoid same instr WB twice in case of stall
}

void pipe_stage_mem() {
	uint64_t addr, rdata, wdata;
	uint8_t ld = getbit(EXM.control,MEM_R);
	uint8_t st = getbit(EXM.control,MEM_W);
	uint8_t size;
	uint32_t memword;

	// forwarding 
	// shadows WB forwarding due to later position in cycle
	if ((EXM.rdest!=31) && getbit(EXM.control,REG_W)) {
		if (EXM.rdest==IDEX.rsrc1) 
			IDEX.rsrc1_val = EXM.alu_res;
		if (EXM.rdest==IDEX.rsrc2)
			IDEX.rsrc2_val = EXM.alu_res;
	}
		IDEX.N = EXM.newN;
		IDEX.Z = EXM.newZ;
	
	// unstall if branch	
	if (getbit(EXM.control,B) || getbit(EXM.control,UNCONDB)) {
		//printf("%s","branch unstall\n");
		if (EXM.btaken){
			IDEX.control = 0x0;
			EXM.control = 0x0;
			DO_IF = 0x1;	/* enable branch target fetch
				  in same cycle */	
		} else {
			DO_IF = 0x1;	// enable branch+8 fetch
			DO_ID = 0x1;	// enable branch+4 fetch
		}
		DO_EX = 0x0;
	}

	if (ld|st) {
		addr = EXM.alu_res;
		size = getbits(EXM.control,MEMSIZE2,MEMSIZE1);
		memword = mem_read_32(addr);
	}
	if (ld) {
		if ((EXM.rdest==IDEX.rsrc1) ||
			((!getbit(IDEX.control,MEM_R))&&(EXM.rdest==IDEX.rsrc2))) {
			// necessary stall;
			// older load 20:16 unused as rsrc2, unaffected 
			printf("%s","load-use\n");			// DB
			printf("EXM.rdest:X%d,IDEX.rsrc1:X%d,IDEX.rsrc2:X%d\n",
				EXM.rdest,IDEX.rsrc1,IDEX.rsrc2);	//DB
			DO_EX = 0x0;
			DO_ID = 0x0;
			DO_IF = 0x0;
		}
		if (size==0x3) {		// ldur
			rdata = mem_read_32(addr+4);
			rdata <<= 32;
			rdata |= memword;
		} else if (size==0x0) {	// ldurb
			rdata = memword & mask(8);
		} else if (size==0x1) {	// ldurh
			rdata = memword & mask(16);
		}	 
	} else if (st) {
		wdata = EXM.rsrc2_val; 
		if (size==0x3) {		// stur
			mem_write_32(addr,(uint32_t)(wdata));
			mem_write_32(addr+4,(uint32_t)(wdata>>32));
		} else if (size==0x0) {	// sturb
			wdata &= mask(8);
			memword &= ~mask(8);
			memword |= (uint32_t)wdata;
			mem_write_32(addr,memword);
		} else if (size==0x1) {	// sturh
			wdata &= mask(16);
			memword &= ~mask(16);
			memword |= (uint32_t)wdata;
			mem_write_32(addr,memword);
		}
	}
	
	MWB.mem_data = rdata;	
	MWB.control = EXM.control & mask(8);
	MWB.rdest = EXM.rdest;
	MWB.alu_res = EXM.alu_res;
	MWB.newN = EXM.newN;
	MWB.newZ = EXM.newZ;
	DO_MEM = 0x0;	// avoid same instr MEM twice
	DO_WB = 0x1;	// enable same instr WB
}

void pipe_branch_taken_flush_IFID(){
	IFID.PC = 0;
	IFID.instr = 0;
	IFID.N = 0;
	IFID.Z = 0;
}

void pipe_stage_execute() {
	uint8_t newN, newZ;
	uint8_t setflgs = getbit(IDEX.control,SETFLAGS);
	
	if (getbit(IDEX.control,HLT)) 
		DO_IF = 0x0;
	
	// ALU computation
	int64_t alu_res;
	int64_t data1 = mux(getbit(IDEX.control,ALUSRC1),
					IDEX.PC,
					IDEX.rsrc1_val);
	int64_t data2 = mux(getbit(IDEX.control,ALUSRC2),
					IDEX.imm,
					IDEX.rsrc2_val);
	alu_res = alu(IDEX.alu_sig,data1,data2);
	if (getbit(IDEX.control,CBTYPE))	// CBNZ
		alu_res = !alu_res;
	EXM.alu_res = alu_res;
	if (setflgs) flagset(alu_res,&newN,&newZ);

	// branch stall & address computation 
	uint64_t bPC;
	uint8_t bres, btaken;
	if (getbit(IDEX.control,B) || getbit(IDEX.control,UNCONDB)) {
		//printf("%s","branch stall\n");				//DB 
		DO_ID = 0x0;
		DO_IF = 0x0;	// stall younger instrs in same cycle
		bPC = mux(getbit(IDEX.control,PCSRC),
					IDEX.rsrc1_val,	
					(uint64_t)add(IDEX.PC,IDEX.imm));
	}
	// cond branch resolution
	if (getbit(IDEX.control,B)) {
		bres = mux(getbit(IDEX.control,BRESSRC),
				alu_res,
				flagtest(IDEX.bcond,IDEX.N,IDEX.Z));
	}

	// change PC if branch taken
	btaken = (getbit(IDEX.control,UNCONDB) || 
	 				(getbit(IDEX.control,B) && bres))
					&&
					(bPC!=(IDEX.PC+4));

	if (bPC == IFID.PC){
		DO_IF = 0x1;
		CURRENT_STATE.PC = bPC;
	} else if (btaken) {
		CURRENT_STATE.PC = bPC;
		pipe_branch_taken_flush_IFID();
		++stat_squash;
	}
	
	EXM.bres = bres;
	EXM.btaken = btaken;
	EXM.rdest = IDEX.rdest;
	EXM.rsrc2_val = IDEX.rsrc2_val;
	EXM.newN = mux(setflgs,newN,IDEX.N);
	EXM.newZ = mux(setflgs,newZ,IDEX.Z);
	EXM.control = IDEX.control & mask(16);
	DO_EX = 0x0;
	DO_MEM = 0x1;
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

	if ((op0>>1)==0x4) { 		// data proc imm
		setbit(&control,ALUSRC2);
		setbit(&control,REG_W);
		
		op0 = getbits(instr,25,23);
		if ((op0>>1)==0x1) {	// add/sub imm
			op = getbit(instr,30);
			IDEX.alu_sig = op ? SUB : ADD;
			S = getbit(instr,29);
			if (S) setbit(&control,SETFLAGS);
			IDEX.imm = getbits(instr,21,10);
		} else if (op0==0x5) {			// mov
			IDEX.alu_sig = MOV;
			IDEX.imm = getbits(instr,20,5);
		} else if (op0==0x6) {			// bitfield (lsl/r)
			int8_t imms = getbits(instr,15,10);
			int8_t immr = getbits(instr,21,16);
			if (imms==0x3f) {
				IDEX.alu_sig = LSR;
				IDEX.imm = immr;
			} else if (imms+1==immr) {
				IDEX.alu_sig = LSL;
				IDEX.imm = (~imms)&mask(6);
			}
		}

	} else if ((op0>>1)==0x5) {// branch,etc.
		op0 = getbits(instr,31,29);
		op1 = getbits(instr,25,22);
		if (op0==0x6) {	
			if (op1<0x4) {			// excgen (hlt)
				setbit(&control,HLT);
			} else if (op1>=0x8) {		// br
				setbit(&control,UNCONDB);
				setbit(&control,PCSRC);	// Rn
			}
		} else {	// PC+offset
			if ((op0==0x2)&&(op1<0x8)) {	// b.cond
				setbit(&control,B);
				IDEX.bcond = getbits(instr,3,0);
				IDEX.imm = signext(getbits(instr,23,5),19)<<2;
			} else if ((op0&mask(2))==0x0) {// b 
				setbit(&control,UNCONDB);
				IDEX.imm = signext(getbits(instr,25,0),26)<<2;
			} else if (((op0&mask(2))==0x1)&&(op1<0x8)) {	// cb
				setbit(&control,B);
				setbit(&control,BRESSRC);// ALU==0?
				setbit(&control,REG2LOC);
				op = getbit(instr,24);
				if (op)  		//CBNZ
					setbit(&control,CBTYPE); 
				IDEX.alu_sig = TST0;
				IDEX.imm = signext(getbits(instr,23,5),19)<<2;
			}
		}
	} else if (getbit(op0,0)==0x0&&getbit(op0,2)==0x1) {	// ld/st
		IDEX.imm = getbits(instr,20,12);
		setbit(&control,ALUSRC2);
		IDEX.alu_sig = ADD;

		op1 = getbits(instr,29,28);
		op3 = getbits(instr,24,23);
		op4 = getbits(instr,21,16);
		op5 = getbits(instr,11,10);
		if ((op1==0x3)&&(op3<0x2)&&
					(op4<0x8)&&(op5==0x0)) {	// ld/stur

			opc = getbits(instr,23,22);
			if (opc==0x0) {			// st 
				setbit(&control,MEM_W);
				setbit(&control,REG2LOC);
			} else if (opc==0x1) {	// ld
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
			if (op2<0x8) {				// logical sr
				opc = getbits(instr,30,29);
				if (opc==0x1) IDEX.alu_sig = OR;
				else if (opc==0x2) IDEX.alu_sig = XOR;
				else IDEX.alu_sig = AND;
				if (opc==0x3) setbit(&control,SETFLAGS);
				
			} else if (getbit(op2,3)==0x1) {	// add/sub er
				op = getbit(instr,30);
				IDEX.alu_sig = op ? SUB : ADD;
				S = getbit(instr,29);
				if (S) setbit(&control,SETFLAGS);
				IDEX.imm = getbits(instr,21,10);
			}
		} else if ((op1==0x1)&&(op2>=0x8)) {	// 3-src (madd->mul)
			IDEX.alu_sig = MUL;
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
	IDEX.PC = IFID.PC;
	IDEX.N = IFID.N;
	IDEX.Z = IFID.Z;
	IDEX.control = control;
	DO_ID = 0x0;
	DO_EX = 0x1;
}

void pipe_stage_fetch() {
	IFID.instr = mem_read_32(CURRENT_STATE.PC);
	IFID.PC = CURRENT_STATE.PC;
	IFID.N = CURRENT_STATE.FLAG_N;
	IFID.Z = CURRENT_STATE.FLAG_Z;
	CURRENT_STATE.PC = add(CURRENT_STATE.PC,4);
	++stat_inst_fetch;
	DO_ID = 0x1;
}
