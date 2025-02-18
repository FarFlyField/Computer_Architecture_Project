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

#ifndef _PIPE_H_
#define _PIPE_H_

#include "shell.h"
#include "stdbool.h"
#include <limits.h>

// main ctrl bits
// 7:0 passed to EX,MEM,WB
// 15:8 passed to EX,MEM
// 23:16 passed to EX only
#define REG_W		0		
#define MEMTOREG    1		// 0-ALU; 1-Mem
#define HLT			2
#define MEM_R		3
#define SETFLAGS	4

#define MEM_W		8
#define MEMSIZE1	9
#define MEMSIZE2	10
#define B			11
#define UNCONDB		12
#define BTBHIT		13

#define REG2LOC		16      // 0-20:16; 1-4:0(ST)
#define ALUSRC1		17		// 0-rsrc1; 1-PC
#define ALUSRC2		18		// 0-rsrc2; 1-imm
#define PCSRC		19		// 0-PC+imm; 1-rsrc1(BR)
#define CBTYPE		20		// 0-CBZ; 1-CBNZ
#define BRESSRC		21		// 0-flags; 1-ALU==0?(CB)

// ALU signal values
#define AND		0x0
#define OR		0x1
#define ADD		0x2
#define MUL		0x3
#define XOR		0x4
#define SUB		0x6
#define TST0	0x7
#define MOV		0x8
#define LSL		0x9
#define LSR		0xa

typedef struct CPU_State {
	/* register file state */
	int64_t REGS[ARM_REGS];
	int FLAG_N;        /* flag N */
	int FLAG_Z;        /* flag Z */
	/* program counter in fetch stage */
	uint64_t PC;
	
} CPU_State;

typedef struct pipereg_IFID {
	uint64_t instrPC, predPC;
	uint32_t instr;
	uint8_t N, Z;

	//for bpc
	uint8_t missed;

	
} pipereg_IFID;

typedef struct pipereg_IDEX {
	uint64_t instrPC, predPC;
	uint8_t N, Z;
	uint8_t rsrc1, rsrc2, rdest;
	int64_t rsrc1_val, rsrc2_val;
	int64_t imm;
	uint32_t control;
	uint8_t ALU_sig;
	uint8_t bcond;

	//for bpc
	uint8_t missed;

	uint32_t instr;		// DB

} pipereg_IDEX;

typedef struct pipereg_EXM {
	uint64_t instrPC, bPC, predPC;
	uint8_t rdest;
	int64_t ALU_res;
	int64_t rsrc2_val;
	uint8_t newN, newZ;
	uint8_t bres;
	uint8_t btaken;
	uint32_t control;

	//for bpc

	uint32_t instr;		// DB

} pipereg_EXM;

typedef struct pipereg_MWB {
	uint8_t rdest;
	int64_t ALU_res;
	uint8_t newN, newZ;
	uint64_t mem_data;
	uint32_t control;

	//for bpc

	uint32_t instr;		// DB

} pipereg_MWB;

int RUN_BIT;
uint8_t DO_IF, DO_ID, DO_EX, DO_MEM, DO_WB;
uint8_t FWDA, FWDB;
int64_t MEM_RES;

/* global variable -- pipeline state */
extern CPU_State CURRENT_STATE;

/* called during simulator startup */
void pipe_init();

/* this function calls the others */
void pipe_cycle();

/* each of these functions implements one stage of the pipeline */
void pipe_stage_fetch();
void pipe_stage_decode();
void pipe_stage_execute();
void pipe_stage_mem();
void pipe_stage_wb();

#endif
