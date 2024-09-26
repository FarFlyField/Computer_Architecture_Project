#include <stdio.h>
#include <inttypes.h>
#include "shell.h"

#define REG_MASK 0x1f
#define ZR 31

enum instr_type {
	ADDSUB_IMM,MOV_IMM,BITFLD,
	BCOND,EXC,BR,B,CB,
	LDSTUR,
	LOGIC_SR,ADDSUB_ER,SRC3
};

// mask: aux that returns a mask for 
// the accepted number of bits
int mask(int num_bits) {
	return ~(~0x0<<num_bits);
}

// nbits: aux that returns a specified number of bits
// starting at a specified index in an accepted int
int nbits(int i, int start_idx, int num_bits) {
	return (i>>start_idx) & mask(num_bits);
}

// bit: aux that returns the bit
// at a specified index for an accepted int
int bit(int i, int idx) {
	return nbits(i,idx,1);
}

// imm_ext: aux that sign-extends an immediate value to 64b
uint64_t imm_ext(uint32_t imm, int imm_len) {
	uint64_t sign = (uint64_t)(imm>>(imm_len-1));
	sign = ~sign+1;
	return (sign<<imm_len) | (uint64_t)imm;
}

// incr_PC: aux that increments PC
void incr_PC() {
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

// flagset: aux that sets the next-state N and Z flags
void flagset(int Rd) {
	uint64_t nxt_Rd_val = NEXT_STATE.REGS[Rd];
	NEXT_STATE.FLAG_N = nxt_Rd_val>>63 ? 0x1 : 0x0;
	NEXT_STATE.FLAG_Z = nxt_Rd_val ? 0x0 : 0x1;
}

void fetch(uint32_t *ip) {
	*ip = mem_read_32(CURRENT_STATE.PC);
}

void decode(uint32_t instr, 
	enum instr_type *itype, int *iid,
	uint32_t *immp, int *Rdp, int *Rnp, int *Rmp, int *Rtp) {
	
	uint32_t op0 = nbits(instr,25,4);
	uint32_t op1,op2,op3,op4,op5;

	if ((op0>>1)==0x4) { 		// data proc imm
		op0 = nbits(instr,23,3);
		if ((op0>>1)==0x1) {	// add/sub imm
			*itype = ADDSUB_IMM;
			*iid = nbits(instr,29,3);	// sf,op,S
			*immp = nbits(instr,10,12);
			*Rdp = instr & REG_MASK;
			*Rnp = (instr>>5) & REG_MASK;
		} else if (op0==0x5) {			// mov
			*itype = MOV_IMM;
			*immp = nbits(instr,5,16);
			*Rdp = instr & REG_MASK;
		} else if (op0==0x6) {			// bitfield (lsl/r)
			*itype = BITFLD;
			*iid = nbits(instr,10,6); 	// imms
			*immp = nbits(instr,16,6);	// immr
			*Rdp = instr & REG_MASK;
			*Rnp = (instr>>5) & REG_MASK;
		}
	} else if ((op0>>1)==0x5) {// branch,etc.
		op0 = nbits(instr,29,3);
		op1 = nbits(instr,22,4);
		if ((op0==0x2)&&(op1<0x8)) {	// b.cond
			*itype = BCOND;
			*iid = nbits(instr,0,4);	// cond
			*immp = nbits(instr,5,19);
		} else if (op0==0x6) {	
			if (op1<0x4) {				// excgen (hlt)
				*itype = EXC;
			} else if (op1>=0x8) {		// br
				*itype = BR;
				*Rnp = (instr>>5) & REG_MASK;
			}
		} else if ((op0&mask(2))==0x0) {// b 
			*itype = B;
			*immp = nbits(instr,0,26);
		} else if (((op0&mask(2))==0x1)&&
					(op1<0x8)) {		// cb
			*itype = CB;
			*iid = (bit(instr,31)<<1) | bit(instr,24);	// sf,op
			*immp = nbits(instr,5,19);
			*Rtp = instr & REG_MASK;
		}
	} else if (bit(op0,0)==0x0&&bit(op0,2)==0x1) {	// ld/st
		op1 = nbits(instr,28,2);
		op3 = nbits(instr,23,2);
		op4 = nbits(instr,16,6);
		op5 = nbits(instr,10,2);
		if ((op1==0x3)&&(op3<0x2)&&
					(op4<0x8)&&(op5==0x0)) {	// ld/stur
			*itype = LDSTUR;
			*iid = nbits(instr,30,2) << 8;
			*iid |= bit(instr,26) << 4;
			*iid |= nbits(instr,22,2);	// B-aligned size,V,opc
			*immp = nbits(instr,12,9);
			*Rtp = instr & REG_MASK;
			*Rnp = (instr>>5) & REG_MASK;
		}
	} else if ((op0&mask(3))==0x5) {// data proc reg
		op0 = bit(instr,30);
		op1 = bit(instr,28);
		op2 = nbits(instr,21,4);
		op3 = bit(instr,11);
		if (op1==0x0) {
			if (op2<0x8) {				// logical sr
				*itype = LOGIC_SR;
				*iid = nbits(instr,29,3) << 1;
				*iid |= bit(instr,21);		// sf,opc,N
				*Rdp = instr & REG_MASK; 
				*Rnp = (instr>>5) & REG_MASK;
				*Rmp = (instr>>16) & REG_MASK;
			} else if (bit(op2,3)==0x1) {	// add/sub er
				*itype = ADDSUB_ER;
				*iid = nbits(instr,29,3);	// sf,op,S
				*Rdp = instr & REG_MASK;
				*Rnp = (instr>>5) & REG_MASK;
				*Rmp = (instr>>16) & REG_MASK;
			}
		} else if ((op1==0x1)&&(op2>=0x8)) {	// 3-src (madd->mul)
			*itype = SRC3;
			*Rdp = instr & REG_MASK;
			*Rnp = (instr>>5) & REG_MASK;
			*Rmp = (instr>>16) & REG_MASK;
		}
	} 
}	

void exec_logsh(uint32_t immr, int imms, int Rd, int Rn);
void exec_ldur(int Rt, uint64_t addr, int size);
void exec_stur(int Rt, uint64_t addr, int size); 
void exec_sr(int srid, int Rd, int Rn, int Rm); 
void exec_er(int erid, int Rd, int Rn, int Rm);
void exec_imm(int immid, uint32_t imm12, int Rd, int Rn); 
void exec_bcond (int cond, uint32_t imm19);

void execute(enum instr_type itype, int iid, 
		uint32_t imm, int Rd, int Rn, int Rm, int Rt) {
	uint64_t offset, addr;
	int ld, size;
	switch (itype) {
		case ADDSUB_IMM: 
			exec_imm(iid,imm,Rd,Rn);
			incr_PC();
			break;
		case MOV_IMM:
			NEXT_STATE.REGS[Rd] = imm;
			incr_PC();
			break;
		case BITFLD: 
			exec_logsh(imm,iid,Rd,Rn);
			incr_PC();
			break;
		case BCOND: 
			exec_bcond(iid,imm);
			break; 
		case EXC: 
			RUN_BIT = 0x0;
			incr_PC();
			break;
		case BR: 
			NEXT_STATE.PC = CURRENT_STATE.REGS[Rn];
			break;
		case B: 
			offset = imm_ext(imm,26)<<2;
			NEXT_STATE.PC = CURRENT_STATE.PC + offset;
			break;
		case CB: 
			offset = imm_ext(imm,19)<<2;
			if (bit(iid,0)) {	// op; cbnz:1, cbz:0
				NEXT_STATE.PC = CURRENT_STATE.PC + 
						(CURRENT_STATE.REGS[Rt] ? offset : 4);
			} else {			// cbz
				NEXT_STATE.PC = CURRENT_STATE.PC +
						(CURRENT_STATE.REGS[Rt] ? 4 : offset);
			}
			break;
   		case LDSTUR: 
			ld = nbits(iid,0,4); // opc; ld:01b, st:00b
			addr = CURRENT_STATE.REGS[Rn] + imm_ext(imm,9);
			size = nbits(iid,8,4); // ur:11b, urb:00b, urh:01b
			if (ld) exec_ldur(Rt,addr,size);
			else exec_stur(Rt,addr,size);
			incr_PC();
			break;
		case LOGIC_SR: 
			exec_sr(iid,Rd,Rn,Rm);
			incr_PC();
			break;
		case ADDSUB_ER: 
			exec_er(iid,Rd,Rn,Rm);
			incr_PC();
			break;
		case SRC3: 
			NEXT_STATE.REGS[Rd] = 
					(uint64_t)(CURRENT_STATE.REGS[Rn]*
								CURRENT_STATE.REGS[Rm]);
			incr_PC();
			break;
		default: break;
	}
}

void exec_logsh(uint32_t immr, int imms, int Rd, int Rn) {
	if (!(imms^mask(6))) {			// lsr
		NEXT_STATE.REGS[Rd] = CURRENT_STATE.REGS[Rn]>>immr;
		NEXT_STATE.REGS[Rd] &= mask(64-immr);
	} else if (imms+1==immr) {		// lsl
		imms = (~imms) & mask(6);
		NEXT_STATE.REGS[Rd] = CURRENT_STATE.REGS[Rn]<<imms;
	}
}
		
void exec_ldur(int Rt, uint64_t addr, int size) {
	uint64_t src_addr = addr;
	uint32_t src_word = mem_read_32(src_addr);
	if (size==0x3) {		// ldur
		NEXT_STATE.REGS[Rt] = mem_read_32(src_addr+4);
		NEXT_STATE.REGS[Rt] <<= 32;
		NEXT_STATE.REGS[Rt] |= src_word;
	} else if (size==0x0) {	// ldurb
		NEXT_STATE.REGS[Rt] = src_word & mask(8);
	} else if (size==0x1) {	// ldurh
		NEXT_STATE.REGS[Rt] = src_word & mask(16);
	}
}	

void exec_stur(int Rt, uint64_t addr, int size) {
	uint64_t dest_addr = addr;
	uint32_t dest_word = mem_read_32(dest_addr);
	uint64_t src_val = CURRENT_STATE.REGS[Rt];
	if (size==0x3) {		// stur
		mem_write_32(dest_addr,(uint32_t)(src_val));
		mem_write_32(dest_addr+4,(uint32_t)(src_val>>32));
	} else if (size==0x0) {	// sturb
		src_val &= mask(8);
		dest_word &= (~0x0)<<8;
		dest_word |= (uint32_t)src_val;
		mem_write_32(dest_addr,dest_word);
	} else if (size==0x1) {	// sturh
		src_val &= mask(16);
		dest_word &= (~0x0)<<16;
		dest_word |= (uint32_t)src_val;
		mem_write_32(dest_addr,dest_word);
	}
}

void exec_sr(int srid, int Rd, int Rn, int Rm) {
	uint64_t Rn_val = CURRENT_STATE.REGS[Rn];
	uint64_t Rm_val = CURRENT_STATE.REGS[Rm];
	if (srid==0x8 || srid==0xe) { 	// and/ands
		NEXT_STATE.REGS[Rd] = Rn_val & Rm_val;
		if (srid==0xe) 
			flagset(Rd);
	} else if (srid==0xc) { // eor
		NEXT_STATE.REGS[Rd] = Rn_val ^ Rm_val;
	} else if (srid==0xa) { // orr
		NEXT_STATE.REGS[Rd] = Rn_val | Rm_val;
	}	
}

void exec_er(int erid, int Rd, int Rn, int Rm) {
	uint64_t Rn_val = CURRENT_STATE.REGS[Rn];
	uint64_t Rm_val = CURRENT_STATE.REGS[Rm];
	if (erid<0x6) {			// add/adds
		NEXT_STATE.REGS[Rd] = Rn_val + Rm_val;
	} else {				// sub/subs/cmp
		NEXT_STATE.REGS[Rd] = Rn_val - Rm_val;
	}
	if (erid & mask(1)) {
		flagset(Rd);
	}
}

void exec_imm(int immid, uint32_t imm12, int Rd, int Rn) {
	uint64_t Rn_val = CURRENT_STATE.REGS[Rn];
	if (immid<0x6) {		// add/adds
		NEXT_STATE.REGS[Rd] = Rn_val + imm12;
	} else {				// sub/subs/cmp
		NEXT_STATE.REGS[Rd] = Rn_val - imm12;
	}
	if (immid & mask(1)) {	// adds/subs/cmp
		flagset(Rd);
	}
}

int bcond_test(int cond, int N, int Z) {
	int C=0, V=0;
	if (cond==0x0) return Z;		// beq
	else if (cond==0x1) return !Z;		// bne
   	else if (cond==0xa) return N==V;	// bge
	else if (cond==0xb) return N!=V;	// blt
	else if (cond==0xc) return (!Z)&&(N==V);// bgt
	else if (cond==0xd) return Z||(N!=V);	// ble	
}

void exec_bcond (int cond, uint32_t imm19) {
	uint64_t offset = imm_ext(imm19,19)<<2;
	NEXT_STATE.PC = CURRENT_STATE.PC + (
		bcond_test(cond,CURRENT_STATE.FLAG_N,CURRENT_STATE.FLAG_Z) ?
		offset :
		4
		);
}

void process_instruction() {
    /* execute one instruction here. You should use CURRENT_STATE and modify
     * values in NEXT_STATE. You can call mem_read_32() and mem_write_32() to
     * access memory. */
	uint32_t instr;
	enum instr_type itype;
	int iid;
	uint32_t imm;
	int Rd,Rm,Rn,Rt;    	
	
	fetch(&instr);
	decode(instr,
			&itype,&iid,
			&imm,&Rd,&Rn,&Rm,&Rt);
	execute(itype,iid,
			imm,Rd,Rn,Rm,Rt);
	NEXT_STATE.REGS[ZR] = 0x0;
}
