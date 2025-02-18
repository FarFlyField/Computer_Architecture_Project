/*
 * CMSC 22200, Fall 2016
 *
 * ARM pipeline timing simulator
 *
 */

#include "cache.h"
#include "pipe.h"
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

// power2: computes 2^exp (for set and block)
int power2(uint32_t exp) {
    int ret = 1;
    for (; exp; exp--) {
        ret *= 2;
    }
    return ret;
}

// map addr set bits to cache set index
int get_sidx(uint64_t addr, cache_t *c) {
	assert(c);
	int s = c->s;
	int b = c->b;
	return (int)getbits_64(addr,s+b-1,b);
}

// map addr offset bits to cache block offset
int get_offset(uint64_t addr, cache_t *c) {
	assert(c);
	int b = c->b;
	return (int)getbits_64(addr,b-1,0);
}

// map addr tag bits to cache line tag
uint64_t get_tag(uint64_t addr, cache_t *c) {
	assert(c);
	int s = c->s;
	int b = c->b;
	return getbits_64(addr,63,s+b);
}

// cache_new: create a cache_t instance on the heap
// sets: number of set bits
// ways: number of lines per set (associativity)
// block: number of block bits
cache_t *cache_new(int sets, int ways, int block)
{
    int S, B;
    S = power2(sets);
    B = power2(block);

	cache_t *c = (cache_t*)malloc(sizeof(cache_t));
	assert(c);
    c->s = sets;
    c->S = S;
	c->E = ways;
    c->b = block;
    c->B = B;
	c->sets = (cline**)malloc(sizeof(cline*)*S);
	assert(c->sets);

    int sidx, lidx; // set and line indices
	for (int sidx=0; sidx<S; sidx++) {
		c->sets[sidx] = (cline*)calloc(sizeof(cline),ways);
		assert(c->sets[sidx]);
		for (int lidx=0; lidx<ways; lidx++) {
			c->sets[sidx][lidx].data = 
                (uint8_t*)calloc(sizeof(uint8_t),B);
			assert(c->sets[sidx][lidx].data);
		}
	}
	c->access_line = &c->sets[0][0];
	return c;	
}

// cache_destroy: free a cache instance from the heap
void cache_destroy(cache_t *c)
{
	assert(c);
	int sidx, lidx; // set and line indices
	assert(c->sets);
	for (sidx=0; sidx<power2(c->s); sidx++) {
		assert(c->sets[sidx]);
		for (lidx=0; lidx<c->E; lidx++) {
			assert(c->sets[sidx][lidx].data);
			free(c->sets[sidx][lidx].data);
		}
		free(c->sets[sidx]);
	}
	free(c->sets);
	free(c);
	return;
}

// read 4B from a cache line
uint32_t cread_32(uint64_t addr, cache_t *c) {
	assert(c);
	cline *accline = c->access_line;
	assert(accline);
	uint8_t *acc_data = accline->data;
	assert(acc_data);
    
	int offset = get_offset(addr,c);

	uint32_t rdata = 0;
	// read 4B starting from block_offset (lowest bits of addr)
	// to rdata
	for (int i=0; i<4; i++) {
		rdata |= (acc_data[offset+i]<<(i*8));
	} 
	return rdata;
}

void cwrite_32(uint64_t addr, uint32_t wdata, cache_t *c) {
	assert(c);
	cline *accline = c->access_line;
	assert(accline);
	uint8_t *acc_data = accline->data;
	assert(acc_data);
	
	int offset = get_offset(addr,c);

    // write 4B wdata byte by byte to cache line
    // starting from block_offset 
    for (int i=0; i<4; i++) {
        acc_data[offset+i] = (uint8_t)(wdata>>(i*8));
    } 
}

// cache_update: returns 1 if hit, 0 if miss;
// point hit_line if hit
int cache_update(cache_t *c, uint64_t addr, int action) {
	assert(c);
	int hit = 0;
	int s, S, E, b, B;
	s = c->s;
	S = c->S;
	E = c->E;
	b = c->b;
	B = c->B;

    // map addr to set and tag
    uint64_t addr_tag = get_tag(addr,c);
	printf("cache:addr_tag:0x%lx\n",addr_tag);		// DB
    int addr_sidx = get_sidx(addr,c);
	printf("cache:addr_sidx:0x%x\n",addr_sidx);		// DB
    assert(addr_sidx<S);
    cline *accset = c->sets[addr_sidx];    // set to access (addr set bits mapped to)

    // search for block in set line by line,
    // as well as for LRU (or first empty) line (min cycles = 0)
    int lidx = 0;   // line index
    int lru_lidx = 0;
    int lru_cycle = accset[0].mr_cycle;

    for (; lidx<E; lidx++) {
        if (accset[lidx].valid) {  // line valid
            if (!(accset[lidx].tag^addr_tag)) {    // tags match
				printf("%s\n","cache:matching line found");	// DB
                c->access_line = &accset[lidx];	// matching line to be read/written
                hit = 1;
            }
        }
		if (accset[lidx].mr_cycle<lru_cycle) {
			lru_lidx = lidx;
			lru_cycle = accset[lidx].mr_cycle;
		}
    }

    // miss
    if (!hit) {
		
        printf("%s\n","cache:miss"); // DB
        if (action==ACTN_POSS_STALL) {
	        printf("%s\n","cache:prepare to stall/stalling"); // DB
			// set access line to LRU line
			// prepare for potential mem retrieval at 10th stall cycle
			c->access_line = &accset[lru_lidx];

		} else if (action==ACTN_UNSTALL) {
        	printf("%s\n","cache:last stall cycle"); // DB
			// LRU line found during 1st stall cycle, pointed to by access_line
			// mem retrieval routine 
			cline *accline = c->access_line;
        	uint8_t *acc_data = accline->data;	// points to cline block
        	uint32_t memword;

			// block base (addr tag and set bits)
			uint64_t bB_addr;
        	// if replacing dirty block, write to mem
        	if (DC_WRITE) { 
				printf("%s\n","cache:LRU line dirty"); // DB
        		uint64_t acc_tag = accline->tag; // existing tag
        		uint64_t memdest_base = (acc_tag<<(s+b)) | (addr_sidx<<(b));
        		bB_addr = memdest_base;
				for (int i=0; i<B; i+=4) {   // iterate through block by word 
					memword = 0;
        	 		for (int j=0; j<4; j++) {   // iterate through word by byte
                   		memword |= acc_data[i+j]<<(j*8);
               		}
               		mem_write_32(bB_addr,memword);
               		bB_addr += 4;
            	}
				DC_WRITE = 0x0;
        	} 
        	// overwrite block
			uint64_t addr_bbase = addr & ~(mask_64(b)); 
			printf("cache:mem acc addr block base:0x%lx\n",addr_bbase);	// DB
			bB_addr = addr_bbase;
        	accline->tag = addr_tag;
        	for (int i=0; i<B; i+=4) {   // iterate through block by word
           		memword = mem_read_32(bB_addr);
           		for (int j=0; j<4; j++) {   // iterate through word by byte
               		acc_data[i+j] = (uint8_t)(memword>>(j*8));
           		}
           		bB_addr += 4;
        	}
        	accline->valid = 1;
   		}
    } else {	// hit
		// DB
		printf("%s","*****\n");
		printf("%s","cache: hit - access line dump\n");
		printf("tag:0x%lx\n",c->access_line->tag);
		printf("tag:0x%lx\n",c->access_line->tag);
		for (int i=0; i<B; i++) {
			printf("block byte %d:0x%x\n",i,c->access_line->data[i]);
		}
		printf("%s","*****\n");
		if (IC_STALL) { 
		// fetch miss,
		// but downstream branch resln directs PC 
		// back to earlier fetched block
			printf("%s","cache:PC redirected to earlier fetched block\n");
			IC_STALL = 0x0;
			IC_STALL_CYCLE = 0x0;
		}
	}
	
	// line to access (hit/replacement) is now MRU line
	c->access_line->mr_cycle = stat_cycles;
	
	return hit;
}

