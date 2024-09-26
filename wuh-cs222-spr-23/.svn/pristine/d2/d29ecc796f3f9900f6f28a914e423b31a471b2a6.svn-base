/*
 * CMSC 22200, Fall 2016
 *
 * ARM pipeline timing simulator
 *
 */
#ifndef _CACHE_H_
#define _CACHE_H_

#include <stdint.h>

#define ACTN_UNSTALL 0
#define ACTN_POSS_STALL 1


typedef struct cline {
	uint8_t valid;
	uint32_t mr_cycle;
	uint64_t tag;
	uint8_t *data;
} cline;

typedef struct cache
{
	int s, S;	// num of set bits & num of sets
	int E;		// num of ways (associativity)
	int b, B;	// num of block bits & num of blocks
	cline **sets;	// data
	cline *access_line;	// cline for current access
} cache_t;

// initiate cache
cache_t *cache_new(int sets, int ways, int block);
// free cache
void cache_destroy(cache_t *c);

// map addr set bits to cache set index
int get_sidx(uint64_t addr, cache_t *c);
// map addr offset bits to cache block offset
int get_offset(uint64_t addr, cache_t *c);
// map addr tag bits to cache line tag
uint64_t get_tag(uint64_t addr, cache_t *c);

// read a word from cache
uint32_t cread_32(uint64_t addr, cache_t *c);
// write a word to cache
void cwrite_32(uint64_t addr, uint32_t wdata, cache_t *c);

int cache_update(cache_t *c, uint64_t addr, int action);

#endif
