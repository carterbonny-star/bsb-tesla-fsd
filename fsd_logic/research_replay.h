#pragma once
#include "fsd_types.h"
#include <stdint.h>
#include <string.h>

typedef struct {
    uint32_t ms;
    unsigned bus;
    CANFRAME frame;
} ReplayInput;

static inline int research_hex(char c) {
    if(c>='0' && c<='9') return c-'0';
    if(c>='a' && c<='f') return c-'a'+10;
    if(c>='A' && c<='F') return c-'A'+10;
    return -1;
}

// Strict classic-CAN replay input. Reject RTR, extended IDs, FD and extra text.
static inline bool research_parse_replay(const char* p, ReplayInput* out) {
    ReplayInput v={};
    if(strncmp(p,"replay ",7)) return false;
    p+=7;
    if(*p<'0' || *p>'9') return false;
    while(*p>='0' && *p<='9') {
        unsigned d=(unsigned)(*p++-'0');
        if(v.ms>(UINT32_MAX-d)/10u) return false;
        v.ms=v.ms*10u+d;
    }
    if(strncmp(p," can",4)) return false;
    p+=4;
    if(*p!='0' && *p!='1') return false;
    v.bus=(unsigned)(*p++-'0');
    if(*p++!=' ') return false;
    unsigned digits=0;
    while(research_hex(*p)>=0) {
        if(++digits>3) return false;
        v.frame.id=v.frame.id*16u+(unsigned)research_hex(*p++);
    }
    if(!digits || v.frame.id>0x7FF || *p++!='#') return false;
    while(*p) {
        if(v.frame.dlc==8) return false;
        int hi=research_hex(*p++);
        if(hi<0 || !*p) return false;
        int lo=research_hex(*p++);
        if(lo<0) return false;
        v.frame.data[v.frame.dlc++]=(uint8_t)((hi<<4)|lo);
    }
    *out=v;
    return true;
}
