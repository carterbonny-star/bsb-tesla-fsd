// Host-only replay tool. Contains no CAN/serial/network driver.
#include "../fsd_logic/research_speed.h"
#include "../fsd_logic/research_replay.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv) {
    FSDState s={};
    s.hw_version=TeslaHW_HW4;
    s.fsd_protocol_mode=14;
    s.op_mode=OpMode_Active;
    s.fsd_unlock=true;
    s.hw3_offset_mode=1;
    s.hw3_custom_pct[0]=30; s.hw3_custom_pct[1]=20;
    s.hw3_custom_pct[2]=10; s.hw3_custom_pct[3]=10;
    if(argc>1) {
        int protocol=atoi(argv[1]);
        if(protocol!=13 && protocol!=14) { fprintf(stderr,"protocol must be 13 or 14\n"); return 2; }
        s.fsd_protocol_mode=(uint8_t)protocol;
    }
    char line[128];
    unsigned lineno=0;
    uint32_t last_ms=0;
    bool seen=false;
    while(fgets(line,sizeof(line),stdin)) {
        ++lineno;
        size_t n=strcspn(line,"\r\n");
        if(n==sizeof(line)-1) { fprintf(stderr,"line %u too long\n",lineno); return 2; }
        line[n]=0;
        if(!line[0] || line[0]=='#') continue;
        ReplayInput input={};
        if(!research_parse_replay(line,&input) || (seen && (int32_t)(input.ms-last_ms)<0)) {
            fprintf(stderr,"invalid replay / backwards timestamp at line %u\n",lineno); return 2;
        }
        last_ms=input.ms; seen=true;
        research_observe(&s,input.bus,&input.frame,input.ms);
        unsigned count=research_calculate(&s,input.bus,&input.frame);
        for(unsigned k=0;k<count;++k) {
            printf("MOCK %u can%u %03X#",input.ms,input.bus,(unsigned)input.frame.id);
            for(unsigned i=0;i<input.frame.dlc;++i) printf("%02X",input.frame.data[i]);
            printf(" offset=%d target=%.3f smooth=%.3f cap=%.0f\n",s.speed_offset,
                   s.research_target_kph,s.hw3_smooth_target_kph,s.research_cap_kph);
        }
    }
    return ferror(stdin) ? 2 : 0;
}
