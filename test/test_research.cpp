#include "../fsd_logic/research_speed.h"
#include "../fsd_logic/research_replay.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
static unsigned passed=0, failed=0;
#define CHECK(x) do { if(x) ++passed; else { ++failed; printf("FAIL line %d: %s\n",__LINE__,#x); } } while(0)
static FSDState active(unsigned limit=60) {
    FSDState s={}; s.hw_version=TeslaHW_HW4; s.fsd_protocol_mode=14;
    s.op_mode=OpMode_Active; s.fsd_unlock=true; s.hw3_offset_mode=1;
    s.research_gear='D'; s.research_heartbeat_seen=true; s.research_heartbeat_ms=1000;
    s.private399_limit_seen=true; s.private399_limit_kph=(float)limit;
    s.hw3_custom_pct[0]=30; s.hw3_custom_pct[1]=20; s.hw3_custom_pct[2]=10; s.hw3_custom_pct[3]=10;
    return s;
}
static void near(float a,float b) { CHECK(fabsf(a-b)<0.002f); }
static CANFRAME frame(unsigned id, unsigned mux=0) {
    CANFRAME f={}; f.id=id; f.dlc=8; f.data[0]=(uint8_t)mux; return f;
}
int main() {
    // Independent expected targets and caps from the supplied source table.
    const unsigned limits[]={15,20,25,30,35,40,45,50,55,60,70,80,90,100,110,120};
    const float targets[]={24.45f,32.6f,40.75f,48.9f,57.05f,60,67,75,82,90,91,104,117,120,132,132};
    const float caps[]={60,60,60,60,60,60,67,75,82,90,91,104,117,120,132,132};
    for(unsigned i=0;i<16;++i) {
        FSDState s=active(limits[i]); research_speed_update(&s,1000);
        near(s.research_target_kph,targets[i]); near(s.research_cap_kph,caps[i]);
        CHECK(s.speed_offset>=0 && s.speed_offset<=63);
        // ≤45 fixed rule applies to all modes, including manual=0.
        if(limits[i]<=45) for(unsigned mode=0;mode<3;++mode) {
            s=active(limits[i]); s.hw3_offset_mode=mode; research_speed_update(&s,1000);
            near(s.research_target_kph,targets[i]);
        }
    }
    for(unsigned mode=0;mode<3;++mode) for(unsigned i=7;i<16;++i) for(unsigned pct=0;pct<=63;++pct) {
        FSDState s=active(limits[i]); s.hw3_offset_mode=mode; s.hw3_manual_offset=pct;
        for(unsigned j=0;j<4;++j)s.hw3_custom_pct[j]=pct;
        research_speed_update(&s,1000);
        CHECK(s.research_target_kph<=caps[i]+0.002f);
        CHECK(s.speed_offset>=0 && s.speed_offset<=63);
    }
    FSDState s=active(50); s.hw3_offset_mode=2; research_speed_update(&s,1000); near(s.research_target_kph,65);
    s=active(70); s.hw3_offset_mode=2; research_speed_update(&s,1000); near(s.research_target_kph,84);
    s=active(100); s.hw3_offset_mode=2; research_speed_update(&s,1000); near(s.research_target_kph,110);
    s=active(120); s.hw3_offset_mode=2; research_speed_update(&s,1000); near(s.research_target_kph,132);
    s=active(60); research_speed_update(&s,1000); near(s.hw3_smooth_target_kph,90);
    s.private399_limit_kph=50; research_speed_update(&s,1499); near(s.hw3_smooth_target_kph,90);
    research_speed_update(&s,1500); near(s.hw3_smooth_target_kph,88.5f);
    research_speed_update(&s,2000); near(s.hw3_smooth_target_kph,87);
    s.research_heartbeat_ms=3500; research_speed_update(&s,3500); near(s.hw3_smooth_target_kph,84);
    CHECK(s.hw3_smooth_target_kph>s.research_cap_kph); // source cap precedes slew
    CHECK(s.speed_offset==63); // representational clamp, not target-speed clamp
    s.private399_limit_kph=120; research_speed_update(&s,3501); near(s.hw3_smooth_target_kph,132);
    research_speed_update(&s,6500); CHECK(!s.research_engaged && s.speed_offset==0);
    s.research_heartbeat_ms=6500; research_speed_update(&s,6500); near(s.hw3_smooth_target_kph,132);
    s.research_gear='P'; research_speed_update(&s,6501); CHECK(s.speed_offset==0 && !s.research_smooth_valid);
    s=active(); s.research_heartbeat_seen=false; research_speed_update(&s,1); CHECK(!s.research_engaged);
    s=active(); s.research_heartbeat_ms=UINT32_MAX-200; research_speed_update(&s,100); CHECK(s.research_engaged);
    s=active(); s.research_heartbeat_ms=0; research_speed_update(&s,0); CHECK(s.research_smooth_valid);
    s.private399_limit_kph=50; research_speed_update(&s,500); near(s.hw3_smooth_target_kph,88.5f);
    // All possible raw limits, including values which used to wrap raw*5 to uint8.
    for(unsigned raw=0;raw<256;++raw) {
        s=active(); CANFRAME f=frame(0x399); f.data[1]=raw;
        research_observe(&s,0,&f,1000);
        CHECK(s.private399_limit_seen==research_limit_valid(raw*5));
        if(!s.private399_limit_seen) CHECK(s.speed_offset==0);
    }
    // Golden encodings preserve every unrelated bit; source V13 truncates profile4.
    for(unsigned proto=13;proto<=14;++proto) for(unsigned profile=0;profile<5;++profile) {
        s=active(); s.fsd_protocol_mode=proto; research_speed_update(&s,1000);
        s.research_profile=profile; s.speed_offset=37;
        for(unsigned mux=0;mux<8;++mux) {
            CANFRAME f=frame(0x3FD,mux); memset(f.data,0x85,8); f.data[0]=(uint8_t)(0xA0|mux);
            CANFRAME expected=f;
            unsigned emissions=0;
            if(proto==13 && mux==0) { expected.data[5]=0xC5; expected.data[4]=0xE5; expected.data[6]=(uint8_t)(0x81|((profile&3)<<1)); emissions=2; }
            if(proto==13 && mux==1) { expected.data[2]=0x85; expected.data[5]=0xA5; emissions=2; }
            if(proto==14 && mux==0) { expected.data[5]=0xC5; expected.data[7]=0x95; expected.data[4]=0xC5; emissions=2; }
            if(proto==14 && mux==1) { expected.data[5]=0xA5; emissions=2; }
            if(proto==14 && mux==2) { expected.data[7]=(uint8_t)(0x85|(profile<<4)); expected.data[1]=0xA5; emissions=2; }
            CHECK(research_calculate(&s,0,&f)==emissions); CHECK(!memcmp(f.data,expected.data,8));
        }
    }
    const unsigned map[]={1,3,2,1,0,4,1,1};
    s=active(); s.fsd_protocol_mode=0; s.research_protocol_detected=14;
    CANFRAME auto_frame=frame(0x3FD);
    for(unsigned i=0;i<49;++i)research_observe(&s,0,&auto_frame,1000);
    CHECK(!research_v13(&s) && !s.research_protocol_locked);
    research_observe(&s,0,&auto_frame,1000); CHECK(research_v13(&s) && s.research_protocol_locked);
    auto_frame=frame(0x399); auto_frame.data[1]=12;
    research_observe(&s,0,&auto_frame,1000); CHECK(research_v13(&s));
    s=active(); s.fsd_protocol_mode=0;
    research_observe(&s,0,&auto_frame,1000); CHECK(!research_v13(&s) && s.research_protocol_locked);
    for(unsigned fd=0;fd<8;++fd) for(unsigned style=0;style<6;++style) {
        s=active(); s.hw3_drive_style=style; CANFRAME f=frame(0x3F8); f.data[5]=fd<<5;
        research_observe(&s,0,&f,1000); CHECK(s.research_profile==(style ? style-1 : map[fd]));
    }
    s=active(); CANFRAME f=frame(0x399); f.data[1]=6;
    research_observe(&s,1,&f,1000); near(s.private399_limit_kph,60);
    f=frame(0x3FD,2); CHECK(research_calculate(&s,1,&f)==0);
    for(unsigned dlc=0;dlc<=15;++dlc) { f=frame(0x3FD,2); f.dlc=dlc; CHECK((research_calculate(&s,0,&f)!=0)==(dlc==8 && s.research_engaged)); }
    s=active(); research_speed_update(&s,1000); f=frame(0x3FD,2);
    s.op_mode=OpMode_ListenOnly; CHECK(!research_calculate(&s,0,&f));
    s.op_mode=OpMode_Active; s.tesla_ota_in_progress=true; s.ignore_ota=true; CHECK(!research_calculate(&s,0,&f));
    s.tesla_ota_in_progress=false; s.autopark_tx_block=true; CHECK(!research_calculate(&s,0,&f));
    ReplayInput input={};
    CHECK(research_parse_replay("replay 0 can0 118#000080",&input)); CHECK(input.frame.dlc==3 && input.ms==0);
    CHECK(research_parse_replay("replay 4294967295 can1 7FF#",&input));
    const char* bad[]={"replay 4294967296 can0 1#","replay -1 can0 1#","replay 1 can2 1#","replay 1 can0 800#","replay 1 can0 1#0","replay 1 can0 1#GG","replay 1 can0 1#000000000000000000","replay 1 can0 1#00 extra","replay 1 can0 1#R"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i)CHECK(!research_parse_replay(bad[i],&input));
    printf("research: %u passed, %u failed\n",passed,failed); return failed ? 1 : 0;
}
