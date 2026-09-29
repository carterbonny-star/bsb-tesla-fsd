#pragma once
/* Offline CanFeather 3.82 model. Source: user-supplied task_can.cpp.
 * No driver dependency, I/O or clock reads. Explicit time makes replay deterministic.
 * These are source semantics, NOT a validated Tesla signal specification.
 */
#include "fsd_state.h"

static inline bool research_v13(const FSDState* s) {
    return s->fsd_protocol_mode == 13 ||
        (s->fsd_protocol_mode == 0 && s->research_protocol_detected == 13);
}

static inline bool research_limit_valid(unsigned limit) {
    const unsigned limits[] = {15,20,25,30,35,40,45,50,55,60,70,80,90,100,110,120};
    for(unsigned i=0; i<sizeof(limits)/sizeof(limits[0]); ++i)
        if(limit == limits[i]) return true;
    return false;
}

static inline void research_speed_reset(FSDState* s) {
    s->speed_offset = 0;
    s->hw3_offset_target = s->hw3_offset_last = 0;
    s->hw3_smooth_target_kph = 0;
    s->research_target_kph = s->research_cap_kph = 0;
    s->research_smooth_valid = false;
}

static inline void research_speed_update(FSDState* s, uint32_t now) {
    // Explicit seen flag avoids the source's false engagement during boot's first 3s.
    s->research_engaged = s->research_gear == 'D' && s->research_heartbeat_seen &&
        (uint32_t)(now - s->research_heartbeat_ms) < 3000u;
    if(!s->research_engaged || !s->private399_limit_seen ||
       !research_limit_valid((unsigned)s->private399_limit_kph)) {
        // Deliberate hardening: invalid input / disengagement clears stale offset.
        research_speed_reset(s);
        return;
    }
    unsigned limit = (unsigned)s->private399_limit_kph;
    unsigned pct = 0, cap = 0;
    if(limit <= 45) {
        cap = limit == 45 ? 67 : 60;
        pct = limit <= 35 ? 63 : 50; // source priority, even in manual/custom mode
    } else {
        switch(limit) {
            case 50: cap=75; break; case 55: cap=82; break;
            case 60: cap=90; break; case 70: cap=91; break;
            case 80: cap=104; break; case 90: cap=117; break;
            case 100: cap=120; break; default: cap=132; break;
        }
        if(s->hw3_offset_mode == 0) pct=s->hw3_manual_offset;
        else if(s->hw3_offset_mode == 1)
            pct=limit<=60 ? 50 : limit<=90 ? 30 : limit<=110 ? 20 : 10;
        else pct=s->hw3_custom_pct[limit<=50 ? 0 : limit<=70 ? 1 : limit<=100 ? 2 : 3];
    }
    if(pct>63) pct=63;
    float target=(float)(limit*(1.0+pct/100.0));
    if(target>(float)cap) target=(float)cap;
    s->research_target_kph=target;
    s->research_cap_kph=(float)cap;
    float desired=(target-(float)limit)/(float)limit*100.0f;
    s->hw3_offset_target=(uint8_t)(desired<0 ? 0 : desired>63 ? 63 : desired);
    if(!s->research_smooth_valid || target>=s->hw3_smooth_target_kph) {
        s->hw3_smooth_target_kph=target;
        s->hw3_smooth_last_ms=now;
        s->research_smooth_valid=true;
    } else {
        float dt=(uint32_t)(now-s->hw3_smooth_last_ms)/1000.0f;
        if(dt>1.0f) dt=1.0f;
        if(dt>=0.5f) {
            s->hw3_smooth_target_kph-=3.0f*dt;
            if(s->hw3_smooth_target_kph<target) s->hw3_smooth_target_kph=target;
            s->hw3_smooth_last_ms=now;
            ++s->hw3_slew_count;
        }
    }
    // Like the source, cap is applied BEFORE slew. During a decrease the smoothed
    // target can temporarily exceed cap. Six-bit, nonnegative encoding cannot
    // represent every smoothed speed across a large limit change.
    float dyn=(s->hw3_smooth_target_kph-(float)limit)/(float)limit*100.0f;
    s->speed_offset=(uint8_t)(dyn<0 ? 0 : dyn>63 ? 63 : dyn); // truncation, not rounding
    s->hw3_offset_last=(uint8_t)s->speed_offset;
}

static inline void research_observe(FSDState* s, unsigned bus, const CANFRAME* f, uint32_t now) {
    if(bus!=0 || f->dlc>8) return; // source: TWAI primary owns speed/protocol signals
    bool was_engaged=s->research_engaged;
    if(!s->research_protocol_locked) {
        if(f->id==0x399 && f->dlc>=6) {
            s->research_protocol_detected=14;
            s->research_protocol_locked=true;
            s->research_3fd_count=0;
        } else if(f->id==0x3FD && f->dlc==8 && ++s->research_3fd_count>=50) {
            s->research_protocol_detected=13;
            s->research_protocol_locked=true;
        }
    }
    if(f->id==0x118 && f->dlc>=3) {
        unsigned gear=f->data[2]>>5;
        s->research_gear=gear==1 ? 'P' : gear==2 ? 'R' : gear==3 ? 'N' : gear==4 ? 'D' : '?';
    }
    if(f->id==0x389 && f->dlc>=2 && f->data[0]==0xFF &&
       (f->data[1]==2 || f->data[1]==3 || f->data[1]==0x42 || f->data[1]==0x43)) {
        s->research_heartbeat_seen=true;
        s->research_heartbeat_ms=now;
    }
    if(f->id==0x399 && f->dlc>=6) {
        unsigned limit=(unsigned)f->data[1]*5u;
        s->private399_raw_limit=f->data[1];
        s->private399_limit_seen=research_limit_valid(limit);
        s->private399_limit_kph=s->private399_limit_seen ? (float)limit : 0;
        s->private399_last_ms=now;
    }
    if(f->id==0x3F8 && f->dlc>=6) {
        const uint8_t map[8]={1,3,2,1,0,4,1,1};
        s->follow_distance_raw=(f->data[5]>>5)&7;
        s->research_profile=map[s->follow_distance_raw];
    }
    if(s->hw3_drive_style>=1 && s->hw3_drive_style<=5)
        s->research_profile=s->hw3_drive_style-1;
    s->research_engaged=s->research_gear=='D' && s->research_heartbeat_seen &&
        (uint32_t)(now-s->research_heartbeat_ms)<3000u;
    // Match source scheduling: update on engagement edges or a limit frame.
    if(was_engaged!=s->research_engaged || (f->id==0x399 && f->dlc>=6))
        research_speed_update(s,now);
}

/* Returns number of mock emissions. The source duplicates modified 0x3FD.
 * Caller must write these only to a mock sink; no physical CAN interface here.
 */
static inline unsigned research_calculate(FSDState* s, unsigned bus, CANFRAME* f) {
    if(bus!=0 || f->dlc!=8 || s->op_mode==OpMode_ListenOnly ||
       s->tesla_ota_in_progress || s->autopark_tx_block) return 0;
    if(f->id==0x3F8 && s->research_engaged) {
        f->data[1]|=0x40;
        return 1;
    }
    if(f->id!=0x3FD || !s->fsd_unlock) return 0;
    unsigned mux=f->data[0]&7;
    bool changed=false;
    if(!research_v13(s)) {
        if(s->research_gear!='D' && s->research_gear!='R') return 0;
        if(mux==0) { f->data[5]|=0x40; f->data[7]|=0x10; f->data[4]|=0x40; changed=true; }
        if(mux==1) { f->data[5]|=0xA0; changed=true; }
        if(mux==2 && s->research_engaged) {
            f->data[7]=(f->data[7]&~0x70)|((s->research_profile&7)<<4);
            f->data[1]=(f->data[1]&~0x3F)|(s->speed_offset&0x3F);
            changed=true;
        }
    } else {
        if(mux==0) {
            f->data[5]|=0x40;
            if(s->research_engaged) {
                f->data[4]=(f->data[4]&~0x3F)|0x40|(s->speed_offset&0x3F);
                f->data[6]=(f->data[6]&~0x06)|((s->research_profile&3)<<1);
            }
            changed=true;
        }
        if(mux==1) { f->data[2]&=~0x08; f->data[5]|=0xA0; changed=true; }
    }
    if(changed) ++s->frames_modified;
    return changed ? 2 : 0;
}
