#pragma once
// Default research builds are physically TX-locked.
// A separate BENCH_PHYSICAL_TX build may enable the CAN controller's normal
// mode for isolated bench testing only. FSD-generated frames still use the
// research mock sink unless a dedicated bench test harness explicitly calls
// CanDriver::send().
#if defined(BENCH_PHYSICAL_TX)
  #define RESEARCH_PHYSICAL_TX 1
#else
  #if defined(RESEARCH_PHYSICAL_TX) && RESEARCH_PHYSICAL_TX != 0
    #error "Physical CAN TX is forbidden outside the dedicated bench build"
  #endif
  #define RESEARCH_PHYSICAL_TX 0
  #ifndef SNIFFER_ONLY
    #define SNIFFER_ONLY 1
  #endif
#endif
