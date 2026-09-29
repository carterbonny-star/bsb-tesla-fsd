#pragma once
// This branch is research-only. No build flag or stored/UI mode can enable TX.
#if defined(RESEARCH_PHYSICAL_TX) && RESEARCH_PHYSICAL_TX != 0
#error "Physical CAN TX is forbidden in the research branch"
#endif
#define RESEARCH_PHYSICAL_TX 0
#ifndef SNIFFER_ONLY
#define SNIFFER_ONLY 1
#endif
