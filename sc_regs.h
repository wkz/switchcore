/* \\/ Westermo - Switchcore driver
 *
 * Copyright (C) 2014  Westermo Teleindustri AB
 *
 * Author: Tobias Waldekranz <tobias.waldekranz@westermo.se>
 *
 */

#ifndef _SC_REGS_H
#define _SC_REGS_H

#define SC_PHY(n)  (n)

#define SC_PHY_IRQSTAT 0x13
#define SC_PHY_IRQSTAT_LINK (1 << 10)
#define SC_PHY_IRQSTAT_ANEG (1 << 11)

#define SC_PHY_IRQSUM  0x14

#define SC_PORT(n) (0x10 + (n))

#define SC_GLOBAL1 0x1b

#define SC_GC 4
#define SC_GC_ATUPROB (1 << 3)
#define SC_GC_PPU     (1 << 14)

#define SC_GS 0
#define SC_GS_PHYINT  (1 << 1)
#define SC_GS_ATUPROB (1 << 3)

#define SC_PCS_FC         (1 << 6)
#define SC_PCS_LINKVALUE  (1 << 5)
#define SC_PCS_FORCELINK  (1 << 4)
#define SC_PCS_DPX        (1 << 3)
#define SC_PCS_FORCEDPX   (1 << 2)
#define SC_PCS_SPEED_1000 (1 << 1)
#define SC_PCS_SPEED_100  1
#define SC_GLOBAL2 0x1c

#define SC_IS 0
#define SC_IM 1

#define SC_SC 0x18
#define SC_SC_BUSY (1 << 15)

#define SC_SD 0x19

#endif	/* _SC_REGS_H */
