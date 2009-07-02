/*
 * linux/include/asm/bootinfo.h:  Include file for boot information
 *                                provided on Motorola phones
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Date         Author          Comment
 * 01/07/2009   Motorola        Initial version
 */

#ifndef __ASMARM_BOOTINFO_H
#define __ASMARM_BOOTINFO_H


#if !defined(__KERNEL__) || defined(CONFIG_BOOTINFO)

/*
 * These #defines are used for the bits in powerup_reason.
 *
 * If a soft reset occurs, then the powerup reason contains the value
 * PU_REASON_SOFT_RESET.  In this case, it is guaranteed that no other
 * bits are set.  If the powerup reason is not PU_REASON_SOFT_RESET,
 * then the powerup reason is a bitfield and any of the remaining reasons
 * defined below may be set.  Multiple bits may be set in the powerup reason
 * if it is not PU_REASON_SOFT_RESET.
 */
#define PU_REASON_SOFT_RESET            0x00000000
#define PU_REASON_WARM_BOOT	        0x80000000
#define PU_REASON_WATCHDOG              0x00008000
#define PU_REASON_MODEL_ASSEMBLY        0x00001000
#define PU_REASON_SYSTEM_REBOOT         0x00000800
#define PU_REASON_CBL_DETECT_REGRESSION 0x00000400
#define PU_REASON_POWERCUT              0x00000200
#define PU_REASON_CBL_DETECT_CHARGER    0x00000100
#define PU_REASON_PWR_KEY_PRESS         0x00000080
#define PU_REASON_PDA_MODE              0x00000040
#define PU_REASON_CBL_DETECT_FACTORY    0x00000020
#define PU_REASON_CBL_DETECT_USB        0x00000010
#define PU_REASON_PCAP_RTC              0x00000008
#define PU_REASON_RESUME_FROM_SLEEP     0x00000004


/*
 * These #defines are used for the battery status at boot.
 * When no battery is present, the status is BATTERY_LO_VOLTAGE.
 */
#define BATTERY_GOOD_VOLTAGE    1
#define BATTERY_LO_VOLTAGE      2
#define BATTERY_UNKNOWN         (-1)

/*
 * /proc/bootinfo has a strict format.  Each line contains a name/value
 * pair which are separated with a colon and a single space on both
 * sides of the colon.  The following defines help you size the
 * buffers used to read the data from /proc/bootinfo.
 *
 * BOOTINFO_MAX_NAME_LEN:  maximum size in bytes of a name in the
 *                         bootinfo line.  Don't forget to add space
 *                         for the NUL if you need it.
 * BOOTINFO_MAX_VAL_LEN:   maximum size in bytes of a value in the
 *                         bootinfo line.  Don't forget to add space
 *                         for the NUL if you need it.
 * BOOTINFO_BUF_SIZE:      size in bytes of buffer that is large enough
 *                         to read a /proc/bootinfo line.  The extra
 *                         3 is for the " : ".  Don't forget to add
 *                         space for the NUL and newline if you
 *                         need them.
 */
#define BOOTINFO_MAX_NAME_LEN    32
#define BOOTINFO_MAX_VAL_LEN    128
#define BOOTINFO_BUF_SIZE       (BOOTINFO_MAX_NAME_LEN + \
					3 + BOOTINFO_MAX_VAL_LEN)

#endif


#if defined(__KERNEL__) && defined(CONFIG_BOOTINFO)

extern struct proc_dir_entry proc_root;

u32  bi_powerup_reason(void);
void bi_set_powerup_reason(u32 powerup_reason);

u32  bi_mbm_version(void);
void bi_set_mbm_version(u32 mbm_version);

u32  bi_mbm_loader_version(void);
void bi_set_mbm_loader_version(u32 mbm_loader_version);

u32  bi_flat_dev_tree_address(void);
void bi_set_flat_dev_tree_address(u32 flat_dev_tree_address);

u16  bi_battery_status_at_boot(void);
void bi_set_battery_status_at_boot(u16 battery_status_at_boot);

#endif /* defined(__KERNEL__) && defined(CONFIG_BOOTINFO) */

#endif
