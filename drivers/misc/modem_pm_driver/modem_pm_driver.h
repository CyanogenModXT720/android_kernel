/*
 * Copyright (C) 2009, Motorola, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 * Motorola 2009-Jan-28 - Initial Creation
 */

#ifndef __MODEM_PM_DRIVER_H__
#define __MODEM_PM_DRIVER_H__

#include <linux/ioctl.h>

#define MODEM_PM_DRIVER_DEV_NAME  "modem_pm_driver"


#define RES_LATENCY_CO          0x5 /* Latency constraint */
#define RES_OPP_CO              0x7 /* OPP constraint */
#define CO_VDD2_OPP3            0x3 /* VDD2 OPP constraint levels */

/* Latency constraint levels */
#define CO_LATENCY_MPUOFF_COREON        0x3
#define CO_LATENCY_MPUOFF_CORERET       0x4

enum MODEM_PM_SHARED_DDR_FREQUENCY_OPP_CONSTRAINT_T {
	MODEM_PM_SHARED_DDR_FREQUENCY_OPP_HIGH,
	MODEM_PM_SHARED_DDR_FREQUENCY_OPP_NO_VOTE,
};

enum MODEM_PM_SHARED_DDR_LOW_POWER_POLICY_CONSTRAINT_T {
	MODEM_PM_SHARED_DDR_LOW_POWER_POLICY_ON_INACTIVE,
	MODEM_PM_SHARED_DDR_LOW_POWER_POLICY_RET,
	MODEM_PM_SHARED_DDR_LOW_POWER_POLICY_NO_VOTE,
};

struct constraint_id {
	int   type;
	void *data;
};

/* Frequency OPP constraint command */
#define MODEM_PM_DRIVER_IOCTL_CMD_HANDLE_FREQUENCY_OPP_CONSTRAINT     (0x00)

/* Low power policy contraint command */
#define MODEM_PM_DRIVER_IOCTL_CMD_HANDLE_LOW_POWER_POLICY_CONSTRAINT  (0x01)

/* Handle frequency OPP constraint */
#define MODEM_PM_DRIVER_IOCTL_HANDLE_FREQUENCY_OPP_CONSTRAINT \
    _IOW(0, MODEM_PM_DRIVER_IOCTL_CMD_HANDLE_FREQUENCY_OPP_CONSTRAINT, \
	 enum MODEM_PM_SHARED_DDR_FREQUENCY_OPP_CONSTRAINT_T)

/* Handle low power policy constraint */
#define MODEM_PM_DRIVER_IOCTL_HANDLE_LOW_POWER_POLICY_CONSTRAINT \
    _IOW(0, MODEM_PM_DRIVER_IOCTL_CMD_HANDLE_LOW_POWER_POLICY_CONSTRAINT, \
	 enum MODEM_PM_SHARED_DDR_LOW_POWER_POLICY_CONSTRAINT_T)

#endif

