/*
 * drivers/video/omap2/misc/dispsw-mr.h
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * -----------------------------------------------------------------------
 *
 * This interface is intended to handle all aspects of multiple resolution
 * support for the platform displays.
 */

#include <mach/display.h>

#include "dispsw.h"

#ifndef __DISPSW_MR_H__
#define __DISPSW_MR_H__

#define DISPSW_MR_MAX_RES_SUPPORTED	(20)

struct dispsw_mr_support {
	char dev_name[DISPSW_MAX_NAME_SIZE + 1];
	char res_name[DISPSW_MAX_NAME_SIZE + 1];
	struct omap_video_timings dev_timing;
};

struct dispsw_mr_data {
	struct mutex mtx; /* Lock for all device accesses */

	int num_entries;
	struct dispsw_mr_support *res[DISPSW_MR_MAX_RES_SUPPORTED];
};

/* Intended to be called at driver init and removal times */
int  dispsw_mr_init(struct dispsw_mr_data *mr);
void dispsw_mr_remove(struct dispsw_mr_data *mr);

int  dispsw_mr_set_supported_mode(struct dispsw_mr_data *mr, int idx,
				struct dispsw_mr_support *mode);

bool  dispsw_mr_is_multi_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev);
char *dispsw_mr_get_active_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev);
char *dispsw_mr_get_idx_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev, int idx);
int   dispsw_mr_set_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev, char *name);

#endif /* __DISPSW_MR_H__ */

