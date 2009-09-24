
#ifndef __PANEL_SHOLES_H__
#define __PANEL_SHOLES_H__

#define SHOLES_IOCTL_MAGIC	'g'
#define SHOLES_IOCTL_BASE	0x60

/* Freeze On Disable */
#define SHOLES_G_FOD		_IOR(SHOLES_IOCTL_MAGIC, \
				    SHOLES_IOCTL_BASE+0, int)
#define SHOLES_S_FOD		_IOW(SHOLES_IOCTL_MAGIC, \
				    SHOLES_IOCTL_BASE+1, int)

#endif /* __PANEL_SHOLES_H__ */

