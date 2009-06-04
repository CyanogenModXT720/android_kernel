#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>

#define DEBUG(fmt, arg...)  printk("%s(): " fmt, __FUNCTION__, ##arg)

static DECLARE_WAIT_QUEUE_HEAD(adb_enabled_wait_q);
static int usb_adb_enabled_status = -1;
static int otg_switch_to_mode = -1;
                                                                                
static int adb_enable_open(struct inode *ip, struct file *fp)
{
        return 0;
}
static int adb_enable_release(struct inode *ip, struct file *fp)
{
        return 0;
}
                                                                                
static unsigned int adb_enable_poll(struct file *file, struct poll_table_struct *wait)
{
        poll_wait(file, &adb_enabled_wait_q, wait);
        if(usb_adb_enabled_status == -1 && otg_switch_to_mode == -1) {
            return 0;
        }
        return POLLIN | POLLRDNORM;
}
static ssize_t adb_enable_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
        int ret;
        ret = copy_to_user(buf, &otg_switch_to_mode, sizeof(int));
        usb_adb_enabled_status = -1;
        otg_switch_to_mode = -1;
        return sizeof(int);
}
                                                                                
static ssize_t adb_enable_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
        char ch;
        int ret;
        ret = copy_from_user(&ch, buf, sizeof(char));
        if (ch == '2')
                usb_adb_enabled_status = 2;
        else
                usb_adb_enabled_status = 1;
        wake_up_interruptible(&adb_enabled_wait_q);
        return sizeof(int);
}

void otg_require_switch(u16 mode)
{
        DEBUG("request mode=%d\n", mode);
        otg_switch_to_mode = mode;
        wake_up_interruptible(&adb_enabled_wait_q);
}

EXPORT_SYMBOL(otg_require_switch);

static struct file_operations adb_enable_fops = {
        .owner =   THIS_MODULE,
        .open =    adb_enable_open,
        .release = adb_enable_release,
        .poll =    adb_enable_poll,
        .read =    adb_enable_read,
        .write =    adb_enable_write,
};
static struct miscdevice adb_enable_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "android_adb_enable",
        .fops = &adb_enable_fops,
};

static int __init init(void)
{
        /* register adb enable device to support Andriod UI */
        usb_adb_enabled_status = 0;
        misc_register(&adb_enable_device);
        return 0;
}

module_init(init);
                                                                                
static void __exit cleanup(void)
{
        misc_deregister(&adb_enable_device);
}
module_exit(cleanup);


