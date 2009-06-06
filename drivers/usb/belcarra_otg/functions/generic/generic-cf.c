/*
 * otg/functions/generic/generic-cf.c
 * @(#) sl@belcarra.com/whiskey.enposte.net|otg/functions/generic/generic-cf.c|20070425221028|63290
 *
 *      Copyright (c) 2003-2005 Belcarra
 *	Copyright (c) 2005-2006 Belcarra Technologies 2005 Corp
 *
 * By:
 *      Stuart Lynne <sl@belcarra.com>,
 *      Bruce Balden <balden@belcarra.com>
 *
 */

/*!
 * @file otg/functions/generic/generic-cf.c
 * @brief Generic Configuration Function Driver
 *
 * This implements a generic composite function
 *
 *         - idVendor              16 bit word
 *         - idProduct             16 bit word
 *         - bcd_device            16 bit word
 *
 *         - bDeviceClass          byte
 *         - device_sub-class      byte
 *         - bcdDevice             byte
 *
 *         - vendor                string
 *         - iManufacturer         string
 *         - product               string
 *         - serial                string
 *
 *         - power                 byte
 *         - remote_wakeup         bit
 *
 *         - functions             string
 *
 *
 * The functions string would contain a list of names, the first would
 * be the composite function driver, the rest would be the interface
 * functions. For example:
 *
 *
 *         "cmouse-cf mouse-if"
 *         "mcpc-cf dun-if dial-if obex-if dlog-if"
 *
 * There are also a set of pre-defined configurations that
 * can be loaded singly or in toto.
 *
 * @ingroup GenericFunction
 */

#include <otg/otg-compat.h>
#include <otg/otg-module.h>

#include <otg/usbp-chap9.h>
#include <otg/usbp-func.h>
#include <otg/usbp-cdc.h>
#include <otg/otg-trace.h>
#include <otg/otg-api.h>
#include <otg/otg-utils.h>

#include "generic.h"
//#include "generic.c"



/* Module Parameters ******************************************************** */
/* !
 * @name XXXXX MODULE Parameters
 */
/* ! @{ */


MOD_AUTHOR ("sl@belcarra.com");
EMBED_LICENSE();
MOD_DESCRIPTION ("Generic Composite Function");


MOD_PARM_STR (driver_name, "Driver Name", NULL);

MOD_PARM_INT(idVendor, "Device Descriptor idVendor field", 0);
MOD_PARM_INT(idProduct, "Device Descriptor idProduct field", 0);
MOD_PARM_INT(bcddevice, "Device Descriptor bcdDevice field", 0);

MOD_PARM_INT (bDeviceClass, "Device Descriptor bDeviceClass field", 0);
MOD_PARM_INT (bDeviceSubClass, "Device Descriptor bDeviceSubClass field", 0);
MOD_PARM_INT (bDeviceProtocol, "Device Descriptor bDeviceProtocol field", 0);
MOD_PARM_INT (bcdDevice, "Device Descriptor bcdDevice field", 0);

MOD_PARM_STR (iConfiguration, "Configuration Descriptor iConfiguration field", NULL);
MOD_PARM_STR (iManufacturer, "Device Descriptor iManufacturer field", NULL);
MOD_PARM_STR (iProduct, "Device Descriptor iProduct field", NULL);
MOD_PARM_STR (iSerialNumber, "Device Descriptor iSerialNumber field", NULL);

MOD_PARM_STR (class_name, "Class Function Name", NULL);
MOD_PARM_STR (interface_names, "Interface Function Names List", NULL);

MOD_PARM_STR (config_name, "Pre-defined Configuration", NULL);
MOD_PARM_BOOL (load_all, "Load all pre-defined Configurations", 1);
MOD_PARM_BOOL (override, "Override idVendor and idProduct", 0);

/* Setup the pre-defined configuration name from configuration
 * option if available.
 *
 * N.B. This list needs to be synchronized with both pre-defined
 * configurations (see below) and the Kconfig list
 */
char *generic_predefined_configuration = "";


/* ! *} */


/* Setup the pre-defined configuration name from configuration
 * option if available.
 *
 * N.B. This list needs to be synchronized with both pre-defined
 * configurations (see below) and the Kconfig list
 */
#ifdef CONFIG_OTG_GENERIC_CONFIG_MOUSE
static char default_predefined_configuration[] = "hid";
#elif defined(CONFIG_OTG_GENERIC_CONFIG_NET_BLAN)
static char default_predefined_configuration[] = "mdlm-blan";
#elif defined(CONFIG_OTG_GENERIC_CONFIG_NET_SAFE)
static char default_predefined_configuration[] = "mdlm-safe";
#elif defined(CONFIG_OTG_GENERIC_CONFIG_NET_CDC)
static char default_predefined_configuration[] = "cdc-ecm";
#elif defined(CONFIG_OTG_GENERIC_CONFIG_NET_EEM)
static char default_predefined_configuration[] = "cdc-eem";
#elif defined(CONFIG_OTG_GENERIC_CONFIG_ACM_TTY)
static char default_predefined_configuration[] = "serial";
#elif defined(CONFIG_OTG_GENERIC_CONFIG_MSC)
static char default_predefined_configuration[] = "mass";
#elif defined(CONFIG_OTG_GENERIC_CONFIG_HID2)
static char default_predefined_configuration[] = "hid2";
#elif defined(CONFIG_OTG_GENERIC_CONFIG_HID2_NON_IAD)
static char default_predefined_configuration[] = "hid2-non_iad";
#elif defined(CONFIG_OTG_GENERIC_CONFIG_MSC_HID)
static char default_predefined_configuration[] = "msc-hid";
#elif defined(CONFIG_OTG_GENERIC_CONFIG_ANDROID_ADB)
static char default_predefined_configuration[] = "android_adb";
#else
static char default_predefined_configuration[] = "";
#endif

static char *generic_config_name(void){
        if(MODPARM(config_name) && strlen(MODPARM(config_name))) return MODPARM(config_name);
        printk(KERN_INFO"%s: %s\n", __FUNCTION__, default_predefined_configuration); 
        return default_predefined_configuration;
}

static int preset_config_name(void){
        return (strlen(generic_config_name()) > 0);
}


/* Pre-defined configurations *********************************************** */

static struct generic_config generic_cf_configs[] = {
        /* deprecated - use cdc-eem */
        #if defined(CONFIG_OTG_NETWORK_BLAN) || defined(_OTG_DOXYGEN)
        {
                .composite_driver = { .driver = { .name = "network", }, },
                .interface_names = "net-blan-if",
                .configuration_description = {
                        .iConfiguration = "Motorola Network Device",
                },
                .device_description = {
                        .bDeviceClass = COMMUNICATIONS_DEVICE_CLASS,
                        .bDeviceSubClass = 2,
                        .bDeviceProtocol = 0,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41D4),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
        },
        #endif /* defined(CONFIG_OTG_NETWORK_BLAN) */

        #if (defined(CONFIG_OTG_ACM_MODULE) || defined(CONFIG_OTG_ACM)) && defined(CONFIG_OTG_NETWORK_BLAN) && (defined(CONFIG_OTG_MTP) || defined(CONFIG_OTG_MTP_MODULE))
        {
        .composite_driver.driver.name = "ngp",
        .interface_names = "tty-if:net-blan-if:mtp-if",
        .configuration_description.iConfiguration = CONFIG_OTG_GENERIC_CONFIGURATION,
        .device_description.bDeviceClass = 0xff,
        .device_description.bDeviceSubClass = 0xff,
        .device_description.bDeviceProtocol = 0xff,   
        .device_description.idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
        .device_description.idProduct = __constant_cpu_to_le16(0x41D8),
        .device_description.bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
        .device_description.iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
        .device_description.iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME,
        },
        #endif

        #if defined(CONFIG_OTG_MTP) || defined(CONFIG_OTG_MTP_MODULE)
        {
                .composite_driver = { .driver = { .name = "mtp", }, },
                .interface_names = "mtp-if",
                .configuration_description = {
                        .iConfiguration = "Motorola MTP Device",
                },
                .device_description = {
                        .bDeviceClass = 0,
                        .bDeviceSubClass = 0,
                        .bDeviceProtocol = 0,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41D6),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
        },
        #endif

        #if defined(CONFIG_OTG_MSC) || defined(CONFIG_OTG_MSC_MODULE)
        {
                .composite_driver = { .driver = { .name = "msc", }, },
                .interface_names = "msc-if",
                .configuration_description = {
                        .iConfiguration = "Motorola Mass Storage Device",
                },
                .device_description = {
                        .bDeviceClass = 0,
                        .bDeviceSubClass = 0,
                        .bDeviceProtocol = 0,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41D9),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
        },
        #endif /* defined(CONFIG_OTG_MSC)  */
        #if defined(CONFIG_OTG_MOUSE) || defined(CONFIG_OTG_MOUSE_MODULE)
        {
                .composite_driver = { .driver = { .name = "hid", }, },
                .interface_names = "mouse-if",
                .configuration_description = {
                        .iConfiguration = "Motorola Charging Device",
                },
                .device_description = {
                        .bDeviceClass = 0,
                        .bDeviceSubClass = 0,
                        .bDeviceProtocol = 0,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41DF),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
        },
        #endif /* defined(CONFIG_OTG_MOUSE) */

        #if (defined(CONFIG_OTG_ACM_MODULE) || defined(CONFIG_OTG_ACM)) && defined(CONFIG_OTG_NETWORK_BLAN) && (defined(CONFIG_OTG_MTP) || defined(CONFIG_OTG_MTP_MODULE)) && (defined(CONFIG_OTG_ANDROID_ADB) || defined(CONFIG_OTG_ANDROID_ADB_MODULE))
        {
        .composite_driver.driver.name = "ngp_adb",
        .interface_names = "tty-if:net-blan-if:mtp-if:android_adb-if",
        .configuration_description.iConfiguration = CONFIG_OTG_GENERIC_CONFIGURATION,
        .device_description.bDeviceClass = 0xff,
        .device_description.bDeviceSubClass = 0xff,
        .device_description.bDeviceProtocol = 0xff,   
        .device_description.idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
        .device_description.idProduct = __constant_cpu_to_le16(0x41DA),
        .device_description.bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
        .device_description.iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
        .device_description.iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME,
        },
        #endif

        #if (defined(CONFIG_OTG_MTP) || defined(CONFIG_OTG_MTP_MODULE)) && (defined(CONFIG_OTG_ANDROID_ADB) || defined(CONFIG_OTG_ANDROID_ADB_MODULE))

        {
                .composite_driver = { .driver = { .name = "mtp_adb", }, },
                .interface_names = "mtp-if:android_adb-if",
                .configuration_description = {
                        .iConfiguration = CONFIG_OTG_GENERIC_CONFIGURATION,
                },
                .device_description = {
                        .bDeviceClass = 0xff,
                        .bDeviceSubClass = 0xff,
                        .bDeviceProtocol = 0xff,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41DC),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
        },
        #endif
        #if (defined(CONFIG_OTG_MSC) || defined(CONFIG_OTG_MSC_MODULE)) && (defined(CONFIG_OTG_ANDROID_ADB) || defined(CONFIG_OTG_ANDROID_ADB_MODULE))
        {
                .composite_driver = { .driver = { .name = "msc_adb", }, },
                .interface_names = "msc-if:android_adb-if",
                .configuration_description = {
                        .iConfiguration = CONFIG_OTG_GENERIC_CONFIGURATION,
                },
                .device_description = {
                        .bDeviceClass = 0x0,
                        .bDeviceSubClass = 0x0,
                        .bDeviceProtocol = 0x0,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41DB),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
        },
        #endif

        #if defined(CONFIG_OTG_ANDROID_ADB) || defined(CONFIG_OTG_ANDROID_ADB_MODULE)
        {
                .composite_driver = { .driver = { .name = "android_adb_only", }, },
                .interface_names = "android_adb-if",
                .configuration_description = {
                        .iConfiguration = "Motorola ADB Device",
                },
                .device_description = {
                        .bDeviceClass = 0xff,
                        .bDeviceSubClass = 0xff,
                        .bDeviceProtocol = 0xff,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41D2),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
        },
        #endif
        #if defined(CONFIG_OTG_ACM) ||  defined(CONFIG_OTG_ACM_MODULE) || defined(_OTG_DOXYGEN)
        {
                .composite_driver = { .driver = { .name = "serial", }, },
                .interface_names = "tty-if",
                .configuration_description = {
                        .iConfiguration = "Motorola TTY Device",
                },
                .device_description = {
                        .bDeviceClass = COMMUNICATIONS_DEVICE_CLASS,
                        .bDeviceSubClass = 2,
                        .bDeviceProtocol = 0,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41D3),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
                                                                                                
        },
        {
                .composite_driver = { .driver = { .name = "modem", }, },
                .interface_names = "tty-if",
                .configuration_description = {
                        .iConfiguration = "Motorola TTY Device",
                },
                .device_description = {
                        .bDeviceClass = COMMUNICATIONS_DEVICE_CLASS,
                        .bDeviceSubClass = 2,
                        .bDeviceProtocol = 0,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41D3),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
                                                                                                
        },
        #if defined(CONFIG_OTG_ANDROID_ADB) || defined(CONFIG_OTG_ANDROID_ADB_MODULE)
        {
                .composite_driver = { .driver = { .name = "modem_adb", }, },
                .interface_names = "tty-if:android_adb-if",
                .configuration_description = {
                        .iConfiguration = CONFIG_OTG_GENERIC_CONFIGURATION,
                },
                .device_description = {
                        .bDeviceClass = 0,
                        .bDeviceSubClass = 0,
                        .bDeviceProtocol = 0,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41DD),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
                                                                                                
        },
        #endif
        #endif /* defined(CONFIG_OTG_ACM) */
        #if defined(CONFIG_OTG_MSC) || defined(CONFIG_OTG_MSC_MODULE) || defined(_OTG_DOXYGEN)
        {
                .composite_driver = { .driver = { .name = "mass", }, },
                .interface_names = "msc-if",
                .configuration_description = {
                        .iConfiguration = "Motorola Mass Storage Device",
                },
                .device_description = {
                        .bDeviceClass = 0,
                        .bDeviceSubClass = 0,
                        .bDeviceProtocol = 0,
                        .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                        .idProduct = __constant_cpu_to_le16(0x41D9),
                        .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                        .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                        .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
        },
        #endif /* defined(CONFIG_OTG_MSC)  */
                                                                                                
        #if (defined(CONFIG_OTG_ACM_MODULE) || defined(CONFIG_OTG_ACM)) && defined(CONFIG_OTG_NETWORK_BLAN)
        {
        .composite_driver.driver.name = "tty-blan-motorola",
        .interface_names = "tty-if:net-blan-if",
        .configuration_description.iConfiguration = CONFIG_OTG_GENERIC_CONFIGURATION,
        .device_description.bDeviceClass = 0xff,
        .device_description.bDeviceSubClass = 0xff,
        .device_description.bDeviceProtocol = 0xff,
        .device_description.idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
        .device_description.idProduct = __constant_cpu_to_le16(0x41D5),
        .device_description.bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
        .device_description.iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
        .device_description.iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME,
        },
        #endif

        {  },
};

static struct generic_config generic_config = {
        .composite_driver = { .driver = { .name = "generic", }, },
        .device_description = {
                .idVendor = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_VENDORID),
                .idProduct = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_PRODUCTID),
                .bcdDevice = __constant_cpu_to_le16(CONFIG_OTG_GENERIC_BCDDEVICE),
                .iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER,
                .iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME, },
};




static otg_tag_t GENERIC;

/*!
 * generic_cf_modinit() - module init
 *
 * This is called by the Linux kernel; either when the module is loaded
 * if compiled as a module, or during the system intialization if the
 * driver is linked into the kernel.
 *
 * This function will parse module parameters if required and then register
 * the generic driver with the USB Device software.
 *
 */
int generic_cf_modinit (void)
{
        /* load config or configs
         */

        #if defined(CONFIG_OTG_TRACE) || defined(CONFIG_OTG_TRACE_MODULE)
        GENERIC = otg_trace_obtain_tag(NULL, "generic-cf");
        #endif

        if (preset_config_name() || MODPARM(load_all)) {
                TRACE_MSG0(GENERIC, "PRESET");
                return generic_modinit(generic_cf_configs,
                                generic_config_name(),
                                MODPARM(override),
                                MODPARM(idVendor),
                                MODPARM(idProduct),
                                MODPARM(iSerialNumber), 
                                GENERIC);
        }
        else {
                struct generic_config *config = &generic_config;

                TRACE_MSG0(GENERIC, "SEARCH");

                printk (KERN_INFO "%s: idVendor: %04x idProduct: %04x\n", __FUNCTION__, MODPARM(idVendor), MODPARM(idProduct));
                printk (KERN_INFO "%s: class_name: \"%s\" _interface_names: \"%s\"\n",
                                __FUNCTION__, MODPARM(class_name), MODPARM(interface_names));

                if (MODPARM(driver_name) && strlen(MODPARM(driver_name)))
                        config->composite_driver.driver.name = MODPARM(driver_name);

                if (MODPARM(class_name) && strlen(MODPARM(class_name)))
                        config->class_name = MODPARM(class_name);

                if (MODPARM(interface_names) && strlen(MODPARM(interface_names)))
                        config->interface_names = MODPARM(interface_names);

                if (MODPARM(iConfiguration) && strlen(MODPARM(iConfiguration)))
                        config->configuration_description.iConfiguration = MODPARM(iConfiguration);

                if (MODPARM(bDeviceClass))
                        config->device_description.bDeviceClass = MODPARM(bDeviceClass);

                if (MODPARM(bDeviceSubClass))
                        config->device_description.bDeviceSubClass = MODPARM(bDeviceSubClass);

                if (MODPARM(bDeviceProtocol))
                        config->device_description.bDeviceProtocol = MODPARM(bDeviceProtocol);

                if (MODPARM(idVendor))
                        config->device_description.idVendor = MODPARM(idVendor);
                else
                        config->device_description.idVendor = CONFIG_OTG_GENERIC_VENDORID;

                if (MODPARM(idProduct))
                        config->device_description.idProduct = MODPARM(idProduct);
                else
                        config->device_description.idProduct = CONFIG_OTG_GENERIC_PRODUCTID;

                if (MODPARM(bcdDevice))
                        config->device_description.bcdDevice = MODPARM(bcdDevice);
                else
                        config->device_description.bcdDevice = CONFIG_OTG_GENERIC_BCDDEVICE;

                if (MODPARM(iManufacturer) && strlen(MODPARM(iManufacturer)))
                        config->device_description.iManufacturer = MODPARM(iManufacturer);
                else
                        config->device_description.iManufacturer = CONFIG_OTG_GENERIC_MANUFACTURER;

                if (MODPARM(iProduct) && strlen(MODPARM(iProduct)))
                        config->device_description.iProduct = MODPARM(iProduct);
                else
                        config->device_description.iProduct = CONFIG_OTG_GENERIC_PRODUCT_NAME;

/*
                if (MODPARM(iSerialNumber) && strlen(MODPARM(iSerialNumber))){
                        config->device_description.iSerialNumber = MODPARM(iSerialNumber);
                }
*/

                if (MODPARM(interface_names))
                        config->interface_names = MODPARM(interface_names);

                generic_register(config, NULL, FALSE, 0, 0, NULL, GENERIC);
                return  0;
        }
}

module_init (generic_cf_modinit);

/*!
 * generic_cf_modexit() - module init
 *
 * This is called by the Linux kernel; when the module is being unloaded
 * if compiled as a module. This function is never called if the
 * driver is linked into the kernel.
 * @return void
 */
void generic_cf_modexit (void)
{
        generic_modexit(generic_cf_configs, GENERIC);

        if (generic_config.registered)
                usbd_deregister_composite_function (&generic_config.composite_driver);

        if (generic_config.interface_list)
                LKFREE(generic_config.interface_list);

	otg_trace_invalidate_tag(GENERIC);

}
#if OTG_EPILOGUE
module_exit (generic_cf_modexit);
#endif

