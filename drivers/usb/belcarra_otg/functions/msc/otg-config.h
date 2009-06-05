/*
 *
 * @(#) balden@belcarra.com|otg/functions/msc/otg-config.h|20060419204258|23307
 *
 */


/*
 * tristate "  Mass Storage Function"
 */
/* #define CONFIG_OTG_MSC */

/*
 * hex "VendorID (hex value)"
 * default "0x15ec"
 */
#define CONFIG_OTG_MSC_VENDORID 0x15ec

/*
 * hex "ProductID (hex value)"
 * default "0xf006"
 */
#define CONFIG_OTG_MSC_PRODUCTID 0xf006

/*
 * hex "bcdDevice (binary-coded decimal)"
 * default "0x0100"
 */
#define CONFIG_OTG_MSC_BCDDEVICE 0x0100

/*
 * string "iManufacturer (string)"
 * default "Belcarra"
 */
#define CONFIG_OTG_MSC_MANUFACTURER "Belcarra"

/*
 * string "iProduct (string)"
 * default "Mass Storage Class - Bulk Only"
 */
#define CONFIG_OTG_MSC_PRODUCT_NAME "Mass Storage Class - Bulk Only"

/*
 * string "MSC Bulk Only iInterface (string)"
 * default "MSC BO Data Intf"
 */
#define CONFIG_OTG_MSC_INTF "MSC BO Data Intf"

/*
 * string "Data Interface iConfiguration (string)"
 * default "MSC BO Configuration"
 */
#define CONFIG_OTG_MSC_DESC "MSC BO Configuratino"

/*
 * bool "  MSC Tracing"
 * default n
 */
#define CONFIG_OTG_MSC_REGISTER_TRACE
