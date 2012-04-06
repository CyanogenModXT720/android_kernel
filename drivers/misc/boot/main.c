#include "types.h"
#include "stdio.h"
#include "error.h"
#include "atag.h"
#include "common.h"
#include "memory.h"
#include "hw_misc.h"
#include "images.h"
#include "dsp.h"
#include "atlas.h"
#include "images.h"

#define ARCH_NUMBER 2196

void critical_error(error_t err) {
  if (console_initialized()) {
    printf("Critical error %d\n", (int)err);
  }
  while (1);
}

void __attribute__((__naked__)) enter_kernel(int zero, int arch, int *atags, int kern_addr) {
    __asm__ volatile (
        "bx r3\n"
    );
}

void __attribute__((__naked__)) enter_mbm(int mbm_addr) {
    __asm__ volatile (
        "bx r0\n"
    );
}

unsigned __attribute__((__naked__)) fix_c1c0(void) {
    __asm__ volatile (
        "mrc p15, 0, r0, c1, c0, 0\n"
	"bic r0, #0x2000\n"
	"mcr p15, 0, r0, c1, c0, 0\n"
	"bx lr\n"
    );
}

char s[20];
int main() {
  struct memory_image image;
  
  image_complete();

  printf("milestone loader rev %s.\n", LDR_VERSION);
  image_dump_stats();
  write32(2, 0x48200010);
  while(!(read32(0x48200014)&1));

  fix_c1c0();

  printf("entering mbm\n");
  enter_mbm(0x8f310000);
  printf("returned from mbm\n");
  printf("going to endless loop\n");
  while(1);

  //dsp_reboot();
  //hw_preboot();

  if (image_find(IMG_LINUX, &image) != NULL) {
    enter_kernel(0, ARCH_NUMBER, atag_build(), KERNEL_DEST);
    //jump_to_linux(image.data, 1024, atag_build());
  } else {
    critical_error(IMG_NOT_PROVIDED);
  }

  return 0;
}
