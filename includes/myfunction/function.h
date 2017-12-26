#include "nrf.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

#define FILE_NAME   "Test.TXT"
#define TEST_STRING "SD card example.\r\n"

void SDCtest();
bool Getfile(char* fileName, char *buffT);
bool Writefile(char* fileName, char *buffT,int size);	