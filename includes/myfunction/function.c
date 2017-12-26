#include "function.h"
void SDCtest()
{
		//SDC_init();
		static FATFS fs;
    static FIL file;

    uint32_t bytes_written;
    FRESULT ff_result;
    
		
			
		ff_result = f_mount(&fs, "", 0);
    if (ff_result)
    {
				
        return;
    }
		ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND | FA_OPEN_ALWAYS);
				
		

    if (ff_result != FR_OK)
    {		
			
        return;
    }
		
		ff_result = f_write(&file, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        return;
    }
		(void) f_close(&file);
		return;
		
}
bool Getfile(char* fileName, char *buffT)
{
	static FATFS fs;
	static FIL file;

	uint32_t bytes_written;
	FRESULT ff_result;
	char read[3];
	
	ff_result = f_mount(&fs, "", 0);
	if (ff_result)
  {
		return false;
  }
	ff_result = f_open(&file, fileName, FA_READ | FA_WRITE  | FA_OPEN_ALWAYS);
	if (ff_result)
  {
		return false;
  }
	ff_result = f_read(&file,read, sizeof(read), &bytes_written);
	if (ff_result)
  {
		return false;
  }
	(void) f_close(&file);
	
	for(int i=0;i<sizeof(read);i++)
	{
		*(buffT+i)=read[i];
	}
	return true;
}
bool Writefile(char* fileName, char *buffT,int size)
{
	static FATFS fs;
	static FIL file;

	uint32_t bytes_written;
	FRESULT ff_result;
	char read[3];
	
	ff_result = f_mount(&fs, "", 0);
	if (ff_result)
  {
		return false;
  }
	ff_result = f_open(&file, fileName, FA_READ | FA_WRITE  | FA_OPEN_ALWAYS);
	if (ff_result)
  {
		return false;
  }
	ff_result = f_write(&file,buffT, size, &bytes_written);
	if (ff_result)
  {
		return false;
  }
	(void) f_close(&file);

	return true;
}
