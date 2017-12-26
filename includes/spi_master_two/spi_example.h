#include "app_util_platform.h"
#include "bsp.h"
#include "spi_master.h"

bool SPI_Write( uint8_t register_address, uint8_t *value, uint8_t number_of_bytes );
bool SPI_Read( uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes );
