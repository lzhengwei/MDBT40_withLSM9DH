
#include "spi_example.h"

static void spi_master_init(spi_master_hw_instance_t   spi_master_instance )
{
    uint32_t err_code = NRF_SUCCESS;
 
    // Configure SPI master.
    spi_master_config_t spi_config = SPI_MASTER_INIT_DEFAULT;
    spi_config.SPI_CONFIG_CPHA=SPI_CONFIG_CPHA_Leading ;
    spi_config.SPI_CONFIG_CPOL=SPI_CONFIG_CPOL_ActiveHigh;
    spi_config.SPI_Pin_SCK  = SPIM0_SCK_PIN;
    spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
    spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
    spi_config.SPI_Pin_SS   = SPIM0_SS_PIN;
    spi_config.SPI_Freq     = SPI_FREQUENCY_FREQUENCY_M4;
    spi_config.SPI_CONFIG_ORDER = SPI_CONFIG_ORDER_MsbFirst;
    spi_config.SPI_PriorityIRQ = APP_IRQ_PRIORITY_LOW;
    spi_config.SPI_DisableAllIRQ = 0;
    err_code = spi_master_open(spi_master_instance, &spi_config);
    APP_ERROR_CHECK(err_code);
}
 
static void spi_send_recv(const spi_master_hw_instance_t spi_master_hw_instance, uint8_t * const p_tx_data, uint8_t * const p_rx_data, const uint16_t len)
{
    
    // Start transfer.
  spi_master_init(spi_master_hw_instance);
    uint32_t err_code = spi_master_send_recv(spi_master_hw_instance, p_tx_data, len, p_rx_data, len);
    APP_ERROR_CHECK(err_code);
  for (int i = 0 ; i < 1000 ; i++ ) {
   if ( spi_master_get_state(spi_master_hw_instance) != SPI_MASTER_STATE_BUSY )
    break;
  }
  spi_master_close(spi_master_hw_instance);
}
 
bool SPI_Write( uint8_t register_address, uint8_t *value, uint8_t number_of_bytes )
{
  #define i2c_write_data_len 6
 
  uint8_t w2_data[i2c_write_data_len+1];
  uint8_t r2_data[i2c_write_data_len+1];
  uint8_t i;
 
  if(number_of_bytes > 0x01)
  {
   register_address |= (uint8_t)MULTIPLEBYTE_CMD;
  }
 
  w2_data[0] = register_address;
  for ( i = 0 ; i < number_of_bytes ; i++ ) {
   w2_data[i+1] = value[i];
  }
 
  spi_send_recv(SPI_MASTER_0, w2_data, r2_data, number_of_bytes +1);
 
                return true;
}
 
bool SPI_Read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
 
  #define i2c_read_data_len 8
 
  uint8_t w2_data[i2c_write_data_len+1];
  uint8_t r2_data[i2c_write_data_len+1];
  uint8_t i;
 
  if(number_of_bytes > 0x01)
  {
   register_address |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
   register_address |= (uint8_t)READWRITE_CMD;
  }
 
  w2_data[0] = register_address;
  spi_send_recv(SPI_MASTER_0, w2_data, r2_data, number_of_bytes+1);
 
  for( i = 0 ; i < number_of_bytes ; i++ ) {
   destination[i] = r2_data[i+1];
  }
 
                return true;
}