# MDBT40_withLSM9DH
This project for MCU MDBT40, which is base on Nordic IC Nrf51822.

Goal of this project is collect triaxial-accelerometer data from LSM9DS or LIS3DH IC through bluetooth.

# Artitecture 
MDBT40(nrf51822) connect to android or windows10 UWP application send raw data through Bluetooth Low Energy(BlE).
(I have another project for UWP and Android as Gatt Server, could use it connect to MDBT40(nrf51822))

LSM9DS、LIS3DH and Micro SD card connect to MDBT40 through SPI protcol.
![artitecture](https://github.com/lzhengwei/UWP_Nordic_Uart_Transmitter/blob/master/Structure.jpg)

# MDBT40 program Environment
Use Kil uVison to develope MDBT(nrf51822) project.
softdevice : s130_nrf51_2.0.1
SDK : nRF5_SDK_12.3.0

Initial of project is ble_peripheral/ble_app_uart.
Extend other fnction SPI、Timer and FATFS.

# Send/Receive Message through Ble uart
## Send
Ble Uart send function is :

ble_nus_string_send(&m_nus,accdata, 18);

m_nus : BLE nus parameter. Initial project had Declared.
accdata,18 : sending data and length. Accdata is an uint8_t array. Send byte array according the length you set, Max length is 21.Gatt pakage definition limit data length.  

## Receive
Ble Uart Receive event is on 

"nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)"

and receive data is on "p_data" with "length".
p_data is a byte array. 

# SPI function
## Write 
For LSM9DS or LIS3DH, need to set some register first.
uint8_t CTRL_REG1_DATA[2]; // an byte array
CTRL_REG1_DATA[0]=0x20; // Here is register address. For LSM9DS, 0x20 is CTRL_REG1_XM's address.
CTRL_REG1_DATA[1]=0x77; // value to write. For LSM9DS, it is enable accelerometer and set frequency.
spi_master_tx_rx(spi_address, 2, (const uint8_t *)CTRL_REG1_DATA, rx_temp,CSselect);// this function is for write and read in SPI. 
if want to write for SPI, just use this function and It will write CTRL_REG1_DATA value after excute this.

## read
It is same way to read value through SPI
uint8_t rx_temp[2];
uint8_t CTRL_REG1_DATA[2]; // byte array
CTRL_REG1_DATA[0]=0xA0; // Here is register address which you wnat read. For LSM9DS, 0x20 is CTRL_REG1_XM's address, but in read mode need to set first bit,0x20->0xA0.
CTRL_REG1_DATA[1]=0x00; // this is trash code maybe. In read mode dont need this code I guess.
spi_master_tx_rx(spi_address, 2, (const uint8_t *)CTRL_REG1_DATA, rx_temp,CSselect);// this function is for write and read in SPI. 
after excute this function, the vaule will save at "rx_temp[1]" not "rx_temp[0]". I don't know why happen this. 

for LSM9DS or LIS3DH you could declare an byte array for Data-Out register.

Like this "static uint8_t ACC_read_address[6][2]={{0xA9,0xFF},{0xA8,0xFF},{0xAB,0xFF},{0xAA,0xFF},{0xAD,0xFF},{0xAC,0xFF}};"

Then use loop to read sensor data.

for(int i=0;i<6;i++)
	{	
		spi_master_tx_rx(spi_address, 2, (const uint8_t *)ACC_read_address[i], rxx,CSselect);		//CSselect is for LSM9DS to select Acceleromter or gyroscope.
		accdata[i]=rxx[1];	//save raw data at array accdata.
	}
  
  Here just simply introduce some key function at ble-uart and spi.
  
  If you want learn more about all project work or other problems, could send E-mail to contact me.
  
 I'd be glad to help if you don't mind my bad English.


