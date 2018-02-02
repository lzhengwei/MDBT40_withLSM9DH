# MDBT40_withLSM9DH
This project for MCU MDBT40, which is base on Nordic IC Nrf51822.

Goal of this project is collect triaxial-accelerometer data from LSM9DH or LIS3DH IC through bluetooth.

# Artitecture 
MDBT40(nrf51822) connect to android or windows10 UWP application send raw data through Bluetooth Low Energy(BlE).
LSM9DH、LIS3DH and Micro SD card through SPI connect to MDBT40.
![artitecture](https://github.com/lzhengwei/UWP_Nordic_Uart_Transmitter/blob/master/Structure.jpg)

# MDBT40 code

