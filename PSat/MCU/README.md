# MCU board

# v2.0.x
The MCU board contains an [MSP430FR2355](https://www.ti.com/product/MSP430FR2355) to operate as the 'brains' of the reference stack. The PCB itself breaks out a lot of the GPIO pins for debugging purposes, and includes onboard LEDs too. Programming is done over a Molex picoblade connector, for use with the APSS programming adapter.

The MCU board contains either three separate LEDs (D1 + D2 + D3), or a single RGB LED (D4). The RGB LED is active low.

The MCU board uses three of the MSP's communication buses:
- UCA0: N/A (shared with the MSP programming pins)
- UCA1: GPS UART (exposed to stack)
- UCB0: Sensor I2C (exposed to stack)
- UCB1: Sensor SPI (exposed to stack)

![alt text](../Docs/v2.0pinout.drawio.png)

MCU inputs: MISO, GPS Tx

MCU outputs: MOSI, SCK, LoRa CS, GPS Rx

# v2.1.x
In addition to the contents of the v2.0.x board, the v2.1.x board also contains:
- A [BMP390](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf) barometer / altimeter,
- [ICM-42670-P](https://invensense.tdk.com/wp-content/uploads/2021/07/DS-000451-ICM-42670-P-v1.0.pdf) Inertial Measurement Unit (gyro + accelerometer), 
- Onboard 2MB SPI flash memory IC ([MX25V16066M1I02](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/6165/MX25V1606625V16Mbv14.pdf)), and 
- a microSD card slot.

The SD card is not suitable for use in flight due to the spring-loaded contacts (vibrations in flight), but is an easy way to offload data either after apogee, after landing, or after recovery, depending on your risk tolerance and/or data usage needs. 

The SD card shares the MSP programming pins. Do not flash the MSP while an SD card is inserted.

See [here](https://elm-chan.org/docs/mmc/mmc_e.html) for info on writing to SD cards via SPI. There are many embedded libraries for working with filesystems like FAT32 or exFAT. 

The programming header has changed from picoblade to picoclasp, as picoblade has proved difficult to insert and remove repeatedly.

The MCU board uses all four of the MSP's communication buses:
- UCA0: SD card SPI (shared with the MSP programming pins)
- UCA1: GPS UART (exposed to stack)
- UCB0: Sensor I2C (exposed to stack)
- UCB1: Sensor SPI (exposed to stack)

![alt text](../Docs/v2.1pinout.drawio.png)

MCU inputs: MISO, GPS Tx

MCU outputs: MOSI, SCK, LoRa CS, GPS Rx, GPS Reset, LoRa Reset
