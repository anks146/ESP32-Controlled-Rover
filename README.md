# ESP32-Controlled-Rover
A rover designed for being controlled using an ESP32 connected to a mobile phone. 
This rover is based on STM32H743ZI2 Nucleo Board which is connected to an ESP32 via UART connection. The user connects to the ESP32 Wi-Fi network via
a mobile/laptop, it can give instructions to the rover to move forward/backword/left or right. The ESP32 will then send the corresponding
instructions to the STM32 via the UART interface and the STM32H743 will do the needful.
This project was a part of my internship project and was completed implemented and successfully tested for all the relevant criterias 
pertaining to the same.

### Files descriptions 
###### Code Folder
The .ioc file uploaded is an STM32CubeMX file which can be used for generation a KEIL IDE project.
The main.c and main.h file is having the actual code for the rover which was implemented
The additional files required are in the Libraries directory in the same repository.
The stm32h7xx_it.c file is the interrupt file generated in the Keil project for the STM32H743 board
The Rover_final.uvprojx is the Keil Project file

##### Library Folder
The files in this folder is used by the main files and for the additional functionalites that are not present from the HAL Library support 
available for the STM32H743ZI2 Nucleo Boards


### NOTE
Although this code will help you generate and implement this project without any problems I recommend that you try to build your 
own project for getting to know the various probles and debugging those problems which will surely help you learn more about embedded progeamming then these codes. 
