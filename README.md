![fritzing_sketch_bb](https://user-images.githubusercontent.com/8354691/111726589-7b146100-8826-11eb-99d7-6f830656344b.png)
![IMG_20210315_002914](https://user-images.githubusercontent.com/8354691/111726613-84053280-8826-11eb-8790-67b465b992d5.jpg)

**Wireless Controller Adapter for the original XBOX (2001)**

Tools: STM32CubeIDE, NUCLEO-F446RE, USB Host Shield, SSD1306 OLED Screen, Generic USB BT Dongle (e.g. 1Mii USB Bluetooth 4.0)

       1. Use wireless controllers (e.g wireless PS4 controller) on the original XBOX using STM32 + USB Host Shield. (Wireless PS4 is the only controller supported atm)
       2. FreeRTOS for tasks (USB, Bluetooth, display, controls, etc.) 
       3. Very low input lag.
       4. Rumble Support works, even in THPS 2.
       5. Much faster hardware than the OGX360.
       6. USB Host Shield Libraries mostly stock, easy to add new controllers.


**This project uses two USB libraries.**

       A. It uses the USB Host Shield for communicating via BT to the wireless controller.

       B. It uses the ST USB Middleware libraries to communicate with the XBOX.

Most important files outlined below:

       Core\Src\main.cpp (main.c excluded from build, need C++ support).

       Middlewares\ST\STM32_USB_Device_Library\Class\HID\usbd_hid.c (descriptors, callbacks, this is the most important file)

       USB_DEVICE\App\usb_desc.c (USB descriptor here (not HID), this identifies device as an XBOX controller) 

       USB_Host_Shield_2_0\ (USB Host Shield libraries, examples excluded from build as they are for Arduino)
       Arduino_libs\Arduino.h (We have to change the millis, micros, delayMicroseconds with STM32 HAL)
       
Connecting the setup:
       OLED Screen is a SSD1306
       
       The first image made in fritzing shows a V1 USB Host Shield Board, most will have a V2.
       
       Second image shows a v2 board connected.
       
       The USB Host Shield V2 uses the ICSP connector for SPI communication.
       
       To connect the USB Host Shield V2 to the Nucleo board:
       
       
Nucleo | USB_Host_shield
-- | --
D10 | SS
D9 | INT
  |  
D11 | ICSP MOSI
D12 | ICSP MISO
D13 | ICSP SCK
not connected | ICSP VCC
3.3V | ICSP RST
  |  
5v | 5V input under reset button
3.3V | 3.3V input under reset button

Reference image of ICSP:

![d593600bb7b1cd07e141c122db083c03](https://user-images.githubusercontent.com/8354691/108958528-f750c580-7627-11eb-8029-66873d097963.jpg)

Building the Project:

       Open the project with STM32CubeIDE.
       
       Should be able to build right away.
       
       Be careful about changing the configuration with CubeMX. It will destroy the changes made to the USB files. Use gitkraken/source tree to restore those.
       


Special Thanks to:

USB Host Shield team, especially Lauszus and bobbatcomcastdotnet for helping with USB Host Shield Connections.
https://github.com/felis/USB_Host_Shield_2.0/issues/605

OGX360 team 


Useful Resources:

USB Host Shield on STM32: https://github.com/Lauszus/Nucleo_F446RE_USBHost

Getting started with USB HID on STM32: https://notes.iopush.net/blog/2016/stm32-custom-usb-hid-step-by-step-2/

XBOX HID Custom Vendor Requests: https://github.com/Ryzee119/ogx360/blob/master/Firmware/src/xiddevice.c#L122-#L200

Getting data from USB Control Requests: https://iamjustinwang.blogspot.com/2016/07/stm32f401411-usb-device-hid-testcontrol.html
