Wireless Controller Adapter for the original XBOX (2001)

Tools: STM32CubeIDE, NUCLEO-F446RE, USB Host Shield 

1. Use wireless controllers (e.g wireless PS4 controller) on the original XBOX using STM32 + USB Host Shield.
2. FreeRTOS for managing the different tasks (USB, Bluetooth, display, controls, etc.)
3. Successfully implemented custom XBOX USB vendor requests. (We must intercept these)
4. Much faster hardware than the OGX360 (another similar project) but that uses AVR and is much more expensive.
5. We also benefit from keeping the USB Host Shield libraries stock, so we have much more wireless controller support. 

Primary Layout + Files: 
1. This project uses two USB libraries.
A. It uses the USB Host Shield for communicating via BT to the wireless controller.

B. It uses the ST USB Middleware libraries to communicate with the XBOX. Unfortunately, the ST library is quite barebones, so we must implement a lot of the USB functionality ourselves. ST’s libraries are documented here. 

Most important files outlined below:
1.    Core\Src\main.cpp (main.c excluded from build, need C++ support).

2.    Middlewares\ST\STM32_USB_Device_Library\Class\HID\ (Hid setup, most work done here)
       Middlewares\ST\STM32_USB_Device_Library\Class\HID\usbd_hid.c (OG XBOX USB HID descriptor here, callbacks)

USB_DEVICE\App\usb_desc.c (USB descriptor here (not HID), this identifies device as an XBOX controller) 

3.    USB_Host_Shield_2_0\ (USB Host Shield libraries, exclude example folder from build, examples are for Arduino)
       Arduino_libs\Arduino.h (We have to change the millis, micros, delayMicroseconds with STM32 HAL)
