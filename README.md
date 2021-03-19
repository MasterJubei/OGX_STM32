**Wireless Controller Adapter for the original XBOX (2001)**

Tools: STM32CubeIDE, NUCLEO-F446RE, USB Host Shield 

       1. Use wireless controllers (e.g wireless PS4 controller) on the original XBOX using STM32 + USB Host Shield.
       2. FreeRTOS for tasks (USB, Bluetooth, display, controls, etc.)
       3. Successfully implemented custom XBOX USB vendor requests.
       4. Much faster hardware than the OGX360.
       5. USB Host Shield Libraries mostly stock, easy to add new controllers.


**This project uses two USB libraries.**

       A. It uses the USB Host Shield for communicating via BT to the wireless controller.

       B. It uses the ST USB Middleware libraries to communicate with the XBOX.

Most important files outlined below:

       Core\Src\main.cpp (main.c excluded from build, need C++ support).

       Middlewares\ST\STM32_USB_Device_Library\Class\HID\ (HID setup, most work done here)
       Middlewares\ST\STM32_USB_Device_Library\Class\HID\usbd_hid.c (OG XBOX USB HID descriptor here, callbacks)

USB_DEVICE\App\usb_desc.c (USB descriptor here (not HID), this identifies device as an XBOX controller) 

       USB_Host_Shield_2_0\ (USB Host Shield libraries, exclude example folder from build, examples are for Arduino)
       Arduino_libs\Arduino.h (We have to change the millis, micros, delayMicroseconds with STM32 HAL)
