MSGPSCAN
===========

This project leverages the [ESP32](https://www.espressif.com/en/products/socs/esp32) microcontroller to interface with the [Megasquirt 3]((http://www.msextra.com/) open-source standalone engine controller. It achieves the following:

* CAN Bus Communication: Facilitates seamless communication between the ESP32 and Megasquirt 3 using the CAN protocol.
* GPS Integration: Collects real-time location and navigation data from UBLOX-compatible GPS modules using their binary protocol.
* Data Transmission: Relays GPS information to Megasquirt 3.

## Hardware used
* [ESP-WROOM-32 + Expansion Board](https://www.amazon.com/gp/product/B0B82BBKCY) 
* [HGLRC Mini M100](https://www.amazon.com/gp/product/B0BX65QZJ8) UBLOX compatible 10Hz GPS module
* [SN65HVD230](https://www.amazon.com/gp/product/B07ZT7LLSK) CAN Bus Transceiver 

## WIP
* [ICM-42688](https://www.amazon.com/gp/product/B0CZF512CT) High Precision 6-Axis MEMS MotionTracking Device

