# 3_DoF_RC_Car
Arduino based Mecanum wheel robot with ExpressLRS RC link. Using a standard arduino chip with a baud rate of 115200 and 500Hz updates from the ExpressLRS trasmitter, I get ~5 crc failures per minute.

## Parts
- [Amazon listing for Yahboom Omniduino robot](https://www.amazon.com/Yahboom-Programmable-Mecanum-Omnidirectional-Chassis/dp/B0CB3XQ4ZX/)
  - [Yahboom online instructions](http://www.yahboom.net/study/Omniduino#!)
- [RADIOMASTER RP2 ExpressLRS 2.4GHz Nano Receiver w/Fixed SMT Antenna](https://www.amazon.com/RADIOMASTER-ExpressLRS-2-4GHz-Receiver-Antenna/dp/B0BKH4SM98/)

## Arduino_Code
### ExpressLRS CRSF decoding
All packet finding and decoding is performed by passing serial bytes to the `search_for_packet(...)` function.
If a seemingly valid packet is found, it is passed to `process_crsf_packet(...)` where its CRC is checked, then it is processed by message type.
