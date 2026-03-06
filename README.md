# MountainCar
This repository provides a simple C/C++ implementation of RL algorithms without using complex libraries.
At this time, SARSA and Q-learning are supported, and they are used to solve the Mountain Car problem.

It currently uses float32 arithmetic, but the goal is to use it to compare the efficiency of float and Posit numbers.
Hence, this program could become an example for a Posit library. However, no such library exists yet for ESP32-C3.

## History / Release notes
I first started the design to test OLED displays with the [WokWi](https://wokwi.com/projects/455240433632517121) simulator. I tried to use the OLED_Display_SSD1306 library, but ran into some troubles :
 1. That library won't compile as such, the yield added for ESP8266 is defined without #ifdef which makes it break for ESP32
 2. I corrected the library by adding #ifdef and copying the .h and .cpp files in the Wokwi project, sent pull request
 3. Unfortunately, while it compiled and ran, the refresh was incredibly slow ! So I stopped that track.

Then I selected the pretty nice [ABRobot ESP32-C3 w/ 0.48" I2C OLED](https://github.com/zhuhai-esp/ESP32-C3-ABrobot-OLED/) board. Tiny, but pretty cheap !
<img src="https://github.com/zhuhai-esp/ESP32-C3-ABrobot-OLED/blob/main/Document/%E5%BC%95%E8%84%9A%E5%9B%BE.png" alt="picture of ABRobot ESP-C3 board" align="middle">
