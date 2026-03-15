# MountainCar
This repository provides a simple C/C++ implementation of RL algorithms without using complex libraries.
At this time, SARSA and Q-learning are supported, and they are used to solve the Mountain Car problem.

It currently uses float32 arithmetic, but the goal is to use it to compare the efficiency of float and Posit numbers.
Hence, this program could become an example for a Posit library. However, no such library exists yet for ESP32-C3.

## History / Release notes
### 1. Simulation with Wokwi 
I first started the design to test OLED displays with the [WokWi](https://wokwi.com/projects/455240433632517121) simulator. I tried to use the OLED_Display_SSD1306 library, but ran into some troubles :
 1. That library won't compile as such, the yield added for ESP8266 is defined without #ifdef which makes it break for ESP32
 2. I corrected the library by adding #ifdef and copying the .h and .cpp files in the Wokwi project, sent pull request
 3. Unfortunately, while it compiled and ran, the refresh was incredibly slow ! So I stopped that track.

### 2. Going physical    
Then I selected the pretty nice [ABRobot ESP32-C3 w/ 0.48" I2C OLED](https://github.com/zhuhai-esp/ESP32-C3-ABrobot-OLED/) board. Tiny, but really cheap !
<img src="https://github.com/zhuhai-esp/ESP32-C3-ABrobot-OLED/blob/main/Document/%E5%BC%95%E8%84%9A%E5%9B%BE.png" alt="picture of ABRobot ESP-C3 board" align="middle">
The first iteration didn't include RL, simply 'e' for going left (or reverse) and 'r' for going right.

Instead of blindly using floats, and because someday I want to try Posits, the _track_ is using a mapping of the sin(3x) on the 0-71 range of the OLED screen.
So I first scale the x to smaller numbers by dividing by 40 and removing 1.0. This brings the range to **[-1.0, +0.8[** <br>
Then, since sinx ~ x-x^3/3!, I approximate the sin(3x) by 3x-(3x)^3/6 or 3x * (1-9x^2/6) or 3x * (1-3*(x^2/2)) to limit error propagation. This creates a steep left side, making the chance of success higher than in the original mountain car game. Unfortunately, adding the fifth term *-x^5/5!* flattens the curve so much that it doesn't work anymore.

### 3. Adding Reinformcement Learning
I then added SARSA with the usual -1 reward, and +100/-100 rewards when limits were reached. This was later replaced by the sandard null reward for success. 
