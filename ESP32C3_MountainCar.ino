/*************************************************************
 * Sketch for testing ESP32-C3 with LCD screen.
 *
 * This sketch Copyright Tochinet 2026, MIT license
 *
 * Tried the proposed OLED_Display_SSD1306 library of WokWi
 * - won't compile, the yield added for ESP8266 is defined without #ifdef
 * - corrected by adding #ifdef and copying .h and .cpp files here
 * - incredibly slow refresh ! ABORT
 * Migrated to ABRobot ESP32-C3 w/ 0.48" I2C OLED and U8G2
 *   https://github.com/zhuhai-esp/ESP32-C3-ABrobot-OLED/ 
 * - U8G2 is supposed very fast (close to lcdgfx)
 * - objective is to demonstrate Mountain Car problem
 *   https://en.wikipedia.org/wiki/Mountain_car_problem
 * - manual acceleration : with E=left, R=right
 *
 ************************************************************/
 
#define LEDPIN 8 // GPIO for LED, to indicate Accelerations
#define SCLPIN 6 // Non standard I2C pins on this board
#define SDAPIN 5
#define SCREEN_WIDTH  72 //
#define SCREEN_HEIGHT 40 //
#define TESTTYPE float

#define NUM_ACTIONS 3 // -1,0,1
#define NUM_TILES  10 // Tiles to discretize position and speed
#define MAXEPISODES 600
#define ALPHA 0.1
#define GAMMA 0.99
#define EPSILON 0.1

#include <Arduino.h>
#include <Wire.h> // FlexWire recommended by WokWi but U8g2 requires Wire

//#include <lcdgfx.h> // dropped for the moment, using U8g2 instead
#include <U8g2lib.h>

// Screen object definition for ABRobot ESP32-C3 w/ 0.42" OLED
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, SCLPIN, SDAPIN);  

// Arrays for height/slope of track
uint8_t trackHeight[SCREEN_WIDTH];
TESTTYPE trackSlope[SCREEN_WIDTH];
TESTTYPE lastY=0; // memory to calculate slope
uint8_t lastAction, action=0; // can be [0,1,2]
TESTTYPE lastVelocity, velocity=0; 
TESTTYPE lastPosition, position=0; // best expressed in x or float?
uint8_t minHeight=0; // to calculate starting position
uint16_t gamesLost=0,gamesWon=0;

// RL arrays and vars
TESTTYPE Q[9][9][NUM_ACTIONS]; // Q table for each S,A tuple
int8_t reward; // bounded to +/-100
uint16_t loopCount=0,loopsNeeded[MAXEPISODES];

// TESTTYPE F[NUM_ACTIONS][NUM_TILES]; // No tiling yet

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-C3, ready to shine !");

  pinMode(LEDPIN, OUTPUT);

  u8g2.begin();   // default I2C address 0x3C OK
  u8g2.enableUTF8Print(); // for accents etc.
  u8g2.setFont(u8g2_font_7x13_tf);    // Medium font with accents
  u8g2.drawStr(0,13,"ESP32-C3");      // write message to internal memory first 
  u8g2.setFont(u8g2_font_ncenB08_tf); // Small font with accents
  u8g2.drawStr(0,22,"Mountain Car");
  u8g2.drawStr(0,31,"expériment");    // accent test
  u8g2.sendBuffer();                  // transfer internal memory to the display */
  delay(2000);
  u8g2.clear();

  u8g2.setDrawColor(2); // Inverting each time
  for(uint8_t x=0; x<SCREEN_HEIGHT/2; x++) {
    u8g2.drawBox(x,x,SCREEN_WIDTH-2*x,SCREEN_HEIGHT-2*x);
    delay(50);
    u8g2.sendBuffer();
  }
  delay(1000);
  u8g2.clear();

  // calculate approximation of sin(3x) for mountain car
  for(uint8_t x=0; x<SCREEN_WIDTH; x++) {
    digitalWrite(LEDPIN, HIGH); // LED OFF
    TESTTYPE y= x/40.0 - 1.0;       // first scaling screen [0,70] to approx [-.8,.6]
    TESTTYPE halfSquare = y*y/2.0; // Intermediate value for x^3 and x^5
    y= 3.0*y*(1.0-3.0*halfSquare/*(1.0-halfSquare)*/) ; // sin(3x) approximation (Taylor sinx = x-x^3/6+x^5/5!) 
    y= 20.0-19.0*y; // scaling sin(3x) to [39,1]
    Serial.print("(");Serial.print(x);Serial.print(",");Serial.print(y);Serial.print(") ");
    if (y<0) y=0;
    trackHeight[x] = y+0.5; // Conversion to closest int
    if (trackHeight[x] > minHeight) {
      minHeight = trackHeight[x]; // minHeight stores highest Y value
      position = x;
      digitalWrite(LEDPIN, LOW); // LED ON
    }
    trackSlope[x] = lastY - y; // negative -(y - lastY)
    if (x % 10 ==0) Serial.println();
    u8g2.drawPixel(x, y); // in memory
    u8g2.sendBuffer();    // transfer internal memory to the display */
    lastY=y;
    delay(100);  
  }
  minHeight = position ; // minheight now stores X value of lowest point for replays
}

void loop() {
  if (loopCount <65535) loopCount++; // Need to avoid overflow
  u8g2.clear(); // clear internal memory _and_ screen
  u8g2.setDrawColor(1); // White
  lastPosition = position;
  lastVelocity = velocity;
  lastAction = action;
  action = 1; // default no acceleration
  reward = -1; // default negative to reduce length
  digitalWrite(LEDPIN, HIGH); // LED OFF
  for(int x=0; x<SCREEN_WIDTH; x++) {
    u8g2.drawPixel(x,trackHeight[x]);
  }
  drawCart(position); // includes sendBuffer
  action = selectAction(position, velocity);
  velocity += updateVelocity(action, position);
  //velocity *= 0.98; // Attenuation, makes it more difficult 
  position += velocity;
  if (position<0 ) { // Reached left boundary 
    gamesLost++;
    reward = -100;
    gameOver("\nGame Lost!");
  }
  if (position>70) { // Reached right boundary 
    gamesWon++;
    reward = +100;
    gameOver("\nGame Won!");
  }
  reward += (int)position/25; // To favorise going right...
  updateQ(lastPosition,lastVelocity, lastAction, reward, position, velocity, action);
} // end of loop()

void drawCart(uint8_t x) {
  uint8_t rear  = trackHeight[x-1] ;
  uint8_t front = trackHeight[x+1] ;
  int8_t dx = front - rear ; // orthogonal to track 
  int8_t dy = -2 ;

  u8g2.drawDisc(x+dx,trackHeight[x]+dy,4);
  u8g2.setCursor(9,12);
  u8g2.print(velocity);
  u8g2.setCursor(42,30);
  u8g2.print(position);
  u8g2.sendBuffer();
}
  
TESTTYPE updateVelocity(uint8_t action, uint8_t x) {
  TESTTYPE delta = 0.1 * (action-1) - 0.25 * trackSlope[x]; 
  return delta; 
}

void gameOver(String Message) {
  Serial.print(Message);Serial.print(": ");
  Serial.print(gamesLost);Serial.print('-');
  Serial.println(gamesWon);
  u8g2.setCursor(10,22);
  u8g2.print(Message);
  u8g2.setCursor(0,39);
  u8g2.print(gamesLost);
  u8g2.setCursor(60,39);
  u8g2.print(gamesWon);
  u8g2.setDrawColor(2); // Inverting each time
  for (uint8_t i=0;i<6;i++) {
    u8g2.drawBox(0,0,SCREEN_WIDTH-1,SCREEN_HEIGHT-1);
      u8g2.sendBuffer();
      delay(1000);
  }
  if (gamesWon+gamesLost < MAXEPISODES) loopsNeeded[gamesWon+gamesLost]=loopCount;
  loopCount=0;
  if (gamesWon+gamesLost == MAXEPISODES) {
    Serial.print("Loop counts : ");
    long loopsAveraged =0;
    for (int i=0;i<MAXEPISODES;i++) { // Forget about zero on its own line
      loopsAveraged += loopsNeeded[i];
      Serial.print(loopsNeeded[i]);
      Serial.print(' ');
      if ((i+1)%10==0) {
        Serial.print("avg: ");
        Serial.println(loopsAveraged);
        loopsAveraged=0;
      }
    }
    for(;;); // STOP 
  }
  position = minHeight;
  velocity = 0;
}

TESTTYPE getQ(TESTTYPE position, TESTTYPE velocity, uint8_t action) {
  uint8_t intPos = (uint8_t)position >>3; // 0-71 -> 0-8
  velocity=constrain(velocity, -4.0, 4.0);
  uint8_t intVel = (uint8_t)(velocity+4.0); // +/-4 -> 0-8
  return Q[intPos][intVel][action];
}

uint8_t selectAction(TESTTYPE position, TESTTYPE velocity) {
  uint8_t bestAction=1; // default still
  TESTTYPE maxQ = getQ(position, velocity, 0);
  if (Serial.available()) {
    digitalWrite(LEDPIN, LOW); // LED ON
    if (Serial.peek()=='e') {
      Serial.read(); return(0); // go left
    }
    if (Serial.read()=='r') return(2); // go right
  }
  if (random(100)/100.0<EPSILON) { // Issue with random() with no arguments ???
    digitalWrite(LEDPIN, LOW); // LED ON
    return random(NUM_ACTIONS);
  }
  for (action = 1; action< NUM_ACTIONS;action++) {
    TESTTYPE currentQ = getQ(position, velocity, action);
    if (currentQ > maxQ) {
      bestAction = action;
      maxQ=currentQ;
    }
  }
  return(bestAction);
}

void updateQ (TESTTYPE position, TESTTYPE velocity, uint8_t action, int8_t reward, TESTTYPE nextPos, TESTTYPE nextVel, uint8_t nextAction) { 
  uint8_t intPos = (uint8_t)position >>3;
  velocity=constrain(velocity, -4.0, 4.0);
  uint8_t intVel = (uint8_t)(velocity+4.0); // +/-4 -> 0-8
  TESTTYPE maxQ = max(Q[intPos][intVel][0],max(Q[intPos][intVel][1],Q[intPos][intVel][2])); // TODO improve for NUM_ACTIONS !=2
  //Q-Learning
  //Q[intPos][intVel][action] += ALPHA *(reward + GAMMA*getQ(nextPos,nextVel,nextAction) - Q[intPos][intVel][action]);
  //SARSA
  Q[intPos][intVel][action] += ALPHA *(reward + GAMMA*getQ(nextPos,nextVel,nextAction) - Q[intPos][intVel][action]);
  Serial.print(position);Serial.print(' ');
  Serial.print(velocity);Serial.print(", ");
  Serial.print(action);Serial.print(",  ");
  Serial.print(Q[intPos][intVel][action]);Serial.print(",   ");
  Serial.print(intPos);Serial.print(' ');
  Serial.print(intVel);Serial.print(' ');
  Serial.println(reward); // */
}
