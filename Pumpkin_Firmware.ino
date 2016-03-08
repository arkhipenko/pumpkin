/* -------------------------------------
 Firmware for interactive halloween pumpkin - v1.0.0

 Change Log:
  2015-10-XX:
    v1.0.0 - Initial release

*/


// --------------------
// LIBRARIES

#include <DirectIO.h>
#include <DlpFilter.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

#include <PinChangeInt.h>
#include <avr/sleep.h>
#include <avr/power.h>



// ----------------------
// DEFINES

//#define _DEBUG_

#define PIN_MP3_TX            0   // Serial Tx pin on the MP3 board
#define PIN_MP3_RX            1   // Serial Rx pin on the MP3 board
#define PIN_MOTION_ACTIVATION 2   // IRQ pin activated by two motion sensors and a mic. Active-LOW
#define PIN_LED3_RED          3   // RED pin on the 3 color LED (PWM)
#define PIN_LASER1            4   // Activation pin on Laser #1
#define PIN_M1_ENABLE         5   // Enable pin on the L293D chip for eccentric motor #1 (PWM) 
#define PIN_M2_ENABLE         6   // Enable pin on the L293D chip for eccentric motor #2 (PWM) 
#define PIN_LASER2            7   // Activation pin on Laser #2
#define PIN_US_ECHO1          8   // Echo pin for ultrasonic sensor #1
#define PIN_LED3_BLUE         10  // BLUE pin on the 3 color LED (PWM)
#define PIN_LED3_GREEN        11  // GREEN pin on the 3 color LED (PWM)
#define PIN_US_TRIGGER1       12  // Trigger pin for ultrasonic sensor #1
#define PIN_MP3_READY         9  // MP3 IRQ pin for idle/busy playing status check

/** ----------------------
 *  Speed of sound is 340 m/s at sea level
 *    that is
 *    0.002941176 seconds per meter, or
 *    2.941176474 milliseconds per meter, or
 *    2941.176474 microseconds per meter, or
 *    29.41176474 microseconds per centimeter
 *    NEED to multiply by 2 for both ways
 */

#define DIST_FLASH            18000L  // Distance to start flashing lights (~3 m)
#define DIST_SCREAM           6000L  // Distance to go full concert - scream, flash, etc. (1 m)
#define DIST_MAXRANGE         20000L // Max range for timeout (10000 microseconds 
#define DIST_K                128     // Damping factor for low pass filter

// -----------------------
// PINS
Output<PIN_US_TRIGGER1>       pUsTrig1;
Input<PIN_MOTION_ACTIVATION>  pMotion;
AnalogOutput<PIN_LED3_RED>    pRed;
Output<PIN_LASER1>            pLaser1;
AnalogOutput<PIN_M1_ENABLE>   pM1;
AnalogOutput<PIN_M2_ENABLE>   pM2;
Output<PIN_LASER2>            pLaser2;
Input<PIN_US_ECHO1>           pUsEcho1;
AnalogOutput<PIN_LED3_BLUE>   pBlue;
AnalogOutput<PIN_LED3_GREEN>  pGreen;
Input<PIN_MP3_READY>          pMP3Rdy;
Output<13>                    pLED;

// ------------------------
// TYPES

typedef struct {
  bool          busy;
  bool          timeout;
  unsigned long pulseStart;
  unsigned long pulseStop;
  bool          clockStarted;
  unsigned long distance;
} PulseSensor;

// ------------------------
// TASKS
#define TASK_TIMEOUT          30000L  // in millis
#define TASK_DIST_INTERVAL    100
#define TASK_DIST_TIMEOUT     10
#define TASK_FLASH            10000L  // 10 seconds
#define TASK_WINK             12000L  
#define TASK_SCREAM           30000L

Scheduler ts;

Task tMotion(0, 1, &MotionDetectedCallback, &ts);
Task tWink(TASK_WINK, 1, NULL, &ts); // Second run is empty - just allowing a delay before OnDisable is executed
Task tFlash(TASK_FLASH, 2, &FlashCallback, &ts);
Task tScream(TASK_SCREAM, 2, &ScreamCallback, &ts);

Task tTimeout(TASK_TIMEOUT, 1, &TimeoutCallback, &ts);

Task tDistance1(TASK_DIST_INTERVAL, -1, &DistanceTriggerCallback1, &ts);
Task tDistanceTimeout1(TASK_DIST_TIMEOUT, 1, &DistanceTimeoutCallback1, &ts);
Task tDistanceCalc(0, 1, &DistanceCalculate, &ts);

Task tEyes (0, -1, NULL, &ts);
Task tMotor (0, -1, NULL, &ts);
Task tLED (0, -1, NULL, &ts);


// -----------------------
// UTILITY FUNCTIONS

/** LED3 - utility to set levels of RGB on the 3 color led.
 * levels of each color are remembered
 * @param aRed (0 .. 255) - level of red PWM. 0 = off.
 * @param aBlue (0 .. 255) - level of blue PWM
 * @param aGreen (0 .. 255) - level of green PWM
 */

static int sRedLevel, sBlueLevel, sGreenLevel;
void LED3(int aRed, int aBlue, int aGreen) {
  pRed = sRedLevel = aRed & 255;
  pBlue = sBlueLevel = aBlue & 255;
  pGreen = sGreenLevel = aGreen & 255;
}

void LED3Off() {
  LED3(0, 0, 0);
}

/** LED3Step - utility to change levels of RGB on the 3 color led incrementally
 *  changed levels of each color will be remembered
 * @param aDRed (-255 .. 255) - increment (+) or decrement (-) of the red component. The target value will be forced to 0 .. 255 range.
 * @param aDBlue (-255 .. 255) - increment (+) or decrement (-) of the blue component. The target value will be forced to 0 .. 255 range.
 * @param aDGreen (-255 .. 255) - increment (+) or decrement (-) of the green component. The target value will be forced to 0 .. 255 range.
 */
void LED3Step(int aDRed, int aDBlue, int aDGreen) {
  sRedLevel += aDRed; if (sRedLevel < 0) sRedLevel = 0; if (sRedLevel > 255) sRedLevel = 255;
  sBlueLevel  += aDBlue; if (sBlueLevel < 0) sBlueLevel = 0; if (sBlueLevel > 255) sBlueLevel = 255;
  sGreenLevel += aDGreen; if (sGreenLevel < 0) sGreenLevel = 0; if (sGreenLevel > 255) sGreenLevel = 255;
  LED3(sRedLevel, sBlueLevel, sGreenLevel);
}


/** LaserXXX utility functions - control lasers attached to the eyes of the pumpkin
 *  turning them on, off, depending on a aStatusvariable, all on, all off.
*/
void Laser1(bool aStatus) {
  pLaser1 = aStatus ? HIGH : LOW;
}
void Laser2(bool aStatus) {
  pLaser2 = aStatus ? HIGH : LOW;
}

void Laser1On() {
  Laser1(true);
}
void Laser1Off() {
  Laser1(false);
}

void Laser2On() {
  Laser2(true);
}
void Laser2Off() {
  Laser2(false);
}

void LasersOn() {
  Laser1On();
  Laser2On();
}
void LasersOff() {
  Laser1Off();
  Laser2Off();
}

/** MotorXXX - utility to control eccentric motors
 *  Motors are controlled by PWM enabled pins
 *
*/

static int sM1Speed, sM2Speed;
void Motor1(int aSpeed) {
  pM1 = sM1Speed = aSpeed & 255;
}
void Motor2(int aSpeed) {
  pM2 = sM2Speed = aSpeed & 255;
}

void Motor1Stop() {
  Motor1(0);
}
void Motor2Stop() {
  Motor2(0);
}

void MotorsOff() {
  Motor1Stop();
  Motor2Stop();
}

/** MotorXXX - utility to control eccentric motors
 *  Motors are controlled by PWM enabled pins
 *
*/
void MotionDetected() {
  tMotion.restart();
}

void MP3BoardInit() {
  Serial.begin(4800, SERIAL_8N1); //Set to 4800 bps
  Serial.write(0xEF); // Reset board
  delay(100);
  Serial.write(200 + 15); // volume command = 2000 + (0 .. 31)
  delay(100);
}

void MP3Stop() {
  Serial.write(0xEF);
}

void PrepareToSleep() {
  ts.disableAll();

//  All of the below should be covered by the OnDisable functions of the respective Tasks
//  //  LED3(0, 0, 0);
//  //  MotorsOff();
//  //  LasersOff();
//  //  MP3Stop();
  Serial.end();
}

void Sleep() {

  //   * The 5 different modes are:
  //   *     SLEEP_MODE_IDLE         -the least power savings
  //   *     SLEEP_MODE_ADC
  //   *     SLEEP_MODE_PWR_SAVE
  //   *     SLEEP_MODE_STANDBY
  //   *     SLEEP_MODE_PWR_DOWN     -the most power savings

  PrepareToSleep();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  sleep_bod_disable();
  sei();
  PCintPort::attachInterrupt(PIN_MOTION_ACTIVATION, &MotionDetected, FALLING);
  //  power_all_disable();
  sleep_cpu();

  // z..z..z..Z..Z..Z
  
  /* wake up here */
  sleep_disable();
  //  power_adc_enable();
  //  power_spi_enable();
  //  power_twi_enable();
  //  power_usart0_enable();  // this sketch uses UART only, no TWI/SPI
  MP3BoardInit();
}

// ---------------------------
// CALLBACK FUNCTIONS

/** Motion Detected Callback:
 *    1. Activate distance sensing to detect person getting closer
 *    2. Activate timeout timer (put device to sleep in 1 minute if no motion/sound/distance detected)
 *    3. Activate Random Eyes flashing
 *    4. Activate Random Shaking
 *    5. When ultrasound sensor senses person proximity, activate 3 color LED and sound, + more shaking and eyes flashing
 */
void MotionDetectedCallback() {

  tTimeout.restartDelayed();   // Set overall timeout to put device to sleep if inactive
  tDistance1.enableIfNot();    // Start distance measuring

  if ( tScream.isEnabled()  || tFlash.isEnabled() || tWink.isEnabled() ) return;

  tWink.set(TASK_WINK, 1, NULL, &WinkOnEnable, &WinkOnDisable);
  tWink.enable();
}

void TimeoutCallback() {
  Sleep();
}



// ---------------------------
// ULTRASONIC SENSOR 1 CODE


volatile PulseSensor us1;

void DistanceTriggerCallback1() {
#ifdef _DEBUG_
  //Serial.print("In DistanceTriggerCallback1. Echo="); Serial.println(pUsEcho1);
#endif;
  if (us1.busy) return;
  if (pUsEcho1 == HIGH) return;

  us1.busy = true;
  us1.timeout = false;
  pUsTrig1 = LOW;
  delayMicroseconds(4);
  pUsTrig1 = HIGH;

  us1.pulseStart = 0;
  us1.clockStarted = false;

  tDistanceTimeout1.restartDelayed();
  PCintPort::attachInterrupt(PIN_US_ECHO1, &DistanceStartStopClock1, CHANGE);

  //  for statements above should already provide over 10 microseconds of delay
  //  delayMicroseconds(10);
  pUsTrig1 = LOW;
  //  enableInterrupt(PIN_US_ECHO1, &DistanceStartStopClock1, CHANGE);
}

void DistanceStartStopClock1 () {
  if (us1.clockStarted) {
    us1.pulseStop = micros();
    PCintPort::detachInterrupt(PIN_US_ECHO1);
    us1.busy = false;
    tDistanceTimeout1.disable();
    tDistanceCalc.restart();
  }
  else {
    us1.pulseStart = micros();
    us1.clockStarted = true;
  }
}

void DistanceTimeoutCallback1() {
  us1.clockStarted = true;
  DistanceStartStopClock1();
  us1.timeout = true;
}

dlpFilter dF(DIST_MAXRANGE, DIST_K);
void DistanceCalculate() {

  us1.distance = dF.value( us1.timeout ? DIST_MAXRANGE : us1.pulseStop - us1.pulseStart );

  bool busy = tScream.isEnabled();
  if ( busy || us1.distance < DIST_SCREAM ) {
    if ( !busy ) {
      tScream.set(TASK_TIMEOUT, 2, &ScreamCallback, &ScreamOnEnable, &ScreamOnDisable);
      tScream.enableDelayed();
    }
    return;
  }

  busy = tFlash.isEnabled();
  if ( busy || us1.distance < DIST_FLASH ) {
    if ( !busy ) {
      tFlash.set(TASK_FLASH, 2, &FlashCallback, &FlashOnEnable, &FlashOnDisable);
      tFlash.enable();
    }
    return;
  }
  
#ifdef _DEBUG_
  //Serial.print("distance=");Serial.println(us1.distance);
#endif;

}

// ---------------------------------
// STAGE 1 - WINK
//
//  * At this stage general motion is detected

bool WinkOnEnable() {
  tEyes.set(1000 + random(4) * 1000, 1, NULL, &EyesOnEnable, &EyesOnDisable);  // MAX time = 5 sec
  tEyes.restartDelayed(); 
  
  tMotor.set(1000 + random(4000), 2, &ShakeCallback, NULL, &ShakeOnDisable);  // MAX delay is 5 sec
  tMotor.restartDelayed();
  
  return true;
}

void WinkOnDisable() {
  tEyes.disable();
  tMotor.disable();
}


bool EyesOnEnable() {
  switch (random(3)) {
    case 0:
      Laser1On();
      break;
    case 1:
      Laser2On();
      break;
    case 2:
      LasersOn();
      break;
  }
  return true;
}

void EyesOnDisable() {
  LasersOff();
}


void ShakeCallback() {
  if (tMotor.isFirstIteration()) {
    tMotor.setInterval( 1000 + random(2000) );  // MAX delay is 3 sec. Total is 8 seconds
    Motor1(255);
    Motor2(255);
//    switch (random(3)) {
//      case 0:
//        Motor1(200 + random(56));
//        break;
//      case 1:
//        Motor2(200 + random(56));
//        break;
//    }
  }
}

void ShakeOnDisable() {
  MotorsOff();
}


// ---------------------------------
// STAGE 2 - FLASH LED
//
//  * At this stage someone is approaching, so pumpkin will flash the LED

int red, blue, green;

bool FlashOnEnable() {
  tWink.disable(); 
  red = blue = green = 0;
  tTimeout.restartDelayed();   // Set overall timeout to put device to sleep if inactive

  return true;
}

void FlashCallback() {
#ifdef _DEBUG_
  //Serial.print( "FlashCallback=" );Serial.println( tFlash.getRunCounter() );
  //Serial.print( "millis=" );Serial.println( millis() );
#endif;

  if (tFlash.isFirstIteration()) {
    int r  = random(3);
    if (r == 0) red = 10;
    if (r == 1) blue = 10;
    if (r == 2) green = 10;
    tLED.set(50, -1, &LEDGlowCallback, NULL, &LEDGlowOnDisable);
    tLED.enable();
  }
}

void FlashOnDisable() {
  tLED.disable(); 
}


void LEDGlowCallback() {
  LED3Step(red, blue, green);
  red = (sRedLevel == 0 || sRedLevel == 255) ? -red : red;
  blue = (sBlueLevel == 0 || sBlueLevel == 255) ? -blue : blue;
  green = (sGreenLevel == 0 || sGreenLevel == 255) ? -green : green;
}

void LEDGlowOnDisable() {
  LED3(0, 0, 0);
}


// ---------------------------------
// STAGE 3 - SCREAM
//
//  * At this stage someone is reaching down for a candy - flash all read and scream 

bool ScreamOnEnable() {
  tWink.disable();
  tFlash.disable(); 
  blue = green = 0;
  red = 10;
  
  tEyes.set(1000, -1, &EyesOn, NULL, &EyesOnDisable);
  tEyes.enable();

  tLED.set(50, -1, &LEDGlowCallback, NULL, &LEDGlowOnDisable);
  tLED.enable();

  Speak();

  PCintPort::attachInterrupt(PIN_MP3_READY, &MP3Stopped, RISING);

  tTimeout.restartDelayed();   // Set overall timeout to put device to sleep if inactive

  return true;
}

void ScreamCallback() {
  if ( tScream.isFirstIteration() ) {
    tLED.disable();
    
    tMotor.set(1000 + random(2000), 2, &ShakeCallback, NULL, &ShakeOnDisable);
    tMotor.restartDelayed();
    
    tScream.setInterval(5000); // Additional 5 sec pause after action
  }
}

void ScreamOnDisable() {
  MP3Stop();
  PCintPort::detachInterrupt(PIN_MP3_READY);
  tEyes.disable();
  tLED.disable();
  tMotor.disable();
}


void EyesOn () {
  LasersOn();
  tEyes.set(1000, -1, &EyesOff, NULL, &EyesOnDisable);
  tEyes.delay();
  tTimeout.restartDelayed();
}

void EyesOff () {
  LasersOff();
  tEyes.set(500, -1, &EyesOn, NULL, &EyesOnDisable);
  tEyes.delay();
}

void Speak() {
  Serial.write(0x00);
}

void MP3Stopped() {
  tScream.forceNextIteration();
}

// ---------------------------
// MAIN ARDUINO CODE

void setup() {

  power_spi_disable();
  power_twi_disable();

  LED3(255, 0, 0); delay(100);
  LED3(0, 255, 0); delay(100);
  LED3(0, 0, 255); delay(100);
  LED3Off();

  randomSeed(analogRead(A5));
  Sleep();  // immediately sleep and wait for motion
}

void loop() {
  ts.execute();
}





