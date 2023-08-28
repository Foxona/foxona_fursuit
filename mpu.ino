// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <FastLED.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define LED_R 12
#define LED_L 32
#define LED_BUILTIN 2

#define PIN_BTN1 0
#define PIN_BTN2 17

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 3  // Popular NeoPixel ring size
CRGB ledsR[NUMPIXELS];
CRGB ledsL[NUMPIXELS];

#define PIN_FAN 4
#define NUM_LED_MODES 4
#define NUM_FAN_MODES 4

#define DELAYVAL 50  // Time (in milliseconds) to pause between pixels


Adafruit_MPU6050 mpu;
bool mpu_available = false;
int iter_id = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(10);
  } else {
    Serial.println("MPU6050 Found!");
    mpu_available = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);  // deg/s
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("");
  }

  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  FastLED.addLeds<NEOPIXEL, LED_R>(ledsR, NUMPIXELS);
  FastLED.addLeds<NEOPIXEL, LED_L>(ledsL, NUMPIXELS);
}

const int btnPins[] = { PIN_BTN1, PIN_BTN2 };
long _last[] = { 0, 0 };
int _prev[] = { 0, 0 };
#define DEBOUNCE_MS 200

bool getBtnState(int btn) {
  auto curState = (digitalRead(btnPins[btn]) == LOW);
  if (!curState) _last[btn] = 0;
  else if (!_last[btn]) _last[btn] = millis();
  return (millis() > _last[btn] + DEBOUNCE_MS);
}

bool onBtnPress(int btn) {
  if (getBtnState(btn)) {
    if (!_prev[btn]) _prev[btn] = iter_id;
  } else _prev[btn] = 0;
  return _prev[btn] == iter_id;
}

void btnLoop(uint8_t* led_mode, uint8_t* fan_mode) {
  if (onBtnPress(0)) {
    *led_mode = (*led_mode + 1) % NUM_LED_MODES;
    Serial.println("Button 0 pressed: switching led mode");
  }
  if (onBtnPress(1)) {
    *fan_mode = (*fan_mode + 1) % NUM_FAN_MODES;
    Serial.println("Button 1 pressed: switching fan mode");
  }
}


void commandsLoop(uint8_t* led_mode) {
  if (Serial.available()) {
    String res = Serial.readStringUntil('\n');
    Serial.write("Read: ");
    Serial.write(res.c_str());
    Serial.write("\n");
    if (res.startsWith("mode=")) {
      auto newmode = res.substring(5).toInt();
      Serial.printf("Switching to mode %d", newmode);
      *led_mode = newmode;
    }
  }
}


// led modes

void rainbow() {
  static uint8_t hue = 0;
  hue += 1;
  // fill_rainbow(ledsR, NUMPIXELS, i, 5);
  CHSV hsv{ hue, 255, 255 };
  CRGB rgb;
  hsv2rgb_rainbow(hsv, rgb);
  FastLED.showColor(rgb);
}

void red() {
  FastLED.showColor({ 255, 0, 0 });
}

void random_color() {
  int r = rand();
  int g = rand();
  int b = rand();
  FastLED.showColor({ r, g, b });
}


void loop_leds(uint8_t led_mode, sensors_event_t& g) {
  auto norm = [](float accel) {
    return uint8_t(abs(accel) * 2);
  };
  static float pos[3]{ 0 };
  pos[0] += g.acceleration.x;
  pos[1] += g.acceleration.y;
  pos[2] += g.acceleration.z;
  const auto SHPEED = -0.3;
  pos[0] += pos[0] < 0 ? -SHPEED : SHPEED;
  pos[1] += pos[1] < 0 ? -SHPEED : SHPEED;
  pos[2] += pos[2] < 0 ? -SHPEED : SHPEED;

  switch (led_mode) {
    case 0:
      rainbow();
      break;
    case 1:
      red();
      break;
    case 2:
      random_color();
      break;
    case 3:
      FastLED.showColor({ norm(pos[2]), norm(pos[1]), norm(pos[0]) });
      break;
    default:
      Serial.printf("Invalid mode %d\n", led_mode);
  }
}

// void print_mpu(sensors_event_t& a, sensors_event_t& g, sensors_event_t& temp) {

//   /* Print out the values */
//   Serial.print(",Acc_X:");
//   Serial.print(a.acceleration.x);
//   Serial.print(",Acc_Y:");
//   Serial.print(a.acceleration.y);
//   Serial.print(",Acc_Z:");
//   Serial.print(a.acceleration.z);
//   // Serial.println(" m/s^2");

//   Serial.print(",Rot_X:");
//   Serial.print(g.gyro.x);
//   Serial.print(",Rot_Y:");
//   Serial.print(g.gyro.y);
//   Serial.print(",Rot_Z:");
//   Serial.print(g.gyro.z);
//   // Serial.println(" rad/s");

//   Serial.print(",Temp:");
//   Serial.print(temp.temperature);
//   // Serial.println(" degC");
//   Serial.println("");
// }

uint8_t led_mode = 0;
uint8_t fan_mode = 0;

void loop() {
  iter_id++;

  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  commandsLoop(&led_mode);
  btnLoop(&led_mode, &fan_mode);

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  if (mpu_available) {
    mpu.getEvent(&a, &g, &temp);
  }
  // Serial.printf("%d %d\n", led_mode, fan_mode);

  // print_mpu(a, g, temp);
  uint8_t fanspeed = fan_mode * 85;
  analogWrite(PIN_FAN, fanspeed);

  loop_leds(led_mode, g);
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  delay(DELAYVAL);
}