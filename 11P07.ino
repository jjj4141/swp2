#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm) -> 18cm
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm) -> 36cm

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

#define _EMA_ALPHA 0.3    // EMA weight of new sample (range: 0 to 1)

#define _DUTY_MIN 550  // servo full clockwise position (0 degree)
#define _DUTY_NEU ((_DUTY_MIN + _DUTY_MAX) / 2) // servo neutral position (90 degree)
#define _DUTY_MAX 2400 // servo full counterclockwise position (180 degree)

// global variables
float dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;       // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  // LED 초기화
  digitalWrite(PIN_LED, HIGH);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_MIN); // 초기 위치: 0도

  // initialize USS related variables
  dist_prev = _DIST_MIN;
  dist_ema = _DIST_MIN;

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_filtered;
  
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // 초음파 거리 측정 (mm)
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 범위 필터 (Range Filter) - 18cm~36cm 밖이거나 측정 실패(0.0) 시 이전 값 유지
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX) || (dist_raw < _DIST_MIN)) {
      dist_filtered = dist_prev;
  } else {    // 유효 측정 범위 내 (180mm ~ 360mm)
      dist_filtered = dist_raw;
      dist_prev = dist_raw;
  }

  // Modify the below line to implement the EMA equation
  dist_ema = (_EMA_ALPHA * dist_filtered) + ((1.0 - _EMA_ALPHA) * dist_ema);

  // 거리에 비례한 서보모터 제어 (18cm 이하: 0도, 36cm 이상: 180도, 사이 구간 선형 매핑)
  long target_duty = map((long)dist_ema, (long)_DIST_MIN, (long)_DIST_MAX, _DUTY_MIN, _DUTY_MAX);
  target_duty = constrain(target_duty, _DUTY_MIN, _DUTY_MAX);

  myservo.writeMicroseconds(target_duty);

  // LED 제어 (측정 범위 18cm~36cm 내에 들어오면 점등)
  if (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX) {
    digitalWrite(PIN_LED, LOW);  // LED ON
  } else {
    digitalWrite(PIN_LED, HIGH); // LED OFF
  }

  // output the distance to the serial port
  Serial.print("Min:");       Serial.print(_DIST_MIN);
  Serial.print(",dist:");     Serial.print(dist_raw);
  Serial.print(",ema:");      Serial.print(dist_ema);
  Serial.print(",Servo:");    Serial.print(myservo.read());
  Serial.print(",Max:");      Serial.print(_DIST_MAX);
  Serial.println("");
 
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
