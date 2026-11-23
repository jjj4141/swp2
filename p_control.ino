#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9
#define PIN_SERVO 10
#define PIN_IR    A0

// Event interval parameters
#define _INTERVAL_DIST    20  // distance sensor interval (unit: ms)
#define _INTERVAL_SERVO   20  // servo interval (unit: ms)
#define _INTERVAL_SERIAL  100 // serial interval (unit: ms)

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.30    // EMA weight of new sample (range: 0 to 1)

// Servo adjustment (실습 3에서 찾은 값)
#define _DUTY_MAX 2030
#define _DUTY_NEU 1230
#define _DUTY_MIN 550

#define _SERVO_ANGLE_DIFF  122  // |D - E| degree
#define _SERVO_SPEED       300  // servo speed (deg/s)

// Target Distance
#define _DIST_TARGET    155 // Center of the rail (unit: mm)

// Sensor offset: B에 공 놓고 측정값 - 155 (지금은 170-155 = 15)
#define _DIST_OFFSET    15.0

// PID parameters
#define _KP  2.0    // proportional gain (실험하며 조정)
//#define _KD 0.0
//#define _KI 0.0

Servo myservo;
float dist_ema;

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

int duty_change_per_interval;
int duty_target, duty_current;

int error_current;
float pterm /*, dterm, iterm */;

void setup()
{
  pinMode(PIN_LED, OUTPUT);

  myservo.attach(PIN_SERVO);

  myservo.writeMicroseconds(_DUTY_NEU - 200);
  delay(3000);
  
  duty_target = duty_current = _DUTY_NEU;
  myservo.writeMicroseconds(duty_current);

  duty_change_per_interval =
    (float)(_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE_DIFF) * (_INTERVAL_SERVO / 1000.0);

  Serial.begin(1000000);

  // EMA 초기값을 0이 아닌 실제 측정값으로 시작 (초반 오동작 방지)
  dist_ema = volt_to_distance(ir_sensor_filtered(30, 0.5, 0));
}

void loop()
{
  unsigned long time_curr = millis();

  if (time_curr >= (last_sampling_time_dist + _INTERVAL_DIST)) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  if (event_dist) {
    float dist_filtered;
    int control;

    event_dist = false;

    dist_filtered = volt_to_distance(ir_sensor_filtered(30, 0.5, 0));
    dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;

    // Update PID variables
    error_current = _DIST_TARGET - dist_ema;   // 오차 = 목표 - 현재 (+: 공이 B보다 가까움)
    pterm = _KP * error_current;               // 비례항 = Kp x 오차

    control = pterm /* + dterm + iterm */;
    duty_target = _DUTY_NEU + control;

    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    if (error_current > 0) digitalWrite(PIN_LED, 1);
    else                   digitalWrite(PIN_LED, 0);
  }

  if (event_servo) {
    event_servo = false;

    if (duty_target > duty_current) {
      duty_current += duty_change_per_interval;
      if (duty_current > duty_target) duty_current = duty_target;
    } else {
      duty_current -= duty_change_per_interval;
      if (duty_current < duty_target) duty_current = duty_target;
    }

    myservo.writeMicroseconds(duty_current);
  }

  if (event_serial) {
    event_serial = false;
    // 시리얼 출력 형식은 템플릿 그대로 유지 (교안 요구사항)
    Serial.print("DIST:");
    Serial.print(dist_ema);
    Serial.print(",_DIST_TARGET:");
    Serial.print(_DIST_TARGET);
    Serial.print(",ERROR:");
    Serial.print(error_current);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_current:");
    Serial.print(duty_current);
    Serial.print(",(");
    Serial.print(duty_change_per_interval);
    Serial.print("),pterm:");
    Serial.println(pterm);
  }
}

float volt_to_distance(int a_value)
{
  // 기존 커브피팅 식 - 오프셋 (재피팅했다면 새 식으로 교체하고 오프셋은 0으로)
  return 644.0 - 2.22*a_value + 1.9E-03*a_value*a_value - _DIST_OFFSET;
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  unsigned int *ir_val, ret_val;
  unsigned int start_time;

  if (verbose >= 2)
    start_time = millis();

  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1))
    return 0;

  if (position == 1.0)
    position = 0.999;

  if (verbose == 1) {
    Serial.print("n: "); Serial.print(n);
    Serial.print(", position: "); Serial.print(position);
    Serial.print(", ret_idx: ");  Serial.println((unsigned int)(n * position));
  }

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;

  if (verbose == 1)
    Serial.print("IR:");

  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
    if (verbose == 1) {
      Serial.print(" ");
      Serial.print(ir_val[i]);
    }
  }

  if (verbose == 1)
    Serial.print("  => ");

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];

  if (verbose == 1) {
    for (int i = 0; i < n; i++) {
      Serial.print(" ");
      Serial.print(ir_val[i]);
    }
    Serial.print(" :: ");
    Serial.println(ret_val);
  }
  free(ir_val);

  if (verbose >= 2) {
    Serial.print("Elapsed time: "); Serial.print(millis() - start_time); Serial.println("ms");
  }

  return ret_val;
}
