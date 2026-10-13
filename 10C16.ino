/*
 * 1. Sigmoid 기반 제어 코드
 * 
#include <Servo.h>
#include <math.h>

#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 180.0
#define _DIST_MAX 360.0

#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)

#define _EMA_ALPHA 0.5
#define _TARGET_DIST 220.0 // 감지 거리 (22cm)

#define _DUTY_MIN 900   // 차단기 닫힘 
#define _DUTY_NEU 2000  // 차단기 열림 

#define HOLDING_TIME 1000 // 차량 통과 후 대기 시간 (1초)

float dist_ema, dist_prev = _DIST_MAX;
unsigned long last_sampling_time;

unsigned long move_start_time = 0;
unsigned long car_passed_time = 0; // 차량 통과 시점 기록
bool is_raising = false;
bool is_lowering = false;
int current_duty = _DUTY_MIN;

Servo myservo;

float get_sigmoid_ratio(float t) {
  float x = -6.0 + 12.0 * t;
  return 1.0 / (1.0 + exp(-x));
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_MIN);

  dist_prev = _DIST_MIN;
  dist_ema = _DIST_MAX;

  Serial.begin(57600);
  last_sampling_time = millis();
}

void loop() {
  float dist_raw, dist_filtered;
  
  if (millis() < last_sampling_time + INTERVAL)
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  if (dist_raw == 0.0 || dist_raw > _DIST_MAX) {
      dist_filtered = _DIST_MAX;
  } else if (dist_raw < _DIST_MIN) {
      dist_filtered = _DIST_MIN;
  } else {
      dist_filtered = dist_raw;
  }
  dist_prev = dist_filtered;

  dist_ema = (_EMA_ALPHA * dist_filtered) + ((1.0 - _EMA_ALPHA) * dist_ema);

  // 1. 차량 접근 감지 및 1초 대기 후 하강 조건 판단
  if (dist_ema <= _TARGET_DIST) {
    digitalWrite(PIN_LED, LOW); // 차량 감지: LED ON
    car_passed_time = millis(); // 차량이 있는 동안 시간 갱신

    if (!is_raising && current_duty < _DUTY_NEU) {
      is_raising = true;
      is_lowering = false;
      move_start_time = millis();
    }
  } else {
    digitalWrite(PIN_LED, HIGH); // 차량 없음: LED OFF

    // 감지 범위를 벗어난 후 HOLDING_TIME(1초) 이상 지났을 때만 하강 시작
    if (millis() - car_passed_time >= HOLDING_TIME) {
      if (!is_lowering && current_duty > _DUTY_MIN) {
        is_lowering = true;
        is_raising = false;
        move_start_time = millis();
      }
    }
  }

  // 2. Sigmoid 부드러운 서보 제어
  unsigned long MOVING_TIME = 1500;
  if (is_raising || is_lowering) {
    unsigned long elapsed = millis() - move_start_time;
    float t = (float)elapsed / (float)MOVING_TIME;

    if (t >= 1.0) {
      t = 1.0;
      if (is_raising) current_duty = _DUTY_NEU;
      if (is_lowering) current_duty = _DUTY_MIN;
      is_raising = false;
      is_lowering = false;
    } else {
      float ratio = get_sigmoid_ratio(t);
      if (is_raising) {
        current_duty = _DUTY_MIN + (int)((_DUTY_NEU - _DUTY_MIN) * ratio);
      } else if (is_lowering) {
        current_duty = _DUTY_NEU - (int)((_DUTY_NEU - _DUTY_MIN) * ratio);
      }
    }

    myservo.writeMicroseconds(current_duty);
  }

  Serial.print("Min:");      Serial.print(_DIST_MIN);
  Serial.print(",Target:");  Serial.print(_TARGET_DIST);
  Serial.print(",dist:");    Serial.print(dist_ema);
  Serial.print(",Duty:");    Serial.print(current_duty);
  Serial.print(",Max:");     Serial.print(_DIST_MAX);
  Serial.println("");
 
  last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}

*/ 





// 2. Cosine 기반 제어 코드
#include <Servo.h>
#include <math.h> // cos(), M_PI 사용을 위해 포함

#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

#define _EMA_ALPHA 0.5    
#define _TARGET_DIST 220.0 // 감지 목표 거리 (22cm)

#define _DUTY_MIN 900   // 차단기 닫힘 (0도)
#define _DUTY_NEU 2000  // 차단기 열림 (90도)

#define HOLDING_TIME 1000 // 차량 통과 후 대기 시간 (1초 = 1000ms)

// global variables
float dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;       // unit: ms

unsigned long move_start_time = 0;
unsigned long car_passed_time = 0; // 차량 통과 시점 기록
bool is_raising = false;
bool is_lowering = false;
int current_duty = _DUTY_MIN;

Servo myservo;

// Cosine 기반 Ease-in-Out 보간 비율 계산 함수 (t: 0.0 ~ 1.0)
float get_cosine_ratio(float t) {
  return (1.0 - cos(M_PI * t)) / 2.0;
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_MIN); // 초기 상태: 차단기 닫힘

  dist_prev = _DIST_MIN;
  dist_ema = _DIST_MAX;

  Serial.begin(57600);
  last_sampling_time = millis(); // setup 시점의 샘플링 타임 정확히 초기화
}

void loop() {
  float dist_raw, dist_filtered;
  
  if (millis() < last_sampling_time + INTERVAL)
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 타임아웃(0.0) 또는 MAX 초과 시 최대 거리(_DIST_MAX)로 처리
  if (dist_raw == 0.0 || dist_raw > _DIST_MAX) {
      dist_filtered = _DIST_MAX;
  } else if (dist_raw < _DIST_MIN) {
      dist_filtered = _DIST_MIN;
  } else {
      dist_filtered = dist_raw;
  }
  dist_prev = dist_filtered;

  // EMA filter
  dist_ema = (_EMA_ALPHA * dist_filtered) + ((1.0 - _EMA_ALPHA) * dist_ema);

  // 1. 차량 접근 감지 및 1초 대기 후 하강 조건 판단
  if (dist_ema <= _TARGET_DIST) {
    digitalWrite(PIN_LED, LOW); // 차량 감지: LED ON
    car_passed_time = millis(); // 차량이 감지되는 동안 시간 계속 갱신

    if (!is_raising && current_duty < _DUTY_NEU) {
      is_raising = true;
      is_lowering = false;
      move_start_time = millis(); // 승강 시작 시간 저장
    }
  } else {
    digitalWrite(PIN_LED, HIGH); // 차량 없음: LED OFF

    // 감지 범위를 벗어난 후 HOLDING_TIME(1초) 지났을 때만 하강 시작
    if (millis() - car_passed_time >= HOLDING_TIME) {
      if (!is_lowering && current_duty > _DUTY_MIN) {
        is_lowering = true;
        is_raising = false;
        move_start_time = millis(); // 하강 시작 시간 저장
      }
    }
  }

  // 2. Cosine 기반 부드러운 서보모터 동작 제어
  unsigned long MOVING_TIME = 1500; // 차단기 동작 시간: 1.5초
  if (is_raising || is_lowering) {
    unsigned long elapsed = millis() - move_start_time;
    float t = (float)elapsed / (float)MOVING_TIME;

    if (t >= 1.0) {
      t = 1.0;
      if (is_raising) current_duty = _DUTY_NEU;  // 도착 시 딱 붙도록 목표치 고정
      if (is_lowering) current_duty = _DUTY_MIN; // 도착 시 딱 붙도록 목표치 고정
      is_raising = false;
      is_lowering = false;
    } else {
      float ratio = get_cosine_ratio(t); // Cosine 함수 적용
      if (is_raising) {
        current_duty = _DUTY_MIN + (int)((_DUTY_NEU - _DUTY_MIN) * ratio);
      } else if (is_lowering) {
        current_duty = _DUTY_NEU - (int)((_DUTY_NEU - _DUTY_MIN) * ratio);
      }
    }

    myservo.writeMicroseconds(current_duty);
  }

  // Serial output
  Serial.print("Min:");      Serial.print(_DIST_MIN);
  Serial.print(",Target:");  Serial.print(_TARGET_DIST);
  Serial.print(",dist:");    Serial.print(dist_ema);
  Serial.print(",Duty:");    Serial.print(current_duty);
  Serial.print(",Max:");     Serial.print(_DIST_MAX);
  Serial.println("");
 
  last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
