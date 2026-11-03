#include <Servo.h>

#define PIN_SERVO 10

#define _DUTY_MIN 550  // 0도 위치의 펄스 폭 (us)
#define _DUTY_NEU 1475 // 90도 위치의 펄스 폭 (us)
#define _DUTY_MAX 2400 // 180도 위치의 펄스 폭 (us)

// [도전과제 13C02 설정]
// 1) 60초/180도 이동 실험 : _SERVO_SPEED 3.0  , TARGET_ANGLE 180.0
// 2) 300초/90도 이동 실험 : _SERVO_SPEED 0.3  , TARGET_ANGLE 90.0
#define _SERVO_SPEED 3.0   // 각속도 (deg/sec)
#define TARGET_ANGLE 180.0 // 목표 이동 각도 (deg)

#define INTERVAL 20        // 서보 제어 및 위치 갱신 주기 (20ms = 50Hz)

Servo myservo;

float duty_change_per_interval; // 20ms 간격당 이동해야 할 펄스 폭 변화량 (float 필수)
float duty_curr;                // 현재 위치 펄스 폭 (정밀 누적 계산용 float)
float duty_target;              // 목표 위치 펄스 폭

unsigned long last_sampling_time;

void setup() {
  Serial.begin(57600);
  myservo.attach(PIN_SERVO);

  //  매 INTERVAL(20ms) 마다 변경해야 할 duty 계산
  //  (float) 형변환을 적용하여 정수 버림(truncation)으로 인한 속도 오차 방지
  duty_change_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * ((float)INTERVAL / 1000.0);

  // 초기 시작 위치(0도) 및 목표 각도에 따른 target duty 산출
  duty_curr = _DUTY_MIN;
  duty_target = _DUTY_MIN + (_DUTY_MAX - _DUTY_MIN) * (TARGET_ANGLE / 180.0);

  // 초기 위치로 서보 이동 및 시간 기록
  myservo.writeMicroseconds((int)duty_curr);
  last_sampling_time = millis();
}

void loop() {
  unsigned long time_curr = millis();

  // 설정한 INTERVAL(20ms) 주기마다 아래 로직 실행
  if (time_curr < (last_sampling_time + INTERVAL))
    return;

  last_sampling_time += INTERVAL;

  // 목표 위치(duty_target)를 향해 duty_change_per_interval만큼 미세 증감
  if (duty_curr < duty_target) {
    duty_curr += duty_change_per_interval;
    if (duty_curr > duty_target)
      duty_curr = duty_target;
  } else if (duty_curr > duty_target) {
    duty_curr -= duty_change_per_interval;
    if (duty_curr < duty_target)
      duty_curr = duty_target;
  }

  // 실제로 계산된 펄스 폭 신호를 서보모터에 전달 (int 형변환)
  myservo.writeMicroseconds((int)duty_curr);

  // 시리얼 모니터 및 플로터 출력
  Serial.print("Target:"); Serial.print(duty_target);
  Serial.print(",Current:"); Serial.print(duty_curr);
  Serial.print(",ChangePerInterval:"); Serial.println(duty_change_per_interval);
}
