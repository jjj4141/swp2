#define PIN_LED 7

unsigned long g_period; // t 수식 오버플로우 방지 위해 long 선언
int g_duty;

void set_period(unsigned long period) {
  g_period = period;
}

void set_duty(int duty) {
  g_duty = duty;
}

void pwm_write() {               // period, duty값에 따른 pwm 펄스 함수
  unsigned long t = 0;
  t = g_period * g_duty / 100;  // led 켜져있는 시간 계산
   
  digitalWrite(PIN_LED, LOW);   // 회로 특성: LOW -> led 켜짐
  if (t > 0) {
    delayMicroseconds(t);
  }

  digitalWrite(PIN_LED, HIGH);  // led 꺼짐
  if (g_period - t > 0) {
    delayMicroseconds(g_period - t);
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  set_period(10000);             // 0.1ms -> 100, 1ms -> 1000, 10ms -> 10000 대입
  set_duty(0);
}

void loop() {
  unsigned long accumulated_time;             // 각 duty에서 누적된 실행 시간
  
  for (g_duty = 0; g_duty<=100; g_duty++) {   // led 0.5초간 밝아짐
    accumulated_time = 0;
    while( accumulated_time < 5000 ) {
      pwm_write();
      accumulated_time += g_period;
    }
  }

  for (g_duty=100; g_duty>=0; g_duty--) {     // led 0.5초간 어두워짐
    accumulated_time = 0;
    while( accumulated_time < 5000 ) {
      pwm_write();
      accumulated_time += g_period;
    }
  }
}
