// 과제코드: 20C10
// 적외선 거리센서 자동 curve fitting 프로그램

#define PIN_IR      A0
#define NUM_POINTS  7        // 0, 50, 100, 150, 200, 250, 300 mm (총 7개)
#define STEP_MM     50
#define MAX_DEG     6        // 데이터 7개 기준 최대 6차까지
#define ADC_MAX     1023.0

int   g_method;              // 1: 다항식, 2: 유리함수
int   g_deg;                 // 다항식 차수
int   g_n;                   // 필터링 샘플 개수 n
float g_coef[MAX_DEG + 1];   // 다항식 계수 저장
float g_u0, g_us;            // 정규화(scaling)용 변수
float g_ra, g_rb, g_rc;      // 유리함수 계수 a, b, c

unsigned int g_adc[NUM_POINTS];   // 측정된 센서값
float        g_dist[NUM_POINTS];  // 실제 거리(mm)

int compare(const void *a, const void *b);
unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose);

void setup()
{
  Serial.begin(1000000);
  while (!Serial) ;
}

void loop()
{
  Serial.println();
  Serial.println(F("=== IR sensor curve fitting (20C10) ==="));

  // 1. 피팅 방식, 차수, 샘플 수 입력받기
  g_method = ask(F("Method (1: polynomial, 2: rational a/(x-c)+b)"), 1, 2);
  if (g_method == 1)
    g_deg = ask(F("Polynomial degree (1 ~ 6)"), 1, MAX_DEG);
  g_n = ask(F("Samples per point n (1 ~ 100)"), 1, 100);

  // 2. 0mm ~ 300mm까지 50mm 간격으로 측정
  for (int i = 0; i < NUM_POINTS; i++) {
    g_dist[i] = i * STEP_MM;
    Serial.print(F("["));  Serial.print(i + 1);
    Serial.print(F("/"));  Serial.print(NUM_POINTS);
    Serial.print(F("] Place the ball at "));
    Serial.print((int)g_dist[i]);
    Serial.println(F(" mm, then press Enter."));

    wait_enter();
    g_adc[i] = ir_sensor_filtered((unsigned int)g_n, 0.5, 0); // 중앙값 필터 사용
    Serial.print(F("    sensor = ")); Serial.println(g_adc[i]);
  }

  // 3. curve fitting 계산 후 코드 출력
  bool ok = (g_method == 1) ? fit_polynomial() : fit_rational();
  if (ok)
    print_result();
  else
    Serial.println(F("Curve fitting failed. Check the readings and try again."));

  Serial.println(F("\nPress Enter to restart."));
  wait_enter();
}

// 엔터 입력 대기 및 시리얼 버퍼 비우기
void wait_enter()
{
  while (Serial.available() == 0) ;
  delay(30);
  while (Serial.available() > 0) Serial.read();
}

// lo ~ hi 범위 내의 정수 입력받기
int ask(const __FlashStringHelper *msg, int lo, int hi)
{
  char buf[12];
  while (true) {
    Serial.print(msg); Serial.print(F(": "));

    int len = 0;
    while (Serial.available() == 0) ;
    unsigned long last = millis();
    while (true) {
      if (Serial.available() > 0) {
        char c = Serial.read();
        last = millis();
        if (c == '\n' || c == '\r') break;
        if (len < (int)sizeof(buf) - 1) buf[len++] = c;
      } else if (millis() - last > 100) {
        break;
      }
    }
    delay(30);
    while (Serial.available() > 0) Serial.read();
    buf[len] = '\0';

    int val = atoi(buf);
    if (len > 0 && val >= lo && val <= hi) {
      Serial.println(val);
      return val;
    }
    Serial.print(F("\n  -> Enter a number between "));
    Serial.print(lo); Serial.print(F(" and ")); Serial.println(hi);
  }
}

// 최소제곱 다항식 회귀 (QR 분해로 float 오차 방지)
bool lsq_poly(const float *u, const float *y, int n, int p, float *c)
{
  float A[NUM_POINTS][MAX_DEG + 1];
  float R[MAX_DEG + 1][MAX_DEG + 1] = {0};

  for (int i = 0; i < n; i++) {
    A[i][0] = 1.0;
    for (int j = 1; j < p; j++)
      A[i][j] = A[i][j - 1] * u[i];
  }
  // Gram-Schmidt 직교화
  for (int j = 0; j < p; j++) {
    for (int k = 0; k < j; k++) {
      float dot = 0.0;
      for (int i = 0; i < n; i++) dot += A[i][k] * A[i][j];
      R[k][j] = dot;
      for (int i = 0; i < n; i++) A[i][j] -= dot * A[i][k];
    }
    float norm = 0.0;
    for (int i = 0; i < n; i++) norm += A[i][j] * A[i][j];
    norm = sqrt(norm);
    if (norm < 1e-6) return false;
    R[j][j] = norm;
    for (int i = 0; i < n; i++) A[i][j] /= norm;
  }
  // 후진 대입으로 계수 계산
  for (int j = p - 1; j >= 0; j--) {
    float s = 0.0;
    for (int i = 0; i < n; i++) s += A[i][j] * y[i];
    for (int k = j + 1; k < p; k++) s -= R[j][k] * c[k];
    c[j] = s / R[j][j];
  }
  return true;
}

bool fit_polynomial()
{
  float t[NUM_POINTS];
  float xmin = 1e9, xmax = -1e9;

  for (int i = 0; i < NUM_POINTS; i++) {
    t[i] = g_adc[i] / ADC_MAX;
    if (t[i] < xmin) xmin = t[i];
    if (t[i] > xmax) xmax = t[i];
  }
  if (xmax - xmin < 1e-6) return false;

  // 거듭제곱 시 아두이노 float 정밀도 깨지는 걸 막기 위해 [-1, 1]로 정규화
  g_u0 = (xmin + xmax) / 2.0;
  g_us = 2.0 / (xmax - xmin);
  for (int i = 0; i < NUM_POINTS; i++)
    t[i] = (t[i] - g_u0) * g_us;

  return lsq_poly(t, g_dist, NUM_POINTS, g_deg + 1, g_coef);
}

// 유리함수 y = a / (x - c) + b 의 오차제곱합 계산
float rational_try(float c, float *pa, float *pb)
{
  float z[NUM_POINTS], zm = 0.0, ym = 0.0;

  // z = 1/(x - c) 로 치환해서 선형 회귀
  for (int i = 0; i < NUM_POINTS; i++) {
    z[i] = 1.0 / ((float)g_adc[i] - c);
    zm += z[i];
    ym += g_dist[i];
  }
  zm /= NUM_POINTS;
  ym /= NUM_POINTS;

  float szz = 0.0, szy = 0.0;
  for (int i = 0; i < NUM_POINTS; i++) {
    szz += (z[i] - zm) * (z[i] - zm);
    szy += (z[i] - zm) * (g_dist[i] - ym);
  }
  if (szz < 1e-20) return 1e30;

  float a = szy / szz;
  float b = ym - a * zm;
  float sse = 0.0;
  for (int i = 0; i < NUM_POINTS; i++) {
    float e = g_dist[i] - (a * z[i] + b);
    sse += e * e;
  }
  *pa = a;
  *pb = b;
  return sse;
}

bool fit_rational()
{
  float min_x = g_adc[0];
  for (int i = 1; i < NUM_POINTS; i++)
    if (g_adc[i] < min_x) min_x = g_adc[i];

  float c_hi = min_x - 1.0; // 분모가 0 이하가 되지 않도록 제한
  float best_sse = 1e30, a, b, sse;
  float best_c = 0.0;

  // 1차로 1.0 단위로 대략적인 c 탐색 후 0.05 단위로 정밀 탐색
  for (float c = -500.0; c <= c_hi; c += 1.0) {
    sse = rational_try(c, &a, &b);
    if (sse < best_sse) { best_sse = sse; best_c = c; g_ra = a; g_rb = b; }
  }
  if (best_sse > 1e29) return false;

  float hi = best_c + 1.0;
  if (hi > c_hi) hi = c_hi;
  for (float c = best_c - 1.0; c <= hi; c += 0.05) {
    sse = rational_try(c, &a, &b);
    if (sse < best_sse) { best_sse = sse; best_c = c; g_ra = a; g_rb = b; }
  }
  g_rc = best_c;
  return true;
}

// R^2 계산을 위한 모델 예측값 반환
float model_eval(unsigned int a_value)
{
  if (g_method == 2) {
    float d = (float)a_value - g_rc;
    if (d < 1.0) d = 1.0;
    return g_ra / d + g_rb;
  }
  float t = (a_value / ADC_MAX - g_u0) * g_us;
  float v = g_coef[g_deg]; // Horner's method
  for (int k = g_deg - 1; k >= 0; k--)
    v = v * t + g_coef[k];
  return v;
}

// 계수를 C 코드에 바로 넣을 수 있게 지수 표기법으로 출력
void print_sci(float v)
{
  if (v == 0.0) { Serial.print(F("0.0")); return; }
  if (v < 0.0)  { Serial.print('-'); v = -v; }

  int e = (int)floor(log10(v));
  float m = v / pow(10.0, e);
  if (m >= 10.0) { m /= 10.0; e++; }
  if (m < 1.0)   { m *= 10.0; e--; }
  if (m >= 9.9999995) { m = 1.0; e++; }

  Serial.print(m, 6);
  Serial.print('e');
  if (e >= 0) Serial.print('+');
  Serial.print(e);
}

// 결정계수 R^2 및 volt_to_distance() 함수 코드 출력
void print_result()
{
  float mean = 0.0, ss_res = 0.0, ss_tot = 0.0;
  for (int i = 0; i < NUM_POINTS; i++) mean += g_dist[i];
  mean /= NUM_POINTS;
  for (int i = 0; i < NUM_POINTS; i++) {
    float f = model_eval(g_adc[i]);
    ss_res += (g_dist[i] - f) * (g_dist[i] - f);
    ss_tot += (g_dist[i] - mean) * (g_dist[i] - mean);
  }
  Serial.print(F("\nR^2 = ")); Serial.println(1.0 - ss_res / ss_tot, 5);

  Serial.println(F("\n// ===== Copy this function into your sketch ====="));
  Serial.println(F("float volt_to_distance(unsigned int a_value)   // returns mm"));
  Serial.println(F("{"));

  if (g_method == 2) {
    Serial.print(F("  float d = (float)a_value - (")); print_sci(g_rc); Serial.println(F(");"));
    Serial.println(F("  if (d < 1.0) d = 1.0;"));
    Serial.print(F("  return ")); print_sci(g_ra);
    Serial.print(F(" / d + (")); print_sci(g_rb); Serial.println(F(");"));
  } else {
    Serial.println(F("  float x = a_value / 1023.0;"));
    Serial.print(F("  float t = (x - ")); print_sci(g_u0);
    Serial.print(F(") * "));             print_sci(g_us); Serial.println(';');

    Serial.print(F("  return "));
    for (int k = 0; k < g_deg; k++) {
      print_sci(g_coef[k]);
      Serial.print(F(" + t * ("));
    }
    print_sci(g_coef[g_deg]);
    for (int k = 0; k < g_deg; k++) Serial.print(')');
    Serial.println(';');
  }
  Serial.println(F("}"));
  Serial.println(F("// ================================================"));
}

// --- 이하 20_example_1.ino 필터 함수 그대로 사용 ---
int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  // Eliminate spiky noise of an IR distance sensor by repeating measurement and taking a middle value
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
    Serial.print  ("  => ");

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
