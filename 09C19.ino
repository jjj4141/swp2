#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0     
#define INTERVAL 25       
#define PULSE_DURATION 10 
#define _DIST_MIN 100     
#define _DIST_MAX 300     

#define TIMEOUT ((INTERVAL / 2) * 1000.0) 
#define SCALE (0.001 * 0.5 * SND_VEL)     

#define _EMA_ALPHA 0.5    

#define N 30

unsigned long last_sampling_time;   
float dist_ema;                     

float samples[N];
int sample_idx = 0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_median;
  
  if (millis() < last_sampling_time + INTERVAL)
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  samples[sample_idx] = dist_raw;
  sample_idx = (sample_idx + 1) % N;

  float buf[N];
  for (int i = 0; i < N; i++) {
    buf[i] = samples[i];
  }

  for (int i = 0; i < N - 1; i++) {
    for (int j = 0; j < N - i - 1; j++) {
      if (buf[j] > buf[j + 1]) {
        float temp = buf[j];
        buf[j] = buf[j + 1];
        buf[j + 1] = temp;
      }
    }
  }

  if (N % 2 == 1) {
    dist_median = buf[N / 2];
  } else {
    dist_median = (buf[(N / 2) - 1] + buf[N / 2]) / 2.0;
  }

  dist_ema = _EMA_ALPHA * dist_median + (1.0 - _EMA_ALPHA) * dist_ema;

  // output the read value to the serial port
  Serial.print("Min:");      Serial.print(_DIST_MIN);
  Serial.print(",raw:");     Serial.print(dist_raw);
  Serial.print(",ema:");     Serial.print(dist_ema);
  Serial.print(",median:");  Serial.print(dist_median);
  Serial.print(",Max:");     Serial.print(_DIST_MAX);
  Serial.println("");

  if ((dist_raw < _DIST_MIN) || (dist_raw > _DIST_MAX))
    digitalWrite(PIN_LED, 1);
  else
    digitalWrite(PIN_LED, 0);

  last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
