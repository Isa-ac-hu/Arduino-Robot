
#include <Arduino.h>

#if defined(ARDUINO_ARCH_AVR)
  #include <Arduino_FreeRTOS.h>
  #include <semphr.h>
  #include <timers.h>
#elif defined(ARDUINO_ARCH_ESP32)
  #include <FreeRTOS.h>
  #include <semphr.h>
  #include <timers.h>
#else
  #include <Arduino_FreeRTOS.h>
  #include <semphr.h>
  #include <timers.h>
#endif

#include <SPI.h>
#include <SD.h>
#include <TMRpcm.h>

// -------------------- Pins --------------------
static const uint8_t PIN_LED_STATUS   = A3;   // status LED for PIR
static const uint8_t PIN_PIR          = 2;    // PIR (has external interrupt on AVR)
static const uint8_t PIN_MOTOR        = A1;   // L293 input (simple on/off)
static const uint8_t PIN_TRIG         = 7;    // Ultrasonic trigger
static const uint8_t PIN_ECHO         = A4;   // Ultrasonic echo (analog pin as digital)
static const uint8_t PIN_RGB_R        = 6;
static const uint8_t PIN_RGB_G        = 5;
static const uint8_t PIN_RGB_B        = 10;
static const uint8_t PIN_POT          = A0;
static const uint8_t PIN_SPEAKER      = 9;
static const uint8_t PIN_SD_CS        = 4;

static const uint8_t PIN_MOTOR_DIR = A1;   // PIN_MOTOR
static const uint8_t PIN_MOTOR_EN  = 3;    // PWM-capable pin

// ---- PID tuning ----
static const uint16_t DIST_TARGET_MM = 280;   
static const float Kp = 1.0f;         
static const float Ki = 0.15f;          
static const float Kd = 0.1f;         
static const uint8_t PWM_MAX = 255;
static const uint8_t PWM_MIN = 0;

pinMode(PIN_MOTOR_DIR, OUTPUT);
pinMode(PIN_MOTOR_EN,  OUTPUT);
digitalWrite(PIN_MOTOR_DIR, HIGH);   // forward by default
analogWrite(PIN_MOTOR_EN, 0); 

// -------------------- RTOS Objects --------------------
static QueueHandle_t qSensors; // SensorFrame queue
static QueueHandle_t qEvents; // Audio/LED events
static SemaphoreHandle_t pirSem; // Given from PIR ISR on rising edge
static TimerHandle_t ledTicker; // Software timer to tick LED breathing

// -------------------- Audio --------------------
TMRpcm tmrpcm;

// -------------------- Types --------------------
enum : uint8_t { EVT_NONE=0, EVT_MOTION, EVT_NEAR_OBSTACLE, EVT_CLEAR_PATH, EVT_PLAY_SUN, EVT_PLAY_BACK };

struct SensorFrame {
  uint16_t distance_mm; // filtered distance in mm
  bool motion; // PIR motion latched since last frame
  uint16_t pot_raw;    
  TickType_t ts;
};

// -------------------- Tuning --------------------
static const uint16_t ULTRA_PERIOD_MS = 25;     // 40 Hz sensor loop
static const uint16_t CONTROL_PERIOD_MS = 20;     // 50 Hz control loop
static const uint16_t LED_PERIOD_MS = 20;     // 50 Hz LED loop (via SW timer tick)
static const uint16_t POT_DEBOUNCE_MS = 40;

static const uint16_t DIST_STOP_MM = 200;   
static const uint16_t DIST_MAX_MM = 2000;   // cap sensor
static const uint8_t  AUDIO_VOL = 5;

// Map pot regions to modes
static const int POT_THRESH_LED = 700;
static const int POT_THRESH_AUDIO = 400;
static const int POT_FORCE_SUN = 100;

// -------------------- State shared via queue --------------------
volatile bool pirEdgeSeen = false;   // set in ISR, consumed in Sensor task

// -------------------- Ultrasonic helper (non-blocking) --------------------
// Minimal non-blocking state machine: trigger pulse then measure echo with timeout
static uint16_t readUltrasonicMM() {
  // trigger 10us pulse
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Use pulseIn with small timeout so we don't block too long (<30ms)
  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 30000UL); // μs
  if (dur == 0) return DIST_MAX_MM;                     // timeout = far
  // distance in mm = time(μs)*0.343/2
  float mm = (dur * 0.343f) / 2.0f;
  if (mm < 0) mm = 0;
  if (mm > DIST_MAX_MM) mm = DIST_MAX_MM;
  return (uint16_t)mm;
}

// -------------------- LED utils --------------------
static void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(PIN_RGB_R, r);
  analogWrite(PIN_RGB_G, g);
  analogWrite(PIN_RGB_B, b);
}

// -------------------- ISR --------------------
void IRAM_ATTR pirISR() {
  BaseType_t xHigherWoken = pdFALSE;
  pirEdgeSeen = true;
  if (pirSem) xSemaphoreGiveFromISR(pirSem, &xHigherWoken);
  portYIELD_FROM_ISR(xHigherWoken);
}

// -------------------- TASK: Sensors --------------------
static void TaskSensors(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();

  uint16_t potPrev = 0;
  TickType_t potLastChange = 0;
  bool pirLatched = false;

  for (;;) {
    // PIR: wait at most one cycle for an edge (doesn't block other sensors)
    if (xSemaphoreTake(pirSem, 0) == pdTRUE || pirEdgeSeen) {
      pirLatched = true;
      pirEdgeSeen = false;
      digitalWrite(PIN_LED_STATUS, HIGH);
    } else {
      digitalWrite(PIN_LED_STATUS, LOW);
    }

    // Ultrasonic
    uint16_t dist = readUltrasonicMM();

    // Potentiometer with debounce 
    uint16_t pot = analogRead(PIN_POT);
    if (abs((int)pot - (int)potPrev) > 10 || (xTaskGetTickCount() - potLastChange) >= pdMS_TO_TICKS(POT_DEBOUNCE_MS)) {
      potPrev = pot;
      potLastChange = xTaskGetTickCount();
    } else {
      pot = potPrev;
    }

    // Ship frame
    SensorFrame f { dist, pirLatched, pot, xTaskGetTickCount() };
    pirLatched = false; // consume the latch
    xQueueSend(qSensors, &f, 0);

    if (dist < DIST_STOP_MM) xQueueSend(qEvents, (void*)&(uint8_t){EVT_NEAR_OBSTACLE}, 0);
    else xQueueSend(qEvents, (void*)&(uint8_t){EVT_CLEAR_PATH}, 0);
    if (pot < POT_FORCE_SUN) xQueueSend(qEvents, (void*)&(uint8_t){EVT_PLAY_SUN}, 0);
    else if (pot < POT_THRESH_AUDIO) xQueueSend(qEvents, (void*)&(uint8_t){EVT_PLAY_BACK}, 0);
    if (xTaskGetTickCount() - last < pdMS_TO_TICKS(ULTRA_PERIOD_MS)) {
      vTaskDelayUntil(&last, pdMS_TO_TICKS(ULTRA_PERIODMS)); // typo fix below
    } else {
      last = xTaskGetTickCount();
      vTaskDelay(pdMS_TO_TICKS(ULTRA_PERIOD_MS));
    }
  }
}
#undef ULTRA_PERIODMS
#define ULTRA_PERIODMS ULTRA_PERIOD_MS

// -------------------- TASK: Control (motor + safety) --------------------
static void TaskControl(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();

  // PID state
  float integral = 0.0f;
  float prevErr  = 0.0f;

  // Simple LPF on distance (damps ultrasonic jitter)
  float distFilt = DIST_MAX_MM;

  SensorFrame f {};
  for (;;) {
    // Always pick up the latest sensor reading if present
    if (xQueueReceive(qSensors, &f, 0) == pdTRUE) {
      // 1st-order low-pass: alpha ~ 0.3 at 50 Hz
      const float alpha = 0.3f;
      distFilt = alpha * f.distance_mm + (1.0f - alpha) * distFilt;
    }

    uint16_t dmm = (uint16_t)distFilt;

    uint8_t pwmCmd = 0;

    if (dmm < DIST_STOP_MM) {
      // Hard-stop zone
      pwmCmd = 0;
      integral = 0;                 
    } else {
      // PID on distance error to shape approach speed
      float err = (float)dmm - (float)DIST_TARGET_MM;   // >0 means far
      float dt  = CONTROL_PERIOD_MS / 1000.0f;

      // Integrate with clamping (anti-windup)
      integral += err * dt;
      integral = constrain(integral, -500.0f, 500.0f);

      float deriv = (err - prevErr) / dt;
      prevErr = err;

      float u = Kp*err + Ki*integral + Kd*deriv;

      // Map controller output to PWM
      if (u <= 0)      pwmCmd = 0;
      else if (u >= PWM_MAX) pwmCmd = PWM_MAX;
      else             pwmCmd = (uint8_t)u;

      if (dmm > 800) pwmCmd = min<uint8_t>(pwmCmd, 220);
    }

    // Apply command
    digitalWrite(PIN_MOTOR_DIR, HIGH);   // forward
    analogWrite(PIN_MOTOR_EN, pwmCmd);

    vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}


// -------------------- TASK: LEDs (RGB animation + distance color) --------------------
static volatile uint8_t ledTick = 0;

static void ledTimerCb(TimerHandle_t) {
  ledTick++;
}

static void TaskLED(void* arg) {
  (void)arg;
  uint8_t r= random(0,255), g= random(0,255), b= random(0,255);
  int8_t dr = +1, dg = +1, db = +1;

  SensorFrame f {};
  uint16_t lastDist = DIST_MAX_MM;

  for (;;) {
    // If there’s a fresh sensor frame, use distance to bias color
    if (xQueueReceive(qSensors, &f, 0) == pdTRUE) {
      lastDist = f.distance_mm;
    }

    // Two modes: “status gradient” by distance + slow breathing animation
    // Distance → red near, green far
    uint8_t nearVal = (lastDist >= 255*2) ? 0 : (uint8_t)max(0, 255 - (int)lastDist/2);
    uint8_t farVal  = (lastDist >= 255*2) ? 255 : (uint8_t)min(255, (int)lastDist/2);

    if (ledTick) {
      ledTick = 0;
      r = constrain((int)r + dr*random(1,6), 0, 255);
      g = constrain((int)g + dg*random(1,6), 0, 255);
      b = constrain((int)b + db*random(1,6), 0, 255);
      if (r==0||r==255) dr = -dr;
      if (g==0||g==255) dg = -dg;
      if (b==0||b==255) db = -db;
    }
    setRGB(nearVal, farVal, b);
    vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
  }
}

// -------------------- TASK: Audio (sole owner of TMRpcm) --------------------
static void TaskAudio(void* arg) {
  (void)arg;
  uint8_t evt = EVT_NONE;

  for (;;) {
    if (xQueueReceive(qEvents, &evt, pdMS_TO_TICKS(20)) == pdTRUE) {
      switch (evt) {
        case EVT_PLAY_SUN:
          tmrpcm.play((char*)"sun.wav");
          break;
        case EVT_PLAY_BACK:
          if (!tmrpcm.isPlaying()) tmrpcm.play((char*)"back.wav");
          break;
        case EVT_NEAR_OBSTACLE:
          break;
        default:
          break;
      }
    }
  }
}

// -------------------- TASK: Logger (optional serial prints) --------------------
static void TaskLogger(void* arg) {
  (void)arg;
  SensorFrame f {};
  for (;;) {
    if (xQueueReceive(qSensors, &f, pdMS_TO_TICKS(100)) == pdTRUE) {
      Serial.print(F("dist(mm)=")); Serial.print(f.distance_mm);
      Serial.print(F("  pot="));     Serial.print(f.pot_raw);
      Serial.print(F("  motion="));  Serial.println(f.motion ? F("1") : F("0"));
    }
  }
}

// -------------------- Arduino setup --------------------
void setup() {
  // IO
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);

  Serial.begin(9600);

  // SD / Audio
  tmrpcm.speakerPin = PIN_SPEAKER;
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println(F("SD init failed"));
  } else {
    tmrpcm.setVolume(AUDIO_VOL);
  }

  // RTOS objects
  qSensors = xQueueCreate(8, sizeof(SensorFrame));
  qEvents = xQueueCreate(8, sizeof(uint8_t));
  pirSem = xSemaphoreCreateBinary();

  // LED timer (breathing tick)
  ledTicker = xTimerCreate("ledTick", pdMS_TO_TICKS(LED_PERIOD_MS), pdTRUE, nullptr, ledTimerCb);
  xTimerStart(ledTicker, 0);

  // PIR interrupt (rising edge = motion)
  attachInterrupt(digitalPinToInterrupt(PIN_PIR), pirISR, RISING);

  xTaskCreate(TaskSensors, "Sensors", 256, nullptr, tskIDLE_PRIORITY + 3, nullptr);
  xTaskCreate(TaskControl, "Control",  256, nullptr, tskIDLE_PRIORITY + 3, nullptr);
  xTaskCreate(TaskLED,     "LED",      192, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  xTaskCreate(TaskAudio,   "Audio",    256, nullptr, tskIDLE_PRIORITY + 2, nullptr);
  xTaskCreate(TaskLogger,  "Logger",   256, nullptr, tskIDLE_PRIORITY + 1, nullptr);

}

void loop() {
}
