#include<Arduino_FreeRTOS.h>
#include<semphr.h>
#include<timers.h>

#define configTIMER_TASK_STACK_DEPTH 256//configMINIMAL_STACK_SIZE


/* constants */
#define MAX_VEL 255
#define WARN_VEL 0.88 * MAX_VEL
//#define VEL_INC 0.2 * MAX_VEL
#define VEL_INC 0.1 * MAX_VEL
#define CURR_FACTOR 0.03

#define AX_LEN 10.0f

/* pins */
#define AD0 14
#define AD1 15
#define AD2 16
#define PLUS_LEFT_PIN 10
#define MINUS_LEFT_PIN AD2
#define PLUS_RIGHT_PIN 2
#define MINUS_RIGHT_PIN 3
#define BRAKES_ENABLE_PIN 5
#define DIAGNOSTIC_MODE_PIN 7

#define PWM_A 4
#define PWM_B 11
#define BRA_A 8
#define BRA_B 9
#define DIR_A 12
#define DIR_B 13

#define SENS_PIN_A AD1
#define SENS_PIN_B AD0

/* macros */
#define START_TIMER(timerName, timerFunction) \
  do { \
    timerName = timerFunction; \
  } while (timerName == NULL); \
  timerStarted = false; \
  do { \
    timerStarted = (xTimerStart(timerName, 0) != pdPASS); \
  } while (!timerStarted) \

/* global semaphore declarations */
SemaphoreHandle_t semaphoreMotors;
SemaphoreHandle_t semaphoreSerial;
SemaphoreHandle_t semaphoreInputs;
SemaphoreHandle_t semaphoreControl;
SemaphoreHandle_t semaphoreMaxV;

TimerHandle_t timerWarnVel, timerMonitorVel, timerSensor, timerPrintMaxCurr;
bool timerStarted;

typedef struct motor_t {
  uint8_t vel;
  bool doIncreaseVel;
  bool doDecreaseVel;
} motor_t;

typedef struct pins_t {
  uint8_t plusLeft : 1;
  uint8_t minusLeft : 1;
  uint8_t plusRight : 1;
  uint8_t minusRight : 1;
  uint8_t engageBrakes : 1;
  uint8_t toggleDiagnostic : 1;
} pins_t;

typedef union inputs_t {
  uint8_t port;
  pins_t pins;
} inputs_t;

typedef struct control_t {
  bool diagnosticMode;
  bool brakesEngaged;
} control_t;

typedef struct transform_t {
  float x;
  float y;
  float theta;
} transform_t;

typedef struct vec_t {
  float x;
  float y;
} vec_t;

/* global state */
motor_t leftMotor = { .vel = MAX_VEL/2, .doIncreaseVel = false, .doDecreaseVel = false };
motor_t rightMotor = { .vel = MAX_VEL/2, .doIncreaseVel = false, .doDecreaseVel = false };
control_t control = { .diagnosticMode = false, .brakesEngaged = false };
inputs_t inputs = { .port = 0x00 };

//uint8_t maxCurrCount = 0;

const uint8_t width = 6;
uint8_t count = 0;
uint8_t vals[2*width] = {0};
uint16_t rollSumLeft = 0; uint16_t rollSumRight = 0;

uint16_t maxVA2s, maxVB2s, maxVA, maxVB;

/* setup */
void setup() {
  semaphoreMotors = xSemaphoreCreateMutex();
  semaphoreSerial = xSemaphoreCreateMutex();
  semaphoreControl = xSemaphoreCreateMutex();
  semaphoreInputs = xSemaphoreCreateMutex();
  semaphoreMaxV = xSemaphoreCreateMutex();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PLUS_LEFT_PIN, INPUT_PULLUP);
  pinMode(MINUS_LEFT_PIN, INPUT_PULLUP);
  pinMode(PLUS_RIGHT_PIN, INPUT_PULLUP);
  pinMode(MINUS_RIGHT_PIN, INPUT_PULLUP);
  pinMode(BRAKES_ENABLE_PIN, INPUT_PULLUP);
  pinMode(DIAGNOSTIC_MODE_PIN, INPUT_PULLUP);
  pinMode(SENS_PIN_A, INPUT);
  pinMode(SENS_PIN_B, INPUT);

  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(BRA_A, OUTPUT);
  pinMode(BRA_B, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  
  Serial.begin(19200);
  while(!Serial);

  xTaskCreate(TaskPollInputs, "Poll in", 64, NULL, 0, NULL);
  xTaskCreate(TaskProcessInputs, "Process in", 96, NULL, 0, NULL);
  xTaskCreate(TaskUpdateMaxV, "upd v", 64, NULL, 0, NULL);
  xTaskCreate(TaskPrintMaxCurr, "curr", 96, NULL, 0, NULL);
  xTaskCreate(TaskKinematics, "kine", 128, NULL, 0, NULL);
  START_TIMER(timerMonitorVel, xTimerCreate("vel monitor", pdMS_TO_TICKS(500), pdTRUE, (void *) 0, monitorVelTimerCallback));
  START_TIMER(timerSensor, xTimerCreate("sensors", pdMS_TO_TICKS(20), pdTRUE, (void *) 0, sensorTimerCallback));
  timerWarnVel = xTimerCreate("warn blink", pdMS_TO_TICKS(250), pdTRUE, (void *) 0, warnVelTimerCallback);
  vTaskStartScheduler();
}


void loop() {}


transform_t calc_new_transform(transform_t t, transform_t v, float dt) {
  float r = 0.5f * AX_LEN * (v.x + v.y) / (0.001f + v.y - v.x);
  float wdt = ((v.y - v.x) / AX_LEN) * dt;
  vec_t icc = { .x = t.x - r * sin(t.theta), .y = t.y + r * cos(t.theta) };
  float cwdt = cos(wdt);
  float swdt = sin(wdt);
  float dx = t.x - icc.x;
  float dy = t.y - icc.y;
  return {
    .x = cwdt * dx - swdt * dy + icc.x,
    .y = swdt * dx + cwdt * dy + icc.y,
    .theta = t.theta + wdt
  };
}


void TaskKinematics(void * params) {
  bool doPrint = false;
  const uint16_t periodPrint = 1000;
  uint16_t tLastPrint = 0;
  uint16_t t = 0;
  uint16_t tLastIter = 0;
  transform_t currTf = { .x = 0.0f, .y = 0.0f, .theta = 0.0f };
  transform_t lastTf = { .x = 0.0f, .y = 0.0f, .theta = 0.0f };
  transform_t currVel = { .x = 0.0f, .y = 0.0f, .theta = 0.0f };
  transform_t lastVel = { .x = 0.0f, .y = 0.0f, .theta = 0.0f };
  
  for (;;) {
    tLastIter = t;
    t = millis();

    lastVel = { .x = currVel.x, .y = currVel.y, .theta = currVel.theta };
    if (xSemaphoreTake(semaphoreMotors, (TickType_t) 5) == pdPASS) {
      currVel.x = (float) map(leftMotor.vel, 0, MAX_VEL, -1000, 1000)/1000;
      currVel.y = (float) map(rightMotor.vel, 0, MAX_VEL, -1000, 1000)/1000;
      xSemaphoreGive(semaphoreMotors);
    }

    lastTf = { .x = currTf.x, .y = currTf.y, .theta = currTf.theta };
    currTf = calc_new_transform(currTf, currVel, (float)(t - tLastIter) / 1000.0f);
    
    if ((currVel.x != lastVel.x) || (currVel.y != lastVel.y) || (currVel.theta != lastVel.theta)) {
      // At least one of the velocity values has changed
      doPrint = true;
    } else {
      if (t - tLastPrint >= periodPrint) {
        tLastPrint = millis();
        doPrint = true; // At least 1000ms have passed
      }
    }
    if (doPrint) {
      doPrint = false;
      if (xSemaphoreTake(semaphoreSerial, (TickType_t) 5 == pdTRUE)) {
        Serial.print("lin vel: (");
        Serial.print(currVel.x);
        Serial.print(", ");
        Serial.print(currVel.y);
        Serial.print("), ang vel: ");
        Serial.print(currVel.theta);
        Serial.print(", pos: (");
        Serial.print(currTf.x);
        Serial.print(", ");
        Serial.print(currTf.y);
        Serial.print("), orient : ");
        Serial.println(currTf.theta);
        xSemaphoreGive(semaphoreSerial);
      }
    }
    vTaskDelay(5);
  }
}

void TaskPrintMaxCurr(void * params) {
  bool diagnosticMode = true;  uint16_t va2s, vb2s, va, vb;
  const uint32_t periodMs = 2000;
  uint32_t currTime, lastTime;
  for (;;) {
    currTime = millis();
    if (currTime - lastTime >= periodMs) {
      lastTime = millis();
      if (xSemaphoreTake(semaphoreControl, (TickType_t) 5 == pdTRUE)) {
        diagnosticMode = control.diagnosticMode;
        xSemaphoreGive(semaphoreControl);
      }
      if (diagnosticMode) {
        if (xSemaphoreTake(semaphoreMaxV, (TickType_t) 5 == pdTRUE)) {
          va = maxVA;
          vb = maxVB;
          va2s = maxVA2s;
          vb2s = maxVB2s;
          maxVA2s = 0;
          maxVB2s = 0;
          xSemaphoreGive(semaphoreMaxV);
        }
        if (xSemaphoreTake(semaphoreSerial, (TickType_t) 5 == pdTRUE)) {
          Serial.print("last 2s: (A:");
          Serial.print(va2s * CURR_FACTOR);
          Serial.print(", B:");
          Serial.print(vb2s * CURR_FACTOR);
          Serial.print("), since diag on: (A:");
          Serial.print(va * CURR_FACTOR);
          Serial.print(", B:");
          Serial.print(vb * CURR_FACTOR);
          Serial.println(")");
          xSemaphoreGive(semaphoreSerial);
        }
      }
    }
    vTaskDelay(5);
  }
}


void monitorVelTimerCallback(TimerHandle_t handle) {
  bool diagnosticMode;
  rollSumLeft -= vals[count]; rollSumRight -= vals[count + width];

  if (xSemaphoreTake(semaphoreControl, (TickType_t) 5 == pdTRUE)) {
    diagnosticMode = control.diagnosticMode;
    xSemaphoreGive(semaphoreControl);
  }
  if (xSemaphoreTake(semaphoreMotors, (TickType_t) 5 == pdTRUE)) {
    vals[count] = leftMotor.vel;
    vals[count+width] = rightMotor.vel;
    xSemaphoreGive(semaphoreMotors);
  }
  rollSumLeft += vals[count]; rollSumRight += vals[count + width];
  if (diagnosticMode == true) {
    if (rollSumLeft/width > WARN_VEL && rollSumRight/width > WARN_VEL) {
      xTimerStart(timerWarnVel, 0);
    }
  }
  count = (count + 1) % width;
}


void sensorTimerCallback(TimerHandle_t handle) {
  const TickType_t t = 5 / portTICK_PERIOD_MS;
  vTaskDelay(t);
}


void warnVelTimerCallback(TimerHandle_t handle) {
  const uint8_t expiresAfter = 12;
  uint8_t warnVelCount;
  warnVelCount = (uint8_t) pvTimerGetTimerID(handle);
  if (++warnVelCount > expiresAfter) {
    warnVelCount = 0;
    vTimerSetTimerID(handle, (void *) warnVelCount);
    xTimerStop(handle, 0);
  } else {
    vTimerSetTimerID(handle, (void *) warnVelCount);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}


void TaskUpdateMaxV(void *params) {
  uint16_t va, vb;
  bool diagnosticMode;
  for (;;) {
    if (xSemaphoreTake(semaphoreControl, (TickType_t) 5 == pdTRUE)) {
      diagnosticMode = control.diagnosticMode;
      xSemaphoreGive(semaphoreControl);
    }
    va = digitalRead(SENS_PIN_A);
    vb = digitalRead(SENS_PIN_B);
    if (xSemaphoreTake(semaphoreMaxV, (TickType_t) 5 == pdTRUE)) {
      if (va > maxVA2s) maxVA2s = va;
      if (vb > maxVB2s) maxVB2s = vb;
      if (!diagnosticMode) {
        maxVA = 0;
        maxVB = 0;
      } else {
        if (va > maxVA) maxVA = va;
        if (vb > maxVB) maxVB = vb;
      }
      xSemaphoreGive(semaphoreMaxV);
    }
    vTaskDelay(1);
  }
}


void TaskPollInputs(void *params) {
  inputs_t state = { .port = 0x00 };
  inputs_t prevState = { .port = 0x00 };
  uint32_t prevDebounceTime = 0;
  uint32_t debounceDelay = 50;
  for (;;) {
    prevState.port = state.port;
    state.pins.plusLeft = !digitalRead(PLUS_LEFT_PIN);
    state.pins.minusLeft = !digitalRead(MINUS_LEFT_PIN);
    state.pins.plusRight = !digitalRead(PLUS_RIGHT_PIN);
    state.pins.minusRight = !digitalRead(MINUS_RIGHT_PIN);
    state.pins.engageBrakes = !digitalRead(BRAKES_ENABLE_PIN);
    state.pins.toggleDiagnostic = !digitalRead(DIAGNOSTIC_MODE_PIN);
    
    if (state.port != prevState.port) {
      prevDebounceTime = millis();
    }
    if ((millis() - prevDebounceTime) > debounceDelay) {
      if (state.port != prevState.port) {
        state.port = prevState.port;
      }
    }
    
    if (xSemaphoreTake(semaphoreInputs, portMAX_DELAY) == pdPASS) {
      inputs.port = state.port;
      xSemaphoreGive(semaphoreInputs);
    }
    vTaskDelay(1);
  }
}


void TaskProcessInputs(void *params) {
  bool doIncreaseVelL, doDecreaseVelL;
  bool prevInputIncVelL, prevInputDecVelL;
  bool doIncreaseVelR, doDecreaseVelR;
  bool prevInputIncVelR, prevInputDecVelR;
  bool toggleDiagnostic, brakesEngaged;
  bool prevInputDiag, prevInputBrakes;
  
  for (;;) {
    if (xSemaphoreTake(semaphoreInputs, (TickType_t) 5) == pdPASS) {
      doIncreaseVelL = (!prevInputIncVelL && inputs.pins.plusLeft);
      prevInputIncVelL = inputs.pins.plusLeft;
      doDecreaseVelL = (!prevInputDecVelL && inputs.pins.minusLeft);
      prevInputDecVelL = inputs.pins.minusLeft;
      
      doIncreaseVelR = (!prevInputIncVelR && inputs.pins.plusRight);
      prevInputIncVelR = inputs.pins.plusRight;
      doDecreaseVelR = (!prevInputDecVelR && inputs.pins.minusRight);
      prevInputDecVelR = inputs.pins.minusRight;

      toggleDiagnostic = (!prevInputDiag && inputs.pins.toggleDiagnostic);
      prevInputDiag = inputs.pins.toggleDiagnostic;
      brakesEngaged = inputs.pins.engageBrakes;
      xSemaphoreGive(semaphoreInputs);
    }
    if (xSemaphoreTake(semaphoreControl, (TickType_t) 5) == pdPASS) {
      if (toggleDiagnostic) {
        control.diagnosticMode = !(control.diagnosticMode);
      }
      control.brakesEngaged = brakesEngaged;
      xSemaphoreGive(semaphoreControl);
    }
    if (xSemaphoreTake(semaphoreMotors, (TickType_t) 5) == pdPASS) {
      if (brakesEngaged) {
        leftMotor.vel = MAX_VEL/2;
        rightMotor.vel = MAX_VEL/2;
        digitalWrite(BRA_A, true);
        digitalWrite(BRA_B, true);
      } else {
        if (doIncreaseVelL) {
          if (leftMotor.vel <= MAX_VEL - VEL_INC)
            leftMotor.vel += VEL_INC;
          else
            leftMotor.vel = MAX_VEL;
        }
        if (doDecreaseVelL) {
          if (leftMotor.vel >= VEL_INC)
            leftMotor.vel -= VEL_INC;
          else
            leftMotor.vel = 0;
        }
        if (doIncreaseVelR) {
          if (rightMotor.vel <= MAX_VEL - VEL_INC)
            rightMotor.vel += VEL_INC;
          else
            rightMotor.vel = MAX_VEL;
        }
        if (doDecreaseVelR) {
          if (rightMotor.vel >= VEL_INC)
            rightMotor.vel -= VEL_INC;
          else
            rightMotor.vel = 0;
        }
      }
      digitalWrite(DIR_A, (leftMotor.vel < MAX_VEL/2));
      digitalWrite(DIR_B, (rightMotor.vel < MAX_VEL/2));
      
      analogWrite(PWM_A, leftMotor.vel);
      analogWrite(PWM_B, rightMotor.vel);
      xSemaphoreGive(semaphoreMotors);
    }
  }
  vTaskDelay(1);
}
