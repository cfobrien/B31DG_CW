#include<Arduino_FreeRTOS.h>
#include<semphr.h>

SemaphoreHandle_t serial_sem;
SemaphoreHandle_t left_motor_sem, right_motor_sem;
SemaphoreHandle_t diag_mode_sem;
SemaphoreHandle_t roll_avg_sem;

typedef struct roll_avg_t {
  SemaphoreHandle_t roll_avg_sem;
  uint16_t avg;
  uint8_t count;
  uint8_t width;
} roll_avg_t;

typedef struct motor_t {
  SemaphoreHandle_t motor_sem;
  uint16_t max_vel;
  uint16_t warn_vel;
  uint16_t vel;
  uint16_t vel_increment;
} motor_t;

volatile motor_t left_motor, right_motor;

TaskHandle_t sensor_processing_th;
TaskHandle_t left_motor_controller_th;
TaskHandle_t right_motor_controller_th;
TaskHandle_t update_rolling_avg_th;

void task_sensor_processing(void *params);
void task_motor_controller(void *params);

void setup() {
  serial_sem = xSemaphoreCreateMutex();
  left_motor_sem = xSemaphoreCreateMutex();
  right_motor_sem = xSemaphoreCreateMutex();
  roll_avg_sem = xSemaphoreCreateMutex();

  roll_avg_t roll_avg = { .roll_avg_sem = roll_avg_sem, .avg = 0, .count = 0, .width = 6 };
  left_motor = { .motor_sem = left_motor_sem, .max_vel = 1023, .warn_vel = (uint16_t) 0.88*1023, .vel = 0, .vel_increment = (uint16_t) 0.2 * 1023 };
  right_motor = { .motor_sem = right_motor_sem, .max_vel = 1023, .warn_vel = (uint16_t) 0.88*1023, .vel = 0, .vel_increment = (uint16_t) 0.2 * 1023 };
  
  Serial.begin(19200);
  while(!Serial);

  xTaskCreate(task_sensor_processing, "Sensor Processing task", 128, NULL, 1, &sensor_processing_th);
  xTaskCreate(task_motor_controller, "Left Motor Controller", 128, (void *)&left_motor, 1, &left_motor_controller_th);
  xTaskCreate(task_motor_controller, "Right Motor Controller", 128, (void *)&right_motor, 1, &right_motor_controller_th);
  xTaskCreate(task_update_rolling_avg, "Update rolling average", 128, (void *)&roll_avg, 1, &update_rolling_avg_th);
}

void loop() {}

void task_update_rolling_avg(void *params) {
  roll_avg_t * roll_avg;
  uint8_t count;
  TickType_t delay = 500 / portTICK_PERIOD_MS;
  for (;;) {
    roll_avg = (roll_avg_t *)params;
    if (roll_avg->roll_avg_sem != NULL) {
      if (xSemaphoreTake(roll_avg->roll_avg_sem, (TickType_t) 5 == pdTRUE)) {
//        roll_avg->count += 1 % roll_avg->width;
        count = roll_avg->count;
        count += 1 % roll_avg->width;
        count = roll_avg->width;
        roll_avg->count = count;
        xSemaphoreGive(roll_avg->roll_avg_sem);
      }
    }
    if (serial_sem != NULL) {
      if (xSemaphoreTake(serial_sem, (TickType_t) 5 == pdTRUE)) {
        Serial.println(count);
        xSemaphoreGive(serial_sem);
      }
    }
    vTaskDelay(delay); //0.5s busy wait (2Hz rate)
  }
}

void task_motor_controller(void *params) {
  motor_t * motor;
  uint16_t max_vel;
  for (;;) {
    motor = (motor_t *)params;
    if (motor->motor_sem != NULL) {
      if (xSemaphoreTake(motor->motor_sem, (TickType_t) 5 == pdTRUE)) {
        max_vel = motor->max_vel;
        xSemaphoreGive(motor->motor_sem);
      }
    }
    if (serial_sem != NULL) {
      if (xSemaphoreTake(serial_sem, (TickType_t) 5 == pdTRUE)) {
        Serial.println(max_vel);
        xSemaphoreGive(serial_sem);
      }
    }
    vTaskDelay((TickType_t)1000/portTICK_PERIOD_MS);
  }
}

void task_sensor_processing(void *params __attribute__((unused))) {
  TickType_t delay = 5 / portTICK_PERIOD_MS;
  for (;;) {
    vTaskDelay(delay); //5ms busy wait
    vTaskDelay(1); //15ms delay
  }
}
