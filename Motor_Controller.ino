#include<Arduino_FreeRTOS.h>
#include<semphr.h>

TickType_t ms;

SemaphoreHandle_t left_motor_sem;
SemaphoreHandle_t right_motor_sem;
SemaphoneHandle_t diag_mode_sem;

typedef struct motor_t {
  uint8_t max_vel;
  uint8_t warn_vel;
  uint8_t vel;
  uint8_t vel_increment;
} motor_t;

TaskHandle_t sensor_processing_th;
TaskHandle_t left_motor_controller_th;
TaskHandle_t right_motor_controller_th;

void task_sensor_processing(void *params);
void task_motor_controller(void *params);

void setup() {
  ms = portTICK_PERIOD_MS;

  Serial.begin(9600);
  while(Serial == NULL);

  motor_t left_motor = { .max_vel = 1023, .warn_vel = 0.88*1023, .vel = 0, .vel_increment = 0.2 * 1023 };
  motor_t right_motor = { .max_vel = 1023, .warn_vel = 0.88*1023, .vel = 0, .vel_increment = 0.2 * 1023 };

  xTaskCreate(
    task_sensor_processing,
    "Sensor Processing task",
    128,
    NULL,
    1,
    &sensor_processing_th);

  xTaskCreate(
    task_motor_controller,
    "Left Motor Controller",
    128,
    NULL,
    1,
    &left_motor_controller_th);

  xTaskCreate(
    task_motor_controller,
    "Right Motor Controller",
    128,
    NULL,
    1,
    &right_motor_controller_th);
}

void loop() {}

void task_sensor_processing(void *params __attribute__((unused))) {
  TickType_t delay = 1000 / portTICK_PERIOD_MS;
  for (;;) {
    Serial.println("test");
    vTaskDelay(delay); //5ms busy wait
    vTaskDelay(1); //15ms delay
  } 
}

void task_motor_controller(void *params __attribute__((unused))) {
  for (;;) {
    if (xSemaphoreTake(left_motor_sem, (TickType_t) 5 == pdTRUE)) {
      // Access motor struct
      xSemaphoreGive(left_motor_sem);
    }
    vTaskDelay(1);
  }
}
