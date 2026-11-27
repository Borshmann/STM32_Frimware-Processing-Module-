#ifndef PLATA_H
#define PLATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * Plata — отладочный контроллер усилия на STM32F103C8T6.
 *
 * Аппаратная конфигурация:
 *   - тензодатчик YZC-133 + NAU7802 на I2C1 (второй канал — I2C2);
 *   - шаговый двигатель NEMA17 + MKS SERVO42D на USART2/RS485;
 *   - внешний пульт/терминал на USART1/RS485.
 *
 * Файл содержит общие константы, типы и прототипы прикладных модулей.
 * Код разделён на:
 *   main.c    — HAL, тактирование, RS485, retarget printf;
 *   sensor.c  — NAU7802: чтение, тара, калибровка;
 *   motor.c   — MKS SERVO42D: кадры, режимы, скорость, позиция;
 *   control.c — демо-связка «тензодатчик → каретка»;
 *   app.c     — CLI, команды, автоматический запуск AUTO.
 */

/* Версия прошивки. Метка историческая: функционально это уже демо с AUTO. */
#define FW_VERSION            "0.5-step3"

/* Целевой рабочий диапазон усилия для будущего PI-регулятора, граммы. */
#define TARGET_MIN_G          100
#define TARGET_MAX_G          600

/*
 * Ограничения ручного JOG.
 * JOG_MAX_MS ограничивает длительность одной команды, чтобы случайно
 * не отправить мотор в длительное движение.
 */
#define JOG_MAX_RPM           30
#define JOG_MAX_MS            5000U
#define JOG_DEFAULT_MS        2000U

/* Максимальная длина строки CLI (без учёта завершающего '\0'). */
#define CLI_LINE_MAX          96U

/* Код режима SR_vFOC в протоколе MKS (команда 82H). */
#define MOTOR_MODE_SR_VFOC    0x05U

/*
 * Состояние одного канала тензодатчика NAU7802.
 *
 * tare_raw       — сырой отсчёт при нулевой нагрузке (тара);
 * counts_per_g   — коэффициент калибровки: сколько counts на 1 грамм;
 * tared          — тара выполнена;
 * calibrated     — коэффициент известен;
 * present        — чип обнаружен на шине;
 * initialized    — чип инициализирован и готов к чтению.
 */
typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t present;
  uint8_t initialized;
  uint8_t revision;
  uint8_t tared;
  uint8_t calibrated;
  int32_t tare_raw;
  float counts_per_g;
} Sensor;

/*
 * Состояние одного драйвера MKS SERVO42D.
 *
 * huart   — UART, к которому подключён RS485 драйвера;
 * de_port — порт пина DE/RE передатчика RS485;
 * de_pin  — номер пина DE/RE;
 * addr    — адрес драйвера на шине (сейчас 1).
 */
typedef struct
{
  UART_HandleTypeDef *huart;
  GPIO_TypeDef *de_port;
  uint16_t de_pin;
  uint8_t addr;
} Motor;

/* Общее состояние приложения (пока используется только IDLE). */
typedef enum
{
  APP_STATE_IDLE = 0,
  APP_STATE_READY,
  APP_STATE_RUN,
  APP_STATE_FAULT
} AppState;

/*
 * Состояние демо-контура «усилие → скорость каретки».
 *
 * home_pos / far_pos     — крайние точки рабочего хода в counts энкодера;
 * max_travel             — длина хода в counts;
 * home_margin/far_margin — программные запасы до краёв;
 * encoder_sign           — знак пересчёта позиции в направление «от home»;
 * motor_sign             — знак команды скорости «от home»;
 * gain_rpm_per_g         — прибавка скорости на грамм сверх deadband;
 * max_rpm                — ограничение скорости движения;
 * return_rpm             — скорость возврата к home;
 * deadband_g             — порог начала движения;
 * release_g              — порог возврата;
 * max_force_g            — аварийный порог перегрузки, граммы;
 * read_failures          — счётчик подряд идущих ошибок чтения;
 * jog_failures           — счётчик подряд идущих ошибок команды скорости.
 *
 * Счётчики нужны, чтобы краткие сбои сразу после homing не выключали демо.
 */
typedef struct
{
  Sensor *sensor;
  Motor *motor;

  uint8_t active;
  uint8_t home_valid;
  uint8_t far_valid;
  uint8_t limits_valid;
  uint8_t read_failures;
  uint8_t jog_failures;

  int32_t home_pos;
  int32_t far_pos;
  int32_t max_travel;
  int32_t home_margin;
  int32_t far_margin;
  int8_t encoder_sign;
  int8_t motor_sign;

  float gain_rpm_per_g;
  int16_t max_rpm;
  int16_t return_rpm;
  int16_t deadband_g;
  int16_t release_g;
  float max_force_g;

  int16_t last_cmd_rpm;
  int32_t last_position;
  float last_force_g;
  uint32_t last_tick;
} Control;

/* main.c: низкоуровневая передача по RS485 с управлением DE/RE. */
void RS485_ExtInit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef RS485_Transmit(UART_HandleTypeDef *huart,
                                 GPIO_TypeDef *de_port,
                                 uint16_t de_pin,
                                 const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout);
HAL_StatusTypeDef RS485_ExtSend(const uint8_t *data, uint16_t len);

/* sensor.c: NAU7802 — инициализация, чтение, тара и калибровка. */
void Sensor_Init(Sensor *sensor, I2C_HandleTypeDef *hi2c);
uint8_t Sensor_Probe(Sensor *sensor);
uint8_t Sensor_Begin(Sensor *sensor);
uint8_t Sensor_SetSampleRate(Sensor *sensor, uint16_t sps);
uint16_t Sensor_GetSampleRate(Sensor *sensor);
uint8_t Sensor_ReadRevision(Sensor *sensor, uint8_t *revision);
uint8_t Sensor_ReadRaw(Sensor *sensor, int32_t *raw);
uint8_t Sensor_ReadMedian3(Sensor *sensor, int32_t *raw);
uint8_t Sensor_ReadAverage(Sensor *sensor, uint8_t samples, int32_t *raw);
uint8_t Sensor_Tare(Sensor *sensor);
void Sensor_SetCoefficient(Sensor *sensor, float counts_per_g);
uint8_t Sensor_Calibrate(Sensor *sensor, float known_grams);
float Sensor_ConvertToGrams(Sensor *sensor, int32_t raw);
uint8_t Sensor_ReadGrams(Sensor *sensor, int32_t *raw, float *grams);

/* motor.c: MKS SERVO42D — кадры, режимы, скорость, позиция, homing. */
void Motor_Init(Motor *motor,
                UART_HandleTypeDef *huart,
                GPIO_TypeDef *de_port,
                uint16_t de_pin,
                uint8_t addr);

uint8_t Motor_ReadEnable(Motor *motor, uint8_t *enabled);
uint8_t Motor_ReadStatus(Motor *motor, uint8_t *status);
uint8_t Motor_ReadSpeed(Motor *motor, int16_t *rpm);
uint8_t Motor_ReadPosition(Motor *motor, int32_t *position);
uint8_t Motor_GoHome(Motor *motor);
uint8_t Motor_ReadVersionRaw(Motor *motor, uint8_t *response, uint8_t response_len);

uint8_t Motor_SetMode(Motor *motor, uint8_t mode);
uint8_t Motor_BroadcastCommand(Motor *motor,
                               uint8_t cmd,
                               const uint8_t *data,
                               uint8_t data_len);
uint8_t Motor_Unlock(Motor *motor);
uint8_t Motor_Recover(Motor *motor);
uint8_t Motor_SetEnabled(Motor *motor, uint8_t enabled);
uint8_t Motor_Jog(Motor *motor, int16_t rpm, uint8_t accel);
uint8_t Motor_Stop(Motor *motor);
uint8_t Motor_EmergencyStop(Motor *motor);

/* control.c: демо-контур усилия, мягкие пределы и возврат к home. */
void Control_Init(Control *control, Sensor *sensor, Motor *motor);
void Control_Loop(Control *control);
uint8_t Control_Start(Control *control);
void Control_Stop(Control *control);
void Control_SetHome(Control *control, int32_t home);
void Control_SetFar(Control *control, int32_t far);
void Control_SetMargins(Control *control, int32_t home, int32_t far);
void Control_SetMotorSign(Control *control, int8_t sign);
void Control_SetGain(Control *control, float gain);
void Control_SetMaxRpm(Control *control, int16_t rpm);
void Control_SetReturnRpm(Control *control, int16_t rpm);
void Control_SetDeadband(Control *control, int16_t grams);
void Control_SetRelease(Control *control, int16_t grams);
void Control_SetMaxForce(Control *control, float grams);
void Control_PrintStatus(Control *control);

/* app.c: инициализация приложения, CLI и главный цикл. */
void App_Init(I2C_HandleTypeDef *i2c1,
              I2C_HandleTypeDef *i2c2,
              UART_HandleTypeDef *cli_uart,
              UART_HandleTypeDef *motor_uart);
void App_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* PLATA_H */
