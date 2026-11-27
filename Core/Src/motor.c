#include "plata.h"

/*
 * motor.c — драйвер MKS SERVO42D по RS485 (USART2).
 *
 * Формат кадра запроса:  FA addr cmd data... checksum
 * Формат ответа:         FB addr cmd status/data... checksum
 * checksum — сумма всех предыдущих байтов по модулю 256.
 *
 * Важно: активные ответы драйвера отключены (команда 8C 01 00).
 * Перед каждой командой RX очищается, иначе лишний ответ после JOG
 * остаётся в USART2 и следующая команда читает мусор (ложный OFFLINE).
 *
 * F6 (speed mode): data[0] = бит направления 0x80 + старший байт скорости,
 * data[1] = младший байт скорости, data[2] = acc (0..255).
 * Чем больше acc, тем быстрее разгон/торможение; acc=0 — без разгона.
 *
 * Ответ обычно: FB addr cmd status/data... checksum.
 * response[3] — status или первый байт данных. Для команды 31H
 * данные занимают response[3..8] (6 байт, 48-битная позиция).
 */


#define MOTOR_RESPONSE_TIMEOUT_MS 50U

#define MOTOR_CMD_READ_ENABLE     0x3AU
#define MOTOR_CMD_READ_STATUS     0xF1U
#define MOTOR_CMD_READ_SPEED      0x32U
#define MOTOR_CMD_READ_VERSION    0x40U
#define MOTOR_CMD_SET_MODE        0x82U
#define MOTOR_CMD_ENABLE          0xF3U
#define MOTOR_CMD_SPEED           0xF6U
#define MOTOR_CMD_EMERGENCY_STOP  0xF7U
#define MOTOR_CMD_SET_ADDR        0x8BU
#define MOTOR_CMD_SET_RSP         0x8CU
#define MOTOR_CMD_SET_BAUD        0x8AU
#define MOTOR_CMD_UNLOCK          0x8FU

/* Контрольная сумма MKS: сумма всех предыдущих байтов по модулю 256.
 * Пример: FA 01 3A 00 → FA+01+3A+00 = 0x135 → младший байт 0x35. */
static uint8_t motor_checksum(const uint8_t *data, uint16_t len)
{
  uint8_t sum = 0U;
  uint16_t i;

  for (i = 0U; i < len; i++)
  {
    sum = (uint8_t)(sum + data[i]);
  }

  return sum;
}

/* Сбрасывает ошибки UART и вычитывает все уже принятые байты.
 *
 * ORE — переполнение приёма, FE — ошибка кадра, NE — шум на линии.
 * Если их не очистить, следующий HAL_UART_Receive может вернуть ошибку.
 * Чтение DR опустошает RXNE, пока в буфере есть байты. */
static void motor_flush_rx(Motor *motor)
{
  if (motor == NULL || motor->huart == NULL)
  {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(motor->huart);
  __HAL_UART_CLEAR_FEFLAG(motor->huart);
  __HAL_UART_CLEAR_NEFLAG(motor->huart);

  while (__HAL_UART_GET_FLAG(motor->huart, UART_FLAG_RXNE) != RESET)
  {
    (void)motor->huart->Instance->DR;
  }
}

/* Собирает и отправляет один кадр MKS, затем принимает и проверяет ответ.
 *
 * Шаги:
 *   1. проверяем указатели и длину данных;
 *   2. очищаем RX от хвостов прошлых команд;
 *   3. собираем FA addr cmd data... checksum;
 *   4. отправляем через RS485 (DE поднимается внутри RS485_Transmit);
 *   5. если ответ не нужен — выходим;
 *   6. принимаем response_len байт с таймаутом 50 мс;
 *   7. проверяем FB, адрес, команду и контрольную сумму.
 *
 * Возвращает 1 при успехе, 0 при любой ошибке. */
static uint8_t motor_command(Motor *motor,
                             uint8_t cmd,
                             const uint8_t *data,
                             uint8_t data_len,
                             uint8_t *response,
                             uint8_t response_len)
{
  uint8_t frame[16];
  uint8_t frame_len = 0U;
  uint8_t i;
  uint8_t ok;

  if (motor == NULL || motor->huart == NULL)
  {
    return 0U;
  }

  if ((uint16_t)data_len + 4U > sizeof(frame))
  {
    return 0U;
  }

  /* Убираем возможный «хвост» предыдущего ответа. */
  motor_flush_rx(motor);

  frame[frame_len++] = 0xFAU;
  frame[frame_len++] = motor->addr;
  frame[frame_len++] = cmd;

  for (i = 0U; i < data_len; i++)
  {
    frame[frame_len++] = data[i];
  }

  /* Контрольная сумма считается по всем байтам до неё. */
  frame[frame_len] = motor_checksum(frame, frame_len);
  frame_len++;

  if (RS485_Transmit(motor->huart,
                     motor->de_port,
                     motor->de_pin,
                     frame,
                     frame_len,
                     50U) != HAL_OK)
  {
    return 0U;
  }

  /* Некоторые команды не требуют ответа (broadcast-подобные). */
  if (response == NULL || response_len == 0U)
  {
    return 1U;
  }

  if (HAL_UART_Receive(motor->huart,
                       response,
                       response_len,
                       MOTOR_RESPONSE_TIMEOUT_MS) != HAL_OK)
  {
    return 0U;
  }

  /* Проверяем, что ответ действительно от нашего драйвера и команды. */
  if (response[0] != 0xFBU ||
      response[1] != motor->addr ||
      response[2] != cmd)
  {
    return 0U;
  }

  /* Последняя проверка — контрольная сумма ответа. */
  ok = (motor_checksum(response, (uint16_t)(response_len - 1U)) ==
        response[response_len - 1U]) ? 1U : 0U;

  return ok;
}

/* Broadcast-команда (адрес 0x00). Ответ не ожидается.
 * Все драйверы на шине выполнят её. Используется для UNLOCK и RECOVER. */
uint8_t Motor_BroadcastCommand(Motor *motor,
                               uint8_t cmd,
                               const uint8_t *data,
                               uint8_t data_len)
{
  uint8_t frame[16];
  uint8_t frame_len = 0U;
  uint8_t i;

  if (motor == NULL || motor->huart == NULL)
  {
    return 0U;
  }

  if ((uint16_t)data_len + 4U > sizeof(frame))
  {
    return 0U;
  }

  frame[frame_len++] = 0xFAU;
  frame[frame_len++] = 0x00U; /* широковещательный адрес */
  frame[frame_len++] = cmd;

  for (i = 0U; i < data_len; i++)
  {
    frame[frame_len++] = data[i];
  }

  frame[frame_len] = motor_checksum(frame, frame_len);
  frame_len++;

  return (RS485_Transmit(motor->huart,
                         motor->de_port,
                         motor->de_pin,
                         frame,
                         frame_len,
                         50U) == HAL_OK) ? 1U : 0U;
}

/* Разблокировка кнопок драйвера broadcast-командой 8F. */
uint8_t Motor_Unlock(Motor *motor)
{
  uint8_t data = 0x00U;

  return Motor_BroadcastCommand(motor, MOTOR_CMD_UNLOCK, &data, 1U);
}

/* Восстановление связи broadcast-командами. Порядок важен:
 *   1. UNLOCK — разблокировать кнопки;
 *   2. SET_MODE — перевести в SR_vFOC;
 *   3. SET_ADDR — адрес 1;
 *   4. SET_RSP — отвечать на команды, но без active-потока;
 *   5. SET_BAUD — 38400.
 * После RECOVER нужно передёрнуть питание драйвера. */
uint8_t Motor_Recover(Motor *motor)
{
  uint8_t mode = MOTOR_MODE_SR_VFOC;
  uint8_t addr = 0x01U;
  uint8_t rsp[2] = {0x01U, 0x00U}; /* отвечать на команды, без active-потока */
  uint8_t baud = 0x04U; /* 38400 бит/с */
  uint8_t ok = 1U;

  ok &= Motor_BroadcastCommand(motor, MOTOR_CMD_UNLOCK, &((uint8_t){0x00U}), 1U);
  HAL_Delay(5);
  ok &= Motor_BroadcastCommand(motor, MOTOR_CMD_SET_MODE, &mode, 1U);
  HAL_Delay(5);
  ok &= Motor_BroadcastCommand(motor, MOTOR_CMD_SET_ADDR, &addr, 1U);
  HAL_Delay(5);
  ok &= Motor_BroadcastCommand(motor, MOTOR_CMD_SET_RSP, rsp, 2U);
  HAL_Delay(5);
  ok &= Motor_BroadcastCommand(motor, MOTOR_CMD_SET_BAUD, &baud, 1U);

  return ok;
}

/* Заполняет структуру драйвера и переводит DE/RE в приём. */
void Motor_Init(Motor *motor,
                UART_HandleTypeDef *huart,
                GPIO_TypeDef *de_port,
                uint16_t de_pin,
                uint8_t addr)
{
  if (motor == NULL)
  {
    return;
  }

  motor->huart = huart;
  motor->de_port = de_port;
  motor->de_pin = de_pin;
  motor->addr = addr;

  if (de_port != NULL)
  {
    HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_RESET);
  }
}

/* Читает состояние «мотор включён» командой 3A.
 * response[3] — 0/1. */
uint8_t Motor_ReadEnable(Motor *motor, uint8_t *enabled)
{
  uint8_t response[5];

  if (enabled == NULL)
  {
    return 0U;
  }

  if (!motor_command(motor,
                     MOTOR_CMD_READ_ENABLE,
                     NULL,
                     0U,
                     response,
                     sizeof(response)))
  {
    return 0U;
  }

  *enabled = response[3];
  return 1U;
}

/* Читает статус драйвера командой F1. response[3] — код статуса. */
uint8_t Motor_ReadStatus(Motor *motor, uint8_t *status)
{
  uint8_t response[5];

  if (status == NULL)
  {
    return 0U;
  }

  if (!motor_command(motor,
                     MOTOR_CMD_READ_STATUS,
                     NULL,
                     0U,
                     response,
                     sizeof(response)))
  {
    return 0U;
  }

  *status = response[3];
  return 1U;
}

/* Читает текущую скорость командой 32.
 * response[3] — старший байт, response[4] — младший; знаковое RPM. */
uint8_t Motor_ReadSpeed(Motor *motor, int16_t *rpm)
{
  uint8_t response[6];
  uint16_t raw;

  if (rpm == NULL)
  {
    return 0U;
  }

  if (!motor_command(motor,
                     MOTOR_CMD_READ_SPEED,
                     NULL,
                     0U,
                     response,
                     sizeof(response)))
  {
    return 0U;
  }

  raw = ((uint16_t)response[3] << 8) | response[4];
  *rpm = (int16_t)raw;
  return 1U;
}

/* Читает позицию энкодера командой 31.
 *
 * Ответ: 6 байт response[3..8], старший байт первым.
 * Склеиваем в int64_t, затем расширяем знак по биту 47.
 * Единицы — counts: 16384 counts = 1 оборот.
 *
 * На OLED-экране MKS позиция показывается в 1/400 оборота:
 *   screen = counts / 40.96.
 * Поэтому 6.3k на экране ≈ 258000 counts в этой функции. */
uint8_t Motor_ReadPosition(Motor *motor, int32_t *position)
{
  uint8_t response[10];
  int64_t value = 0;
  uint8_t i;

  if (position == NULL)
  {
    return 0U;
  }

  if (!motor_command(motor,
                     0x31U,
                     NULL,
                     0U,
                     response,
                     sizeof(response)))
  {
    return 0U;
  }

  for (i = 3U; i < 9U; i++)
  {
    value = (value << 8) | response[i];
  }

  if ((value & 0x800000000000LL) != 0)
  {
    value |= ~0xFFFFFFFFFFFFLL;
  }

  *position = (int32_t)value;
  return 1U;
}

/* Запускает homing командой 91, mode 0 — по датчику Холла.
 *
 * Драйвер сам едет к датчику, останавливается и обнуляет позицию.
 * STM32 не видит этот датчик — он только ждёт стабильную позицию 0.
 *
 * Возвращает response[3]: 0 — ошибка, 1 — старт, 2 — успех. */
uint8_t Motor_GoHome(Motor *motor)
{
  uint8_t data = 0x00U;
  uint8_t response[5];

  if (!motor_command(motor,
                     0x91U,
                     &data,
                     1U,
                     response,
                     sizeof(response)))
  {
    return 0U;
  }

  return response[3];
}

/* Читает сырой ответ версии командой 40. */
uint8_t Motor_ReadVersionRaw(Motor *motor, uint8_t *response, uint8_t response_len)
{
  if (response == NULL || response_len < 8U)
  {
    return 0U;
  }

  return motor_command(motor,
                       MOTOR_CMD_READ_VERSION,
                       NULL,
                       0U,
                       response,
                       8U);
}

/* Устанавливает режим драйвера командой 82.
 * Для этого проекта нужен MOTOR_MODE_SR_VFOC = 0x05. */
uint8_t Motor_SetMode(Motor *motor, uint8_t mode)
{
  uint8_t response[5];

  if (!motor_command(motor,
                     MOTOR_CMD_SET_MODE,
                     &mode,
                     1U,
                     response,
                     sizeof(response)))
  {
    return 0U;
  }

  return response[3];
}

/* Включает/выключает мотор командой F3.
 * enabled=1 — вал держит момент, enabled=0 — вал отпущен. */
uint8_t Motor_SetEnabled(Motor *motor, uint8_t enabled)
{
  uint8_t data = enabled ? 1U : 0U;
  uint8_t response[5];

  if (!motor_command(motor,
                     MOTOR_CMD_ENABLE,
                     &data,
                     1U,
                     response,
                     sizeof(response)))
  {
    return 0U;
  }

  return response[3];
}

/* Задаёт скорость в режиме F6.
 *
 * Кадр F6: FA addr F6 d0 d1 acc CRC.
 *   d0 = бит направления 0x80 | старшие 7 бит скорости;
 *   d1 = младшие 8 бит скорости;
 *   acc = параметр разгона 0..255.
 *
 * rpm < 0 — отрицательное направление (бит 0x80).
 * acc: время на 1 RPM = (256 - acc) * 50 мкс.
 *   acc=180 → 3.8 мс/RPM → ~0.34 с до 90 RPM;
 *   acc=200 → 2.8 мс/RPM → ~0.25 с торможения с 90 RPM.
 * acc=0 — без разгона, сразу заданная скорость. */
uint8_t Motor_Jog(Motor *motor, int16_t rpm, uint8_t accel)
{
  uint8_t data[3];
  uint8_t response[5];
  uint16_t speed;

  if (rpm < 0)
  {
    speed = (uint16_t)(-rpm);
    data[0] = (uint8_t)(0x80U | ((speed >> 8) & 0x7FU));
  }
  else
  {
    speed = (uint16_t)rpm;
    data[0] = (uint8_t)((speed >> 8) & 0x7FU);
  }

  data[1] = (uint8_t)(speed & 0xFFU);
  data[2] = accel;

  if (!motor_command(motor,
                     MOTOR_CMD_SPEED,
                     data,
                     sizeof(data),
                     response,
                     sizeof(response)))
  {
    return 0U;
  }

  return response[3];
}

/* Контролируемая остановка: F6, скорость 0, acc=200.
 * Большой acc нужен, чтобы на 90 RPM каретка не проскакивала мягкий
 * предел: торможение занимает около 0.25 с вместо ~1.1 с при acc=10. */
uint8_t Motor_Stop(Motor *motor)
{
  /* F6, скорость 0, быстрое торможение */
  return Motor_Jog(motor, 0, 200U);
}

/* Аварийная остановка командой F7 — без плавного торможения.
 * Использовать только в аварийной ситуации. */
uint8_t Motor_EmergencyStop(Motor *motor)
{
  uint8_t response[5];

  if (!motor_command(motor,
                     MOTOR_CMD_EMERGENCY_STOP,
                     NULL,
                     0U,
                     response,
                     sizeof(response)))
  {
    return 0U;
  }

  return response[3];
}
