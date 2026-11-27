#include "plata.h"

/*
 * app.c — прикладной слой Plata: CLI, команды и вспомогательные потоки.
 *
 * Задачи файла:
 *   - принять строку команд из внешнего RS485 (USART1);
 *   - разобрать команду и вызвать нужный модуль (sensor/motor/control);
 *   - вести потоки STREAM/FAST и автостоп JOG по таймеру;
 *   - выполнить AUTO — полный запуск демо одной командой;
 *   - мигать светодиодом и обслуживать главный цикл App_Loop().
 *
 * Внешний CLI: 115200 8N1, команды регистронезависимые, конец строки — Enter.
 * Внутренний протокол MKS и модуль control описаны в motor.c и control.c.
 */


#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Настройки AUTO для текущего станка.
 * AUTO_HOME_POS — дальний (рабочий) край, AUTO_FAR_POS — ближний край.
 */
#define AUTO_HOME_POS         258623L
#define AUTO_FAR_POS          0L
#define AUTO_HOME_MARGIN      15000L
#define AUTO_FAR_MARGIN       55000L
#define AUTO_GAIN_RPM_PER_G   0.16f
#define AUTO_MAX_RPM          90
#define AUTO_RETURN_RPM       40
#define AUTO_DEADBAND_G       30
#define AUTO_RELEASE_G        15
#define AUTO_MAX_FORCE_G      2000.0f
#define AUTO_COUNTS_PER_G     1116.0f
#define AUTO_HOMING_TIMEOUT_MS 30000U

/* UART внешнего CLI (USART1/RS485). Используется для printf. */
static UART_HandleTypeDef *app_cli_uart = NULL;

/* Два экземпляра датчика: I2C1 и I2C2. */
static Sensor sensor1;
static Sensor sensor2;
static Motor motor1;
static Control control1;
static AppState app_state = APP_STATE_IDLE;

/* Буфер приёма CLI.
 * app_rx_byte — один байт из прерывания USART1;
 * app_line — накопленная строка;
 * app_line_len — сколько уже накоплено;
 * app_line_ready — 1, когда пришёл Enter и строку можно разбирать. */
static uint8_t app_rx_byte;
static char app_line[CLI_LINE_MAX];
static volatile uint16_t app_line_len = 0U;
static volatile uint8_t app_line_ready = 0U;

/* Состояние ручного JOG: активен ли и когда его останавливать. */
static uint8_t app_jog_active = 0U;
static uint32_t app_jog_end_tick = 0U;
static uint32_t app_led_tick = 0U;

/*
 * ВРЕМЕННО: калибровка для текущего датчика и монтажа.
 * Позже заменить хранением во flash или перекалибровкой через T1/CAL1.
 */
#define DEFAULT_TARE_RAW       (-67471L)
#define DEFAULT_COUNTS_PER_G   1116.0f

/* Состояние потока STREAM: датчик, период и время следующего вывода. */
static Sensor *app_stream_sensor = NULL;
static uint32_t app_stream_period_ms = 0U;
static uint32_t app_stream_next_tick = 0U;

/* Состояние быстрого потока FAST (320 SPS). */
static Sensor *app_fast_sensor = NULL;
/* 1 — не печатать приглашение "> " (например, во время FAST). */
static uint8_t app_quiet_prompt = 0U;

/* Применяет временную калибровку, зашитую в прошивку:
 * tare_raw=-67471, counts_per_g=1116. */
static void app_apply_default_calibration(Sensor *sensor)
{
  if (sensor == NULL)
  {
    return;
  }

  sensor->tare_raw = DEFAULT_TARE_RAW;
  sensor->counts_per_g = DEFAULT_COUNTS_PER_G;
  sensor->tared = 1U;
  sensor->calibrated = 1U;
}

/* Проверяет, не вышел ли сырой отсчёт за разумный диапазон
 * (признак обрыва/перегрузки датчика). */
static uint8_t app_raw_saturated(int32_t raw)
{
  return (raw > 8000000L || raw < -8000000L) ? 1U : 0U;
}

/* Сравнивает аргумент CLI со строкой без учёта регистра. */
static uint8_t app_arg_eq(const char *arg, const char *value)
{
  if (arg == NULL || value == NULL)
  {
    return 0U;
  }

  while (*arg != '\0' && *value != '\0')
  {
    if (tolower((unsigned char)*arg) != tolower((unsigned char)*value))
    {
      return 0U;
    }
    arg++;
    value++;
  }

  return (*arg == '\0' && *value == '\0') ? 1U : 0U;
}

static const char *app_state_name(AppState state)
{
  switch (state)
  {
    case APP_STATE_IDLE:  return "IDLE";
    case APP_STATE_READY: return "READY";
    case APP_STATE_RUN:   return "RUN";
    case APP_STATE_FAULT: return "FAULT";
    default:              return "?";
  }
}

/* Печатает состояние датчика: найден, инициализирован, калиброван. */
static void app_print_sensor(const char *name, Sensor *sensor)
{
  if (sensor == NULL || sensor->hi2c == NULL)
  {
    printf("%s: not configured\r\n", name);
    return;
  }

  if (sensor->initialized)
  {
    printf("%s: NAU7802 OK, rev=0x%02X%s\r\n",
           name,
           (unsigned)sensor->revision,
           sensor->calibrated ? ", calibrated" : "");
    return;
  }

  if (Sensor_Probe(sensor))
  {
    printf("%s: NAU7802 0x2A found, not initialized (send INIT)\r\n", name);
  }
  else
  {
    printf("%s: NAU7802 0x2A NOT FOUND\r\n", name);
  }
}

/* Печатает граммы в формате X.X без float printf:
 * в этой сборке newlib не выводит плавающую точку. */
static void app_print_grams(float grams)
{
  /* Формируем строку вручную: printf с %f в этой newlib не работает.
   * Отправляем напрямую в RS485, чтобы не добавлять приглашение "> ". */
  int32_t grams10 = (int32_t)(grams * 10.0f +
                              ((grams >= 0.0f) ? 0.5f : -0.5f));

  if (grams10 < 0)
  {
    printf("F=-%ld.%01ld g",
           (long)((-grams10) / 10),
           (long)((-grams10) % 10));
  }
  else
  {
    printf("F=%ld.%01ld g",
           (long)(grams10 / 10),
           (long)(grams10 % 10));
  }
}

/* R/R1/R2: одно чтение — сырой отсчёт и граммы. */
static void app_print_reading(const char *name, Sensor *sensor)
{
  int32_t raw = 0;
  float grams = 0.0f;

  if (sensor == NULL || !sensor->initialized)
  {
    printf("%s: not ready\r\n", name);
    return;
  }

  if (!Sensor_ReadAverage(sensor, 8U, &raw))
  {
    printf("%s: read failed\r\n", name);
    return;
  }

  printf("%s: raw=%ld ", name, (long)raw);

  /* Аварийная защита: при обрыве/перегрузе АЦП уходит в насыщение. */
  if (app_raw_saturated(raw))
  {
    printf("SATURATED ");
  }

  if (sensor->calibrated)
  {
    grams = Sensor_ConvertToGrams(sensor, raw);
    app_print_grams(grams);
  }
  else
  {
    printf("F=uncalibrated");
  }

  printf("\r\n");
}

/* Печатает один отсчёт в пакете B1/B2. */
static void app_print_sample(const char *name, Sensor *sensor, uint8_t index)
{
  int32_t raw = 0;
  float grams = 0.0f;

  if (!Sensor_ReadRaw(sensor, &raw))
  {
    printf("%s[%u]: read failed\r\n", name, (unsigned)index);
    return;
  }

  printf("%s[%u]: raw=%ld ", name, (unsigned)index, (long)raw);

  if (app_raw_saturated(raw))
  {
    printf("SATURATED ");
  }

  if (sensor->calibrated)
  {
    grams = Sensor_ConvertToGrams(sensor, raw);
    app_print_grams(grams);
  }
  else
  {
    printf("F=uncalibrated");
  }

  printf("\r\n");
}

/* B1/B2: пакет сырых отсчётов для оценки шума. */
static void app_batch_sensor(const char *name, Sensor *sensor, uint8_t count)
{
  uint8_t i;

  if (sensor == NULL || !sensor->initialized)
  {
    printf("%s: not ready\r\n", name);
    return;
  }

  if (count == 0U)
  {
    count = 10U;
  }

  if (count > 50U)
  {
    count = 50U;
  }

  printf("%s: batch %u samples\r\n", name, (unsigned)count);

  for (i = 0U; i < count; i++)
  {
    app_print_sample(name, sensor, (uint8_t)(i + 1U));
  }
}

/* Останавливает поток STREAM. */
static void app_stream_stop(void)
{
  /* Просто снимаем указатель: App_Loop перестанет вызывать тик. */
  app_stream_sensor = NULL;
  app_stream_period_ms = 0U;
  printf("STREAM: stopped\r\n");
}

/* Запускает STREAM: только граммы, медиана 3, заданный период. */
static void app_stream_start(const char *name, Sensor *sensor, uint32_t period_ms)
{
  if (sensor == NULL || !sensor->initialized)
  {
    printf("%s: not ready\r\n", name);
    return;
  }

  if (!sensor->calibrated)
  {
    printf("%s: not calibrated (T1 then CAL1 <g>)\r\n", name);
    return;
  }

  /* Период ограничиваем 100..5000 мс, чтобы не забивать UART. */
  if (period_ms < 100U)
  {
    period_ms = 100U;
  }

  if (period_ms > 5000U)
  {
    period_ms = 5000U;
  }

  /* STREAM и FAST взаимоисключающие: включая один, выключаем другой. */
  app_fast_sensor = NULL;
  app_stream_sensor = sensor;
  app_stream_period_ms = period_ms;
  app_stream_next_tick = HAL_GetTick();

  printf("STREAM: %s, period=%lu ms, grams (median3)\r\n",
         name,
         (unsigned long)period_ms);
}

/* Вызывается из App_Loop: печатает очередной отсчёт STREAM по таймеру. */
static void app_stream_tick(void)
{
  int32_t raw = 0;
  float grams = 0.0f;

  if (app_stream_sensor == NULL)
  {
    return;
  }

  /* Проверяем, пришло ли время очередного вывода. */
  if ((HAL_GetTick() - app_stream_next_tick) < app_stream_period_ms)
  {
    return;
  }

  app_stream_next_tick = HAL_GetTick();

  /* Одно чтение с медианой 3: одиночный импульсный выброс игнорируется. */
  if (!Sensor_ReadMedian3(app_stream_sensor, &raw))
  {
    printf("STREAM: read failed, stopped\r\n");
    app_stream_sensor = NULL;
    app_stream_period_ms = 0U;
    return;
  }

  if (app_raw_saturated(raw))
  {
    printf("STREAM: SATURATED, stopped\r\n");
    app_stream_sensor = NULL;
    app_stream_period_ms = 0U;
    return;
  }

  grams = Sensor_ConvertToGrams(app_stream_sensor, raw);

  printf("STREAM: ");
  app_print_grams(grams);
  printf("\r\n");
}

/* Печатает только число граммов для FAST (без префикса). */
static void app_fast_send_grams(float grams)
{
  int32_t grams10 = (int32_t)(grams * 10.0f +
                              ((grams >= 0.0f) ? 0.5f : -0.5f));
  char buffer[20];
  int len;

  if (grams10 < 0)
  {
    len = snprintf(buffer,
                   sizeof(buffer),
                   "-%ld.%01ld\r\n",
                   (long)((-grams10) / 10),
                   (long)((-grams10) % 10));
  }
  else
  {
    len = snprintf(buffer,
                   sizeof(buffer),
                   "%ld.%01ld\r\n",
                   (long)(grams10 / 10),
                   (long)(grams10 % 10));
  }

  if (len > 0)
  {
    (void)RS485_ExtSend((const uint8_t *)buffer, (uint16_t)len);
  }
}

/* Быстрый поток FAST: 320 SPS, одиночные отсчёты, без фильтра. */
static void app_fast_tick(void)
{
  int32_t raw = 0;

  if (app_fast_sensor == NULL)
  {
    return;
  }

  if (!Sensor_ReadRaw(app_fast_sensor, &raw))
  {
    printf("FAST: read failed, stopped\r\n");
    app_fast_sensor = NULL;
    return;
  }

  if (app_raw_saturated(raw))
  {
    printf("FAST: SATURATED, stopped\r\n");
    app_fast_sensor = NULL;
    return;
  }

  app_fast_send_grams(Sensor_ConvertToGrams(app_fast_sensor, raw));
}

/* Запускает FAST на 320 SPS. */
static void app_fast_start(const char *name, Sensor *sensor)
{
  int32_t dummy = 0;
  uint8_t i;

  if (sensor == NULL || !sensor->initialized)
  {
    printf("%s: not ready\r\n", name);
    return;
  }

  if (!sensor->calibrated)
  {
    printf("%s: not calibrated (T1 then CAL1 <g>)\r\n", name);
    return;
  }

  /* Переводим NAU7802 на 320 SPS. */
  if (!Sensor_SetSampleRate(sensor, 320U))
  {
    printf("%s: cannot set 320 SPS\r\n", name);
    return;
  }

  /* Выбрасываем первые преобразования после смены частоты. */
  /* После смены частоты первые 3 преобразования могут быть старыми. */
  HAL_Delay(100);
  for (i = 0U; i < 3U; i++)
  {
    (void)Sensor_ReadRaw(sensor, &dummy);
  }

  /* FAST отключает STREAM, чтобы не читать один датчик из двух мест. */
  app_stream_sensor = NULL;
  app_stream_period_ms = 0U;
  app_fast_sensor = sensor;
  app_quiet_prompt = 1U;
}

/* Останавливает FAST. */
static void app_fast_stop(void)
{
  /* 3. Потоки: FAST имеет приоритет над STREAM. */
  if (app_fast_sensor != NULL)
  {
    app_fast_sensor = NULL;
    printf("FAST: stopped\r\n");
  }
  else
  {
    printf("FAST: not active\r\n");
  }
}

/* T/T1/T2: тара датчика без нагрузки (32 отсчёта). */
static void app_tare_sensor(const char *name, Sensor *sensor)
{
  if (sensor == NULL || !sensor->initialized)
  {
    printf("%s: not ready\r\n", name);
    return;
  }

  if (Sensor_Tare(sensor))
  {
    printf("%s: tare raw=%ld", name, (long)sensor->tare_raw);
    if (app_raw_saturated(sensor->tare_raw))
    {
      printf(" WARNING: saturated, check load cell");
    }
    printf("\r\n");
  }
  else
  {
    printf("%s: tare failed\r\n", name);
  }
}

/* CAL1/CAL2: калибровка по известному весу.
 * Считает counts_per_g = (raw - tare_raw) / known_grams. */
static void app_calibrate_sensor(const char *name, Sensor *sensor, float known_grams)
{
  if (sensor == NULL || !sensor->initialized)
  {
    printf("%s: not ready\r\n", name);
    return;
  }

  if (!sensor->tared)
  {
    printf("%s: tare first (T1/T2)\r\n", name);
    return;
  }

  if (app_raw_saturated(sensor->tare_raw))
  {
    printf("%s: tare is saturated, check load cell\r\n", name);
    return;
  }

  if (known_grams <= 0.0f)
  {
    printf("Usage: CAL1 <grams> / CAL2 <grams>\r\n");
    return;
  }

  if (Sensor_Calibrate(sensor, known_grams))
  {
    long counts_per_g = (long)((sensor->counts_per_g >= 0.0f) ?
                               (sensor->counts_per_g + 0.5f) :
                               (sensor->counts_per_g - 0.5f));

    printf("%s: calibrated, counts/g=%ld\r\n", name, counts_per_g);
  }
  else
  {
    printf("%s: calibration failed (check load and tare)\r\n", name);
  }
}

/* SCAN/I2C: перебирает адреса 1..127 и печатает найденные устройства. */
static void app_scan_bus(const char *name, I2C_HandleTypeDef *hi2c)
{
  uint8_t addr;
  uint8_t found = 0U;

  if (hi2c == NULL)
  {
    printf("%s: not configured\r\n", name);
    return;
  }

  printf("%s scan:", name);

  for (addr = 0x08U; addr <= 0x77U; addr++)
  {
    if (HAL_I2C_IsDeviceReady(hi2c,
                              (uint16_t)(addr << 1),
                              1U,
                              2U) == HAL_OK)
    {
      printf(" 0x%02X", (unsigned)addr);
      found++;

      if (addr == 0x2AU)
      {
        Sensor temp;
        uint8_t rev = 0U;

        Sensor_Init(&temp, hi2c);
        if (Sensor_ReadRevision(&temp, &rev))
        {
          printf("(NAU7802 rev=0x%02X)", (unsigned)rev);
        }
      }
    }
  }

  if (!found)
  {
    printf(" nothing");
  }

  printf("\r\n");
}

/* PING: печатает ONLINE/OFFLINE, enabled, status и скорость MKS. */
static void app_print_motor_status(void)
{
  uint8_t enabled = 0U;
  uint8_t status = 0U;
  int16_t rpm = 0;

  if (motor1.huart == NULL)
  {
    printf("MKS: not configured\r\n");
    return;
  }

  if (!Motor_ReadEnable(&motor1, &enabled))
  {
    /*
     * Некоторые конфигурации отключают ответ на 3A, но продолжают
     * отвечать на F1. Пробуем F1, прежде чем объявлять мотор OFFLINE.
     */
    if (Motor_ReadStatus(&motor1, &status))
    {
      printf("MKS addr=%u: ONLINE, 3A no reply, status=%u\r\n",
             (unsigned)motor1.addr,
             (unsigned)status);
      return;
    }

    printf("MKS addr=%u: OFFLINE (check 38400, addr, A/B, GND)\r\n",
           (unsigned)motor1.addr);
    return;
  }

  printf("MKS addr=%u: ONLINE, enabled=%u",
         (unsigned)motor1.addr,
         (unsigned)enabled);

  if (Motor_ReadStatus(&motor1, &status))
  {
    printf(", status=%u", (unsigned)status);
  }

  if (Motor_ReadSpeed(&motor1, &rpm))
  {
    printf(", speed=%d RPM", (int)rpm);
  }

  printf("\r\n");
}

/* STATUS: сводка по тактированию, датчикам, MKS и JOG. */
static void app_print_status(void)
{
  printf("STATE=%s\r\n", app_state_name(app_state));
  printf("SYS: SYSCLK=%lu Hz HCLK=%lu Hz PCLK1=%lu Hz PCLK2=%lu Hz\r\n",
         (unsigned long)HAL_RCC_GetSysClockFreq(),
         (unsigned long)HAL_RCC_GetHCLKFreq(),
         (unsigned long)HAL_RCC_GetPCLK1Freq(),
         (unsigned long)HAL_RCC_GetPCLK2Freq());

  app_print_sensor("I2C1", &sensor1);
  if (sensor1.initialized)
  {
    app_print_reading("I2C1", &sensor1);
  }

  app_print_sensor("I2C2", &sensor2);
  if (sensor2.initialized)
  {
    app_print_reading("I2C2", &sensor2);
  }

  app_print_motor_status();

  if (app_jog_active)
  {
    printf("JOG: active, remaining=%lu ms\r\n",
           (unsigned long)(app_jog_end_tick - HAL_GetTick()));
  }
  else
  {
    printf("JOG: inactive\r\n");
  }
}

/* STOP: останавливает мотор и снимает признак активного JOG. */
static void app_stop_motor(void)
{
  (void)Motor_Stop(&motor1);
  app_jog_active = 0U;
}

/* JOG: запускает вращение на заданное время.
 * Перед запуском проверяет, что драйвер отвечает и мотор включён. */
static void app_start_jog(int16_t rpm, uint32_t duration_ms)
{
  uint8_t enabled = 0U;

  if (!Motor_ReadEnable(&motor1, &enabled))
  {
    printf("JOG: MKS does not respond\r\n");
    return;
  }

  if (!enabled)
  {
    printf("JOG: motor disabled. Send EN 1 first.\r\n");
    return;
  }

  if (!Motor_Jog(&motor1, rpm, 10U))
  {
    printf("JOG: command failed\r\n");
    return;
  }

  app_jog_active = 1U;
  app_jog_end_tick = HAL_GetTick() + duration_ms;

  printf("JOG: %d RPM, dir=%s, duration=%lu ms\r\n",
         (int)rpm,
         (rpm < 0) ? "CW" : "CCW",
         (unsigned long)duration_ms);
}

/* Автостоп JOG по истечении заданного времени. */
static void app_jog_tick(void)
{
  if (app_jog_active &&
      (int32_t)(HAL_GetTick() - app_jog_end_tick) >= 0)
  {
    app_jog_active = 0U;
    (void)Motor_Stop(&motor1);
    printf("\r\nJOG auto-stop\r\n> ");
  }
}

/* CLOCK: печатает SYSCLK/HCLK/PCLK1/PCLK2. */
static void app_print_clock(void)
{
  printf("SYSCLK=%lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
  printf("HCLK=%lu Hz\r\n", (unsigned long)HAL_RCC_GetHCLKFreq());
  printf("PCLK1=%lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK1Freq());
  printf("PCLK2=%lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK2Freq());
}

/* VERSION: печатает сырой ответ версии MKS. */
static void app_print_version(void)
{
  uint8_t response[8];
  uint8_t i;

  if (!Motor_ReadVersionRaw(&motor1, response, sizeof(response)))
  {
    printf("MKS version: no response\r\n");
    return;
  }

  printf("MKS version raw:");
  for (i = 0U; i < sizeof(response); i++)
  {
    printf(" %02X", (unsigned)response[i]);
  }
  printf("\r\n");
}

/* Ожидает завершения homing: позиция должна стать близкой к нулю
 * и стабильной несколько опросов подряд. */
static uint8_t app_wait_homing(int32_t *position)
{
  /* start нужен для общего таймаута: homing не должен длиться вечно. */
  uint32_t start = HAL_GetTick();
  int32_t previous = 0;
  int32_t current = 0;
  uint8_t stable = 0U;

  if (position == NULL)
  {
    return 0U;
  }

  *position = 0;

  while ((HAL_GetTick() - start) < AUTO_HOMING_TIMEOUT_MS)
  {
    /* Опрашиваем позицию раз в 400 мс — чаще нет смысла. */
    HAL_Delay(400U);

    if (!Motor_ReadPosition(&motor1, &current))
    {
      stable = 0U;
      continue;
    }

    /* Считаем, что homing завершён, когда позиция около нуля
     * (в пределах ±2000 counts). */
    if (current > -2000 && current < 2000)
    {
      /* Три подряд идущих опроса с разницей < 50 counts — позиция
       * стабильна. 3 × 400 мс ≈ 1.2 с. */
      if (stable > 0U &&
          (current - previous) < 50 &&
          (previous - current) < 50)
      {
        stable++;
      }
      else
      {
        stable = 1U;
      }

      if (stable >= 3U)
      {
        *position = current;
        return 1U;
      }
    }
    else
    {
      stable = 0U;
    }

    previous = current;
  }

  /* Таймаут: отдаём последнюю позицию, чтобы вызывающий увидел,
   * где остановился драйвер. */
  *position = current;
  return 0U;
}

/* AUTO: полный запуск демо одной командой.
 *
 * Порядок:
 *   init датчика → тара → коэффициент 1116 → enable мотора →
 *   homing → повторная тара → пределы/скорости → DEMO 1.
 * Во время выполнения нельзя трогать датчик. */
static void app_auto_demo(void)
{
  uint8_t home_status;
  int32_t position = 0;

  /* Шаг 1. Останавливаем старое демо, мотор и поток, чтобы
   * дальнейшие операции не конфликтовали. */
  Control_Stop(&control1);
  (void)Motor_Stop(&motor1);
  app_jog_active = 0U;
  app_stream_stop();

  /* Шаг 2. Переинициализируем датчик 1. Второй датчик пробуем
   * инициализировать, но его отсутствие не мешает AUTO. */
  printf("AUTO: init sensor 1...\r\n");

  if (!Sensor_Begin(&sensor1))
  {
    printf("AUTO: sensor 1 init failed\r\n");
    return;
  }

  (void)Sensor_Begin(&sensor2);

  /* Шаг 3. Тара до homing: запоминаем текущий ноль. */
  printf("AUTO: tare, do not touch the sensor...\r\n");

  if (!Sensor_Tare(&sensor1))
  {
    printf("AUTO: tare failed\r\n");
    return;
  }

  Sensor_SetCoefficient(&sensor1, AUTO_COUNTS_PER_G);
  printf("AUTO: tare raw=%ld, coef=%ld counts/g\r\n",
         (long)sensor1.tare_raw,
         (long)(sensor1.counts_per_g + 0.5f));

  /* Шаг 4. После тары принудительно ставим известный коэффициент
   * 1116 counts/g — он не зависит от текущего смещения. */
  printf("AUTO: enable motor...\r\n");

  /* Шаг 5. Включаем мотор: homing требует удерживающего момента. */
  if (!Motor_SetEnabled(&motor1, 1U))
  {
    printf("AUTO: motor enable failed\r\n");
    return;
  }

  HAL_Delay(100U);

  printf("AUTO: homing, up to 30 s...\r\n");

  /* Шаг 6. Запускаем homing по датчику Холла MKS. */
  home_status = Motor_GoHome(&motor1);
  if (home_status == 0U)
  {
    printf("AUTO: homing command failed\r\n");
    return;
  }

  /* Шаг 7. Ждём, пока драйвер доедет до нуля и позиция станет
   * стабильной. */
  if (!app_wait_homing(&position))
  {
    printf("AUTO: homing timeout, pos=%ld\r\n", (long)position);
    return;
  }

  printf("AUTO: homing done, pos=%ld\r\n", (long)position);
  /* Шаг 8. Тара после homing: механизм мог слегка сместиться,
   * поэтому ноль датчика лучше запомнить в рабочем положении. */
  printf("AUTO: tare after homing...\r\n");

  if (!Sensor_Tare(&sensor1))
  {
    printf("AUTO: tare failed\r\n");
    return;
  }

  Sensor_SetCoefficient(&sensor1, AUTO_COUNTS_PER_G);
  printf("AUTO: tare raw=%ld, coef=%ld counts/g\r\n",
         (long)sensor1.tare_raw,
         (long)(sensor1.counts_per_g + 0.5f));

  /* Шаг 9. Задаём пределы, запасы и параметры демо.
   * home — дальний край, far — ближний. */
  Control_SetHome(&control1, AUTO_HOME_POS);
  Control_SetFar(&control1, AUTO_FAR_POS);
  Control_SetMargins(&control1, AUTO_HOME_MARGIN, AUTO_FAR_MARGIN);
  Control_SetMotorSign(&control1, 1);
  Control_SetGain(&control1, AUTO_GAIN_RPM_PER_G);
  Control_SetMaxRpm(&control1, AUTO_MAX_RPM);
  Control_SetReturnRpm(&control1, AUTO_RETURN_RPM);
  Control_SetDeadband(&control1, AUTO_DEADBAND_G);
  Control_SetRelease(&control1, AUTO_RELEASE_G);
  Control_SetMaxForce(&control1, AUTO_MAX_FORCE_G);

  /* Даём MKS время выйти из режима homing. */
  HAL_Delay(1000U);

  /* Шаг 10. Включаем демо. Оно само вернёт каретку к home
   * со скоростью RETURN, потому что travel сейчас максимален. */
  if (!Control_Start(&control1))
  {
    printf("AUTO: demo not ready\r\n");
    return;
  }

  printf("AUTO: DEMO ON, returning to home 6.3k...\r\n");
}

/* HELP: список доступных команд. */
static void app_print_help(void)
{
  printf("Commands:\r\n");
  printf("  HELP              this help\r\n");
  printf("  STATUS            state, sensors, MKS\r\n");
  printf("  AUTO              full setup: init, tare, home, limits, DEMO\r\n");
  printf("\r\n");
  printf("  T1                tare sensor 1, no load\r\n");
  printf("  CAL1 <g>          calibrate sensor 1 with known weight\r\n");
  printf("  COEF <counts/g>   set sensor 1 coefficient manually\r\n");
  printf("  R1 / B1 [n]       one reading / batch of raw samples\r\n");
  printf("  FAST 1|0          320 SPS stream, grams only\r\n");
  printf("\r\n");
  printf("  PING / POS / HOME MKS status / position / homing\r\n");
  printf("  EN 1|0            enable/disable motor\r\n");
  printf("  JOG <rpm> [ms]    manual run, max +/-30 RPM\r\n");
  printf("  STOP              stop motor\r\n");
  printf("  RECOVER           broadcast MKS recovery\r\n");
  printf("\r\n");
  printf("  LIMIT HOME|FAR [n]  set limit by position or number\r\n");
  printf("  MARGIN <home> <far>  soft-limit margins, counts\r\n");
  printf("  DIR 1|-1            motor sign away from home\r\n");
  printf("  GAIN <rpm/g>        force-to-speed gain\r\n");
  printf("  MAXRPM <rpm>        speed limit\r\n");
  printf("  RETURN <rpm>        return-to-home speed\r\n");
  printf("  DEADBAND <g>        start threshold\r\n");
  printf("  RELEASE <g>         return threshold\r\n");
  printf("  MAXFORCE <g>        overload limit, default 2000 g\r\n");
  printf("  DEMO 1|0            start/stop demo\r\n");
  printf("  CTRL                demo status\r\n");
  printf("\r\n");
  printf("JOG positive = CCW, negative = CW.\r\n");
  printf("Before JOG: send EN 1.\r\n");
}

/* Стартовый баннер: версия, подсказка HELP и состояние датчиков. */
static void app_print_startup(void)
{
  uint8_t enabled = 0U;

  printf("\r\n\r\n");
  printf("========================================\r\n");
  printf(" Plata force controller - Step 3 test\r\n");
  printf(" FW: %s\r\n", FW_VERSION);
  printf(" SYSCLK=%lu Hz HCLK=%lu Hz\r\n",
         (unsigned long)HAL_RCC_GetSysClockFreq(),
         (unsigned long)HAL_RCC_GetHCLKFreq());
  printf(" PCLK1=%lu Hz PCLK2=%lu Hz\r\n",
         (unsigned long)HAL_RCC_GetPCLK1Freq(),
         (unsigned long)HAL_RCC_GetPCLK2Freq());

  app_print_sensor("I2C1", &sensor1);
  app_print_sensor("I2C2", &sensor2);

  if (Motor_ReadEnable(&motor1, &enabled))
  {
    printf(" MKS addr 1: ONLINE, enabled=%u\r\n", (unsigned)enabled);
  }
  else
  {
    printf(" MKS addr 1: OFFLINE\r\n");
    printf("   Check: 38400 8N1, UartAddr=1, A/B, GND\r\n");
  }

  printf("========================================\r\n");
}

/* Разбирает одну строку CLI и выполняет команду.
 * Команда и аргументы разделяются пробелами/табуляцией. */
static void app_execute(char *line)
{
  char *cmd;
  char *arg1;
  char *arg2;
  char *p;

  /* Разбираем строку: команда — первое слово, arg1/arg2 — следующие.
   * strtok модифицирует строку, вставляя \0 на месте разделителей. */
  cmd = strtok(line, " \t");
  if (cmd == NULL)
  {
    return;
  }

  for (p = cmd; *p != '\0'; p++)
  {
    *p = (char)tolower((unsigned char)*p);
  }

  arg1 = strtok(NULL, " \t");
  arg2 = strtok(NULL, " \t");

  /* HELP / ? — печать списка команд. */
  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
  {
    app_print_help();
  }
  /* STATUS — сводка: тактирование, датчики, MKS, вес, JOG. */
  else if (strcmp(cmd, "status") == 0)
  {
    app_print_status();
  }
  /* SCAN / I2C — сканировать обе шины I2C. */
  else if (strcmp(cmd, "scan") == 0 || strcmp(cmd, "i2c") == 0)
  {
    app_scan_bus("I2C1", sensor1.hi2c);
    app_scan_bus("I2C2", sensor2.hi2c);
  }
  /* AUTO / SETUP / AUTODEMO — полный запуск демо одной командой. */
  else if (strcmp(cmd, "auto") == 0 ||
           strcmp(cmd, "setup") == 0 ||
           strcmp(cmd, "autodemo") == 0)
  {
    app_auto_demo();
  }
  /* INIT — переинициализировать оба NAU7802 и применить временную
   * калибровку (tare=-67471, counts/g=1116). */
  else if (strcmp(cmd, "init") == 0)
  {
    uint8_t ok1;
    uint8_t ok2;

    Control_Stop(&control1);
    ok1 = Sensor_Begin(&sensor1);
    ok2 = Sensor_Begin(&sensor2);

    if (ok1)
    {
      app_apply_default_calibration(&sensor1);
    }

    printf("I2C1: %s%s\r\n",
           ok1 ? "init OK" : "init failed",
           ok1 ? " (default calibration)" : "");
    printf("I2C2: %s\r\n", ok2 ? "init OK" : "init failed");
  }
  /* RATE [10|20|40|80|320] — установить/показать частоту NAU7802. */
  else if (strcmp(cmd, "rate") == 0)
  {
    if (arg1 == NULL)
    {
      printf("RATE: I2C1=%u SPS", (unsigned)Sensor_GetSampleRate(&sensor1));
      if (sensor2.initialized)
      {
        printf(", I2C2=%u SPS", (unsigned)Sensor_GetSampleRate(&sensor2));
      }
      printf("\r\n");
    }
    else
    {
      int sps = atoi(arg1);
      uint8_t ok1 = 0U;
      uint8_t ok2 = 0U;

      if (sensor1.initialized)
      {
        ok1 = Sensor_SetSampleRate(&sensor1, (uint16_t)sps);
      }
      if (sensor2.initialized)
      {
        ok2 = Sensor_SetSampleRate(&sensor2, (uint16_t)sps);
      }

      if (ok1 || ok2)
      {
        printf("RATE: %d SPS\r\n", sps);
      }
      else
      {
        printf("Usage: RATE 10|20|40|80|320\r\n");
      }
    }
  }
  /* R — одно чтение обоих датчиков (raw + граммы). */
  else if (strcmp(cmd, "r") == 0)
  {
    app_print_reading("I2C1", &sensor1);
    app_print_reading("I2C2", &sensor2);
  }
  /* R1 — одно чтение датчика 1. */
  else if (strcmp(cmd, "r1") == 0)
  {
    app_print_reading("I2C1", &sensor1);
  }
  /* R2 — одно чтение датчика 2. */
  else if (strcmp(cmd, "r2") == 0)
  {
    app_print_reading("I2C2", &sensor2);
  }
  /* B — пакет сырых отсчётов с обоих датчиков. */
  else if (strcmp(cmd, "b") == 0)
  {
    int count = (arg1 != NULL) ? atoi(arg1) : 10;
    if (count <= 0) count = 10;
    if (count > 50) count = 50;
    app_batch_sensor("I2C1", &sensor1, (uint8_t)count);
    app_batch_sensor("I2C2", &sensor2, (uint8_t)count);
  }
  /* B1 [n] — пакет n отсчётов с датчика 1. */
  else if (strcmp(cmd, "b1") == 0)
  {
    int count = (arg1 != NULL) ? atoi(arg1) : 10;
    if (count <= 0) count = 10;
    if (count > 50) count = 50;
    app_batch_sensor("I2C1", &sensor1, (uint8_t)count);
  }
  /* B2 [n] — пакет n отсчётов с датчика 2. */
  else if (strcmp(cmd, "b2") == 0)
  {
    int count = (arg1 != NULL) ? atoi(arg1) : 10;
    if (count <= 0) count = 10;
    if (count > 50) count = 50;
    app_batch_sensor("I2C2", &sensor2, (uint8_t)count);
  }
  /* FAST 1|2|0 — быстрый поток 320 SPS, только число граммов. */
  else if (strcmp(cmd, "fast") == 0)
  {
    int channel = (arg1 != NULL) ? atoi(arg1) : -1;

    if (channel == 0)
    {
      app_fast_stop();
    }
    else if (channel == 1)
    {
      app_fast_start("I2C1", &sensor1);
    }
    else if (channel == 2)
    {
      app_fast_start("I2C2", &sensor2);
    }
    else
    {
      printf("Usage: FAST 0|1|2\r\n");
    }
  }
  /* STREAM 1|2|0 [ms] — поток с медианой 3 и заданным периодом. */
  else if (strcmp(cmd, "stream") == 0)
  {
    int channel = (arg1 != NULL) ? atoi(arg1) : -1;
    int period = (arg2 != NULL) ? atoi(arg2) : 200;

    if (channel == 0)
    {
      app_stream_stop();
    }
    else if (channel == 1)
    {
      app_stream_start("I2C1", &sensor1, (uint32_t)((period > 0) ? period : 200));
    }
    else if (channel == 2)
    {
      app_stream_start("I2C2", &sensor2, (uint32_t)((period > 0) ? period : 200));
    }
    else
    {
      printf("Usage: STREAM 0|1|2 [ms]\r\n");
    }
  }
  /* T — тара обоих датчиков без нагрузки. */
  else if (strcmp(cmd, "t") == 0)
  {
    app_tare_sensor("I2C1", &sensor1);
    app_tare_sensor("I2C2", &sensor2);
  }
  /* T1 — тара датчика 1. */
  else if (strcmp(cmd, "t1") == 0)
  {
    app_tare_sensor("I2C1", &sensor1);
  }
  /* T2 — тара датчика 2. */
  else if (strcmp(cmd, "t2") == 0)
  {
    app_tare_sensor("I2C2", &sensor2);
  }
  /* CAL1 <г> / CAL <г> — калибровка датчика 1 по известному весу. */
  else if (strcmp(cmd, "cal1") == 0 || strcmp(cmd, "cal") == 0)
  {
    int known = (arg1 != NULL) ? atoi(arg1) : 0;
    app_calibrate_sensor("I2C1", &sensor1, (float)known);
  }
  /* CAL2 <г> — калибровка датчика 2. */
  else if (strcmp(cmd, "cal2") == 0)
  {
    int known = (arg1 != NULL) ? atoi(arg1) : 0;
    app_calibrate_sensor("I2C2", &sensor2, (float)known);
  }
  /* COEF <counts/g> — задать коэффициент датчика 1 вручную. */
  else if (strcmp(cmd, "coef") == 0)
  {
    float value = (arg1 != NULL) ? (float)atof(arg1) : 0.0f;

    if (value <= 0.0f)
    {
      printf("Usage: COEF <counts_per_g>\r\n");
    }
    else
    {
      Sensor_SetCoefficient(&sensor1, value);
      printf("I2C1: counts/g=%.1f\r\n", (double)sensor1.counts_per_g);
    }
  }
  /* PING — проверить MKS и напечатать enabled/status/speed. */
  else if (strcmp(cmd, "ping") == 0)
  {
    app_print_motor_status();
  }
  /* POS — прочитать позицию энкодера MKS (counts). */
  else if (strcmp(cmd, "pos") == 0)
  {
    int32_t position = 0;

    if (Motor_ReadPosition(&motor1, &position))
    {
      printf("MKS position: %ld\r\n", (long)position);
    }
    else
    {
      printf("MKS position: no response\r\n");
    }
  }
  /* HOME — запустить homing MKS по датчику Холла. */
  else if (strcmp(cmd, "home") == 0)
  {
    uint8_t status;

    Control_Stop(&control1);
    status = Motor_GoHome(&motor1);

    if (status == 1U)
    {
      printf("MKS: homing started\r\n");
    }
    else if (status == 2U)
    {
      printf("MKS: homing complete\r\n");
    }
    else
    {
      printf("MKS: homing failed or no response\r\n");
    }
  }
  /* LIMIT HOME|FAR [n] — задать предел текущей позицией или числом. */
  else if (strcmp(cmd, "limit") == 0)
  {
    int32_t position = 0;
    uint8_t position_ok;

    Control_Stop(&control1);

    if (arg2 != NULL)
    {
      position = (int32_t)atol(arg2);
      position_ok = 1U;
    }
    else
    {
      position_ok = Motor_ReadPosition(&motor1, &position);
    }

    if (!position_ok)
    {
      printf("LIMIT: no position response\r\n");
    }
    else if (app_arg_eq(arg1, "home"))
    {
      Control_SetHome(&control1, position);
      printf("LIMIT HOME: %ld%s\r\n",
             (long)position,
             (arg2 != NULL) ? " (set)" : "");
    }
    else if (app_arg_eq(arg1, "far"))
    {
      Control_SetFar(&control1, position);
      printf("LIMIT FAR: %ld%s\r\n",
             (long)position,
             (arg2 != NULL) ? " (set)" : "");
    }
    else
    {
      printf("Usage: LIMIT HOME|FAR [counts]\r\n");
    }
  }
  /* MARGIN <home> <far> — программные запасы до краёв, counts. */
  else if (strcmp(cmd, "margin") == 0)
  {
    int32_t home = (arg1 != NULL) ? (int32_t)atol(arg1) : -1;
    int32_t far = (arg2 != NULL) ? (int32_t)atol(arg2) : -1;

    if (home < 0 || far < 0)
    {
      printf("Usage: MARGIN <home_counts> <far_counts>\r\n");
    }
    else
    {
      Control_SetMargins(&control1, home, far);
      printf("DEMO: margins home=%ld far=%ld\r\n",
             (long)control1.home_margin,
             (long)control1.far_margin);
    }
  }
  /* DIR 1|-1 — знак мотора для направления «от home». */
  else if (strcmp(cmd, "dir") == 0)
  {
    int value = (arg1 != NULL) ? atoi(arg1) : 0;

    if (value != 1 && value != -1)
    {
      printf("Usage: DIR 1|-1\r\n");
    }
    else
    {
      Control_SetMotorSign(&control1, (int8_t)value);
      printf("DEMO: motor sign=%d\r\n", value);
    }
  }
  /* GAIN <rpm/г> — прибавка скорости на грамм сверх deadband. */
  else if (strcmp(cmd, "gain") == 0)
  {
    float gain = (arg1 != NULL) ? (float)atof(arg1) : 0.0f;

    if (gain <= 0.0f)
    {
      printf("Usage: GAIN <rpm per gram>\r\n");
    }
    else
    {
      int gain100;

      Control_SetGain(&control1, gain);
      gain100 = (int)(control1.gain_rpm_per_g * 100.0f + 0.5f);
      printf("DEMO: gain=%d.%02d RPM/g\r\n",
             gain100 / 100,
             gain100 % 100);
    }
  }
  /* MAXRPM <rpm> — максимальная скорость демо. */
  else if (strcmp(cmd, "maxrpm") == 0)
  {
    int rpm = (arg1 != NULL) ? atoi(arg1) : 0;

    if (rpm <= 0)
    {
      printf("Usage: MAXRPM <rpm>\r\n");
    }
    else
    {
      Control_SetMaxRpm(&control1, (int16_t)rpm);
      printf("DEMO: max RPM=%d\r\n", (int)control1.max_rpm);
    }
  }
  /* RETURN <rpm> — скорость возврата к home. */
  else if (strcmp(cmd, "return") == 0)
  {
    int rpm = (arg1 != NULL) ? atoi(arg1) : 0;

    if (rpm <= 0)
    {
      printf("Usage: RETURN <rpm>\r\n");
    }
    else
    {
      Control_SetReturnRpm(&control1, (int16_t)rpm);
      printf("DEMO: return RPM=%d\r\n", (int)control1.return_rpm);
    }
  }
  /* DEADBAND <г> — порог начала движения. */
  else if (strcmp(cmd, "deadband") == 0)
  {
    int grams = (arg1 != NULL) ? atoi(arg1) : 0;

    if (grams <= 0)
    {
      printf("Usage: DEADBAND <grams>\r\n");
    }
    else
    {
      Control_SetDeadband(&control1, (int16_t)grams);
      printf("DEMO: deadband=%d g, release=%d g\r\n",
             (int)control1.deadband_g,
             (int)control1.release_g);
    }
  }
  /* RELEASE <г> — порог возврата. */
  else if (strcmp(cmd, "release") == 0)
  {
    int grams = (arg1 != NULL) ? atoi(arg1) : -1;

    if (grams < 0)
    {
      printf("Usage: RELEASE <grams>\r\n");
    }
    else
    {
      Control_SetRelease(&control1, (int16_t)grams);
      printf("DEMO: release=%d g, deadband=%d g\r\n",
             (int)control1.release_g,
             (int)control1.deadband_g);
    }
  }
  /* MAXFORCE <г> — аварийный порог перегрузки, по умолчанию 2000 г. */
  else if (strcmp(cmd, "maxforce") == 0 || strcmp(cmd, "overload") == 0)
  {
    int grams = (arg1 != NULL) ? atoi(arg1) : 0;

    if (grams <= 0)
    {
      printf("Usage: MAXFORCE <grams> (100..2000)\r\n");
    }
    else
    {
      Control_SetMaxForce(&control1, (float)grams);
      printf("DEMO: max force=%ld g\r\n",
             (long)(control1.max_force_g + 0.5f));
    }
  }
  /* DEMO 1|0 — включить/выключить демо. */
  else if (strcmp(cmd, "demo") == 0)
  {
    int enable = (arg1 != NULL) ? atoi(arg1) : -1;

    if (enable == 0)
    {
      Control_Stop(&control1);
      printf("DEMO: stopped\r\n");
    }
    else if (enable == 1)
    {
      app_stream_sensor = NULL;
      app_fast_sensor = NULL;

      if (Control_Start(&control1))
      {
        printf("DEMO: ON, press sensor to move carriage\r\n");
      }
      else
      {
        printf("DEMO: not ready. Need calibrated sensor and LIMIT HOME/FAR\r\n");
      }
    }
    else
    {
      printf("Usage: DEMO 1|0\r\n");
    }
  }
  /* CTRL — напечатать состояние демо. */
  else if (strcmp(cmd, "ctrl") == 0)
  {
    Control_PrintStatus(&control1);
  }
  /* MODE — перевести MKS в SR_vFOC (сохраняется в драйвере). */
  else if (strcmp(cmd, "mode") == 0)
  {
    if (Motor_SetMode(&motor1, MOTOR_MODE_SR_VFOC))
    {
      printf("MKS: mode set to SR_vFOC (05h), saved by driver\r\n");
    }
    else
    {
      uint8_t mode = MOTOR_MODE_SR_VFOC;

      if (Motor_BroadcastCommand(&motor1, 0x82U, &mode, 1U))
      {
        printf("MKS: no response, broadcast SR_vFOC sent\r\n");
        printf("Power-cycle MKS and send PING\r\n");
      }
      else
      {
        printf("MKS: mode broadcast failed\r\n");
      }
    }
  }
  /* UNLOCK — разблокировать кнопки MKS broadcast-командой. */
  else if (strcmp(cmd, "unlock") == 0)
  {
    if (Motor_Unlock(&motor1))
    {
      printf("MKS: unlock broadcast sent\r\n");
    }
    else
    {
      printf("MKS: unlock broadcast failed\r\n");
    }
  }
  /* RECOVER — broadcast: unlock, SR_vFOC, addr=1, RSP без active, 38400. */
  else if (strcmp(cmd, "recover") == 0)
  {
    if (Motor_Recover(&motor1))
    {
      printf("MKS: recovery broadcast sent (unlock, SR_vFOC, addr=1, RSP=on, 38400)\r\n");
      printf("Power-cycle MKS and send PING\r\n");
    }
    else
    {
      printf("MKS: recovery broadcast failed\r\n");
    }
  }
  /* EN 1|0 — включить/выключить мотор. */
  else if (strcmp(cmd, "en") == 0)
  {
    int value = (arg1 != NULL) ? atoi(arg1) : -1;

    if (value != 0 && value != 1)
    {
      printf("Usage: EN 0|1\r\n");
    }
    else if (Motor_SetEnabled(&motor1, (uint8_t)value))
    {
      printf("MKS: %s\r\n", value ? "enabled" : "disabled");
    }
    else
    {
      printf("MKS: enable command failed\r\n");
    }
  }
  /* JOG <rpm> [ms] — ручное вращение с автостопом. */
  else if (strcmp(cmd, "jog") == 0)
  {
    int rpm = (arg1 != NULL) ? atoi(arg1) : 0;
    int ms = (arg2 != NULL) ? atoi(arg2) : (int)JOG_DEFAULT_MS;

    if (rpm < -JOG_MAX_RPM)
    {
      rpm = -JOG_MAX_RPM;
    }
    if (rpm > JOG_MAX_RPM)
    {
      rpm = JOG_MAX_RPM;
    }
    if (ms <= 0)
    {
      ms = (int)JOG_DEFAULT_MS;
    }
    if ((uint32_t)ms > JOG_MAX_MS)
    {
      ms = (int)JOG_MAX_MS;
    }

    if (rpm == 0)
    {
      app_stop_motor();
      printf("JOG: stop\r\n");
    }
    else
    {
      app_start_jog((int16_t)rpm, (uint32_t)ms);
    }
  }
  /* STOP — остановить мотор и снять JOG. */
  else if (strcmp(cmd, "stop") == 0)
  {
    app_stop_motor();
    printf("STOP\r\n");
  }
  /* VERSION — сырой ответ версии MKS. */
  else if (strcmp(cmd, "version") == 0)
  {
    app_print_version();
  }
  /* CLOCK — напечатать частоты тактирования. */
  else if (strcmp(cmd, "clock") == 0)
  {
    app_print_clock();
  }
  else
  {
    printf("Unknown command: %s\r\n", cmd);
    printf("Type HELP\r\n");
  }
}

/* Забирает байты из USART1 и собирает строку до Enter. */
static void app_cli_poll(void)
{
  if (!app_line_ready)
  {
    return;
  }

  app_line_ready = 0U;
  app_line[app_line_len] = '\0';
  app_line_len = 0U;

  printf("\r\n");
  app_execute(app_line);

  if (app_quiet_prompt)
  {
    app_quiet_prompt = 0U;
  }
  else
  {
    printf("\r\n> ");
  }
}

/* Инициализация приложения: структуры датчиков, мотора и контура,
 * начальное состояние и стартовый баннер. */
void App_Init(I2C_HandleTypeDef *i2c1,
              I2C_HandleTypeDef *i2c2,
              UART_HandleTypeDef *cli_uart,
              UART_HandleTypeDef *motor_uart)
{
  app_cli_uart = cli_uart;

  /* Создаём оба датчика. Второй может отсутствовать на шине. */
  Sensor_Init(&sensor1, i2c1);
  Sensor_Init(&sensor2, i2c2);
  if (Sensor_Begin(&sensor1))
  {
    app_apply_default_calibration(&sensor1);
  }
  (void)Sensor_Begin(&sensor2);

  /* Мотор 1: USART2, DE на PB1, адрес 1. */
  Motor_Init(&motor1,
             motor_uart,
             MKS_RS485_DE_GPIO_Port,
             MKS_RS485_DE_Pin,
             1U);
  /* Связываем демо-контур с датчиком 1 и мотором 1. */
  Control_Init(&control1, &sensor1, &motor1);

  /* Сбрасываем состояние CLI, JOG и потоков. */
  app_state = APP_STATE_IDLE;
  app_line_len = 0U;
  app_line_ready = 0U;
  app_jog_active = 0U;
  app_led_tick = HAL_GetTick();
  app_stream_sensor = NULL;
  app_stream_period_ms = 0U;
  app_fast_sensor = NULL;
  app_quiet_prompt = 0U;

  if (app_cli_uart != NULL)
  {
  /* Запускаем приём по одному байту в прерывании. */
    /* Перезапускаем приём следующего байта. */
  (void)HAL_UART_Receive_IT(app_cli_uart, &app_rx_byte, 1U);
  }

  /* Печатаем баннер и краткую справку. */
  app_print_startup();
  app_print_help();
  printf("\r\n> ");
}

/* Главный цикл приложения:
 *   JOG-автостоп, шаг демо, потоки STREAM/FAST, CLI и мигание LED. */
void App_Loop(void)
{
  /* 1. Проверяем, не истекло ли время ручного JOG. */
  app_jog_tick();
  /* 2. Один шаг демо: сила → скорость/возврат. */
  Control_Loop(&control1);

  if (app_fast_sensor != NULL)
  {
    app_fast_tick();
  }
  else
  {
    app_stream_tick();
  }

  /* 4. Разбираем накопленную строку CLI. */
  app_cli_poll();

  /* 5. Мигаем светодиодом примерно 1 Гц. */
  if ((HAL_GetTick() - app_led_tick) >= 500U)
  {
    app_led_tick = HAL_GetTick();
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
  }
}

/* Прерывание приёма UART: складывает байты CLI в буфер. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t c;

  if (huart != app_cli_uart)
  {
    return;
  }

  /* Копируем байт, принятый прерыванием. */
  c = app_rx_byte;

  /* Enter: если строка непустая — помечаем её готовой. */
  if (c == '\r' || c == '\n')
  {
    if (!app_line_ready && app_line_len > 0U)
    {
      app_line_ready = 1U;
    }
  }
  /* Backspace: удаляем последний символ, если строка ещё не готова. */
  else if (c == 0x08U || c == 0x7FU)
  {
    if (!app_line_ready && app_line_len > 0U)
    {
      app_line_len--;
    }
  }
  /* Печатный ASCII: добавляем в буфер, пока есть место. */
  else if (c >= 0x20U && c <= 0x7EU)
  {
    if (!app_line_ready && app_line_len < (CLI_LINE_MAX - 1U))
    {
      app_line[app_line_len++] = (char)c;
    }
  }

  (void)HAL_UART_Receive_IT(app_cli_uart, &app_rx_byte, 1U);
}

/* Обработка ошибок UART: перезапускает приём, чтобы CLI не завис. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == app_cli_uart)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    (void)HAL_UART_Receive_IT(app_cli_uart, &app_rx_byte, 1U);
  }
}
