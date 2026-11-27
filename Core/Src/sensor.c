#include "plata.h"

/*
 * sensor.c — работа с 24-битным АЦП NAU7802 (тензодатчик YZC-133).
 *
 * Адрес на I2C — 0x2A (фиксированный), поэтому два датчика требуют
 * двух отдельных шин I2C1/I2C2.
 *
 * Основные регистры:
 *   PU_CTRL (0x00) — питание, сброс, старт преобразований;
 *   CTRL1   (0x01) — VLDO[5:3] (напряжение внутреннего LDO) и GAIN[2:0];
 *   CTRL2   (0x02) — частота дискретизации и калибровка;
 *   ADCO_B2 (0x12) — старший байт 24-битного результата.
 *
 * В текущей прошивке: внутренний LDO включён (AVDDS=1), VLDO=3.3 В,
 * PGA=128, 320 SPS при инициализации.
 * Если плата питается от 3.3 В, LDO 3.3 В работает в dropout; при
 * необходимости можно попробовать 3.0 В (как в библиотеке Adafruit).
 *
 * Последовательность Sensor_Begin():
 *   1. сброс цифрового блока (RR);
 *   2. включение цифрового питания (PUD), ожидание PUR;
 *   3. включение аналогового питания и внутреннего LDO (PUA|AVDDS);
 *   4. CTRL1: LDO 3.3 В + PGA 128;
 *   5. CTRL2: 320 SPS;
 *   6. включение конденсатора PGA;
 *   7. внутренняя калибровка смещения (CALS);
 *   8. CS=1 — запуск непрерывных преобразований.
 */


#define NAU7802_I2C_ADDR       (0x2AU << 1)

#define NAU7802_REG_PU_CTRL    0x00U
#define NAU7802_REG_CTRL1      0x01U
#define NAU7802_REG_CTRL2      0x02U
#define NAU7802_REG_ADCO_B2    0x12U
#define NAU7802_REG_PGA_PWR    0x1CU
#define NAU7802_REG_DEVICE_REV 0x1FU

#define NAU7802_PU_RR          0x01U
#define NAU7802_PU_PUD         0x02U
#define NAU7802_PU_PUA         0x04U
#define NAU7802_PU_PUR         0x08U
#define NAU7802_PU_CS          0x10U
#define NAU7802_PU_CR          0x20U
#define NAU7802_PU_AVDDS       0x80U

#define NAU7802_CTRL2_CALS     0x04U
#define NAU7802_CTRL2_CAL_ERR  0x08U

#define NAU7802_CTRL1_3V3_128  0x27U
#define NAU7802_CTRL2_320SPS   0x70U
#define NAU7802_PGA_CAP_EN     0x80U

#define SENSOR_TARE_SAMPLES    32U
#define SENSOR_CAL_SAMPLES     32U

/* Чтение одного 8-битного регистра NAU7802 по I2C. */
/* Чтение одного 8-битного регистра NAU7802 по I2C.
 * HAL_I2C_Mem_Read сам формирует START, адрес, регистр, повторный START
 * и читает 1 байт. 20 мс — таймаут на шину. */
static uint8_t sensor_reg_read(Sensor *sensor, uint8_t reg, uint8_t *value)
{
  if (sensor == NULL || sensor->hi2c == NULL || value == NULL)
  {
    return 0U;
  }

  return (HAL_I2C_Mem_Read(sensor->hi2c,
                           NAU7802_I2C_ADDR,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           value,
                           1U,
                           20U) == HAL_OK) ? 1U : 0U;
}

/* Запись одного 8-битного регистра NAU7802 по I2C. */
/* Запись одного 8-битного регистра NAU7802.
 * Адрес 0x2A в HAL уже сдвинут на 1 бит влево (см. NAU7802_I2C_ADDR). */
static uint8_t sensor_reg_write(Sensor *sensor, uint8_t reg, uint8_t value)
{
  if (sensor == NULL || sensor->hi2c == NULL)
  {
    return 0U;
  }

  return (HAL_I2C_Mem_Write(sensor->hi2c,
                            NAU7802_I2C_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            &value,
                            1U,
                            20U) == HAL_OK) ? 1U : 0U;
}

/* Ожидание, пока в регистре установится хотя бы один бит из mask. */
/* Ожидание установки бита(ов) mask в регистре.
 * Опрос идёт с шагом 1 мс, пока не истечёт timeout_ms. */
static uint8_t sensor_wait_set(Sensor *sensor, uint8_t reg, uint8_t mask, uint32_t timeout_ms)
{
  uint8_t value = 0U;
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) <= timeout_ms)
  {
    if (!sensor_reg_read(sensor, reg, &value))
    {
      return 0U;
    }

    if ((value & mask) != 0U)
    {
      return 1U;
    }

    HAL_Delay(1);
  }

  return 0U;
}

/* Ожидание, пока в регистре сбросятся все биты из mask. */
/* Ожидание сброса всех битов mask в регистре (например, окончания
 * внутренней калибровки CALS). */
static uint8_t sensor_wait_clear(Sensor *sensor, uint8_t reg, uint8_t mask, uint32_t timeout_ms)
{
  uint8_t value = 0U;
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) <= timeout_ms)
  {
    if (!sensor_reg_read(sensor, reg, &value))
    {
      return 0U;
    }

    if ((value & mask) == 0U)
    {
      return 1U;
    }

    HAL_Delay(1);
  }

  return 0U;
}

/* Заполняет структуру датчика и сбрасывает состояние калибровки. */
/* Обнуляет структуру датчика. Вызывается один раз при старте,
 * до Sensor_Begin(). */
void Sensor_Init(Sensor *sensor, I2C_HandleTypeDef *hi2c)
{
  if (sensor == NULL)
  {
    return;
  }

  sensor->hi2c = hi2c;
  sensor->present = 0U;
  sensor->initialized = 0U;
  sensor->revision = 0U;
  sensor->tared = 0U;
  sensor->calibrated = 0U;
  sensor->tare_raw = 0;
  sensor->counts_per_g = 0.0f;
}

/* Проверяет наличие NAU7802 на шине по адресу 0x2A. */
/* Проверка присутствия: HAL_I2C_IsDeviceReady делает 2 попытки
 * с таймаутом 20 мс. Адрес фиксированный — 0x2A. */
uint8_t Sensor_Probe(Sensor *sensor)
{
  if (sensor == NULL || sensor->hi2c == NULL)
  {
    return 0U;
  }

  sensor->present = (HAL_I2C_IsDeviceReady(sensor->hi2c,
                                            NAU7802_I2C_ADDR,
                                            2U,
                                            20U) == HAL_OK) ? 1U : 0U;

  return sensor->present;
}

/* Читает ревизию чипа (регистр 0x1F). У нас ожидается 0x0F. */
/* Ревизия чипа лежит в регистре 0x1F. У нас читается 0x0F. */
uint8_t Sensor_ReadRevision(Sensor *sensor, uint8_t *revision)
{
  if (sensor == NULL || sensor->hi2c == NULL || revision == NULL)
  {
    return 0U;
  }

  if (!sensor_reg_read(sensor, NAU7802_REG_DEVICE_REV, revision))
  {
    return 0U;
  }

  sensor->revision = *revision;
  return 1U;
}

/* Полная инициализация NAU7802:
 *   сброс, включение питания, LDO, PGA=128, 320 SPS,
 *   внутренняя калибровка смещения, запуск непрерывных преобразований.
 * Возвращает 1 при успехе. */
uint8_t Sensor_Begin(Sensor *sensor)
{
  uint8_t pu_ctrl = 0U;
  uint8_t ctrl2 = 0U;
  uint8_t rev = 0U;

  if (sensor == NULL || sensor->hi2c == NULL)
  {
    return 0U;
  }

  sensor->initialized = 0U;
  sensor->tared = 0U;
  sensor->calibrated = 0U;
  sensor->tare_raw = 0;
  sensor->counts_per_g = 0.0f;

  if (!Sensor_Probe(sensor))
  {
    return 0U;
  }

  /* Шаг 1. Сброс цифрового блока: пишем RR=1, ждём 10 мс,
   * затем RR=0. После сброса все регистры принимают значения
   * по умолчанию. */
  if (!sensor_reg_write(sensor, NAU7802_REG_PU_CTRL, NAU7802_PU_RR))
  {
    return 0U;
  }
  HAL_Delay(10);
  if (!sensor_reg_write(sensor, NAU7802_REG_PU_CTRL, 0x00U))
  {
    return 0U;
  }
  HAL_Delay(10);

  /* Шаг 2. Включаем цифровое питание (PUD=1) и ждём бит PUR:
   * он означает, что цифровая часть вышла из сброса. */
  if (!sensor_reg_write(sensor, NAU7802_REG_PU_CTRL, NAU7802_PU_PUD))
  {
    return 0U;
  }
  if (!sensor_wait_set(sensor, NAU7802_REG_PU_CTRL, NAU7802_PU_PUR, 200U))
  {
    return 0U;
  }

  /* Шаг 3. Включаем аналоговую часть (PUA) и внутренний LDO
   * (AVDDS=1). Именно LDO будет питать мост тензодатчика. */
  if (!sensor_reg_read(sensor, NAU7802_REG_PU_CTRL, &pu_ctrl))
  {
    return 0U;
  }
  pu_ctrl |= (NAU7802_PU_PUA | NAU7802_PU_AVDDS);
  if (!sensor_reg_write(sensor, NAU7802_REG_PU_CTRL, pu_ctrl))
  {
    return 0U;
  }

  /* Шаг 4. CTRL1 = 0x27:
   *   биты VLDO[5:3] = 100 → 3.3 В;
   *   биты GAIN[2:0] = 111 → PGA 128.
   * Шаг 5. CTRL2 = 0x70 → 320 SPS.
   * Шаг 6. Включаем конденсатор PGA (PGA_PWR, бит 7). */
  if (!sensor_reg_write(sensor, NAU7802_REG_CTRL1, NAU7802_CTRL1_3V3_128))
  {
    return 0U;
  }
  if (!sensor_reg_write(sensor, NAU7802_REG_CTRL2, NAU7802_CTRL2_320SPS))
  {
    return 0U;
  }
  if (!sensor_reg_write(sensor, NAU7802_REG_PGA_PWR, NAU7802_PGA_CAP_EN))
  {
    return 0U;
  }

  HAL_Delay(250);

  /* Шаг 7. Внутренняя калибровка смещения:
   *   - обнуляем младшие 2 бита режима калибровки;
   *   - ставим CALS=1 (старт);
   *   - ждём, пока CALS сбросится (до 1 с);
   *   - проверяем CAL_ERR: если 1 — калибровка не удалась. */
  if (!sensor_reg_read(sensor, NAU7802_REG_CTRL2, &ctrl2))
  {
    return 0U;
  }
  ctrl2 &= (uint8_t)~0x03U;
  ctrl2 |= NAU7802_CTRL2_CALS;
  if (!sensor_reg_write(sensor, NAU7802_REG_CTRL2, ctrl2))
  {
    return 0U;
  }
  if (!sensor_wait_clear(sensor, NAU7802_REG_CTRL2, NAU7802_CTRL2_CALS, 1000U))
  {
    return 0U;
  }
  if (!sensor_reg_read(sensor, NAU7802_REG_CTRL2, &ctrl2))
  {
    return 0U;
  }
  if ((ctrl2 & NAU7802_CTRL2_CAL_ERR) != 0U)
  {
    return 0U;
  }

  /* Шаг 8. CS=1 — запускаем непрерывные преобразования.
   * После этого Sensor_ReadRaw может ждать бит CR и читать данные. */
  if (!sensor_reg_read(sensor, NAU7802_REG_PU_CTRL, &pu_ctrl))
  {
    return 0U;
  }
  pu_ctrl |= NAU7802_PU_CS;
  if (!sensor_reg_write(sensor, NAU7802_REG_PU_CTRL, pu_ctrl))
  {
    return 0U;
  }

  (void)Sensor_ReadRevision(sensor, &rev);
  sensor->initialized = 1U;
  HAL_Delay(20);
  return 1U;
}

/* Переключает частоту дискретизации: 10/20/40/80/320 SPS.
 * Перед сменой CTRL2 преобразования останавливаются (CS=0), затем
 * запускаются снова — некоторые платы иначе сохраняют старую частоту. */
uint8_t Sensor_SetSampleRate(Sensor *sensor, uint16_t sps)
{
  uint8_t ctrl2 = 0U;
  uint8_t pu_ctrl = 0U;
  uint8_t rate_bits;

  if (sensor == NULL || !sensor->initialized)
  {
    return 0U;
  }

  switch (sps)
  {
    case 10U:  rate_bits = 0x00U; break;
    case 20U:  rate_bits = 0x10U; break;
    case 40U:  rate_bits = 0x20U; break;
    case 80U:  rate_bits = 0x30U; break;
    case 320U: rate_bits = 0x70U; break;
    default:   return 0U;
  }

  if (!sensor_reg_read(sensor, NAU7802_REG_CTRL2, &ctrl2))
  {
    return 0U;
  }

  ctrl2 &= (uint8_t)~0x70U;
  ctrl2 |= rate_bits;

  /* Важно: сначала CS=0 (остановить преобразования), потом менять
   * CTRL2, потом CS=1. Некоторые платы иначе оставляют старую частоту.
   *
   * Биты частоты в CTRL2[6:4]:
   *   000 — 10 SPS, 001 — 20, 010 — 40, 011 — 80, 111 — 320. */
  if (!sensor_reg_read(sensor, NAU7802_REG_PU_CTRL, &pu_ctrl))
  {
    return 0U;
  }

  pu_ctrl &= (uint8_t)~NAU7802_PU_CS;
  if (!sensor_reg_write(sensor, NAU7802_REG_PU_CTRL, pu_ctrl))
  {
    return 0U;
  }

  if (!sensor_reg_write(sensor, NAU7802_REG_CTRL2, ctrl2))
  {
    return 0U;
  }

  HAL_Delay(5);

  pu_ctrl |= NAU7802_PU_CS;
  if (!sensor_reg_write(sensor, NAU7802_REG_PU_CTRL, pu_ctrl))
  {
    return 0U;
  }

  HAL_Delay(50);
  return 1U;
}

/* Возвращает текущую частоту дискретизации в SPS (0 при ошибке). */
uint16_t Sensor_GetSampleRate(Sensor *sensor)
{
  uint8_t ctrl2 = 0U;

  if (sensor == NULL || !sensor->initialized)
  {
    return 0U;
  }

  if (!sensor_reg_read(sensor, NAU7802_REG_CTRL2, &ctrl2))
  {
    return 0U;
  }

  switch (ctrl2 & 0x70U)
  {
    case 0x00U: return 10U;
    case 0x10U: return 20U;
    case 0x20U: return 40U;
    case 0x30U: return 80U;
    case 0x70U: return 320U;
    default:    return 0U;
  }
}

/* Читает один 24-битный отсчёт АЦП.
 *
 * 1. Ждём бит CR (conversion ready) в PU_CTRL, максимум 250 мс.
 * 2. Читаем 3 байта начиная с ADCO_B2 (0x12): старший, средний, младший.
 * 3. Склеиваем в 24-битное значение.
 * 4. Если установлен бит 23 — расширяем знак до int32_t (отрицательное).
 *
 * Возвращает 1 при успехе, 0 при таймауте/ошибке I2C. */
uint8_t Sensor_ReadRaw(Sensor *sensor, int32_t *raw)
{
  uint8_t pu_ctrl = 0U;
  uint8_t buffer[3];
  int32_t value;
  uint32_t start;

  if (sensor == NULL || raw == NULL || !sensor->initialized)
  {
    return 0U;
  }

  start = HAL_GetTick();
  while (1)
  {
    if (!sensor_reg_read(sensor, NAU7802_REG_PU_CTRL, &pu_ctrl))
    {
      return 0U;
    }

    if ((pu_ctrl & NAU7802_PU_CR) != 0U)
    {
      break;
    }

    if ((HAL_GetTick() - start) > 250U)
    {
      return 0U;
    }

    __NOP();
  }

  if (HAL_I2C_Mem_Read(sensor->hi2c,
                       NAU7802_I2C_ADDR,
                       NAU7802_REG_ADCO_B2,
                       I2C_MEMADD_SIZE_8BIT,
                       buffer,
                       3U,
                       20U) != HAL_OK)
  {
    return 0U;
  }

  value = ((int32_t)buffer[0] << 16) |
          ((int32_t)buffer[1] << 8) |
          (int32_t)buffer[2];

  if ((value & 0x00800000) != 0)
  {
    value |= (int32_t)0xFF000000;
  }

  *raw = value;
  return 1U;
}

/* Читает три отсчёта и возвращает медиану.
 * Это простой фильтр одиночных импульсных помех: если один отсчёт
 * «улетел», медиана его игнорирует. */
/* Ниже — сортировка трёх чисел сетью из трёх сравнений. */
uint8_t Sensor_ReadMedian3(Sensor *sensor, int32_t *raw)
{
  int32_t values[3];
  int32_t temp;
  uint8_t i;

  if (sensor == NULL || raw == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < 3U; i++)
  {
    if (!Sensor_ReadRaw(sensor, &values[i]))
    {
      return 0U;
    }
  }

  if (values[0] > values[1])
  {
    temp = values[0];
    values[0] = values[1];
    values[1] = temp;
  }
  if (values[1] > values[2])
  {
    temp = values[1];
    values[1] = values[2];
    values[2] = temp;
  }
  if (values[0] > values[1])
  {
    temp = values[0];
    values[0] = values[1];
    values[1] = temp;
  }

  *raw = values[1];
  return 1U;
}

/* Читает samples отсчётов и возвращает среднее.
 *
 * Первый отсчёт после предыдущей конверсии может быть старым —
 * поэтому он читается и выбрасывается. Затем читаются samples
 * свежих значений и суммируются в int64_t, чтобы не переполнить
 * 32-битную сумму. Используется для тары и калибровки. */
uint8_t Sensor_ReadAverage(Sensor *sensor, uint8_t samples, int32_t *raw)
{
  int64_t sum = 0;
  int32_t value = 0;
  uint8_t i;

  if (sensor == NULL || raw == NULL || samples == 0U)
  {
    return 0U;
  }

  /* Выбрасываем один устаревший отсчёт после предыдущей конверсии. */
  if (!Sensor_ReadRaw(sensor, &value))
  {
    return 0U;
  }

  for (i = 0U; i < samples; i++)
  {
    if (!Sensor_ReadRaw(sensor, &value))
    {
      return 0U;
    }
    sum += value;
  }

  *raw = (int32_t)(sum / (int32_t)samples);
  return 1U;
}

/* Тара: запоминает средний сырой отсчёт при нулевой нагрузке.
 *
 * После тары сила = (raw - tare_raw) / counts_per_g.
 * Тара не меняет коэффициент калибровки — его можно задать
 * отдельно через COEF или CAL1. */
uint8_t Sensor_Tare(Sensor *sensor)
{
  int32_t raw = 0;

  if (sensor == NULL || !sensor->initialized)
  {
    return 0U;
  }

  if (!Sensor_ReadAverage(sensor, SENSOR_TARE_SAMPLES, &raw))
  {
    return 0U;
  }

  sensor->tare_raw = raw;
  sensor->tared = 1U;
  return 1U;
}

/* Устанавливает известный коэффициент counts/g вручную.
 *
 * Пример: тара сделана, известно, что 200 г дают 223200 counts,
 * значит counts_per_g = 1116. Можно вызвать COEF 1116 и не ждать
 * гирю. После смены коэффициента калибровка считается выполненной. */
void Sensor_SetCoefficient(Sensor *sensor, float counts_per_g)
{
  if (sensor == NULL || counts_per_g <= 0.0f)
  {
    return;
  }

  sensor->counts_per_g = counts_per_g;
  sensor->calibrated = 1U;
}

/* Калибровка по известному весу.
 *
 * Алгоритм:
 *   1. читаем среднее при нагрузке known_grams;
 *   2. delta = |raw - tare_raw|;
 *   3. если delta < 100 counts — считаем, что нагрузки нет, ошибка;
 *   4. counts_per_g = (raw - tare_raw) / known_grams.
 *
 * Требует предварительной тары (T1). */
uint8_t Sensor_Calibrate(Sensor *sensor, float known_grams)
{
  int32_t raw = 0;
  int32_t delta;

  if (sensor == NULL || !sensor->initialized || !sensor->tared ||
      known_grams <= 0.0f)
  {
    return 0U;
  }

  if (!Sensor_ReadAverage(sensor, SENSOR_CAL_SAMPLES, &raw))
  {
    return 0U;
  }

  delta = raw - sensor->tare_raw;
  if (delta < 0)
  {
    delta = -delta;
  }

  if (delta < 100)
  {
    return 0U;
  }

  sensor->counts_per_g = (float)(raw - sensor->tare_raw) / known_grams;
  sensor->calibrated = 1U;
  return 1U;
}

/* Пересчёт сырого отсчёта в граммы:
 *   grams = (raw - tare_raw) / counts_per_g.
 * Если калибровки нет — возвращает 0. */
float Sensor_ConvertToGrams(Sensor *sensor, int32_t raw)
{
  if (sensor == NULL || !sensor->calibrated ||
      sensor->counts_per_g == 0.0f)
  {
    return 0.0f;
  }

  return (float)(raw - sensor->tare_raw) / sensor->counts_per_g;
}

/* Читает один отсчёт и сразу пересчитывает его в граммы. */
uint8_t Sensor_ReadGrams(Sensor *sensor, int32_t *raw, float *grams)
{
  int32_t value = 0;

  if (sensor == NULL || raw == NULL || grams == NULL)
  {
    return 0U;
  }

  if (!Sensor_ReadRaw(sensor, &value))
  {
    return 0U;
  }

  *raw = value;
  *grams = Sensor_ConvertToGrams(sensor, value);
  return 1U;
}
