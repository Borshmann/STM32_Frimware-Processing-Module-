#include "plata.h"

#include <stdio.h>

/*
 * control.c — экспериментальный демо-контур «тензодатчик → каретка».
 *
 * Идея демо:
 *   - пока сила меньше DEADBAND, каретка стоит;
 *   - при силе выше DEADBAND скорость пропорциональна силе;
 *   - при отпускании (сила ниже RELEASE) каретка возвращается к home;
 *   - крайние точки ограничены программно (home_margin/far_margin);
 *   - при перегрузке выше max_force_g (по умолчанию 2000 г)
 *     движение стопорится, но демо остаётся включённым.
 *
 * Это не финальный PI-регулятор, а отладочная логика для проверки
 * механики, направлений, пределов и связи с MKS.
 */

/*
 * Как устроена система координат и единицы
 * -----------------------------------------
 * position  — абсолютная позиция энкодера MKS, counts.
 *             16384 counts = 1 оборот вала.
 * home_pos  — позиция «покоя» (в текущем станке — дальний край).
 * far_pos   — противоположный край рабочего хода.
 * travel    — ход от home: 0 .. max_travel.
 *             travel = (position - home_pos) * encoder_sign.
 * encoder_sign = +1, если far > home, и -1, если far < home.
 *
 * Сила измеряется в граммах, скорость — в «RPM» MKS.
 * ВАЖНО: «RPM» в F6 — условные единицы драйвера. На этом станке
 * 1 RPM ≈ 2228 counts/s (измерено экспериментально); это зависит
 * от микрошага и механики.
 *
 * Алгоритм одного шага Control_Loop():
 *   1. Раз в 50 мс читаем силу (медиана 5) и позицию MKS.
 *   2. Если сила > deadband — едем от home со скоростью,
 *      пропорциональной силе.
 *   3. Если сила < release — возвращаемся к home.
 *   4. Между release и deadband — стоим (гистерезис).
 *   5. У краёв не доезжаем home_margin/far_margin counts.
 *   6. При перегрузке > max_force_g просто стоп.
 */
/*
 * Период работы контура. 50 мс — компромисс между скоростью реакции
 * и временем чтения NAU7802 (медиана из 5 отсчётов; 80 или 320 SPS —
 * зависит от RATE).
 */
#define CONTROL_PERIOD_MS          50U

/* Запасы по умолчанию. В рабочей конфигурации задаются через MARGIN. */
#define CONTROL_DEFAULT_HOME_MARGIN 300
#define CONTROL_DEFAULT_FAR_MARGIN  1500

/* Аварийный порог силы по умолчанию: 2000 г (датчик YZC-133 на 2 кг).
 * Меняется командой MAXFORCE. */
#define CONTROL_DEFAULT_MAX_FORCE_G 2000.0f

/* Количество отсчётов для медианного фильтра силы. */
#define CONTROL_FILTER_N           5U

/* Модуль 32-битного числа без использования библиотеки math. */
static int32_t control_abs32(int32_t value)
{
  return (value < 0) ? -value : value;
}

/*
 * Читает силу как медиану пяти сырых отсчётов NAU7802.
 *
 * Медиана отбрасывает одиночные импульсные помехи (наводки от мотора
 * и RS485), не внося задержки, как это делает длинное среднее.
 * Возвращает 1 при успехе и 0 при ошибке I2C/АЦП.
 */
static uint8_t control_read_force(Control *control, float *force)
{
  /* Здесь будем хранить пять последних сырых отсчётов АЦП. */
  int32_t values[CONTROL_FILTER_N];
  int32_t temp;
  uint8_t i;
  uint8_t j;

  if (control == NULL || control->sensor == NULL || force == NULL)
  {
    return 0U;
  }

  /* Читаем пять отсчётов подряд. Любая ошибка I2C/АЦП — отдаём 0. */
  for (i = 0U; i < CONTROL_FILTER_N; i++)
  {
    if (!Sensor_ReadRaw(control->sensor, &values[i]))
    {
      return 0U;
    }
  }

  /* Сортировка вставками: массив из 5 элементов, qsort здесь избыточен.
   * После сортировки values[2] — медиана. */
  /* Сортируем массив вставками и берём средний элемент. */
  for (i = 1U; i < CONTROL_FILTER_N; i++)
  {
    temp = values[i];
    j = i;

    while (j > 0U && values[j - 1U] > temp)
    {
      values[j] = values[j - 1U];
      j--;
    }

    values[j] = temp;
  }

  /* Переводим медиану counts → граммы по таре и counts_per_g. */
  *force = Sensor_ConvertToGrams(control->sensor,
                                 values[CONTROL_FILTER_N / 2U]);
  return 1U;
}

/*
 * Пересчитывает направление энкодера и полный ход по заданным home/far.
 *
 * Демо не требует, чтобы home был меньше far: важно только взаимное
 * расположение. Например, сейчас home=258623 (дальний край), far=0
 * (ближний край), поэтому encoder_sign = -1.
 */
static void control_recalc_limits(Control *control)
{
  int32_t diff;

  control->limits_valid = 0U;

  if (!control->home_valid || !control->far_valid)
  {
    return;
  }

  /* Разница может быть отрицательной: home и far задаются как угодно. */
  diff = control->far_pos - control->home_pos;
  /* Знак нужен, чтобы travel всегда рос от home к far. */
  control->encoder_sign = (diff >= 0) ? 1 : -1;
  control->max_travel = control_abs32(diff);

  /* Слишком маленький ход считаем ошибкой конфигурации (защита от
   * случайно одинаковых home/far). */
  if (control->max_travel > 200)
  {
    control->limits_valid = 1U;
  }
}

/* Инициализация контура значениями по умолчанию. Реальные пределы, скорости и
 * пределы задаются командами LIMIT/MARGIN/GAIN/... или командой AUTO. */
void Control_Init(Control *control, Sensor *sensor, Motor *motor)
{
  if (control == NULL)
  {
    return;
  }

  /* Связываем контур с уже созданными датчиком и мотором. */
  control->sensor = sensor;
  control->motor = motor;
  control->active = 0U;
  control->home_valid = 0U;
  control->far_valid = 0U;
  control->limits_valid = 0U;
  control->read_failures = 0U;
  control->jog_failures = 0U;
  control->home_pos = 0;
  control->far_pos = 0;
  control->max_travel = 0;
  /* Запасы по умолчанию; в рабочем режиме переопределяются MARGIN. */
  control->home_margin = CONTROL_DEFAULT_HOME_MARGIN;
  control->far_margin = CONTROL_DEFAULT_FAR_MARGIN;
  control->encoder_sign = 1;
  control->motor_sign = 1;
  /* Начальные параметры демо. Реальные значения задаёт AUTO или CLI. */
  control->gain_rpm_per_g = 0.20f;
  control->max_rpm = 20;
  control->return_rpm = 1;
  control->deadband_g = 30;
  control->release_g = 15;
  control->max_force_g = CONTROL_DEFAULT_MAX_FORCE_G;
  control->last_cmd_rpm = 0;
  control->last_position = 0;
  control->last_force_g = 0.0f;
  control->last_tick = 0U;
}

/* Задаёт home (край рабочего хода) и пересчитывает направление/длину хода. */
void Control_SetHome(Control *control, int32_t home)
{
  if (control == NULL)
  {
    return;
  }

  control->home_pos = home;
  control->home_valid = 1U;
  control_recalc_limits(control);
}

/* Задаёт far (противоположный край рабочего хода) и пересчитывает пределы. */
void Control_SetFar(Control *control, int32_t far)
{
  if (control == NULL)
  {
    return;
  }

  control->far_pos = far;
  control->far_valid = 1U;
  control_recalc_limits(control);
}

/* Задаёт программные запасы до краёв в counts. Запас не даёт каретке
 * доехать до механического упора на большой скорости. */
void Control_SetMargins(Control *control, int32_t home, int32_t far)
{
  if (control == NULL)
  {
    return;
  }

  /* Отрицательный запас не имеет смысла. */
  if (home < 0)
  {
    home = 0;
  }
  if (far < 0)
  {
    far = 0;
  }
  /* Ограничиваем сверху, чтобы случайно не задать абсурдный запас. */
  if (home > 200000)
  {
    home = 200000;
  }
  if (far > 200000)
  {
    far = 200000;
  }

  control->home_margin = home;
  control->far_margin = far;
}

/* Задаёт знак мотора: +1 или -1 — направление «от home». */
void Control_SetMotorSign(Control *control, int8_t sign)
{
  if (control == NULL)
  {
    return;
  }

  control->motor_sign = (sign < 0) ? -1 : 1;
}

/* Задаёт прибавку скорости на грамм сверх deadband, RPM/г. */
void Control_SetGain(Control *control, float gain)
{
  if (control == NULL)
  {
    return;
  }

  /* Слишком малый gain делает движение рывковым; слишком большой —
   * опасным. Ограничиваем 0.01..2.0 RPM/г. */
  if (gain < 0.01f)
  {
    gain = 0.01f;
  }
  if (gain > 2.0f)
  {
    gain = 2.0f;
  }

  control->gain_rpm_per_g = gain;
}

/* Ограничивает максимальную скорость движения, 1..100 RPM. */
void Control_SetMaxRpm(Control *control, int16_t rpm)
{
  if (control == NULL)
  {
    return;
  }

  if (rpm < 1)
  {
    rpm = 1;
  }
  /* Верхний предел демо. 100 RPM — уже очень быстро для механики. */
  if (rpm > 100)
  {
    rpm = 100;
  }

  control->max_rpm = rpm;
}

/* Задаёт скорость возврата к home, 1..20 RPM. */
void Control_SetReturnRpm(Control *control, int16_t rpm)
{
  if (control == NULL)
  {
    return;
  }

  if (rpm < 1)
  {
    rpm = 1;
  }
  /* Возврат специально медленнее движения, чтобы не бить в home. */
  if (rpm > 20)
  {
    rpm = 20;
  }

  control->return_rpm = rpm;
}

/* Задаёт порог начала движения в граммах. Порог release автоматически
 * подрезается, чтобы оставаться ниже deadband. */
void Control_SetDeadband(Control *control, int16_t grams)
{
  if (control == NULL)
  {
    return;
  }

  /* Слишком маленький deadband ловит шум датчика. */
  if (grams < 2)
  {
    grams = 2;
  }
  if (grams > 500)
  {
    grams = 500;
  }

  control->deadband_g = grams;

  if (control->release_g >= control->deadband_g)
  {
    control->release_g = (int16_t)(control->deadband_g - 1);
  }
}

/* Задаёт порог возврата в граммах. Он не может быть больше deadband. */
void Control_SetRelease(Control *control, int16_t grams)
{
  if (control == NULL)
  {
    return;
  }

  if (grams < 0)
  {
    grams = 0;
  }
  /* release обязан быть строго ниже deadband, иначе получится
   * одновременное «ехать» и «возвращаться». */
  if (grams >= control->deadband_g)
  {
    grams = (int16_t)(control->deadband_g - 1);
  }

  control->release_g = grams;
}

/* Задаёт аварийный порог перегрузки в граммах.
 * Нижняя граница 100 г — рабочий диапазон начинается с 100 г.
 * Верхняя 2000 г — предел тензодатчика YZC-133 (2 кг). */
void Control_SetMaxForce(Control *control, float grams)
{
  if (control == NULL)
  {
    return;
  }

  if (grams < 100.0f)
  {
    grams = 100.0f;
  }
  if (grams > 2000.0f)
  {
    grams = 2000.0f;
  }

  control->max_force_g = grams;
}

/* Проверяет готовность датчика, мотора и пределов и включает демо. */
uint8_t Control_Start(Control *control)
{
  /* Демо можно включить только если:
   *   - есть датчик и мотор;
   *   - датчик инициализирован и откалиброван;
   *   - заданы и корректны пределы home/far. */
  if (control == NULL ||
      control->sensor == NULL ||
      control->motor == NULL ||
      !control->sensor->initialized ||
      !control->sensor->calibrated ||
      !control->limits_valid)
  {
    return 0U;
  }

  control->active = 1U;
  control->read_failures = 0U;
  control->jog_failures = 0U;
  control->last_cmd_rpm = 0;
  control->last_tick = HAL_GetTick();
  control->last_position = 0;
  control->last_force_g = 0.0f;

  return 1U;
}

/* Выключает демо и останавливает мотор. */
void Control_Stop(Control *control)
{
  if (control == NULL)
  {
    return;
  }

  /* Снимаем флаг демо и сбрасываем последнюю команду. */
  control->active = 0U;
  control->last_cmd_rpm = 0;

  if (control->motor != NULL)
  {
    (void)Motor_Stop(control->motor);
  }
}

/* Главный шаг демо:
 *   1) читает силу и позицию;
 *   2) проверяет перегрузку и мягкие пределы;
 *   3) считает целевую скорость;
 *   4) отправляет F6 в MKS только при изменении скорости. */
void Control_Loop(Control *control)
{
  int32_t position = 0;
  int32_t travel;
  float force;
  int16_t target_rpm = 0;

  if (control == NULL || !control->active)
  {
    return;
  }

  /* Работаем не чаще раза в 50 мс. HAL_GetTick() переполняется
   * корректно благодаря беззнаковой разности. */
  if ((HAL_GetTick() - control->last_tick) < CONTROL_PERIOD_MS)
  {
    return;
  }

  control->last_tick = HAL_GetTick();

  /* Читаем силу и позицию. Если ошибка — не дёргаем мотор, а считаем
   * количество подряд идущих ошибок. 20 ошибок × 50 мс = 1 с:
   * краткий сбой после homing демо не выключит. */
  if (!control_read_force(control, &force) ||
      !Motor_ReadPosition(control->motor, &position))
  {
    control->read_failures++;

    if (control->read_failures >= 20U)
    {
      Control_Stop(control);
    }
    return;
  }

  control->read_failures = 0U;
  control->last_force_g = force;
  control->last_position = position;

  /* Перегрузка в любую сторону. Демо не выключаем: когда сила
   * вернётся в норму, логика продолжит работать. */
  if (force > control->max_force_g || force < -control->max_force_g)
  {
    /* Перегрузка: останавливаем движение, но демо не выключаем —
     * после отпускания датчика логика сможет продолжить работу. */
    if (control->last_cmd_rpm != 0)
    {
      (void)Motor_Stop(control->motor);
      control->last_cmd_rpm = 0;
    }
    return;
  }

  /* Переводим абсолютную позицию MKS в «ход от home» 0..max_travel. */
  /* Переводим абсолютную позицию в ход от home. Например,
   * home=258623, far=0, encoder_sign=-1:
   *   position=258623 → travel=0;
   *   position=0      → travel=258623. */
  travel = (position - control->home_pos) * control->encoder_sign;

  /* Защита от ухода за пределы: при ошибке позиции или люфте
   * ограничиваем travel диапазоном 0..max_travel. */
  if (travel < 0)
  {
    travel = 0;
  }
  if (travel > control->max_travel)
  {
    travel = control->max_travel;
  }

  /* Если сила выше порога — едем от home. Скорость растёт с силой,
   * но ограничена снизу 1 RPM и сверху max_rpm. */
  if (force >= (float)control->deadband_g)
  {
    if (travel < (control->max_travel - control->far_margin))
    {
      /* Скорость = (сила − deadband) × gain.
       * Пример: 200 г, deadband 30, gain 0.16 → 27.2 RPM. */
      float speed = (force - (float)control->deadband_g) *
                    control->gain_rpm_per_g;

      /* Минимум 1 RPM: MKS F6 не умеет дробные RPM, а 0 означал бы
       * «стоп». Поэтому любое нажатие выше порога уже двигает. */
      if (speed < 1.0f)
      {
        speed = 1.0f;
      }
      /* Верхний предел защищает механику и тормозной путь. */
      if (speed > (float)control->max_rpm)
      {
        speed = (float)control->max_rpm;
      }

      /* motor_sign задаёт, какой знак F6 соответствует движению
       * «от home». На текущем станке это +1. */
      target_rpm = (int16_t)(control->motor_sign * (int16_t)speed);
    }
  }
  /* Сила упала ниже release — возвращаемся к home, пока не дойдём
   * до домашнего запаса. */
  else if (force <= (float)control->release_g)
  {
    /* Возвращаемся, пока не дойдём до домашнего запаса.
     * Если travel уже меньше запаса — стоим, чтобы не бить в home. */
    if (travel > control->home_margin)
    {
      target_rpm = (int16_t)(-control->motor_sign * control->return_rpm);
    }
  }
  /* Сила между release и deadband — гистерезис: стоим на месте. */
  else
  {
    target_rpm = 0;
  }

  /* Нулевая цель — штатная остановка. Повторно STOP не шлём. */
  if (target_rpm == 0)
  {
    if (control->last_cmd_rpm != 0)
    {
      (void)Motor_Stop(control->motor);
      control->last_cmd_rpm = 0;
    }
    return;
  }

  /* Новую скорость отправляем только при изменении цели, чтобы не
   * забивать RS485 одинаковыми F6-командами каждые 50 мс. */
  /* F6 отправляем только при изменении скорости. Если цель та же,
   * MKS продолжает движение сам, а мы не забиваем RS485. */
  if (target_rpm != control->last_cmd_rpm)
  {
    /* acc=180: разгон/торможение примерно 0.34 с до 90 RPM.
     * При ошибке сбрасываем last_cmd и повторим на следующем шаге. */
    if (!Motor_Jog(control->motor, target_rpm, 180U))
    {
      control->jog_failures++;
      control->last_cmd_rpm = 0;

      if (control->jog_failures >= 20U)
      {
        Control_Stop(control);
      }
      return;
    }

    control->jog_failures = 0U;
    control->last_cmd_rpm = target_rpm;
  }
}

/* Печатает состояние демо для команды CTRL. */
void Control_PrintStatus(Control *control)
{
  int32_t force10;
  int32_t gain100;

  if (control == NULL)
  {
    return;
  }

  force10 = (int32_t)(control->last_force_g * 10.0f +
                      ((control->last_force_g >= 0.0f) ? 0.5f : -0.5f));
  gain100 = (int32_t)(control->gain_rpm_per_g * 100.0f + 0.5f);

  printf("DEMO=%s force=", control->active ? "ON" : "OFF");

  if (force10 < 0)
  {
    printf("-%ld.%01ld g",
           (long)((-force10) / 10),
           (long)((-force10) % 10));
  }
  else
  {
    printf("%ld.%01ld g",
           (long)(force10 / 10),
           (long)(force10 % 10));
  }

  printf(" pos=%ld travel=%ld/%ld home=%ld far=%ld",
         (long)control->last_position,
         (long)((control->last_position - control->home_pos) *
                control->encoder_sign),
         (long)control->max_travel,
         (long)control->home_pos,
         (long)control->far_pos);

  printf(" gain=%ld.%02ld maxrpm=%d dir=%d db=%d rel=%d maxf=%ld marg=%ld/%ld\r\n",
         (long)(gain100 / 100),
         (long)(gain100 % 100),
         (int)control->max_rpm,
         (int)control->motor_sign,
         (int)control->deadband_g,
         (int)control->release_g,
         (long)(control->max_force_g + 0.5f),
         (long)control->home_margin,
         (long)control->far_margin);
}
