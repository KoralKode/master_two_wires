#include "protocol.h"
#include <string.h>
#include "main.h"
#include "power_control.h"
// ===== Формат передаваемых данных FEED_PLAN =====
// Максимум фаз – 32, поэтому маска занимает максимум 4 байта.
//
// Данные после функциональной команды FEED_PLAN:
//   1 байт  – количество фаз питания;
//   4 байта – суммарная длительность фазы питания, мкс;
//   далее   – байты индивидуальной маски.
#define FEED_PLAN_TX_MASK_BYTES_MAX       ((FEED_PLAN_MAX_PHASES + 7U) / 8U)
#define FEED_PLAN_TX_DURATION_BYTES       4U
#define FEED_PLAN_TX_MASK_OFFSET          (1U + FEED_PLAN_TX_DURATION_BYTES)
#define FEED_PLAN_TX_DATA_MAX_SIZE        (FEED_PLAN_TX_MASK_OFFSET + FEED_PLAN_TX_MASK_BYTES_MAX)
// ========== Внутренние состояния автомата протокола ==========
typedef enum {
    IDLE,

    // ===== Обычный сегмент передачи мастером и приёмный сегмент мастера =====
    SEG_START_BIT,
    SEG_DATA_BITS,
    SEG_RECEIVE_DATA_BITS,
    SEG_RECEIVE_CRC_BITS,
    SEG_CRC_BITS,
    SEG_RECV_ALARM,
    SEG_RECV_INTR,
    SEG_RECV_CRC,
    SEG_RECEIVE_ALARM,
    SEG_RECEIVE_INTR,
    SEG_SEND_CRC_ACK,
    SEG_COMPLETE,

    // ===== Обычный пакет =====
    SEQ_ROM,
    SEQ_POWER_AFTER_ROM,
    SEQ_ADDR,
    SEQ_POWER_AFTER_ADDR,
    SEQ_SIZE,
    SEQ_POWER_AFTER_SIZE,
    SEQ_FUNC,
    SEQ_POWER_AFTER_FUNC,
    SEQ_DATA,
    SEQ_POWER_AFTER_DATA,
    SEQ_RX_DATA,
    SEQ_POWER_AFTER_RX_DATA,
    SEQ_WAIT_PHASE,
    SEQ_END,

    // ===== Пакет ROM-search =====
    SEQ_SEARCH_SEG,
    SEQ_POWER_AFTER_SEARCH_SEG,
    SEQ_SEARCH_CONTINUE,

    // ===== Поисковый сегмент =====
    SSEG_START_BIT,
    SSEG_BODY,
    SSEG_PAD_BIT
} state_t;

// ========== Внутренние переменные автомата ==========
static state_t current_seq_state = IDLE;
static state_t global_state = IDLE;

static uint8_t tx_data = 0;
static uint8_t tx_crc = 0;
static uint8_t tx_bit_index = 0;
static uint8_t rx_flags = 0;
static uint8_t current_segment_byte = 0;
static uint8_t half_slot_phase = 0;

// ===== Приём данных мастером =====
// Эти две переменные НЕ static, потому что они доступны из main.c через extern в protocol.h
uint8_t received_data[MAX_RECEIVED_DATA_SIZE] = {0};
uint8_t received_data_count = 0;

static uint8_t rx_data_byte = 0;
static uint8_t rx_crc = 0;
static uint8_t rx_bit_index = 0;
static uint8_t rx_crc_ok = 0;
static uint8_t receive_packet_active = 0;

// ===== Параметры текущего пакета =====
static uint8_t current_rom_cmd = 0;
static uint8_t current_func_cmd = 0;
static uint8_t current_datasize = 0;
static uint8_t *current_data = NULL;
static uint8_t data_index = 0;
static uint8_t current_slave_id[ROM_ID_LEN] = {0};
static uint8_t addr_index = 0;

// ===== Фаза питания =====
// Счётчик хранит количество прерываний TIM2 длительностью 35 мкс.
// uint32_t нужен, потому что реальная фаза может включать
// дополнительное время перестройки источника, например 250000 мкс.
static uint32_t phase_ticks = 0U;
static state_t next_state_after_phase = IDLE;

/*
 * Номер следующей реальной фазы в принятом плане питания.
 *
 * При первой реальной фазе используется индекс 0.
 * После завершения фазы индекс увеличивается и циклически
 * возвращается к нулю после последней фазы.
 */
static volatile uint8_t real_power_phase_index = 0U;

/*
 * Признак того, что выполняемая сейчас фаза является временной моделью
 * реальной фазы питания, а не короткой имитационной фазой.
 *
 * Он фиксируется при запуске фазы, чтобы изменение внешних признаков
 * не влияло на уже начавшийся временной интервал.
 */
static volatile uint8_t active_real_power_phase = 0U;

/*
 * Индекс фазы, которая выполняется в данный момент.
 * На следующем этапе этот индекс понадобится для выбора напряжения
 * и тока PPE-3323.
 */
static volatile uint8_t active_real_power_phase_index = 0U;

/*
 * Внутреннее состояние выполняемой фазы питания.
 *
 * В имитационном режиме используется только
 * POWER_PHASE_STAGE_SIMULATED.
 *
 * В реальном режиме фаза состоит из двух частей:
 * POWER_PHASE_STAGE_REAL_EXTRA  – ключ выключен, источник перестраивается;
 * POWER_PHASE_STAGE_REAL_ACTIVE – ключ включён, линия получает питание.
 */
typedef enum
{
    POWER_PHASE_STAGE_NONE = 0U,
    POWER_PHASE_STAGE_SIMULATED,
    POWER_PHASE_STAGE_REAL_EXTRA,
    POWER_PHASE_STAGE_REAL_ACTIVE
} power_phase_stage_t;

static volatile power_phase_stage_t power_phase_stage =
        POWER_PHASE_STAGE_NONE;

// ===== Повтор сегмента при ошибке CRC =====
static uint8_t retry_needed = 0;

// ===== ROM-search =====
// Эти две переменные НЕ static, потому что они доступны из main.c через extern в protocol.h
uint8_t found_roms[MAX_FOUND_DEVICES][ROM_ID_LEN] = {0};
uint8_t found_rom_count = 0;

// ===== ROM-search_interrupt =====
// Здесь хранятся адреса ведомых устройств, которые выставили бит прерывания.
// Основной массив found_roms при этом не изменяется.
uint8_t interrupt_roms[MAX_FOUND_DEVICES][ROM_ID_LEN] = {0};
uint8_t interrupt_rom_count = 0;

// ===== ROM-alarm =====
// Здесь хранятся адреса ведомых устройств, которые выставили бит alarm.
// Основной список found_roms[] при этом не изменяется.
uint8_t alarm_roms[MAX_FOUND_DEVICES][ROM_ID_LEN] = {0};
uint8_t alarm_rom_count = 0;

// ===== Параметры питания найденных ведомых устройств =====
// Массив power_params[] связан с found_roms[] по индексу.
// Например, параметры устройства found_roms[3]
// хранятся в power_params[3].
power_params_t power_params[MAX_FOUND_DEVICES] = {0};

// Количество устройств, для которых параметры уже успешно приняты.
uint8_t power_params_count = 0U;
// ===== План питания, полученный от ПК =====
// В дальнейшем эти данные будут использоваться для управления источником питания
// и для передачи индивидуальных масок ведомым устройствам.
feed_plan_data_t feed_plan_data = {0};

// ===== Состояние передачи индивидуального плана питания ведомым =====
// 1 – сейчас мастер последовательно отправляет FEED_PLAN найденным ведомым.
static uint8_t feed_plan_send_processing = 0U;

// 1 – пакет FEED_PLAN уже запущен, ждём его завершения.
static uint8_t feed_plan_send_waiting_packet = 0U;

// Индекс ведомого устройства в found_roms[],
// которому сейчас передаётся индивидуальная маска.
static uint8_t feed_plan_send_index = 0U;

// Буфер данных для одного пакета FEED_PLAN.
// Он static, потому что send_pack_async() хранит указатель на данные,
// а пакет передаётся асинхронно через прерывания таймера.
static uint8_t feed_plan_tx_data[FEED_PLAN_TX_DATA_MAX_SIZE] = {0};

// ===== Готовность к реальной фазе питания =====
// Первый признак устанавливается после успешного обычного ROM-search,
// когда в found_roms[] есть хотя бы один адрес.
static volatile uint8_t search_found_once = 0U;

// Второй признак устанавливается только после того, как все пакеты
// FEED_PLAN успешно переданы ведомым устройствам.
static volatile uint8_t feed_plan_sent_ok = 0U;

// Реальная фаза питания может быть разрешена только при выполнении
// обоих условий. На этапе 1 этот признак не меняет длительность
// и способ выполнения фазы: используется прежняя имитационная фаза.
static volatile uint8_t real_power_phase_enabled = 0U;

// ===== Состояние внутренней процедуры запроса параметров =====
// 1 – сейчас выполняется последовательный запрос PARAMETERS
// у найденных ведомых устройств.
static uint8_t power_params_processing = 0U;

// 1 – пакет PARAMETERS уже отправлен, и мастер ждёт,
// когда RX-пакет завершится и данные можно будет разобрать.
static uint8_t power_params_waiting_response = 0U;

// Индекс текущего ведомого устройства в массиве found_roms[],
// у которого сейчас запрашиваются параметры.
static uint8_t power_params_index = 0U;

static uint8_t search_rom_no[ROM_ID_LEN] = {0};
static uint8_t search_last_rom[ROM_ID_LEN] = {0};

static uint8_t search_bit_index = 0;
static uint8_t search_group_index = 0;
static uint8_t search_group_phase = 0;

static uint8_t search_id_bit = 1;
static uint8_t search_cmp_id_bit = 1;
static uint8_t search_direction = 1;

static uint8_t search_last_discrepancy = 0;
static uint8_t search_new_discrepancy = 0;
static uint8_t search_last_device_flag = 0;

static uint8_t search_device_complete = 0;
static uint8_t search_error = 0;
static uint8_t search_done = 0;
static uint8_t search_success = 0;

// ===== Режимы поиска =====
// Обычный поиск, поиск устройств с прерыванием и будущий поиск alarm
static uint8_t search_normal_active = 0U;
static uint8_t search_interrupt_active = 0U;
static uint8_t search_alarm_active = 0U;

// ===== Куда сохранять результат текущего поиска =====
// Обычный ROM-search сохраняет адреса в found_roms.
// ROM-search_interrupt сохраняет адреса в interrupt_roms.
// ROM-alarm в будущем будет сохранять адреса в alarm_roms.
typedef enum
{
    SEARCH_RESULT_FOUND = 0,
    SEARCH_RESULT_INTERRUPT,
    SEARCH_RESULT_ALARM
} search_result_target_t;

static search_result_target_t search_result_target = SEARCH_RESULT_FOUND;

#define SEARCH_ANY_ACTIVE() \
    (search_normal_active || search_interrupt_active || search_alarm_active)

// ===== Обработка бита прерывания =====
typedef enum
{
    INTR_STEP_NONE = 0,
    INTR_STEP_CHECK_NEW_DEVICE,
    INTR_STEP_SEARCH_NEW_DEVICES,
    INTR_STEP_SEARCH_INTERRUPT_DEVICES
} interrupt_step_t;

static volatile uint8_t interrupt_pending = 0U;
static uint8_t interrupt_processing = 0U;
static interrupt_step_t interrupt_step = INTR_STEP_NONE;

// ===== Обработка бита alarm =====
// Alarm имеет приоритет над обычным прерыванием.
// Если alarm принят в конце сегмента, текущий пакет не продолжается:
// после ближайшей фазы питания запускается ROM-alarm.
typedef enum
{
    ALARM_STEP_NONE = 0,
    ALARM_STEP_SEARCH_ALARM_DEVICES
} alarm_step_t;

static volatile uint8_t alarm_pending = 0U;
static uint8_t alarm_processing = 0U;
static alarm_step_t alarm_step = ALARM_STEP_NONE;

// Событие, которое main.c сможет вывести пользователю
volatile uint8_t protocol_event_type = PROTOCOL_EVENT_NONE;

// Признак того, что пользовательский пакет был прерван из-за alarm.
volatile uint8_t protocol_packet_aborted_by_alarm = 0U;

// ========== Работа с линией PB0 ==========
#define DATA_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define DATA_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define DATA_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)

/*
 * TIM2 вызывает protocol_timer_irq_handler() каждые 35 мкс,
 * то есть в два раза чаще длительности одного тайм-слота.
 */
#define POWER_PHASE_TIMER_TICK_US  (TIMESLOT_US / 2U)

// ========== Внутренние прототипы ==========
static void next_step(void);
static void on_segment_done(void);
static void on_received_segment_done(void);

static void start_timer(void);
static void stop_timer(void);

static void start_power_phase(state_t next_state);
static void update_real_power_phase_enabled(void);
static uint32_t power_phase_us_to_timer_ticks(uint32_t duration_us);
static void advance_real_power_phase(void);

static void power_key_on(void);
static void power_key_off(void);
static void start_real_power_active_part(void);

static bool rom_cmd_has_address(uint8_t rom_cmd);

static void start_segment(uint8_t data, uint8_t crc, state_t seq_state);
static void start_received_segment(void);

static uint8_t rom_get_bit(const uint8_t rom[ROM_ID_LEN], uint8_t bit_index);
static void rom_set_bit(uint8_t rom[ROM_ID_LEN], uint8_t bit_index, uint8_t value);

static void search_prepare_next_device(void);
static bool search_choose_direction(void);
static void search_store_found_device(void);
static void start_search_segment(void);
static void on_search_segment_done(void);

static void feed_plan_send_reset_state(void);
static uint8_t feed_plan_get_mask_byte_count(uint8_t phase_count);
static uint8_t feed_plan_prepare_slave_data(uint8_t device_index);
static bool feed_plan_start_next_slave(void);
static void feed_plan_continue_sending(void);

static void handle_service_flags_after_segment(void);
static bool search_packet_async_with_cmd(uint8_t rom_cmd, uint8_t search_type, uint8_t clear_result_array);
static void clear_search_active_flags(void);
static void search_engine_reset(void);
static bool search_new_devices_packet_async(void);

static void clear_rom_list(uint8_t roms[MAX_FOUND_DEVICES][ROM_ID_LEN],
                           uint8_t *count);

static uint8_t rom_is_equal(const uint8_t a[ROM_ID_LEN],
                            const uint8_t b[ROM_ID_LEN]);

static void append_unique_rom(uint8_t roms[MAX_FOUND_DEVICES][ROM_ID_LEN],
                              uint8_t *count,
                              const uint8_t rom[ROM_ID_LEN]);

static uint8_t search_get_current_result_count(void);
static void start_interrupt_processing(void);
static void continue_interrupt_processing(void);

static uint32_t read_u32_be(const uint8_t *buf);

static uint8_t power_params_count_valid(void);
static uint8_t power_params_is_actual_for_index(uint8_t index);
static void power_params_store_from_received(uint8_t index);

static bool power_params_start_next_request(void);
static void power_params_continue_processing(void);

static void start_alarm_processing(void);
static void continue_alarm_processing(void);

// ========== Управление таймером ==========
static void start_timer(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);   // обнуляем счётчик таймера
    HAL_TIM_Base_Start_IT(&htim2);      // запускаем таймер с прерываниями
}

static  void stop_timer(void) {
    HAL_TIM_Base_Stop_IT(&htim2);       // останавливаем таймер
}

// ========== Управление внешним ключом питания ==========

/*
 * Отключение источника питания от линии данных.
 *
 * Эта функция должна вызываться по умолчанию:
 * при запуске программы, перед началом любой фазы,
 * после завершения активной части фазы и при тревоге.
 */
static void power_key_off(void)
{
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,
                      POWER_KEY_Pin,
                      POWER_KEY_OFF_LEVEL);
}


/*
 * Подключение источника питания к линии данных.
 *
 * На текущем этапе этот GPIO только формирует тестовый сигнал.
 * Реальное подключение PPE-3323 через внешний ключ будет добавлено
 * после подключения UART и проверки аппаратной части.
 */
static void power_key_on(void)
{
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port,
                      POWER_KEY_Pin,
                      POWER_KEY_ON_LEVEL);
}

//функция показывающая нужено ли отправлять адрес
static bool rom_cmd_has_address(uint8_t rom_cmd) {
    return (rom_cmd == ROM_MATCH_CMD);
}

// ========== Обновление разрешения реальной фазы питания ==========
// Реальная фаза будет добавлена на следующих этапах.
// Пока признак нужен только для правильного выбора момента перехода:
// после успешного поиска и после передачи FEED_PLAN всем ведомым.
static void update_real_power_phase_enabled(void)
{
    real_power_phase_enabled =
            ((search_found_once != 0U) && (feed_plan_sent_ok != 0U)) ?
            1U : 0U;
}


// ========== Признаки готовности к реальной фазе питания ==========
bool protocol_search_found_once(void)
{
    return (search_found_once != 0U);
}

bool protocol_feed_plan_sent_ok(void)
{
    return (feed_plan_sent_ok != 0U);
}

bool protocol_real_power_phase_enabled(void)
{
    return (real_power_phase_enabled != 0U);
}


// ========== Перевод длительности фазы в количество прерываний TIM2 ==========
static uint32_t power_phase_us_to_timer_ticks(uint32_t duration_us)
{
    uint32_t ticks;

    /*
     * Обработчик TIM2 вызывается каждые 35 мкс.
     * Округляем количество тиков вверх, чтобы фактическая фаза
     * не была короче времени, заданного в плане питания.
     */
    ticks = duration_us / POWER_PHASE_TIMER_TICK_US;

    if ((duration_us % POWER_PHASE_TIMER_TICK_US) != 0U)
    {
        ticks++;
    }

    /*
     * Защита от нулевой длительности.
     * В нормальной работе она не требуется, но не позволит автомату
     * мгновенно завершить фазу при ошибочных входных данных.
     */
    if (ticks == 0U)
    {
        ticks = 1U;
    }

    return ticks;
}

static void start_real_power_active_part(void)
{
    /*
     * PB0 уже отпущен в start_power_phase().
     *
     * PB1 разрешается включать только когда:
     * - параметры VSET1/ISET1 переданы;
     * - OUT1 передан;
     * - UART не сообщил об ошибке.
     */
    if (power_control_is_phase_ready())
    {
        power_key_on();
    }
    else
    {
        /*
         * Источник не успел подготовиться за T_EXTRA.
         *
         * PB1 остаётся выключенным. Линия не подключается к источнику.
         * Продолжительность активного интервала сохраняется, чтобы
         * ведомые не получили фазу меньшей длительности.
         */
        power_key_off();
        power_control_mark_phase_not_ready();
    }

    power_phase_stage = POWER_PHASE_STAGE_REAL_ACTIVE;

    phase_ticks =
            power_phase_us_to_timer_ticks(
                    feed_plan_data.phase_duration_us);
}

// ========== Начало фазы питания ==========
static void start_power_phase(state_t next_state)
{
    /*
     * При старте любой фазы источник должен быть отключён от линии.
     *
     * Это безопасное состояние используется:
     * - при имитационной фазе;
     * - во время перестройки PPE-3323;
     * - перед переходом к следующему сегменту.
     */
    power_key_off();

    /*
     * PB0 должен быть отпущен.
     *
     * В имитационном режиме это сохраняет прежнюю работу протокола.
     * В реальном режиме это необходимо перед подключением источника
     * через внешний ключ.
     */
    DATA_HIGH();

    next_state_after_phase = next_state;
    half_slot_phase = 0U;

    /*
     * Реальная фаза возможна только после:
     * - успешного поиска хотя бы одного ведомого;
     * - успешной передачи FEED_PLAN всем ведомым;
     * - при наличии корректного плана питания.
     *
     * Во время обработки тревоги реальная фаза запрещается:
     * ключ должен оставаться выключенным.
     */
    if ((real_power_phase_enabled != 0U) &&
        power_control_is_automatic_mode() &&
        (feed_plan_data.valid != 0U) &&
        (feed_plan_data.phase_count != 0U) &&
        (feed_plan_data.phase_count <= FEED_PLAN_MAX_PHASES) &&
        (alarm_pending == 0U) &&
        (alarm_processing == 0U))
    {
        /*
         * При некорректном индексе начинаем цикл питания
         * заново с первой фазы.
         */
        if (real_power_phase_index >= feed_plan_data.phase_count)
        {
            real_power_phase_index = 0U;
        }

        active_real_power_phase = 1U;
        active_real_power_phase_index = real_power_phase_index;
        /*
         * Сохраняем запрос настройки текущей фазы.
         *
         * UART-команды не передаются из TIM2 IRQ.
         * Их обработает power_control_poll() в main().
         */
        power_control_request_phase(
                feed_plan_data.phase_voltage[active_real_power_phase_index],
                feed_plan_data.phase_current[active_real_power_phase_index]);
        /*
         * Если дополнительное время равно нулю, активную часть
         * можно начать сразу.
         */
        if (feed_plan_data.phase_extra_duration_us == 0U)
        {
            start_real_power_active_part();
        }
        else
        {
            /*
             * Первая часть реальной фазы:
             * источник отключён от линии и имеет время
             * на перестройку напряжения и ограничения тока.
             */
            power_phase_stage = POWER_PHASE_STAGE_REAL_EXTRA;

            phase_ticks =
                    power_phase_us_to_timer_ticks(
                            feed_plan_data.phase_extra_duration_us);
        }
    }
    else
    {
        /*
         * До передачи FEED_PLAN применяется короткая имитационная фаза.
         *
         * Она используется при первом поиске, получении PARAMETERS
         * и во время передачи самих пакетов FEED_PLAN.
         */
        active_real_power_phase = 0U;
        active_real_power_phase_index = 0U;
        real_power_phase_index = 0U;

        power_phase_stage = POWER_PHASE_STAGE_SIMULATED;

        phase_ticks =
                power_phase_us_to_timer_ticks(
                        SIMULATED_POWER_PHASE_US);
    }

    global_state = SEQ_WAIT_PHASE;
    start_timer();
}

// ========== Переход к следующей реальной фазе питания ==========
static void advance_real_power_phase(void)
{
    /*
     * Имитационная фаза не относится к плану питания,
     * поэтому её завершение не меняет номер фазы.
     */
    if (active_real_power_phase == 0U)
    {
        return;
    }

    /*
     * Если план был сброшен во время ожидания фазы,
     * дальнейшее выполнение цикла питания запрещается.
     */
    if ((!feed_plan_data.valid) ||
        (feed_plan_data.phase_count == 0U) ||
        (feed_plan_data.phase_count > FEED_PLAN_MAX_PHASES))
    {
        real_power_phase_index = 0U;
        active_real_power_phase = 0U;
        active_real_power_phase_index = 0U;

        return;
    }

    /*
     * После завершения фазы переходим к следующей.
     * После последней фазы снова используется фаза с индексом 0.
     */
    real_power_phase_index =
            (uint8_t)(active_real_power_phase_index + 1U);

    if (real_power_phase_index >= feed_plan_data.phase_count)
    {
        real_power_phase_index = 0U;
    }

    active_real_power_phase = 0U;
}





// ========== CRC-5 (MSB first) ==========
uint8_t crc5(uint8_t data) {
    uint8_t crc = 0x00;                 // начальное значение регистра CRC
    for (int i = 7; i >= 0; i--) {      // проходим по битам от старшего к младшему
        uint8_t bit = (data >> i) & 1;  // извлекаем текущий бит данных
        uint8_t msb = (crc >> 4) & 1;   // старший бит текущего CRC (бит 4)
        crc = ((crc << 1) | bit) & 0x1F; // сдвигаем влево и добавляем бит данных, отсекаем лишнее
        if (msb) crc ^= CRC5_POLY;       // если выдвинулась 1, выполняем XOR с полиномом
    }
    return crc;                         // возвращаем 5-битный CRC
}

bool check_crc5(uint8_t data, uint8_t received_crc){
	uint8_t crc=crc5(data);
	return crc==received_crc;
}

//функция очистки переменных для начала приёма новых данных
void clear_received_data(void)
{
    // Очистить весь буфер принятых данных
    memset(received_data, 0, sizeof(received_data));

    // Сбросить количество успешно принятых байтов
    received_data_count = 0;

    // Сбросить индекс байта внутри принимаемого пакета
    data_index = 0;

    // Сбросить переменные текущего приёмного сегмента
    rx_data_byte = 0;
    rx_crc = 0;
    rx_bit_index = 0;
    rx_crc_ok = 0;
}

// ===== Работа с отдельными битами адреса =====
static uint8_t rom_get_bit(const uint8_t rom[ROM_ID_LEN], uint8_t bit_index)
{
    uint8_t byte_index = bit_index / 8U;   // номер байта
    uint8_t bit_in_byte = bit_index % 8U;  // номер бита внутри байта

    return (rom[byte_index] >> bit_in_byte) & 0x01U;
}

static void rom_set_bit(uint8_t rom[ROM_ID_LEN], uint8_t bit_index, uint8_t value)
{
    uint8_t byte_index = bit_index / 8U;
    uint8_t bit_in_byte = bit_index % 8U;

    if (value)
    {
        rom[byte_index] |= (1U << bit_in_byte);   // записать 1
    }
    else
    {
        rom[byte_index] &= ~(1U << bit_in_byte);  // записать 0
    }
}

//сброс флагов поиска
static void clear_search_active_flags(void)
{
    search_normal_active = 0U;
    search_interrupt_active = 0U;
    search_alarm_active = 0U;
}

// ========== Очистка одного списка ROM-адресов ==========
// Используется для found_roms, interrupt_roms и alarm_roms.
static void clear_rom_list(uint8_t roms[MAX_FOUND_DEVICES][ROM_ID_LEN],
                           uint8_t *count)
{
    memset(roms, 0, MAX_FOUND_DEVICES * ROM_ID_LEN);
    *count = 0U;
}


// ========== Сравнение двух ROM-адресов ==========
// Возвращает 1, если четыре байта адреса совпадают.
static uint8_t rom_is_equal(const uint8_t a[ROM_ID_LEN],
                            const uint8_t b[ROM_ID_LEN])
{
    return (memcmp(a, b, ROM_ID_LEN) == 0) ? 1U : 0U;
}


// ========== Добавление ROM-адреса без повторов ==========
// Если такой адрес уже есть в массиве, он повторно не добавляется.
// Это особенно важно при поиске новых устройств: обычный ROM-search
// может снова найти уже известные адреса, но в found_roms их дублировать нельзя.
static void append_unique_rom(uint8_t roms[MAX_FOUND_DEVICES][ROM_ID_LEN],
                              uint8_t *count,
                              const uint8_t rom[ROM_ID_LEN])
{
    // Сначала проверяем, нет ли такого адреса в массиве.
    for (uint8_t i = 0U; i < *count; i++)
    {
        if (rom_is_equal(roms[i], rom))
        {
            return;
        }
    }

    // Если адрес новый и место ещё есть, добавляем его в конец массива.
    if (*count < MAX_FOUND_DEVICES)
    {
        memcpy(roms[*count], rom, ROM_ID_LEN);
        (*count)++;
    }
}


// ========== Количество адресов в массиве текущего поиска ==========
// Нужно для завершения поиска при заполнении соответствующего массива.
static uint8_t search_get_current_result_count(void)
{
    if (search_result_target == SEARCH_RESULT_INTERRUPT)
    {
        return interrupt_rom_count;
    }

    if (search_result_target == SEARCH_RESULT_ALARM)
    {
        return alarm_rom_count;
    }

    return found_rom_count;
}

// ========== Обработка служебных битов после завершения сегмента ==========
// Служебные биты читаются в конце каждого обычного сегмента.
// Здесь мастер только фиксирует события, а не выполняет всю обработку сразу.
//
// Приоритеты:
// 1. Alarm – самый высокий приоритет.
// 2. CRC/retry – повтор текущего сегмента.
// 3. Interrupt – отложенная обработка после завершения пакета.
static void handle_service_flags_after_segment(void)
{
    // Во время поисковых процедур новые служебные события не запускаем.
    // ROM-search, ROM-search_interrupt и ROM-alarm должны выполняться
    // как отдельные завершённые операции.
    if (SEARCH_ANY_ACTIVE() || interrupt_processing || alarm_processing)
    {
        return;
    }

    // Бит alarm имеет приоритет над CRC и обычным прерыванием.
    // Если alarm и interrupt пришли одновременно, interrupt не фиксируем.
    if (rx_flags & 0x01U)
    {
        /*
         * Тревога имеет приоритет над обычным обменом.
         *
         * Ключ отключается сразу, ещё до запуска ROM-alarm.
         * На текущем этапе это выключает тестовый GPIO PB1.
         * После подключения оборудования это сразу отсоединит
         * PPE-3323 от линии данных.
         */
    	/*
    	 * PB1 отключается сразу.
    	 * OUT0 будет поставлен в очередь из main().
    	 */
    	power_key_off();
    	power_control_disable();

        alarm_pending = 1U;

        // Обычное прерывание не должно быть обработано раньше alarm.
        interrupt_pending = 0U;

        return;
    }

    // Если alarm нет, а сегмент должен повторяться из-за CRC,
    // бит interrupt из этого сегмента не используем.
    if (retry_needed)
    {
        return;
    }

    // Бит прерывания не прерывает текущий пакет.
    // Он только фиксируется и будет обработан после завершения пакета.
    if (rx_flags & 0x02U)
    {
        interrupt_pending = 1U;
    }
}

// ========== Сброс только внутренней логики алгоритма поиска ==========
// Эта функция НЕ очищает массивы found_roms, interrupt_roms и alarm_roms.
// Она очищает только переменные, которые нужны для одного прохода ROM-search.
static void search_engine_reset(void)
{
    memset(search_rom_no, 0, ROM_ID_LEN);
    memset(search_last_rom, 0, ROM_ID_LEN);

    search_bit_index = 0U;
    search_group_index = 0U;
    search_group_phase = 0U;

    search_id_bit = 1U;
    search_cmp_id_bit = 1U;
    search_direction = 1U;

    search_last_discrepancy = 0U;
    search_new_discrepancy = 0U;
    search_last_device_flag = 0U;

    search_device_complete = 0U;
    search_error = 0U;
    search_done = 0U;
    search_success = 0U;

    clear_search_active_flags();

    addr_index = 0U;
    data_index = 0U;
    rx_flags = 0U;
    retry_needed = 0U;
    half_slot_phase = 0U;
}


// ========== Полный сброс результатов обычного поиска ==========
// Эту функцию можно вызывать при инициализации или перед ручным PACK SEARCH,
// если нужно очистить все результаты поиска.
void search_rom_packet_reset(void)
{
    clear_rom_list(found_roms, &found_rom_count);
    clear_rom_list(interrupt_roms, &interrupt_rom_count);
    clear_rom_list(alarm_roms, &alarm_rom_count);
    // После полного сброса актуального списка устройств нет.
    // Поэтому реальная фаза питания не может быть разрешена.
    search_found_once = 0U;
    update_real_power_phase_enabled();

    // Полный сброс поиска должен также очищать параметры питания,
    // потому что они связаны с адресами из found_roms[] по индексу.
    power_params_reset();
    // План питания связан со списком found_roms[].
    // При полном сбросе поиска старый план больше не считается актуальным.
    feed_plan_reset();
    search_result_target = SEARCH_RESULT_FOUND;
    search_engine_reset();

}

// ===== Подготовка поиска следующего адреса =====
static void search_prepare_next_device(void)
{
    // Начинаем новый проход поиска с предыдущего найденного адреса
    memcpy(search_rom_no, search_last_rom, ROM_ID_LEN);

    // Начинаем снова с первого бита адреса
    search_bit_index = 0;

    // В новом поисковом сегменте стартуем с первой тройки
    search_group_index = 0;
    search_group_phase = 0;

    search_id_bit = 1;
    search_cmp_id_bit = 1;
    search_direction = 1;

    // Для нового прохода начинаем искать новую последнюю коллизию
    search_new_discrepancy = 0;

    search_device_complete = 0;
    search_error = 0;
}

// ===== Выбор направления поиска по паре битов =====
// Функция для выбора направления поиска в алгоритме Search ROM
// ===== Выбор направления поиска по паре битов =====
static bool search_choose_direction(void)
{
    // Сочетание 1/1 означает ошибку:
    // либо устройств больше нет, либо ведомые ответили неверно
    if ((search_id_bit == 1U) && (search_cmp_id_bit == 1U))
    {
        return false;
    }

    // Если биты различаются (0/1 или 1/0), коллизии нет
    if (search_id_bit != search_cmp_id_bit)
    {
        search_direction = search_id_bit;
    }
    else
    {
        // Случай 0/0: коллизия, есть устройства и с 0, и с 1

        if ((search_bit_index + 1U) < search_last_discrepancy)
        {
            // До старой коллизии повторяем старое направление
            search_direction = rom_get_bit(search_last_rom, search_bit_index);
        }
        else if ((search_bit_index + 1U) == search_last_discrepancy)
        {
            // На месте старой коллизии идём по противоположной ветке
            search_direction = 1U;
        }
        else
        {
            // Для новой коллизии по умолчанию идём по ветви 0
            search_direction = 0U;
        }

        // Если выбрали 0, запоминаем новую последнюю коллизию
        if (search_direction == 0U)
        {
            search_new_discrepancy = search_bit_index + 1U;
        }
    }

    // Записываем выбранный бит в собираемый адрес
    rom_set_bit(search_rom_no, search_bit_index, search_direction);

    return true;
}

// ===== Сохранение найденного адреса =====
static void search_store_found_device(void)
{
    // Этот адрес станет "предыдущим найденным" для следующего прохода.
    // Это нужно самому алгоритму ROM-search для обхода дерева адресов.
    memcpy(search_last_rom, search_rom_no, ROM_ID_LEN);

    // Сохраняем найденный адрес в массив, соответствующий текущему режиму поиска.
    if (search_result_target == SEARCH_RESULT_INTERRUPT)
    {
        // Поиск устройств с флагом прерывания.
        // Адреса сохраняются отдельно и не попадают в основной список устройств.
        append_unique_rom(interrupt_roms,
                          &interrupt_rom_count,
                          search_rom_no);
    }
    else if (search_result_target == SEARCH_RESULT_ALARM)
    {
    	// Поиск устройств с флагом аварии.
    	// Адреса сохраняются отдельно и не попадают в основной список устройств.
        append_unique_rom(alarm_roms,
                          &alarm_rom_count,
                          search_rom_no);
    }
    else
    {
        // Обычный ROM-search.
        // При ручном PACK SEARCH массив found_roms заранее очищается.
        // При поиске после NEW_DEVICE массив не очищается,
        // поэтому сюда добавятся только новые адреса без дублей.
        append_unique_rom(found_roms,
                          &found_rom_count,
                          search_rom_no);
    }

    // Обновляем позицию последней коллизии.
    search_last_discrepancy = search_new_discrepancy;

    // Если новых коллизий не осталось, значит это было последнее устройство.
    if (search_last_discrepancy == 0U)
    {
        search_last_device_flag = 1U;
    }
}

// ===== Запуск одного поискового сегмента =====
static void start_search_segment(void)
{
    search_group_index = 0;  // начинаем с первой тройки
    search_group_phase = 0;  // сначала читаем id_bit
    half_slot_phase = 0;     // первая половина стартового слота
    global_state = SSEG_START_BIT;
    start_timer();
}

// ===== Завершение одного поискового сегмента =====
static void on_search_segment_done(void)
{
    // Если произошла ошибка, завершаем весь пакет поиска
    if (search_error)
    {
    	clear_search_active_flags();
        search_done = 1U;
        search_success = 0U;

        global_state = SEQ_END;
        next_step();
        return;
    }

    // Иначе после поискового сегмента идёт фаза питания
    global_state = SEQ_POWER_AFTER_SEARCH_SEG;
    next_step();
}

// ===== Запуск полного пакета поиска =====
// rom_cmd:
//   ROM_SEARCH_CMD           – обычный поиск устройств;
//   ROM_SEARCH_INTERRUPT_CMD – поиск устройств с флагом прерывания;
//   ROM_ALARM_CMD            – будущий поиск устройств с флагом аварии.
//
// search_type:
//   0 – обычный поиск;
//   1 – поиск устройств с прерыванием;
//   2 – поиск устройств с аварией.
//
// clear_result_array:
//   1 – перед поиском очистить массив результата;
//   0 – не очищать массив результата, а добавлять новые адреса без дублей.
static bool search_packet_async_with_cmd(uint8_t rom_cmd,
                                         uint8_t search_type,
                                         uint8_t clear_result_array)
{
    if (global_state != IDLE)
    {
        return false;
    }

    // Сбрасываем только внутреннюю механику ROM-search,
    // но не трогаем массивы результатов.
    search_engine_reset();

    if (search_type == 0U)
    {
        // Обычный поиск устройств.
        search_normal_active = 1U;
        search_result_target = SEARCH_RESULT_FOUND;

        // При ручном PACK SEARCH массив нужно очистить.
        // При поиске после NEW_DEVICE очищать нельзя:
        // найденные новые адреса должны добавиться к уже известным.
        if (clear_result_array)
        {
            clear_rom_list(found_roms, &found_rom_count);

            // Новый полный поиск начинает формирование списка заново.
            // До успешного окончания поиска реальная фаза питания запрещена.
            search_found_once = 0U;
            update_real_power_phase_enabled();

            // При полном новом поиске адресов старые параметры питания
            // больше не считаются актуальными, потому что список found_roms[]
            // будет сформирован заново.
            power_params_reset();

            // Старый план питания тоже больше не актуален,
            // потому что он был связан с прежним порядком found_roms[].
            feed_plan_reset();
        }
    }
    else if (search_type == 1U)
    {
        // Поиск устройств, выставивших бит прерывания.
        search_interrupt_active = 1U;
        search_result_target = SEARCH_RESULT_INTERRUPT;

        // Каждый новый ROM-search_interrupt должен давать новый список
        // устройств с прерыванием.
        if (clear_result_array)
        {
            clear_rom_list(interrupt_roms, &interrupt_rom_count);
        }
    }
    else if (search_type == 2U)
    {
        // Будущий поиск устройств с флагом аварии.
        search_alarm_active = 1U;
        search_result_target = SEARCH_RESULT_ALARM;

        if (clear_result_array)
        {
            clear_rom_list(alarm_roms, &alarm_rom_count);
        }
    }
    else
    {
        return false;
    }

    // Это не приёмный пакет.
    receive_packet_active = 0U;

    // Сброс общих переменных перед стартом поискового пакета.
    addr_index = 0U;
    data_index = 0U;
    rx_flags = 0U;
    retry_needed = 0U;
    half_slot_phase = 0U;

    // Подготавливаем первый проход поиска.
    search_prepare_next_device();

    // Первый сегмент пакета – ROM-команда поиска.
    current_rom_cmd = rom_cmd;
    global_state = SEQ_ROM;
    next_step();

    return true;
}

// ===== Обычный поиск устройств по команде пользователя =====
// Перед таким поиском основной список found_roms очищается.
bool search_rom_packet_async(void)
{
    return search_packet_async_with_cmd(ROM_SEARCH_CMD,
                                        0U,
                                        1U);
}


// ===== Поиск новых устройств после команды NEW_DEVICE =====
// Этот поиск использует обычный ROM-search, но НЕ очищает found_roms.
// Если ROM-search снова найдёт старые адреса, они не будут добавлены повторно.
// Если появятся новые адреса, они добавятся в found_roms.
static bool search_new_devices_packet_async(void)
{
    return search_packet_async_with_cmd(ROM_SEARCH_CMD,
                                        0U,
                                        0U);
}


// ===== Поиск устройств с флагом прерывания =====
// Результаты записываются в interrupt_roms.
bool search_interrupt_packet_async(void)
{
    return search_packet_async_with_cmd(ROM_SEARCH_INTERRUPT_CMD,
                                        1U,
                                        1U);
}

// ===== Поиск устройств с флагом аварии =====
// Отправляется ROM_ALARM_CMD.
// В поиске участвуют только ведомые устройства,
// у которых установлен флаг alarm.
// Результаты записываются в alarm_roms[].
bool search_alarm_packet_async(void)
{
    return search_packet_async_with_cmd(ROM_ALARM_CMD,
                                        2U,
                                        1U);
}

// ===== Блокирующая оболочка для отладки =====
bool search_rom_packet_blocking(void)
{
    if (!search_rom_packet_async())
    {
        return false;
    }

    while (!search_done)
    {
    }

    return search_success;
}

// ========== Запуск одного сегмента ==========
static  void start_segment(uint8_t data, uint8_t crc, state_t seq_state) {
	//здесь необходимо запомнить все данные сегмента перед его отправкой
    current_seq_state = seq_state;   // запоминаем этап последовательности
    tx_data = data;
    current_segment_byte = data;   // сохраняем байт для retry
    tx_crc = crc;
    tx_bit_index = 0;
    rx_flags = 0;
    retry_needed = 0;
    half_slot_phase = 0;
    global_state = SEG_START_BIT;
    start_timer();
}

// ========== Запуск приёмного сегмента ==========
// Приёмный сегмент имеет структуру:
// стартовый бит -> 8 бит данных от ведомого -> 5 бит CRC от ведомого
// -> 2 служебных бита от ведомых -> 1 бит подтверждения CRC от мастера
static void start_received_segment()
{
    // Запоминаем, на каком высокоуровневом этапе мы находимся
    current_seq_state = SEQ_RX_DATA;

    // Сбрасываем собираемый байт данных и принимаемый CRC
    rx_data_byte = 0;
    rx_crc = 0;

    // Сбрасываем индекс принимаемого бита
    rx_bit_index = 0;

    // Сбрасываем служебные флаги alarm/interrupt
    rx_flags = 0;

    // По умолчанию считаем, что CRC ещё не подтверждён
    rx_crc_ok = 0;

    // Повтор пока не нужен
    retry_needed = 0;

    // Начинаем с первой половины стартового слота
    half_slot_phase = 0;

    // Стартовый бит общий для передающего и приёмного сегмента
    global_state = SEG_START_BIT;

    start_timer();
}

// ========== Колбэк завершения передающего сегмента ==========
static void on_segment_done(void)
{
    // Сначала анализируем служебные биты.
    // Это важно для alarm: он имеет приоритет даже над повтором по CRC.
    handle_service_flags_after_segment();

    // Если принят бит alarm, текущий пакет не продолжаем.
    // После фазы питания пакет завершается, а protocol_poll()
    // запустит процедуру ROM-alarm.
    if (alarm_pending)
    {
        protocol_packet_aborted_by_alarm = 1U;
        start_power_phase(SEQ_END);
        return;
    }

    // Если alarm нет, но ведомое устройство не подтвердило CRC,
    // повторяем текущий сегмент.
    if (retry_needed)
    {
        start_segment(current_segment_byte,
                      crc5(current_segment_byte),
                      current_seq_state);
        return;
    }

    // Если ошибок нет, переходим к следующему этапу пакета.
    switch (current_seq_state)
    {
    case SEQ_ROM:
        global_state = SEQ_POWER_AFTER_ROM;
        next_step();
        break;

    case SEQ_ADDR:
        addr_index++;
        global_state = SEQ_POWER_AFTER_ADDR;
        next_step();
        break;

    case SEQ_SIZE:
        global_state = SEQ_POWER_AFTER_SIZE;
        next_step();
        break;

    case SEQ_FUNC:
        global_state = SEQ_POWER_AFTER_FUNC;
        next_step();
        break;

    case SEQ_DATA:
        data_index++;
        global_state = SEQ_POWER_AFTER_DATA;
        next_step();
        break;

    default:
        global_state = IDLE;
        break;
    }
}

// ========== Завершение приёмного сегмента ==========
static void on_received_segment_done(void)
{
    // Сначала анализируем служебные биты.
    // Если пришёл alarm, он имеет приоритет над CRC и обычным продолжением пакета.
    handle_service_flags_after_segment();

    // Если принят бит alarm, текущий RX-пакет не продолжаем.
    // После фазы питания пакет завершится, а затем будет запущен ROM-alarm.
    if (alarm_pending)
    {
        protocol_packet_aborted_by_alarm = 1U;
        start_power_phase(SEQ_END);
        return;
    }

    // Если CRC неверный, байт не сохраняем и не увеличиваем data_index.
    // После фазы питания снова запустится приём того же сегмента.
    if (retry_needed)
    {
        global_state = SEQ_POWER_AFTER_RX_DATA;
        next_step();
        return;
    }

    // CRC верный – сохраняем принятый байт.
    if (data_index < MAX_RECEIVED_DATA_SIZE)
    {
        received_data[data_index] = rx_data_byte;
    }

    // Счётчики успешного приёма увеличиваются только после корректного CRC.
    data_index++;
    received_data_count = data_index;

    // Переходим к фазе питания после принятого сегмента.
    global_state = SEQ_POWER_AFTER_RX_DATA;
    next_step();
}

static void next_step(void) {
    uint8_t value;
    switch (global_state) {
    //отправка ROM-команды
    case SEQ_ROM:
         start_segment(current_rom_cmd, crc5(current_rom_cmd), SEQ_ROM);// отправляем rom команду
         break;
    //фаза питания после ROM-команды
    case SEQ_POWER_AFTER_ROM:
        // Если активен специальный пакет поиска,
        // после ROM-команды переходим к поисковым сегментам.
        if (SEARCH_ANY_ACTIVE() &&
            ((current_rom_cmd == ROM_SEARCH_CMD) ||
             (current_rom_cmd == ROM_SEARCH_INTERRUPT_CMD) ||
             (current_rom_cmd == ROM_ALARM_CMD)))
        {
            start_power_phase(SEQ_SEARCH_SEG);
        }
        else if (rom_cmd_has_address(current_rom_cmd))
        {
            start_power_phase(SEQ_ADDR);
        }
        else
        {
            start_power_phase(SEQ_SIZE);
        }
        break;
    //отправка адреса при необходимости
    case SEQ_ADDR:
        value = current_slave_id[addr_index];
        start_segment(value, crc5(value), SEQ_ADDR);
        break;
    //фаза питания после отправки части адреса
    case SEQ_POWER_AFTER_ADDR:
        if (addr_index < ROM_ID_LEN) {
            start_power_phase(SEQ_ADDR);
        } else {
            start_power_phase(SEQ_SIZE);
        }
        break;
        // Отправка поля размера.
        // Поле размера содержит только количество сегментов данных,
        // следующих после функциональной команды.
        // Сама функциональная команда в это количество не входит.
        case SEQ_SIZE:
            start_segment(current_datasize,
                          crc5(current_datasize),
                          SEQ_SIZE);
            break;
    //фаза питания после отправки размера
    case SEQ_POWER_AFTER_SIZE:
    	start_power_phase(SEQ_FUNC);
        break;

    //отправка функциональной команды
    case SEQ_FUNC:
        start_segment(current_func_cmd, crc5(current_func_cmd), SEQ_FUNC);        // отправляем
        break;
    //фаза питания после функциональной команды
    case SEQ_POWER_AFTER_FUNC:
        if (current_datasize > 0U)
        {
            if (receive_packet_active)
            {
                // Если это пакет приёма, после функции начинаем принимать данные
                start_power_phase(SEQ_RX_DATA);
            }
            else
            {
                // Если это обычный пакет передачи, мастер отправляет данные
                start_power_phase(SEQ_DATA);
            }
        }
        else
        {
            start_power_phase(SEQ_END);
        }
        break;
    //отправка данных
    case SEQ_DATA:
        if (data_index < current_datasize) {
            start_segment(current_data[data_index], crc5(current_data[data_index]),SEQ_DATA); // отправляем сегмент
        }
        break;
    case SEQ_POWER_AFTER_DATA:
        if (data_index < current_datasize) {
            start_power_phase(SEQ_DATA);
        } else {
            start_power_phase(SEQ_END);//все данные отправлены
        }
        break;
    case SEQ_RX_DATA:
        if (data_index < current_datasize)
        {
            // Запускаем приём очередного сегмента данных от ведомого
            start_received_segment();
        }
        else
        {
            // Все данные приняты
            start_power_phase(SEQ_END);
        }
        break;

    case SEQ_POWER_AFTER_RX_DATA:
        if (data_index < current_datasize)
        {
            // Между всеми приёмными сегментами сохраняем фазу питания.
            // Если CRC был неверный, data_index не увеличился,
            // поэтому будет повтор того же сегмента.
            start_power_phase(SEQ_RX_DATA);
        }
        else
        {
            // Все запрошенные сегменты успешно приняты
            start_power_phase(SEQ_END);
        }
        break;
    case SEQ_SEARCH_SEG:
        // Запустить очередной поисковый сегмент
        start_search_segment();
        break;

    case SEQ_POWER_AFTER_SEARCH_SEG:
        // После поискового сегмента даём фазу питания
        start_power_phase(SEQ_SEARCH_CONTINUE);
        break;

    case SEQ_SEARCH_CONTINUE:
        // Если в поиске произошла ошибка, завершаем пакет
        if (search_error)
        {
        	clear_search_active_flags();
            search_done = 1U;
            search_success = 0U;

            global_state = SEQ_END;
            next_step();
            break;
        }

        // Если адрес текущего устройства уже полностью найден
        if (search_device_complete)
        {
            // Сохранить найденный адрес
            search_store_found_device();

            // Если найдено последнее устройство или массив переполнен,
            // завершаем весь пакет поиска
            if (search_last_device_flag || (search_get_current_result_count() >= MAX_FOUND_DEVICES))
            {
            	clear_search_active_flags();
                search_done = 1U;
                search_success = (search_get_current_result_count() > 0U) ? 1U : 0U;
                /*
                 * Для обычного ROM-search фиксируем, что существует
                 * актуальный список хотя бы с одним ведомым устройством.
                 *
                 * Поиск по флагу прерывания и ROM-alarm этот признак
                 * не изменяют, так как они не формируют found_roms[].
                 */
                if (search_result_target == SEARCH_RESULT_FOUND)
                {
                    search_found_once = (found_rom_count > 0U) ? 1U : 0U;
                    update_real_power_phase_enabled();
                }
                global_state = SEQ_END;
                next_step();
            }
            else
            {
                // Иначе начинаем поиск следующего устройства в этом же пакете
                search_prepare_next_device();
                global_state = SEQ_SEARCH_SEG;
                next_step();
            }
        }
        else
        {
            // Если текущий адрес ещё не собран полностью,
            // продолжаем его поиск следующим сегментом
            global_state = SEQ_SEARCH_SEG;
            next_step();
        }
        break;
    //конец сегмента
    case SEQ_END:
    	DATA_HIGH();
        global_state = IDLE;                          // возвращаемся в бездействие
        stop_timer();
        break;

    default:
        break;
    }
}

// ========== Функция send_pack_async (вызывает next_step) ==========
bool send_pack_async(uint8_t rom_cmd, uint8_t func_cmd, uint8_t datasize, uint8_t *data, uint8_t *slave_id) {
    if (global_state != IDLE) {
        return false; // мастер ещё занят предыдущим пакетом
    }
    // Это обычный пакет передачи, а не пакет приёма и не ROM-search
    receive_packet_active = 0U;
    clear_search_active_flags();

    // Сброс общих переменных перед новым пакетом
    addr_index = 0;
    data_index = 0; // начинаем с первого байта
    rx_flags = 0;
    retry_needed = 0;
    half_slot_phase = 0;

    current_rom_cmd = rom_cmd;
    current_func_cmd = func_cmd;        // запоминаем функциональную команду
    current_datasize = datasize;        // запоминаем размер данных
    current_data = data;                // запоминаем указатель на данные

    // Если передан адрес, сохраняем его, иначе используем нулевой адрес
    if (slave_id != NULL) {
        memcpy(current_slave_id, slave_id, ROM_ID_LEN);
    } else {
        memset(current_slave_id, 0, ROM_ID_LEN);  // Если адрес не нужен, передаем ноль
    }



    global_state = SEQ_ROM;             // начинаем с отправки ROM-команды
    next_step();                        // запускаем последовательность
    return true;
}

// ========== Запуск пакета приёма данных от конкретного ведомого ==========
// Последовательность пакета:
// ROM_MATCH -> адрес -> размер принимаемых данных + 1 -> функциональная команда
// -> приём current_datasize сегментов данных
bool receive_pack_async(uint8_t func_cmd, uint8_t receive_size, uint8_t *slave_id)
{
    if (global_state != IDLE)
    {
        return false; // мастер занят
    }

    // Приём без адреса невозможен, потому что работаем через ROM_MATCH
    if (slave_id == NULL)
    {
        return false;
    }

    // Проверяем, что размер не превышает буфер
    if (receive_size == 0U || receive_size > MAX_RECEIVED_DATA_SIZE)
    {
        return false;
    }

    // Очистить буфер перед новым приёмом
    clear_received_data();

    // Включаем режим приёма
    receive_packet_active = 1U;

    // Это пакет приёма, а не ROM-search
    clear_search_active_flags();

    // Сброс служебных переменных перед новым приёмом
    rx_flags = 0;
    retry_needed = 0;
    half_slot_phase = 0;

    // Пакет приёма всегда начинается с ROM_MATCH
    current_rom_cmd = ROM_MATCH_CMD;

    // Функциональная команда говорит ведомому, какие данные надо выдать
    current_func_cmd = func_cmd;

    // current_datasize здесь означает количество принимаемых сегментов данных
    current_datasize = receive_size;

    // При приёме мастер не использует current_data
    current_data = NULL;

    // Индексы начинаются с нуля
    data_index = 0;
    addr_index = 0;

    // Сохраняем адрес ведомого
    memcpy(current_slave_id, slave_id, ROM_ID_LEN);



    // Запускаем обычный пакетный автомат с ROM-команды
    global_state = SEQ_ROM;
    next_step();

    return true;
}

// ========== Запуск пакета приёма данных от всех ведомых устройств ==========
// Эта функция похожа на receive_pack_async(), но работает не через ROM_MATCH,
// а через ROM_SKIP.
//
// Структура пакета:
// ROM_SKIP -> фаза питания -> размер -> фаза питания ->
// функциональная команда -> фаза питания -> приём данных.
//
// Используется для команды NEW_DEVICE.
// Эта команда должна быть общей для всех ведомых устройств.
// Если на линии данных есть новое ведомое устройство, оно удержит линию в 0,
// и мастер примет байт 0x00.
bool receive_pack_skip_async(uint8_t func_cmd, uint8_t receive_size)
{
    // Нельзя запускать новый пакет, пока автомат протокола занят.
    if (global_state != IDLE)
    {
        return false;
    }

    // Приём нулевого количества байтов не имеет смысла.
    // Также нельзя выходить за пределы буфера received_data[].
    if ((receive_size == 0U) || (receive_size > MAX_RECEIVED_DATA_SIZE))
    {
        return false;
    }

    // Перед новым приёмом очищаем буфер и счётчик принятых байтов.
    clear_received_data();

    // Включаем режим приёмного пакета.
    // После функциональной команды автомат перейдёт не к SEQ_DATA,
    // а к SEQ_RX_DATA и начнёт принимать байты от ведомых устройств.
    receive_packet_active = 1U;

    // Это не поисковый пакет, поэтому все флаги поиска сбрасываются.
    clear_search_active_flags();

    // Сбрасываем служебные признаки перед началом нового пакета.
    rx_flags = 0U;
    retry_needed = 0U;
    half_slot_phase = 0U;

    // Главное отличие от receive_pack_async():
    // здесь используется ROM_SKIP, поэтому пакет адресован всем ведомым.
    current_rom_cmd = ROM_SKIP_CMD;

    // Функциональная команда, которую должны увидеть все ведомые устройства.
    // Для обработки прерывания здесь будет NEW_DEVICE.
    current_func_cmd = func_cmd;

    // current_datasize в приёмном пакете означает,
    // сколько байтов мастер должен принять после функциональной команды.
    current_datasize = receive_size;

    // При приёме мастер не отправляет массив данных.
    current_data = NULL;

    // Индексы начинаются с нуля.
    data_index = 0U;
    addr_index = 0U;

    // Адрес не используется, потому что ROM_SKIP не требует адреса.
    memset(current_slave_id, 0, ROM_ID_LEN);

    // Запускаем общий автомат пакета с отправки ROM-команды.
    global_state = SEQ_ROM;
    next_step();

    return true;
}


// ========== Чтение uint32_t из четырёх байтов ==========
// Параметры времени передаются старшим байтом вперёд:
// buf[0] – старший байт, buf[3] – младший байт.
//
// Такой порядок выбран по аналогии с передачей адреса:
// сначала передаются старшие части, затем младшие.
static uint32_t read_u32_be(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) |
           ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8)  |
           ((uint32_t)buf[3]);
}


// ========== Подсчёт принятых параметров питания ==========
// Возвращает количество ячеек power_params[],
// в которых valid == 1.
static uint8_t power_params_count_valid(void)
{
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < MAX_FOUND_DEVICES; i++)
    {
        if (power_params[i].valid)
        {
            count++;
        }
    }

    return count;
}


// ========== Проверка актуальности параметров для найденного адреса ==========
// Параметры считаются актуальными, если:
// 1. для этой ячейки valid == 1;
// 2. ROM-адрес внутри power_params[i] совпадает с found_roms[i].
//
// Это нужно для подключения новых устройств:
// если старые адреса уже имеют параметры, повторно их не запрашиваем,
// а параметры новых адресов будут приняты отдельно.
static uint8_t power_params_is_actual_for_index(uint8_t index)
{
    if (index >= found_rom_count)
    {
        return 0U;
    }

    if (!power_params[index].valid)
    {
        return 0U;
    }

    if (memcmp(power_params[index].rom, found_roms[index], ROM_ID_LEN) != 0)
    {
        return 0U;
    }

    return 1U;
}


// ========== Сохранение принятых параметров питания ==========
// Функция вызывается после завершения RX-пакета PARAMETERS.
// В этот момент received_data[] уже содержит принятые байты.
//
// Формат received_data[]:
//   [0]     – напряжение;
//   [1]     – ток;
//   [2..5]  – время заряда, uint32_t, мкс;
//   [6..9]  – время работы, uint32_t, мкс.
static void power_params_store_from_received(uint8_t index)
{
    if (index >= MAX_FOUND_DEVICES)
    {
        return;
    }

    // Если принято не 10 байт, параметры считаем некорректными.
    // Такое возможно, если пакет был прерван alarm или была другая ошибка.
    if (received_data_count != POWER_PARAMS_DATA_SIZE)
    {
        power_params[index].valid = 0U;
        power_params_count = power_params_count_valid();
        return;
    }

    // Сохраняем адрес, к которому относятся параметры.
    memcpy(power_params[index].rom, found_roms[index], ROM_ID_LEN);

    // Напряжение и ток по линии пришли как байты,
    // но в мастере хранятся как uint32_t.
    power_params[index].voltage = (uint32_t)received_data[0];
    power_params[index].current = (uint32_t)received_data[1];

    // Времена передаются четырьмя байтами, старший байт первым.
    power_params[index].charge_time_us = read_u32_be(&received_data[2]);
    power_params[index].work_time_us   = read_u32_be(&received_data[6]);

    // Отмечаем, что параметры этой ячейки успешно получены.
    power_params[index].valid = 1U;

    // Обновляем счётчик успешно принятых параметров.
    power_params_count = power_params_count_valid();
}


// ========== Сброс массива параметров питания ==========
// Используется при полном новом поиске устройств.
// После сброса все параметры считаются неизвестными.
void power_params_reset(void)
{
    memset(power_params, 0, sizeof(power_params));
    power_params_count = 0U;

    power_params_processing = 0U;
    power_params_waiting_response = 0U;
    power_params_index = 0U;
}

// ========== Сброс внутреннего состояния передачи FEED_PLAN ==========
static void feed_plan_send_reset_state(void)
{
    feed_plan_send_processing = 0U;
    feed_plan_send_waiting_packet = 0U;
    feed_plan_send_index = 0U;

    memset(feed_plan_tx_data, 0, sizeof(feed_plan_tx_data));
}


// ========== Сброс плана питания ==========
void feed_plan_reset(void)
{
    // Полностью очищаем сохранённый план.
    memset(&feed_plan_data, 0, sizeof(feed_plan_data));

    // После сброса плана ведомые больше не имеют актуальных
    // индивидуальных масок и суммарной длительности фазы.
    // Поэтому реальная фаза питания должна быть запрещена.
    feed_plan_sent_ok = 0U;
    update_real_power_phase_enabled();

    /*
     * После сброса плана следующая передача нового плана
     * должна начинать цикл питания с нулевой фазы.
     */
    real_power_phase_index = 0U;
    active_real_power_phase = 0U;
    active_real_power_phase_index = 0U;

    // При сбросе плана ключ всегда переводится в безопасное состояние.
    power_phase_stage = POWER_PHASE_STAGE_NONE;
    power_key_off();
    /*
     * После сброса плана реальный источник должен быть выключен.
     *
     * Функция не вызывает UART напрямую, поэтому безопасна даже
     * при вызове из логики поиска устройств.
     */
    power_control_disable();
    // Если в момент сброса шла передача плана ведомым,
    // внутреннее состояние передачи также сбрасывается.
    feed_plan_send_reset_state();
}



// ========== Запись плана питания в переменные микроконтроллера ==========
// Эта функция НЕ передаёт план ведомым устройствам.
// Она только сохраняет данные, полученные от ПК через COM-порт.
bool feed_plan_store(uint8_t phase_count,
                     const uint8_t *phase_voltage,
                     const uint16_t *phase_current,
                     uint32_t phase_duration_us,
                     uint32_t phase_extra_duration_us,
                     const uint32_t *device_masks,
                     uint8_t device_count)
{
    // Количество фаз должно быть ненулевым и помещаться в маску uint32_t.
    if ((phase_count == 0U) || (phase_count > FEED_PLAN_MAX_PHASES))
    {
        return false;
    }

    // Количество масок не может быть больше максимального числа ведомых.
    if (device_count > MAX_FOUND_DEVICES)
    {
        return false;
    }

    // Активная длительность фазы должна быть ненулевой.
    if (phase_duration_us == 0U)
    {
        return false;
    }
    /*
     * Для автоматического управления PPE-3323 дополнительное время
     * обязательно: в этот интервал передаются VSET1, ISET1 и при первом
     * запуске OUT1, а затем источник успевает установить напряжение.
     */
    if (phase_extra_duration_us == 0U)
    {
        return false;
    }
    // Проверяем, что полная длительность фазы
    // phase_duration_us + phase_extra_duration_us
    // помещается в uint32_t.
    if (phase_extra_duration_us > (0xFFFFFFFFUL - phase_duration_us))
    {
        return false;
    }

    if ((phase_voltage == NULL) ||
        (phase_current == NULL) ||
        (device_masks == NULL))
    {
        return false;
    }

    /*
     * Проверяем, что каждая фаза соответствует пределам OUT1 PPE-3323.
     *
     * Эта проверка остаётся в protocol.c, чтобы план нельзя было
     * записать с недопустимыми значениями даже при обходе COM-порта.
     */
    for (uint8_t i = 0U; i < phase_count; i++)
    {
        if (phase_voltage[i] > POWER_SOURCE_MAX_VOLTAGE_V)
        {
            return false;
        }

        if (phase_current[i] > POWER_SOURCE_MAX_CURRENT_MA)
        {
            return false;
        }
    }

    // Перед записью нового плана очищаем старый.
    feed_plan_reset();

    feed_plan_data.phase_count = phase_count;

    // Сохраняем активную длительность питания и дополнительное время отдельно.
    // При передаче ведомому будет отправлена их сумма.
    feed_plan_data.phase_duration_us = phase_duration_us;
    feed_plan_data.phase_extra_duration_us = phase_extra_duration_us;

    feed_plan_data.device_count = device_count;

    // Сохраняем напряжения и токи фаз.
    for (uint8_t i = 0U; i < phase_count; i++)
    {
        feed_plan_data.phase_voltage[i] = phase_voltage[i];
        feed_plan_data.phase_current[i] = phase_current[i];
    }

    // Сохраняем битовые маски ведомых устройств.
    // Порядок соответствует found_roms[].
    for (uint8_t i = 0U; i < device_count; i++)
    {
        feed_plan_data.device_phase_mask[i] = device_masks[i];
    }

    feed_plan_data.valid = 1U;

    return true;
}

// ========== Расчёт количества байт маски ==========
static uint8_t feed_plan_get_mask_byte_count(uint8_t phase_count)
{
    // Округление phase_count / 8 вверх.
    return (uint8_t)((phase_count + 7U) / 8U);
}


// ========== Подготовка данных FEED_PLAN для одного ведомого ==========
// Формат данных FEED_PLAN:
//   байт 0    – количество фаз питания;
//   байты 1–4 – суммарная длительность фазы питания, мкс;
//   далее     – байты индивидуальной маски фаз.
//
// В feed_plan_data.device_phase_mask[] бит 0 соответствует первой фазе,
// бит 1 – второй фазе и так далее.
//
// В пакет маска укладывается иначе:
//   первый значимый бит передаётся как старший бит первого байта.
// Пример для N = 3 и строки "101":
//   в памяти: биты 0 и 2;
//   в пакете: 10100000b.
static uint8_t feed_plan_prepare_slave_data(uint8_t device_index)
{
    uint8_t phase_count = feed_plan_data.phase_count;
    uint8_t mask_bytes = feed_plan_get_mask_byte_count(phase_count);
    uint32_t mask = feed_plan_data.device_phase_mask[device_index];

    /*
     * Ведомому передаётся не только количество фаз и маска,
     * но и полная длительность одной фазы питания.
     *
     * Полная длительность складывается из:
     *   phase_duration_us       – полезное время активного питания;
     *   phase_extra_duration_us – добавочное время на перестройку источника питания.
     *
     * На стороне ведущего эти два времени хранятся отдельно,
     * но ведомому передаётся их сумма.
     */
    uint32_t full_phase_duration_us =
            feed_plan_data.phase_duration_us +
            feed_plan_data.phase_extra_duration_us;

    // Байт 0 – количество фаз питания.
    feed_plan_tx_data[0] = phase_count;

    /*
     * Байты 1..4 – суммарная длительность фазы питания, мкс.
     * Передаём старший байт первым.
     */
    feed_plan_tx_data[1] = (uint8_t)(full_phase_duration_us >> 24);
    feed_plan_tx_data[2] = (uint8_t)(full_phase_duration_us >> 16);
    feed_plan_tx_data[3] = (uint8_t)(full_phase_duration_us >> 8);
    feed_plan_tx_data[4] = (uint8_t)(full_phase_duration_us);

    /*
     * Очищаем область маски перед новой упаковкой.
     * Маска начинается не с байта 1, а с байта 5,
     * потому что байты 1..4 заняты длительностью фазы.
     */
    memset(&feed_plan_tx_data[FEED_PLAN_TX_MASK_OFFSET],
           0,
           FEED_PLAN_TX_MASK_BYTES_MAX);

    /*
     * Упаковываем N бит маски от старшего бита к младшему.
     * phase = 0 соответствует первой фазе питания.
     */
    for (uint8_t phase = 0U; phase < phase_count; phase++)
    {
        if (mask & ((uint32_t)1UL << phase))
        {
            uint8_t byte_index = (uint8_t)(phase / 8U);
            uint8_t bit_index = (uint8_t)(phase % 8U);

            /*
             * bit_index = 0 соответствует старшему биту байта;
             * bit_index = 7 соответствует младшему биту байта.
             */
            feed_plan_tx_data[FEED_PLAN_TX_MASK_OFFSET + byte_index] |=
                    (uint8_t)(1U << (7U - bit_index));
        }
    }

    /*
     * Размер данных после функциональной команды FEED_PLAN:
     *   1 байт количества фаз +
     *   4 байта суммарной длительности +
     *   байты маски.
     *
     * Сама функциональная команда FEED_PLAN в поле размера не входит.
     */
    return (uint8_t)(FEED_PLAN_TX_MASK_OFFSET + mask_bytes);
}


// ========== Запуск передачи FEED_PLAN следующему ведомому ==========
static bool feed_plan_start_next_slave(void)
{
    while (feed_plan_send_index < feed_plan_data.device_count)
    {
        uint8_t data_size = feed_plan_prepare_slave_data(feed_plan_send_index);

        /*
         * Отправляем обычный TX-пакет:
         * ROM_MATCH -> адрес ведомого -> размер -> FEED_PLAN -> данные.
         *

         * В send_pack_async() передаётся только размер данных,
         * следующих после функциональной команды.
         *
         * Сама функциональная команда FEED_PLAN в поле размера не входит.

         */
        if (!send_pack_async(ROM_MATCH_CMD,
                             FEED_PLAN,
                             data_size,
                             feed_plan_tx_data,
                             found_roms[feed_plan_send_index]))
        {
            return false;
        }

        // Пакет запущен, теперь ждём его завершения.
        feed_plan_send_waiting_packet = 1U;
        return true;
    }

    // Все ведомые получили свои индивидуальные маски.
    feed_plan_send_processing = 0U;
    feed_plan_send_waiting_packet = 0U;
    // Только после успешного завершения всех индивидуальных пакетов
    // FEED_PLAN можно разрешить переход к реальной фазе питания.
    feed_plan_sent_ok = 1U;
    update_real_power_phase_enabled();
    /*
     * Все ведомые получили индивидуальные маски.
     *
     * Теперь при начале реальной фазы можно автоматически
     * отправлять VSET1, ISET1 и OUT1 источнику PPE-3323.
     */
    power_control_arm_new_plan();

    protocol_event_type = PROTOCOL_EVENT_FEED_PLAN_SENT;

    return true;
}


// ========== Продолжение передачи индивидуальных FEED_PLAN ==========
static void feed_plan_continue_sending(void)
{
    if (global_state != IDLE)
    {
        return;
    }

    if (!feed_plan_send_processing)
    {
        return;
    }

    if (feed_plan_send_waiting_packet)
    {
        // Предыдущий пакет FEED_PLAN завершён.
        feed_plan_send_index++;
        feed_plan_send_waiting_packet = 0U;

        /*
         * Если во время передачи плана возник alarm или interrupt,
         * не запускаем следующий FEED_PLAN прямо сейчас.
         * Сначала protocol_poll() обработает более приоритетное событие.
         */
        if (alarm_pending || interrupt_pending)
        {
            return;
        }
    }

    if (!feed_plan_start_next_slave())
    {
        /*
         * Очередной пакет FEED_PLAN не удалось запустить.
         * План не считается переданным всем ведомым,
         * поэтому реальная фаза питания остаётся запрещённой.
         */
        feed_plan_send_reset_state();

        feed_plan_sent_ok = 0U;
        update_real_power_phase_enabled();

        power_phase_stage = POWER_PHASE_STAGE_NONE;
        power_key_off();
        power_control_disable();
    }
}


// ========== Запуск передачи индивидуальных масок всем ведомым ==========
bool feed_plan_send_all_async(void)
{
    if (global_state != IDLE)
    {
        return false;
    }

    if (feed_plan_send_processing)
    {
        return false;
    }

    // План должен быть предварительно принят от ПК.
    if (!feed_plan_data.valid)
    {
        return false;
    }

    // Количество фаз должно быть корректным.
    if ((feed_plan_data.phase_count == 0U) ||
        (feed_plan_data.phase_count > FEED_PLAN_MAX_PHASES))
    {
        return false;
    }

    // План должен соответствовать текущему списку найденных устройств.
    if ((feed_plan_data.device_count == 0U) ||
        (feed_plan_data.device_count != found_rom_count))
    {
        return false;
    }
    // Начинается новая передача плана. До её полного завершения
    // ранее переданный план не считается подтверждённым для реальной фазы.
    feed_plan_sent_ok = 0U;
    update_real_power_phase_enabled();

    /*
     * После успешной передачи нового плана выполнение цикла питания
     * всегда должно начаться с первой фазы – фазы с индексом 0.
     */
    real_power_phase_index = 0U;
    active_real_power_phase = 0U;
    active_real_power_phase_index = 0U;

    /*
     * Пока передаётся новый FEED_PLAN, реальное питание запрещено.
     * Ключ должен оставаться выключенным.
     */
    power_phase_stage = POWER_PHASE_STAGE_NONE;
    power_key_off();
    /*
     * Старый план не должен оставлять OUT1 включённым,
     * пока ведомым передаётся новый FEED_PLAN.
     */
    power_control_disable();
    feed_plan_send_processing = 1U;
    feed_plan_send_waiting_packet = 0U;
    feed_plan_send_index = 0U;

    if (!feed_plan_start_next_slave())
    {
        feed_plan_send_reset_state();
        return false;
    }

    return true;
}

// ========== Запуск запроса PARAMETERS для следующего ведомого ==========
// Функция ищет следующий адрес в found_roms[], для которого ещё нет
// актуальных параметров питания, и запускает RX-пакет MATCH + PARAMETERS.
//
// Если все параметры уже есть, обработка завершается.
static bool power_params_start_next_request(void)
{
    // Идём по всем найденным адресам.
    while (power_params_index < found_rom_count)
    {
        // Если параметры для этого адреса уже есть,
        // переходим к следующему адресу.
        if (power_params_is_actual_for_index(power_params_index))
        {
            power_params_index++;
            continue;
        }

        // Перед новым приёмом очищаем буфер received_data[].
        clear_received_data();

        // Запускаем приёмный пакет:
        // ROM_MATCH -> адрес found_roms[power_params_index]
        // -> размер -> PARAMETERS -> приём 10 байт.
        if (!receive_pack_async(PARAMETERS,
                                POWER_PARAMS_DATA_SIZE,
                                found_roms[power_params_index]))
        {
            // Если пакет не запустился, оставляем обработку активной.
            // Следующая попытка будет возможна позже через protocol_poll().
            return false;
        }

        // Запомнили, что теперь ждём завершения RX-пакета.
        power_params_waiting_response = 1U;
        return true;
    }

    // Все найденные адреса просмотрены.
    // Значит запрос PARAMETERS завершён.
    power_params_processing = 0U;
    power_params_waiting_response = 0U;

    // Сообщаем main.c, что параметры питания готовы к выводу на ПК.
    // Сами данные уже сохранены в power_params[].
    protocol_event_type = PROTOCOL_EVENT_POWER_PARAMS_READY;

    return true;
}


// ========== Продолжение процедуры получения параметров питания ==========
// Эта функция вызывается из protocol_poll(), когда автомат протокола свободен.
// Если предыдущий RX-пакет PARAMETERS завершён, она сохраняет данные
// и запускает запрос параметров у следующего ведомого устройства.
static void power_params_continue_processing(void)
{
    // Пока автомат занят, разбирать данные нельзя.
    if (global_state != IDLE)
    {
        return;
    }

    // Если процедура получения параметров не запущена, делать нечего.
    if (!power_params_processing)
    {
        return;
    }

    // Если мы ждали ответ от ведомого, значит RX-пакет уже завершился.
    // Нужно разобрать received_data[] и сохранить параметры.
    if (power_params_waiting_response)
    {
        power_params_store_from_received(power_params_index);

        // Переходим к следующему найденному устройству.
        power_params_index++;
        power_params_waiting_response = 0U;

        // Если во время приёма параметров был выставлен alarm или interrupt,
        // не запускаем следующий PARAMETERS сразу.
        // Сначала protocol_poll() обработает более важное событие.
        if (alarm_pending || interrupt_pending)
        {
            return;
        }
    }

    // Запускаем запрос параметров у следующего ведомого устройства.
    (void)power_params_start_next_request();
}


// ========== Запуск получения параметров питания у всех найденных устройств ==========
// Функция вызывается после завершения ROM-search.
// Она не отправляет параметры на ПК, а только принимает их от ведомых
// и сохраняет во внутренний массив power_params[].
bool power_params_request_all_async(void)
{
    // Нельзя начинать процедуру, пока автомат протокола занят.
    if (global_state != IDLE)
    {
        return false;
    }

    // Если процедура уже идёт, второй раз её не запускаем.
    if (power_params_processing)
    {
        return false;
    }

    // Если устройств нет, запрашивать нечего.
    if (found_rom_count == 0U)
    {
        return false;
    }

    // Начинаем проход по массиву found_roms[].
    power_params_processing = 1U;
    power_params_waiting_response = 0U;
    power_params_index = 0U;

    // Сразу запускаем запрос для первого устройства,
    // у которого ещё нет актуальных параметров.
    return power_params_start_next_request();
}


// ========== Запуск процедуры обработки прерывания ==========
// Эта функция вызывается только тогда, когда текущий пакет уже завершён,
// то есть global_state == IDLE.
//
// По алгоритму сначала нужно проверить, не связано ли прерывание
// с появлением нового ведомого устройства.
// Для этого мастер отправляет общий RX-пакет:
// ROM_SKIP + NEW_DEVICE + приём 1 байта.
static void start_interrupt_processing(void)
{
    // Если автомат ещё занят, обработку начинать нельзя.
    if (global_state != IDLE)
    {
        return;
    }

    // Если обработка уже идёт, повторно её не запускаем.
    if (interrupt_processing)
    {
        return;
    }

    // Сбрасываем отложенный флаг прерывания:
    // событие уже принято в работу.
    interrupt_pending = 0U;

    // Отмечаем, что началась внутренняя обработка прерывания.
    // Пока этот флаг установлен, новые биты прерывания игнорируются,
    // чтобы не получить вложенную обработку.
    interrupt_processing = 1U;

    // Первый этап обработки – проверка новых устройств.
    interrupt_step = INTR_STEP_CHECK_NEW_DEVICE;

    // Отправляем всем ведомым устройствам команду NEW_DEVICE
    // и принимаем один байт ответа.
    //
    // Если новое устройство есть, ожидаем байт 0x00.
    // Если новых устройств нет, линия данных остаётся в 1,
    // и мастер примет 0xFF.
    if (!receive_pack_skip_async(NEW_DEVICE, 1U))
    {
        // Если по какой-то причине пакет не стартовал,
        // откатываем состояние и оставляем прерывание в ожидании.
        interrupt_processing = 0U;
        interrupt_step = INTR_STEP_NONE;
        interrupt_pending = 1U;
    }
}


// ========== Продолжение процедуры обработки прерывания ==========
// Эта функция вызывается после завершения внутренних пакетов,
// которые были запущены для обработки прерывания.
//
// Внутренняя обработка состоит из этапов:
// 1. Проверка NEW_DEVICE.
// 2. Если принят 0x00 – запуск обычного ROM-search.
// 3. Если принят не 0x00 – запуск ROM-search_interrupt.
// 4. После завершения поиска – установка события для main.c.
static void continue_interrupt_processing(void)
{
    // Продолжать обработку можно только когда автомат свободен.
    if (global_state != IDLE)
    {
        return;
    }

    // Если обработка прерывания не запущена, делать нечего.
    if (!interrupt_processing)
    {
        return;
    }

    // ===== Этап 1: обработан ответ на NEW_DEVICE =====
    if (interrupt_step == INTR_STEP_CHECK_NEW_DEVICE)
    {
        // Если принято 0x00, значит новое устройство удержало линию в 0.
        // Следующим пакетом запускается обычный поиск ROM-search.
        if ((received_data_count > 0U) && (received_data[0] == 0x00U))
        {
            interrupt_step = INTR_STEP_SEARCH_NEW_DEVICES;

            // Новое устройство найдено.
            // Запускаем обычный ROM-search, но НЕ очищаем found_roms.
            // Все уже известные адреса будут пропущены как дубли,
            // а новые адреса добавятся в основной массив found_roms.
            if (!search_new_devices_packet_async())
            {
                interrupt_processing = 0U;
                interrupt_step = INTR_STEP_NONE;
                protocol_event_type = PROTOCOL_EVENT_NONE;
            }

            return;
        }

        // Если байт не равен 0x00, новых устройств нет.
        // Значит это обычное прерывание от уже известных ведомых устройств.
        // Запускаем специальный поиск только среди устройств
        // с установленным флагом прерывания.
        interrupt_step = INTR_STEP_SEARCH_INTERRUPT_DEVICES;

        if (!search_interrupt_packet_async())
        {
            // Защита на случай, если специальный поиск не запустился.
            interrupt_processing = 0U;
            interrupt_step = INTR_STEP_NONE;
            protocol_event_type = PROTOCOL_EVENT_NONE;
        }

        return;
    }

    // ===== Этап 2: завершился обычный ROM-search после NEW_DEVICE =====
    if (interrupt_step == INTR_STEP_SEARCH_NEW_DEVICES)
    {
        // В массиве found_roms[] теперь находится обновлённый список устройств.
        // Старые адреса не дублируются, новые адреса добавлены в конец массива.
        interrupt_processing = 0U;
        interrupt_step = INTR_STEP_NONE;

        // После появления нового ведомого старый план питания уже не подходит:
        // в нём нет маски для нового адреса.
        // Новый план должен быть принят от ПК заново.
        feed_plan_reset();

        // Сообщаем main.c, что надо вывести результат поиска новых устройств.
        protocol_event_type = PROTOCOL_EVENT_NEW_DEVICES;
        return;
    }
    // ===== Этап 3: завершился ROM-search_interrupt =====
    if (interrupt_step == INTR_STEP_SEARCH_INTERRUPT_DEVICES)
    {
    	// В массиве interrupt_roms[] теперь находятся адреса устройств,
    	// которые участвовали в поиске по флагу прерывания.
        interrupt_processing = 0U;
        interrupt_step = INTR_STEP_NONE;

        // Сообщаем main.c, что надо вывести адреса устройств с прерыванием.
        protocol_event_type = PROTOCOL_EVENT_INTERRUPT_DEVICES;
        return;
    }

    // Если сюда попали, значит состояние оказалось неизвестным.
    // Сбрасываем внутреннюю обработку, чтобы автомат не завис.
    interrupt_processing = 0U;
    interrupt_step = INTR_STEP_NONE;
}

// ========== Запуск процедуры обработки alarm ==========
// Эта функция вызывается только после того, как текущий пакет уже завершён.
// Alarm имеет приоритет над обычным прерыванием.
static void start_alarm_processing(void)
{
    // Если автомат ещё занят, обработку alarm начинать нельзя.
    if (global_state != IDLE)
    {
        return;
    }

    // Если обработка alarm уже идёт, повторно её не запускаем.
    if (alarm_processing)
    {
        return;
    }

    // Alarm принят в работу.
    alarm_pending = 0U;
    alarm_processing = 1U;
    alarm_step = ALARM_STEP_SEARCH_ALARM_DEVICES;

    // Во время обработки alarm обычное прерывание не запускается.
    // Если interrupt был принят одновременно с alarm, он уже сброшен
    // в handle_service_flags_after_segment().
    interrupt_pending = 0U;

    // Запускаем поиск ведомых устройств с флагом alarm.
    // Используется та же логика дерева поиска, что и в ROM-search,
    // но команда другая – ROM_ALARM_CMD.
    if (!search_alarm_packet_async())
    {
        // Если поиск не удалось запустить, возвращаем alarm в ожидание.
        alarm_processing = 0U;
        alarm_step = ALARM_STEP_NONE;
        alarm_pending = 1U;
    }
}


// ========== Продолжение процедуры обработки alarm ==========
// После завершения ROM-alarm здесь формируется событие для main.c,
// чтобы пользователь увидел адреса ведомых устройств с флагом alarm.
static void continue_alarm_processing(void)
{
    // Продолжать обработку можно только когда автомат свободен.
    if (global_state != IDLE)
    {
        return;
    }

    // Если обработка alarm не запущена, делать нечего.
    if (!alarm_processing)
    {
        return;
    }

    if (alarm_step == ALARM_STEP_SEARCH_ALARM_DEVICES)
    {
        // В массиве alarm_roms[] теперь находятся адреса устройств,
        // которые участвовали в поиске по флагу alarm.
        alarm_processing = 0U;
        alarm_step = ALARM_STEP_NONE;

        // Сообщаем main.c, что нужно вывести список аварийных устройств.
        protocol_event_type = PROTOCOL_EVENT_ALARM_DEVICES;
        return;
    }

    // Защита от неизвестного состояния.
    alarm_processing = 0U;
    alarm_step = ALARM_STEP_NONE;
}


// ========== Фоновая обработка внутренних событий протокола ==========
// Эта функция вызывается из main.c в основном цикле.
//
// Она ничего не делает, пока автомат протокола занят.
// Если протокол свободен, сначала обрабатывается alarm,
// затем сохраняются принятые параметры питания,
// затем обрабатывается обычное прерывание,
// затем продолжается запрос PARAMETERS у следующих ведомых.
void protocol_poll(void)
{
    // Пока автомат занят передачей, приёмом или поиском,
    // обработку внутренних событий не выполняем.
    if (global_state != IDLE)
    {
        return;
    }

    // Если есть событие, которое main.c ещё не вывел пользователю,
    // новую обработку пока не начинаем.
    if (protocol_event_type != PROTOCOL_EVENT_NONE)
    {
        return;
    }

    // Alarm имеет самый высокий приоритет.
    // Если обработка alarm уже началась, продолжаем её.
    if (alarm_processing)
    {
        continue_alarm_processing();
        return;
    }

    // Если alarm был зафиксирован в сегменте, запускаем ROM-alarm.
    if (alarm_pending)
    {
        start_alarm_processing();
        return;
    }

    // Если только что завершился RX-пакет PARAMETERS,
    // сначала сохраняем received_data[] в power_params[].
    //
    // Это важно сделать до обработки interrupt, потому что received_data[]
    // общий и может быть перезаписан внутренним пакетом NEW_DEVICE.
    if (power_params_processing && power_params_waiting_response)
    {
        power_params_continue_processing();
        return;
    }

    // Если обработка прерывания уже начата,
    // продолжаем её после завершения очередного внутреннего пакета.
    if (interrupt_processing)
    {
        continue_interrupt_processing();
        return;
    }

    // Если во время обычного пакета или приёма PARAMETERS
    // был принят бит прерывания, запускаем его обработку.
    if (interrupt_pending)
    {
        start_interrupt_processing();
        return;
    }


    // Если идёт передача индивидуальных масок FEED_PLAN,
    // продолжаем её после завершения очередного пакета.
    if (feed_plan_send_processing)
    {
        feed_plan_continue_sending();
        return;
    }

    // Если идёт процедура получения параметров питания,
    // но сейчас не ожидается уже запущенный RX-пакет,
    // запускаем PARAMETERS для следующего ведомого.
    if (power_params_processing)
    {
        power_params_continue_processing();
        return;
    }


}

// ========== Сброс события после вывода пользователю ==========
// main.c вызывает эту функцию после того, как вывел найденные адреса.
void protocol_clear_event(void)
{
    protocol_event_type = PROTOCOL_EVENT_NONE;
}

// ========== Сброс признака аварийного завершения пакета ==========
// main.c вызывает эту функцию после того, как сообщил пользователю,
// что текущий пакет был остановлен из-за alarm.
void protocol_clear_packet_abort_flag(void)
{
    protocol_packet_aborted_by_alarm = 0U;
}

void protocol_timer_irq_handler(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2)
    {
        return;
    }

    switch (global_state)
    {
    case SEG_START_BIT:
        if (half_slot_phase == 0U)
        {
            DATA_LOW();
            half_slot_phase = 1U;
        }
        else
        {
            half_slot_phase = 0U;

            if (current_seq_state == SEQ_RX_DATA)
            {
                global_state = SEG_RECEIVE_DATA_BITS;
            }
            else
            {
                global_state = SEG_DATA_BITS;
            }
        }
        break;

    case SEG_DATA_BITS:
    {
        uint8_t bit = (tx_data >> (7U - tx_bit_index)) & 1U;

        if (half_slot_phase == 0U)
        {
            if (bit)
            {
                DATA_HIGH();
            }
            else
            {
                DATA_LOW();
            }

            half_slot_phase = 1U;
        }
        else
        {
            half_slot_phase = 0U;

            if (tx_bit_index == 7U)
            {
                tx_bit_index = 0U;
                global_state = SEG_CRC_BITS;
            }
            else
            {
                tx_bit_index++;
            }
        }
        break;
    }

    case SEG_CRC_BITS:
    {
        uint8_t bit = (tx_crc >> (4U - tx_bit_index)) & 1U;

        if (half_slot_phase == 0U)
        {
            if (bit)
            {
                DATA_HIGH();
            }
            else
            {
                DATA_LOW();
            }

            half_slot_phase = 1U;
        }
        else
        {
            half_slot_phase = 0U;

            if (tx_bit_index == 4U)
            {
                global_state = SEG_RECV_ALARM;
            }
            else
            {
                tx_bit_index++;
            }
        }
        break;
    }

    case SEG_RECV_ALARM:
        if (half_slot_phase == 0U)
        {
            DATA_HIGH();
            half_slot_phase = 1U;
        }
        else
        {
            // Активный уровень аварии – 0
            if (!DATA_READ())
            {
                rx_flags |= 0x01U;
            }

            half_slot_phase = 0U;
            global_state = SEG_RECV_INTR;
        }
        break;

    case SEG_RECV_INTR:
        if (half_slot_phase == 0U)
        {
            DATA_HIGH();
            half_slot_phase = 1U;
        }
        else
        {
            // Активный уровень прерывания – 0
            if (!DATA_READ())
            {
                rx_flags |= 0x02U;
            }

            half_slot_phase = 0U;
            global_state = SEG_RECV_CRC;
        }
        break;

    case SEG_RECV_CRC:
        if (half_slot_phase == 0U)
        {
            DATA_HIGH();
            half_slot_phase = 1U;
        }
        else
        {
            // 1 – CRC подтверждён, повтор не нужен
            // 0 – CRC не подтверждён, сегмент нужно повторить
            uint8_t crc_ack = DATA_READ() ? 1U : 0U;

            if (crc_ack)
            {
                rx_flags |= 0x04U;
                retry_needed = 0U;
            }
            else
            {
                rx_flags &= (uint8_t)~0x04U;
                retry_needed = 1U;
            }

            half_slot_phase = 0U;
            stop_timer();
            global_state = IDLE;
            on_segment_done();
        }
        break;

    case SEG_RECEIVE_DATA_BITS:
        if (half_slot_phase == 0U)
        {
            DATA_HIGH();
            half_slot_phase = 1U;
        }
        else
        {
            uint8_t bit = DATA_READ() ? 1U : 0U;

            rx_data_byte = (uint8_t)((rx_data_byte << 1) | bit);

            half_slot_phase = 0U;

            if (rx_bit_index == 7U)
            {
                rx_bit_index = 0U;
                global_state = SEG_RECEIVE_CRC_BITS;
            }
            else
            {
                rx_bit_index++;
            }
        }
        break;

    case SEG_RECEIVE_CRC_BITS:
        if (half_slot_phase == 0U)
        {
            DATA_HIGH();
            half_slot_phase = 1U;
        }
        else
        {
            uint8_t bit = DATA_READ() ? 1U : 0U;

            rx_crc = (uint8_t)(((rx_crc << 1) | bit) & 0x1FU);

            half_slot_phase = 0U;

            if (rx_bit_index == 4U)
            {
                rx_bit_index = 0U;
                rx_crc_ok = check_crc5(rx_data_byte, rx_crc) ? 1U : 0U;
                retry_needed = (rx_crc_ok == 0U) ? 1U : 0U;
                global_state = SEG_RECEIVE_ALARM;
            }
            else
            {
                rx_bit_index++;
            }
        }
        break;

    case SEG_RECEIVE_ALARM:
        if (half_slot_phase == 0U)
        {
            DATA_HIGH();
            half_slot_phase = 1U;
        }
        else
        {
            // Активный уровень аварии – 0
            if (!DATA_READ())
            {
                rx_flags |= 0x01U;
            }

            half_slot_phase = 0U;
            global_state = SEG_RECEIVE_INTR;
        }
        break;

    case SEG_RECEIVE_INTR:
        if (half_slot_phase == 0U)
        {
            DATA_HIGH();
            half_slot_phase = 1U;
        }
        else
        {
            // Активный уровень прерывания – 0
            if (!DATA_READ())
            {
                rx_flags |= 0x02U;
            }

            half_slot_phase = 0U;
            global_state = SEG_SEND_CRC_ACK;
        }
        break;

    case SEG_SEND_CRC_ACK:
        if (half_slot_phase == 0U)
        {
            /*
             * 1 – принятый CRC корректен.
             * 0 – CRC неверен, ведомый должен повторить сегмент.
             *
             * Значение подтверждения устанавливается в начале
             * последнего тайм-слота сегмента.
             */
            if (rx_crc_ok)
            {
                DATA_HIGH();
            }
            else
            {
                DATA_LOW();
            }

            half_slot_phase = 1U;
        }
        else
        {
            /*
             * Вторая половина тайм-слота подтверждения CRC.
             *
             * Здесь нельзя завершать сегмент и запускать фазу питания,
             * потому что start_power_phase() отпускает линию через DATA_HIGH().
             * Иначе NACK будет удерживаться только половину тайм-слота.
             *
             * Переходим в отдельное состояние завершения, которое сработает
             * после ещё одного полутакта TIM2. Благодаря этому ACK/NACK
             * сохраняется на линии полные 70 мкс.
             */
            half_slot_phase = 0U;
            global_state = SEG_COMPLETE;
        }
        break;
    case SEG_COMPLETE:
        /*
         * Полный тайм-слот CRC ACK/NACK завершён.
         *
         * Линию нужно отпустить сразу в начале этого обработчика.
         * Иначе DATA_HIGH() будет вызван только внутри start_power_phase()
         * после выполнения нескольких функций, и длительность NACK
         * станет больше 70 мкс.
         */
        DATA_HIGH();

        /*
         * После отпускания линии можно безопасно завершить сегмент
         * и перейти к следующей фазе или следующему сегменту.
         */
        stop_timer();
        global_state = IDLE;
        on_received_segment_done();
        break;
    case SSEG_START_BIT:
        if (half_slot_phase == 0U)
        {
            DATA_LOW();
            half_slot_phase = 1U;
        }
        else
        {
            half_slot_phase = 0U;
            global_state = SSEG_BODY;
        }
        break;

    case SSEG_BODY:
        if (half_slot_phase == 0U)
        {
            if ((!search_device_complete) && (!search_error))
            {
                if (search_group_phase == 0U)
                {
                    DATA_HIGH();
                }
                else if (search_group_phase == 1U)
                {
                    DATA_HIGH();
                }
                else
                {
                    if (search_direction)
                    {
                        DATA_HIGH();
                    }
                    else
                    {
                        DATA_LOW();
                    }
                }
            }
            else
            {
                DATA_HIGH();
            }

            half_slot_phase = 1U;
        }
        else
        {
            if ((!search_device_complete) && (!search_error))
            {
                if (search_group_phase == 0U)
                {
                    search_id_bit = DATA_READ() ? 1U : 0U;
                }
                else if (search_group_phase == 1U)
                {
                    search_cmp_id_bit = DATA_READ() ? 1U : 0U;

                    if (!search_choose_direction())
                    {
                        search_error = 1U;
                    }
                }
                else
                {
                    search_bit_index++;

                    if (search_bit_index >= SEARCH_TOTAL_BITS)
                    {
                        search_device_complete = 1U;
                    }
                }
            }

            half_slot_phase = 0U;

            if (search_group_phase < 2U)
            {
                search_group_phase++;
            }
            else
            {
                search_group_phase = 0U;
                search_group_index++;

                if (search_group_index >= SEARCH_GROUPS_PER_SEGMENT)
                {
                    global_state = SSEG_PAD_BIT;
                }
            }
        }
        break;

    case SSEG_PAD_BIT:
        if (half_slot_phase == 0U)
        {
            DATA_HIGH();
            half_slot_phase = 1U;
        }
        else
        {
            half_slot_phase = 0U;
            stop_timer();
            on_search_segment_done();
        }
        break;

    case SEQ_WAIT_PHASE:
        if (phase_ticks > 0U)
        {
            phase_ticks--;
        }

        if (phase_ticks == 0U)
        {
            /*
             * Завершилось дополнительное время реальной фазы.
             *
             * Теперь включаем ключ и начинаем активное питание.
             * Таймер не останавливается: сразу начинается отсчёт
             * phase_duration_us.
             */
            if (power_phase_stage == POWER_PHASE_STAGE_REAL_EXTRA)
            {
                start_real_power_active_part();
                break;
            }

            /*
             * Завершилась активная часть реальной фазы
             * или обычная имитационная фаза.
             *
             * Перед продолжением протокола источник обязательно
             * отключается от линии.
             */
            power_key_off();

            stop_timer();
            half_slot_phase = 0U;

            /*
             * Индекс следующей фазы меняется только после завершения
             * активной части реальной фазы.
             */
            if (power_phase_stage == POWER_PHASE_STAGE_REAL_ACTIVE)
            {
                advance_real_power_phase();
            }

            power_phase_stage = POWER_PHASE_STAGE_NONE;

            global_state = next_state_after_phase;
            next_step();
        }
        break;

    default:
        break;
    }
}

// ========== Инициализация протокола ==========
void protocol_init(void)
{
    // Линия данных по умолчанию должна быть отпущена.
    // Так как выход open-drain, DATA_HIGH() фактически отпускает линию.
    DATA_HIGH();
    // Источник питания при запуске должен быть отключён от линии.
    power_key_off();
    // Основной автомат находится в состоянии ожидания.
    global_state = IDLE;
    current_seq_state = IDLE;

    // Сброс временных переменных сегмента.
    half_slot_phase = 0U;
    retry_needed = 0U;
    rx_flags = 0U;

    // Начальное состояние фазы питания.
    phase_ticks = 0U;
    next_state_after_phase = IDLE;
    real_power_phase_index = 0U;
    active_real_power_phase = 0U;
    active_real_power_phase_index = 0U;
    power_phase_stage = POWER_PHASE_STAGE_NONE;
    // Сброс режима приёма.
    receive_packet_active = 0U;
    clear_received_data();

    // Сброс поисковой логики.
    // Это очистит found_roms[], счётчики и флаги поиска.
    search_rom_packet_reset();

    // Сброс обработки прерывания.
    interrupt_pending = 0U;
    interrupt_processing = 0U;
    interrupt_step = INTR_STEP_NONE;

    // Сброс обработки alarm.
    alarm_pending = 0U;
    alarm_processing = 0U;
    alarm_step = ALARM_STEP_NONE;

    // Пользовательский пакет ещё не прерывался по alarm.
    protocol_packet_aborted_by_alarm = 0U;

    // Событий для вывода пользователю пока нет.
    protocol_event_type = PROTOCOL_EVENT_NONE;
}
bool protocol_is_busy(void)
{
    return (global_state != IDLE);
}
