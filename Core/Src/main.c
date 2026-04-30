/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMESLOT_US     70 //длительность временного слота
#define CRC5_POLY       0x05 //полином для реализации crc
#define ROM_ID_LEN       4U  //длинна массива адреса ведомого
#define POWER_PHASE_US   10000U   // 10 мс

#define MAX_FOUND_DEVICES          16U   // максимум адресов, которые сохраним после одного ROM-search
#define SEARCH_GROUPS_PER_SEGMENT  5U    // в одном поисковом сегменте обрабатываем 5 бит адреса
#define SEARCH_TOTAL_BITS          (ROM_ID_LEN * 8U)  // всего бит в адресе: 32

#define MAX_RECEIVED_DATA_SIZE 64  //максимальное количество бит данных в одном бакете

#define ROM_SKIP_CMD    0xCC //код rom-команды для отправки всем устройствам
#define ROM_SEARCH_CMD 0xF0 //код rom-команды для поиска устройств
#define ROM_READ_DATA_CMD 0x33 //код rom-команды для чтения данных от конкретного устройства на шине
#define ROM_MATCH_CMD 0x55 //код rom-команды для отправки адресса и последующего обмена данными только с одним ведомым
#define ROM_ALARM_CMD 0xEC //код rom-команды для обработки аварийных ситуаций
#define ROM_RESUME_CMD 0x69 //код rom-команды для повторного обращения к ведомому, с которым оно велось в предыдущий раз

#define FUNC_OFF        0x01 //код функциональной команды для выключения одного или нескольких устройств
#define FUNC_ON         0x02 //код функциональной команды для включения одного или нескольких устройств
#define PARAMETERS 0x03 //код функциональной команды для приёма параметров от ведомого
#define NEW_DEVICE 0x04 //код функциональной команды для опроса на новые ведомые
#define FEED_PLAN 0x05 //код функциональной команды для отправки плана питания

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t rom_get_bit(const uint8_t rom[ROM_ID_LEN], uint8_t bit_index);
static void rom_set_bit(uint8_t rom[ROM_ID_LEN], uint8_t bit_index, uint8_t value);

static void search_prepare_next_device(void);
static bool search_choose_direction(void);
static void search_store_found_device(void);

static void start_search_segment(void);
static void on_search_segment_done(void);

void search_rom_packet_reset(void);
bool search_rom_packet_async(void);
bool search_rom_packet_blocking(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ========== НАСТРОЙКА ТАЙМЕРА (TIM2) ==========
// Предполагается, что таймер TIM2 уже настроен в CubeMX:
//   Prescaler = SystemCoreClock / 1_000_000 - 1 = 15   (тик 1 мкс)
//   Period = TIMESLOT_US / 2 - 1                  (35-1 = 34)
//   Один логический тайм-слот протокола обрабатывается за два прерывания таймера
//   AutoReload Preload = Enable
//   NVIC: TIM2 global interrupt enabled

// ========== Глобальные переменные для автоматов ==========
// ========== Глобальные переменные для автоматов ==========
typedef enum {
    IDLE,                      // нет активной операции

    // ===== Состояния обычного сегмента =====
    SEG_START_BIT,             // стартовый бит обычного сегмента
    SEG_DATA_BITS,             // 8 бит данных
	SEG_RECEIVE_DATA_BITS,
	SEG_RECEIVE_CRC_BITS,
    SEG_CRC_BITS,              // 5 бит CRC
    SEG_RECV_ALARM,            // приём бита аварии
    SEG_RECV_INTR,             // приём бита прерывания
    SEG_RECV_CRC,              // приём бита подтверждения CRC
    SEG_COMPLETE,

	SEG_RECEIVE_ALARM,          // приём служебного бита аварии в приёмном сегменте
	SEG_RECEIVE_INTR,           // приём служебного бита прерывания в приёмном сегменте
	SEG_SEND_CRC_ACK,           // передача мастером подтверждения CRC

    // ===== Состояния обычного пакета =====
    SEQ_ROM,                   // отправка ROM-команды
    SEQ_POWER_AFTER_ROM,       // фаза питания после ROM-команды
    SEQ_ADDR,                  // отправка адреса
    SEQ_POWER_AFTER_ADDR,      // фаза питания после адреса
    SEQ_SIZE,                  // отправка размера
    SEQ_POWER_AFTER_SIZE,      // фаза питания после размера
    SEQ_FUNC,                  // отправка функциональной команды
    SEQ_POWER_AFTER_FUNC,      // фаза питания после функции
    SEQ_DATA,                  // отправка данных
    SEQ_POWER_AFTER_DATA,      // фаза питания после данных
	SEQ_RX_DATA,                // приём очередного байта данных от ведомого
	SEQ_POWER_AFTER_RX_DATA,    // фаза питания после принятого сегмента
    SEQ_WAIT_PHASE,            // ожидание окончания фазы питания
    SEQ_END,                   // пакет завершён


    // ===== Состояния специального пакета ROM-search =====
    SEQ_SEARCH_SEG,            // запуск очередного поискового сегмента
    SEQ_POWER_AFTER_SEARCH_SEG,// фаза питания после поискового сегмента
    SEQ_SEARCH_CONTINUE,       // анализ: продолжаем поиск / завершаем

    // ===== Состояния одного поискового сегмента =====
    SSEG_START_BIT,            // стартовый бит поискового сегмента
    SSEG_BODY,                 // 5 троек поиска внутри сегмента
    SSEG_PAD_BIT               // последний "пустой" бит сегмента
} state_t;

static void on_received_segment_done(void);
static void next_step(void);   // прототип функции для отправки пакета
static state_t current_seq_state = IDLE;     // сохранённый этап последовательности (SEQ_*)
static state_t global_state = IDLE;   // текущее состояние автомата
static uint8_t tx_data = 0;           // данные для отправки в текущем сегменте
static uint8_t tx_crc = 0;            // CRC для текущего сегмента
static uint8_t tx_bit_index = 0;      // индекс текущего бита при отправке (0..7 или 0..4)
static uint8_t rx_flags = 0;          // биты: 0=alarm, 1=interrupt, 2=crc_bit
static uint8_t current_segment_byte = 0;   // нужен для корректного retry
static uint8_t half_slot_phase = 0;   // 0 - начало тайм-слота, 1 - середина/конец тайм-слота
// ===== Переменные для приёма данных мастером =====

// Буфер данных, принятых в текущем пакете.
// Один принятый сегмент = один байт данных.
static uint8_t received_data[MAX_RECEIVED_DATA_SIZE] = {0};

// Количество байтов, уже успешно принятых в текущем пакете
static uint8_t received_data_count = 0;

// Байт, который сейчас собирается из 8 бит приёмного сегмента
static uint8_t rx_data_byte = 0;

// CRC, который сейчас принимается от ведомого
static uint8_t rx_crc = 0;

// Индекс текущего принимаемого бита:
// для данных 0..7, для CRC 0..4
static uint8_t rx_bit_index = 0;

// Флаг результата проверки CRC принятого сегмента:
// 1 – CRC верный, 0 – CRC неверный
static uint8_t rx_crc_ok = 0;

// Флаг: текущий пакет является пакетом приёма данных от ведомого
static uint8_t receive_packet_active = 0;

// Параметры пакета (из вызова send_pack)
static uint8_t current_rom_cmd = 0;        // ROM-команда текущего пакета
static uint8_t current_func_cmd = 0;       // функциональная команда текущего пакета
static uint8_t current_datasize = 0;       // количество байт данных в пакете
static uint8_t *current_data = NULL;       // указатель на массив данных
static uint8_t data_index = 0;             // индекс текущего байта данных при отправке
static uint8_t current_slave_id[ROM_ID_LEN] = {0};//id устройства, отправляется после rom-match
static uint8_t addr_index = 0;//индекс в id ведомого
// Переменные для неблокирующей фазы питания
static uint16_t phase_ticks = 0;            // сколько тиков (70 мкс) осталось ждать для фазы питания
static state_t next_state_after_phase = IDLE; // состояние после фазы

// Флаги для циклов подтверждения (как do-while)
static uint8_t retry_needed = 0;           // =1 если сегмент нужно повторить (ошибка CRC)

// ===== Переменные для пакетного ROM-search =====

// Массив найденных адресов
static uint8_t found_roms[MAX_FOUND_DEVICES][ROM_ID_LEN] = {0};

// Количество найденных адресов
static uint8_t found_rom_count = 0;

// Адрес, который сейчас собирается по битам
static uint8_t search_rom_no[ROM_ID_LEN] = {0};

// Последний найденный адрес.
// Нужен, чтобы на следующем проходе идти по другой ветке дерева поиска.
static uint8_t search_last_rom[ROM_ID_LEN] = {0};

// Текущий бит адреса (0..31)
static uint8_t search_bit_index = 0;

// Номер текущей тройки внутри одного поискового сегмента (0..4)
static uint8_t search_group_index = 0;

// Фаза внутри одной тройки:
// 0 - читаем id_bit
// 1 - читаем cmp_id_bit
// 2 - мастер передаёт выбранный direction
static uint8_t search_group_phase = 0;

// Бит адреса от ведомых
static uint8_t search_id_bit = 1;

// Инвертированный бит адреса от ведомых
static uint8_t search_cmp_id_bit = 1;

// Бит направления, который выбрал мастер
static uint8_t search_direction = 1;

// Последняя коллизия предыдущего прохода поиска
static uint8_t search_last_discrepancy = 0;

// Последняя коллизия текущего прохода
static uint8_t search_new_discrepancy = 0;

// Флаг: найдено последнее устройство на шине
static uint8_t search_last_device_flag = 0;

// Флаг: активен ли сейчас общий пакет ROM-search
static uint8_t search_packet_active = 0;

// Флаг: адрес текущего устройства уже собран полностью
static uint8_t search_device_complete = 0;

// Флаг: в процессе поиска произошла ошибка
static uint8_t search_error = 0;

// Флаг: весь пакет ROM-search завершён
static uint8_t search_done = 0;

// Флаг: пакет ROM-search завершён успешно
static uint8_t search_success = 0;

// ========== Работа с линией PB0 ==========
// Макросы для быстрого управления выводом PB0 (открытый сток)
#define DATA_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1)   // отпустить линию (1)
#define DATA_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0) // притянуть к земле (0)
#define DATA_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)     // прочитать состояние линии

// ========== Управление таймером ==========
void start_timer(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);   // обнуляем счётчик таймера
    HAL_TIM_Base_Start_IT(&htim2);      // запускаем таймер с прерываниями
}

void stop_timer(void) {
    HAL_TIM_Base_Stop_IT(&htim2);       // останавливаем таймер
}

//функция показывающая нужено ли отправлять адрес
static bool rom_cmd_has_address(uint8_t rom_cmd) {
    return (rom_cmd == ROM_MATCH_CMD);
}

//функция для начала фазы питания
static void start_power_phase(state_t next_state) {
    DATA_HIGH();

    phase_ticks = (POWER_PHASE_US + TIMESLOT_US - 1U) / TIMESLOT_US;
    phase_ticks *= 2U;          // таймер идёт в 2 раза чаще чем 70 мкс
    next_state_after_phase = next_state;
    half_slot_phase = 0;
    global_state = SEQ_WAIT_PHASE;
    start_timer();
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

// ===== Полный сброс логики пакетного ROM-search =====
void search_rom_packet_reset(void)
{
    memset(found_roms, 0, sizeof(found_roms));
    memset(search_rom_no, 0, ROM_ID_LEN);
    memset(search_last_rom, 0, ROM_ID_LEN);

    found_rom_count = 0;

    search_bit_index = 0;
    search_group_index = 0;
    search_group_phase = 0;

    search_id_bit = 1;
    search_cmp_id_bit = 1;
    search_direction = 1;

    search_last_discrepancy = 0;
    search_new_discrepancy = 0;
    search_last_device_flag = 0;

    search_packet_active = 0;
    search_device_complete = 0;
    search_error = 0;
    search_done = 0;
    search_success = 0;

    // Сброс общих переменных автомата, чтобы поиск всегда начинался из чистого состояния
    addr_index = 0;
    data_index = 0;
    rx_flags = 0;
    retry_needed = 0;
    half_slot_phase = 0;
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
    // Этот адрес станет "предыдущим найденным" для следующего прохода
    memcpy(search_last_rom, search_rom_no, ROM_ID_LEN);

    // Сохраняем его в массив найденных адресов
    if (found_rom_count < MAX_FOUND_DEVICES)
    {
        memcpy(found_roms[found_rom_count], search_rom_no, ROM_ID_LEN);
        found_rom_count++;
    }

    // Обновляем позицию последней коллизии
    search_last_discrepancy = search_new_discrepancy;

    // Если новых коллизий не осталось, значит это было последнее устройство
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
        search_packet_active = 0U;
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

// ===== Запуск полного пакета ROM-search =====
bool search_rom_packet_async(void)
{
    // Шина должна быть свободна
    if (global_state != IDLE)
    {
        return false;
    }

    // Сбросить результаты прошлого поиска
    search_rom_packet_reset();

    // Отметить, что пакет поиска активен
    search_packet_active = 1U;

    // Это пакет поиска адресов, а не пакет приёма данных
    receive_packet_active = 0U;

    // Сброс общих индексов перед началом пакета поиска
    addr_index = 0;
    data_index = 0;
    rx_flags = 0;
    retry_needed = 0;
    half_slot_phase = 0;

    // Подготовить поиск первого устройства
    search_prepare_next_device();
    return true;
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
void start_segment(uint8_t data, uint8_t crc, state_t seq_state) {
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
void start_received_segment()
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
// ========== Колбэк завершения сегмента и перехода к следующему ==========
static void on_segment_done(void) {
    if (retry_needed) {
        // Повторяем сегмент: используем сохранённый этап
    	start_segment(current_segment_byte, crc5(current_segment_byte), current_seq_state);
    } else {
        // Успешно – переходим к следующему этапу последовательности
        switch (current_seq_state) {
        case SEQ_ROM:
            // После успешной отправки ROM-команды всегда переходим
            // к фазе питания после ROM.
            // Для ROM-search специальная логика начнётся уже в next_step().
            global_state = SEQ_POWER_AFTER_ROM;
            next_step();
            break;
            case SEQ_ADDR:
                addr_index++; // индекс адреса двигаем только после успешной передачи
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
                data_index++; // индекс данных двигаем только после успешной передачи
                global_state = SEQ_POWER_AFTER_DATA;
                next_step();
                break;

            default:
                global_state = IDLE;
                break;

        }
    }
}

// ========== Завершение приёмного сегмента ==========
static void on_received_segment_done(void)
{
    if (retry_needed)
    {
        // CRC неверный.
        // Байт не сохраняем, data_index не увеличиваем.
        // После фазы питания снова запустится приём того же сегмента.
        global_state = SEQ_POWER_AFTER_RX_DATA;
        next_step();
        return;
    }

    // CRC верный – сохраняем принятый байт
    if (data_index < MAX_RECEIVED_DATA_SIZE)
    {
        received_data[data_index] = rx_data_byte;
    }

    // Счётчики успешного приёма
    data_index++;
    received_data_count = data_index;

    // Переходим к фазе питания после принятого сегмента
    global_state = SEQ_POWER_AFTER_RX_DATA;
    next_step();
}

// ========== Единственный обработчик прерывания таймера 2 ==========
// Вызывается каждые TIMESLOT_US / 2 микросекунд.
// Один логический тайм-слот протокола состоит из двух полутактов таймера.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM2) return;   // проверяем, что прерывание от нужного таймера

    switch (global_state) {
        // --- Отправка сегмента ---
    case SEG_START_BIT:
        if (half_slot_phase == 0)
        {
            // Первая половина стартового слота – мастер тянет линию в 0
            DATA_LOW();
            half_slot_phase = 1;
        }
        else
        {
            // Вторая половина стартового слота – линия всё ещё остаётся в 0.
            // После завершения стартового слота выбираем тип сегмента.
            half_slot_phase = 0;

            if (current_seq_state == SEQ_RX_DATA)
            {
                // Если это приёмный сегмент, дальше мастер принимает данные
                global_state = SEG_RECEIVE_DATA_BITS;
            }
            else
            {
                // Иначе это обычный сегмент передачи мастером
                global_state = SEG_DATA_BITS;
            }
        }
        break;
    case SEG_DATA_BITS:
    {
        uint8_t bit = (tx_data >> (7 - tx_bit_index)) & 1;

        if (half_slot_phase == 0) {
            if (bit) DATA_HIGH();
            else DATA_LOW();
            half_slot_phase = 1;
        } else {
            half_slot_phase = 0;

            if (tx_bit_index == 7) {
                tx_bit_index = 0;
                global_state = SEG_CRC_BITS;
            } else {
                tx_bit_index++;
            }
        }
    }
    break;
    case SEG_RECEIVE_DATA_BITS:
        if (half_slot_phase == 0)
        {
            // В начале слота мастер отпускает линию,
            // чтобы выбранное ведомое устройство могло выдать бит данных
            DATA_HIGH();
            half_slot_phase = 1;
        }
        else
        {
            // Во второй половине слота считываем бит данных
            uint8_t bit = DATA_READ() ? 1U : 0U;

            // Собираем байт от старшего бита к младшему:
            // первый принятый бит станет битом 7, последний – битом 0
            rx_data_byte = (uint8_t)((rx_data_byte << 1) | bit);

            half_slot_phase = 0;

            if (rx_bit_index == 7U)
            {
                // Все 8 бит данных приняты, переходим к приёму CRC
                rx_bit_index = 0;
                global_state = SEG_RECEIVE_CRC_BITS;
            }
            else
            {
                rx_bit_index++;
            }
        }
        break;
    case SEG_CRC_BITS:
    {
        uint8_t bit = (tx_crc >> (4 - tx_bit_index)) & 1;

        if (half_slot_phase == 0) {
            if (bit) DATA_HIGH();
            else DATA_LOW();
            half_slot_phase = 1;
        } else {
            half_slot_phase = 0;

            if (tx_bit_index == 4) {
                global_state = SEG_RECV_ALARM;
            } else {
                tx_bit_index++;
            }
        }
    }
    break;
    case SEG_RECEIVE_CRC_BITS:
        if (half_slot_phase == 0)
        {
            // Отпускаем линию, чтобы ведомое устройство могло передать бит CRC
            DATA_HIGH();
            half_slot_phase = 1;
        }
        else
        {
            // Считываем очередной бит CRC
            uint8_t bit = DATA_READ() ? 1U : 0U;

            // CRC тоже принимаем от старшего бита к младшему
            rx_crc = (uint8_t)(((rx_crc << 1) | bit) & 0x1FU);

            half_slot_phase = 0;

            if (rx_bit_index == 4U)
            {
                // Все 5 бит CRC приняты
                rx_bit_index = 0;

                // Проверяем CRC для принятого байта
                rx_crc_ok = check_crc5(rx_data_byte, rx_crc) ? 1U : 0U;

                // Если CRC неверный, этот сегмент надо будет повторить
                retry_needed = (rx_crc_ok == 0U) ? 1U : 0U;

                // Далее принимаем служебный бит аварии
                global_state = SEG_RECEIVE_ALARM;
            }
            else
            {
                rx_bit_index++;
            }
        }
        break;
        // --- Приём трёх битов ответа ---
    case SEG_RECV_ALARM:
        if (half_slot_phase == 0) {
            DATA_HIGH();             // отпускаем линию в начале слота
            half_slot_phase = 1;
        } else {
            // Если хотя бы одно ведомое устройство притянуло линию к 0,
            // фиксируем аварийный флаг.
            if (!DATA_READ()) rx_flags |= 0x01;   // читаем в середине слота
            half_slot_phase = 0;
            global_state = SEG_RECV_INTR;
        }
        break;
    case SEG_RECEIVE_ALARM:
        if (half_slot_phase == 0)
        {
            // Отпускаем линию, чтобы ведомые могли сообщить об аварии
            DATA_HIGH();
            half_slot_phase = 1;
        }
        else
        {
            // Активный уровень аварии – 0.
            // Если линия осталась в 1, аварии нет.
            // Если хотя бы одно ведомое устройство притянуло линию к 0,
            // фиксируем аварийный флаг.
            if (!DATA_READ())
            {
                rx_flags |= 0x01U;
            }

            half_slot_phase = 0;
            global_state = SEG_RECEIVE_INTR;
        }
        break;

    case SEG_RECEIVE_INTR:
        if (half_slot_phase == 0)
        {
            // Отпускаем линию для бита прерывания
            DATA_HIGH();
            half_slot_phase = 1;
        }
        else
        {
            // Если линия притянута к 0, фиксируем запрос прерывания.
            if (!DATA_READ())
            {
                rx_flags |= 0x02U;
            }

            half_slot_phase = 0;
            global_state = SEG_SEND_CRC_ACK;
        }
        break;
    case SEG_RECV_INTR:
        if (half_slot_phase == 0) {
            DATA_HIGH();
            half_slot_phase = 1;
        } else {
        	// Если линия притянута к 0, фиксируем запрос прерывания.
            if (!DATA_READ()) rx_flags |= 0x02;
            half_slot_phase = 0;
            global_state = SEG_RECV_CRC;
        }
        break;

    case SEG_RECV_CRC:
        if (half_slot_phase == 0)
        {
            // Мастер отпускает линию, чтобы ведомое устройство
            // могло передать бит подтверждения CRC.
            DATA_HIGH();
            half_slot_phase = 1;
        }
        else
        {
            // Считываем бит подтверждения CRC.
            // 1 – CRC принят ведомым верно.
            // 0 – CRC не принят, сегмент нужно повторить.
            uint8_t crc_ack = DATA_READ() ? 1U : 0U;

            if (crc_ack)
            {
                // Ведомое подтвердило корректный приём CRC
                rx_flags |= 0x04U;
                retry_needed = 0U;
            }
            else
            {
                // Ведомое не подтвердило CRC, нужен повтор сегмента
                rx_flags &= (uint8_t)~0x04U;
                retry_needed = 1U;
            }

            half_slot_phase = 0;
            stop_timer();
            global_state = IDLE;
            on_segment_done();
        }
        break;
    case SEG_SEND_CRC_ACK:
        if (half_slot_phase == 0)
        {
            // Мастер передаёт результат проверки CRC.
            // 1 – сегмент принят, 0 – нужно повторить.
            if (rx_crc_ok)
            {
                DATA_HIGH();
            }
            else
            {
                DATA_LOW();
            }

            half_slot_phase = 1;
        }
        else
        {
            // Слот подтверждения CRC завершён
            half_slot_phase = 0;

            stop_timer();

            // Временно переводим автомат в IDLE,
            // дальше переход выполнит on_received_segment_done()
            global_state = IDLE;

            on_received_segment_done();
        }
        break;
        // ===== Стартовый бит поискового сегмента =====
        case SSEG_START_BIT:
            if (half_slot_phase == 0)
            {
                // Первая половина стартового слота - линия в 0
                DATA_LOW();
                half_slot_phase = 1;
            }
            else
            {
                // Вторая половина стартового слота - тоже держим 0
                half_slot_phase = 0;
                global_state = SSEG_BODY;
            }
            break;

        // ===== Основное тело поискового сегмента =====
        case SSEG_BODY:
            if (half_slot_phase == 0)
            {
                // Первая половина текущего тайм-слота

                if ((!search_device_complete) && (!search_error))
                {
                    if (search_group_phase == 0U)
                    {
                        // Первая фаза тройки: читаем id_bit от ведомых
                        // Мастер отпускает линию
                        DATA_HIGH();
                    }
                    else if (search_group_phase == 1U)
                    {
                        // Вторая фаза тройки: читаем cmp_id_bit
                        DATA_HIGH();
                    }
                    else
                    {
                        // Третья фаза тройки: мастер передаёт выбранное направление
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
                    // Если адрес уже найден или произошла ошибка,
                    // оставшиеся слоты в сегменте заполняем 1
                    DATA_HIGH();
                }

                half_slot_phase = 1;
            }
            else
            {
                // Вторая половина текущего тайм-слота

                if ((!search_device_complete) && (!search_error))
                {
                    if (search_group_phase == 0U)
                    {
                        // Считать обычный бит адреса от ведомых
                        search_id_bit = DATA_READ();
                    }
                    else if (search_group_phase == 1U)
                    {
                        // Считать инвертированный бит адреса
                        search_cmp_id_bit = DATA_READ();

                        // После получения пары битов определить направление поиска
                        if (!search_choose_direction())
                        {
                            search_error = 1U;
                        }
                    }
                    else
                    {
                        // Завершена тройка поиска, один бит адреса обработан
                        search_bit_index++;

                        // Если дошли до конца 32-битного адреса,
                        // оставшиеся тройки в этом сегменте будут заполняющими
                        if (search_bit_index >= SEARCH_TOTAL_BITS)
                        {
                            search_device_complete = 1U;
                        }
                    }
                }

                half_slot_phase = 0;

                // Переход внутри тройки
                if (search_group_phase < 2U)
                {
                    search_group_phase++;
                }
                else
                {
                    // Текущая тройка завершена, переходим к следующей
                    search_group_phase = 0;
                    search_group_index++;

                    // После 5 троек переходим к последнему пустому биту сегмента
                    if (search_group_index >= SEARCH_GROUPS_PER_SEGMENT)
                    {
                        global_state = SSEG_PAD_BIT;
                    }
                }
            }
            break;

        // ===== Последний "пустой" бит поискового сегмента =====
        case SSEG_PAD_BIT:
            if (half_slot_phase == 0)
            {
                // Этот бит всегда 1 и не несёт информации
                DATA_HIGH();
                half_slot_phase = 1;
            }
            else
            {
                // Поисковый сегмент завершён
                half_slot_phase = 0;
                stop_timer();
                on_search_segment_done();
            }
            break;

            // --- Ожидание фазы питания (неблокирующее) ---
        case SEQ_WAIT_PHASE:
            if (phase_ticks > 0U) {
            	DATA_HIGH();
                phase_ticks--;
            }
            if (phase_ticks == 0U) {
                stop_timer();
                half_slot_phase = 0;
                global_state = next_state_after_phase;
                next_step();
            }
            break;

        default:
            break;  // для других состояний (SEQ_*) ничего не делаем в прерывании
    }
}

// ========== Определение next_step (перенесено выше) ==========
static void next_step(void) {
    uint8_t value;
    switch (global_state) {
    //отправка ROM-команды
    case SEQ_ROM:
         start_segment(current_rom_cmd, crc5(current_rom_cmd), SEQ_ROM);// отправляем rom команду
         break;
    //фаза питания после ROM-команды
    case SEQ_POWER_AFTER_ROM:
        // Если активен специальный пакет ROM-search,
        // после ROM-команды переходим к поисковым сегментам.
        if (search_packet_active && current_rom_cmd == ROM_SEARCH_CMD)
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
    //отправка размера+1(функциональная команда)
    case SEQ_SIZE:
    	// CRC для байта размера (данные+функц. команда)
        start_segment(current_datasize + 1U, crc5(current_datasize + 1U),SEQ_SIZE);    // отправляем размер
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
            search_packet_active = 0U;
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
            if (search_last_device_flag || (found_rom_count >= MAX_FOUND_DEVICES))
            {
                search_packet_active = 0U;
                search_done = 1U;
                search_success = (found_rom_count > 0U) ? 1U : 0U;

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
    search_packet_active = 0U;

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
    search_packet_active = 0U;

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

// ========== Инициализация протокола ==========
void protocol_init(void) {
    DATA_HIGH();                        // устанавливаем линию в 1 (высокий уровень, резистор подтянет)
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //настройка прерываний от таймера
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  protocol_init();//инициализация протокола
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  search_rom_packet_blocking();
  while(receive_pack_async(PARAMETERS, 4U, found_roms[0]));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint8_t slave_id[ROM_ID_LEN] = {0xA1, 0x10, 0x12, 0x34};
	  int i=0;
	  //тест skip off

	  while(i<1000000){
		  ++i;
	  }
	  i=0;
      // Запускаем пакет асинхронно (без блокировки)
      send_pack_async(ROM_SKIP_CMD, FUNC_OFF, 0, NULL,NULL);

      //тест skip on
      //цикл задержки между отправками пакетов
	  while(i<1000000){
		  ++i;
	  }
	  i=0;
      send_pack_async(ROM_SKIP_CMD, FUNC_ON, 0, NULL,NULL);

      //тест match off
	  while(i<1000000){
		  ++i;
	  }
	  i=0;
      send_pack_async(ROM_MATCH_CMD, FUNC_OFF, 0, NULL,slave_id);

      //тест resume off
	  while(i<1000000){
		  ++i;
	  }
	  i=0;
      send_pack_async(ROM_RESUME_CMD, FUNC_OFF, 0, NULL,NULL);

      //тест resume on
	  while(i<1000000){
		  ++i;
	  }
	  i=0;
      send_pack_async(ROM_RESUME_CMD, FUNC_ON, 0, NULL,NULL);

      //тест skip on 0xA1, 0x10, 0x12, 0x34
	  while(i<1000000){
		  ++i;
	  }
	  i=0;
      send_pack_async(ROM_SKIP_CMD, FUNC_ON, 4U, slave_id,NULL);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 34;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
