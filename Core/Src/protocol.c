#include "protocol.h"
#include <string.h>
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
static uint16_t phase_ticks = 0;
static state_t next_state_after_phase = IDLE;

// ===== Повтор сегмента при ошибке CRC =====
static uint8_t retry_needed = 0;

// ===== ROM-search =====
// Эти две переменные НЕ static, потому что они доступны из main.c через extern в protocol.h
uint8_t found_roms[MAX_FOUND_DEVICES][ROM_ID_LEN] = {0};
uint8_t found_rom_count = 0;

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

static uint8_t search_packet_active = 0;
static uint8_t search_device_complete = 0;
static uint8_t search_error = 0;
static uint8_t search_done = 0;
static uint8_t search_success = 0;

// ========== Работа с линией PB0 ==========
#define DATA_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define DATA_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define DATA_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)

// ========== Внутренние прототипы ==========
static void next_step(void);
static void on_segment_done(void);
static void on_received_segment_done(void);

static void start_timer(void);
static void stop_timer(void);
static void start_power_phase(state_t next_state);

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

// ========== Управление таймером ==========
static void start_timer(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);   // обнуляем счётчик таймера
    HAL_TIM_Base_Start_IT(&htim2);      // запускаем таймер с прерываниями
}

static  void stop_timer(void) {
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
    // Первый сегмент пакета – ROM_SEARCH_CMD
    current_rom_cmd = ROM_SEARCH_CMD;
    global_state = SEQ_ROM;
    next_step();
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
            // 1 – принятый CRC верен
            // 0 – CRC неверен, ведомый должен повторить сегмент
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
            // ВАЖНО: здесь не делаем DATA_HIGH(),
            // чтобы не укоротить ACK/NACK внутри тайм-слота.
            // Линия будет гарантированно отпущена в start_power_phase().
            half_slot_phase = 0U;

            stop_timer();
            global_state = IDLE;
            on_received_segment_done();
        }
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
            stop_timer();
            half_slot_phase = 0U;
            global_state = next_state_after_phase;
            next_step();
        }
        break;

    default:
        break;
    }
}

// ========== Инициализация протокола ==========
void protocol_init(void) {
    DATA_HIGH();                        // устанавливаем линию в 1 (высокий уровень, резистор подтянет)
    global_state = IDLE;
    current_seq_state = IDLE;
    half_slot_phase = 0U;
    retry_needed = 0U;
}

bool protocol_is_busy(void)
{
    return (global_state != IDLE);
}
