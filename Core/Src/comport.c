/*
 * comport.c
 *
 *  Created on: May 2, 2026
 *      Author: stepa
 */
/*
 * comport.c
 *
 * Модуль обработки пользовательского ввода через COM-порт.
 *
 * Назначение файла:
 * 1. Получать готовые строки из USB CDC.
 * 2. Разбирать введённый пользователем пакет.
 * 3. Проверять корректность ROM-команды, адреса, размера, функциональной команды и данных.
 * 4. Записывать разобранные параметры в глобальные переменные.
 * 5. Устанавливать флаг comport_command_ready, по которому main.c запускает нужную функцию протокола.
 *
 * Важно:
 * comport.c сам НЕ запускает протокол.
 * Он только готовит параметры пакета.
 * Запуск send_pack_async(), receive_pack_async() или search_rom_packet_async()
 * выполняется в main.c.
 */

#include "comport.h"
#include "usbd_cdc_if.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>


// ============================================================================
// Глобальные переменные, которые использует main.c
// ============================================================================

// ROM-команда будущего пакета.
// Например: ROM_SKIP_CMD, ROM_MATCH_CMD, ROM_SEARCH_CMD.
uint8_t rom_cmd = 0;

// Размер данных.
// Для TX это количество байтов, которые мастер должен передать.
// Для RX это количество байтов, которые мастер должен принять.
uint8_t data_size = 0;

// Функциональная команда.
// Например: FUNC_OFF, FUNC_ON, PARAMETERS.
uint8_t func_cmd = 0;

// Тип пакета:
// PACKET_TYPE_TX     – передача данных мастером;
// PACKET_TYPE_RX     – приём данных мастером;
// PACKET_TYPE_SEARCH – поиск ведомых устройств.
uint8_t packet_type = PACKET_TYPE_TX;

// Адрес ведомого устройства.
// Используется, если ROM-команда требует адрес, например MATCH ROM.
uint8_t address_arr[ROM_ID_LEN] = {0};

// Буфер данных, которые пользователь хочет передать.
// Используется только для TX-пакетов.
uint8_t data_arr[MAX_RECEIVED_DATA_SIZE] = {0};

// Флаг готовности пакета.
// comport.c устанавливает его в 1 после успешного разбора строки.
// main.c после запуска пакета должен сбросить его через comport_clear_command().
volatile uint8_t comport_command_ready = 0;

// ============================================================================
// Команды ручного тестирования PPE-3323
// ============================================================================

volatile uint8_t ppe_command_ready = 0U;
uint8_t ppe_command_type = PPE_COMMAND_NONE;

uint8_t ppe_command_voltage_v = 0U;
uint16_t ppe_command_current_ma = 0U;

// ============================================================================
// Приём плана питания из COM-порта
// ============================================================================
//
// Формат ввода:
//
// PLAN
// N
// U1 U2 ... UN
// I1 I2 ... IN
// T_ACTIVE_US
// T_EXTRA_US
// MASK_DEVICE_0
// MASK_DEVICE_1
// ...
//
// Количество строк MASK_DEVICE_x равно found_rom_count.
// Порядок масок соответствует порядку адресов found_roms[],
// которые были выведены на ПК после поиска.

typedef enum
{
    FEED_PLAN_INPUT_NONE = 0,
    FEED_PLAN_INPUT_PHASE_COUNT,
    FEED_PLAN_INPUT_VOLTAGES,
    FEED_PLAN_INPUT_CURRENTS,
    FEED_PLAN_INPUT_DURATION,
    FEED_PLAN_INPUT_EXTRA_DURATION,
    FEED_PLAN_INPUT_MASKS
} feed_plan_input_step_t;

static feed_plan_input_step_t feed_plan_input_step = FEED_PLAN_INPUT_NONE;

static uint8_t feed_plan_input_phase_count = 0U;
static uint8_t feed_plan_input_voltage[FEED_PLAN_MAX_PHASES] = {0};
/*
 * Токи фаз в мА.
 *
 * uint16_t позволяет принимать значения до 3000 мА
 * для первого регулируемого выхода PPE-3323.
 */
static uint16_t feed_plan_input_current[FEED_PLAN_MAX_PHASES] = {0};
static uint32_t feed_plan_input_duration_us = 0U;
// Дополнительное время на перестройку источника питания, мкс.
// Оно вводится отдельной строкой после активной длительности фазы.
static uint32_t feed_plan_input_extra_duration_us = 0U;

static uint32_t feed_plan_input_masks[MAX_FOUND_DEVICES] = {0};
static uint8_t feed_plan_input_mask_index = 0U;

// ============================================================================
// Прототипы внутренних функций
// ============================================================================

// Сравнение строк без учёта регистра.
// Нужно, чтобы пользователь мог писать SKIP, skip, Skip и т.д.
static bool str_eq_ci(const char *a, const char *b);

// Проверка, содержит ли строка шестнадцатеричные буквы A..F.
// Нужна для автоматического распознавания HEX-чисел.
static bool token_has_hex_letters(const char *s);

// Проверка, состоит ли строка только из HEX-символов.
// Нужна для разбора адреса, записанного одним 32-битным числом.
static bool token_is_hex_string(const char *s);

// Разбор числа, которое должно быть обычным размером данных.
// Размер удобнее считать десятичным, если нет префикса 0x.
static bool parse_size_token(const char *token, uint8_t *value);

// Разбор байта протокола.
// Для байтов значения без 0x и длиной до двух символов считаются HEX.
// Например: 10 -> 0x10, A1 -> 0xA1, CC -> 0xCC.
static bool parse_byte_token(const char *token, uint8_t *value);

// Разбор типа пакета.
// Поддерживает TX/RX/SEARCH и числовые варианты 0/1/2.
static bool parse_packet_type_token(const char *token, uint8_t *type_out);

// Разбор ROM-команды.
// Поддерживает имена SKIP, MATCH, SEARCH и числовые значения.
static bool parse_rom_token(const char *token, uint8_t *rom_out);

// Разбор функциональной команды.
// Поддерживает имена OFF, ON, PARAMETERS и числовые значения.
static bool parse_func_token(const char *token, uint8_t *func_out);

// Разбор адреса ведомого устройства.
// Поддерживает два варианта:
// 1. Один 32-битный HEX-токен: A1101234 или 0xA1101234.
// 2. Четыре отдельных байта: A1 10 12 34.
static bool parse_address_tokens(char **argv, int argc, int *index);

// Проверка, требует ли ROM-команда адрес.
// Сейчас адрес нужен только для MATCH ROM.
static bool rom_requires_address(uint8_t rom);

// Вывод справки по доступным пакетам.
static void print_help(void);

// Запуск режима построчного ввода плана питания.
static void feed_plan_input_start(void);

// Сброс временного состояния ввода плана.
static void feed_plan_input_reset(void);

// Завершение ввода с ошибкой.
static void feed_plan_input_fail(void);

// Разбор одной строки плана питания.
static void feed_plan_process_input_line(char **argv, int argc);

/*
 * Разбор плана питания, введённого одной строкой.
 *
 * Формат:
 * PLAN N U1 ... UN I1 ... IN T_ACTIVE_US T_EXTRA_US
 *      MASK_ROM0 ... MASK_ROM[n]
 */
static void feed_plan_process_single_line(char **argv, int argc);


// Разбор десятичного uint32_t.
static bool parse_uint32_dec_token(const char *token, uint32_t *value);

// Разбор строки напряжений фаз, В.
static bool parse_feed_plan_voltage_line(char **argv,
                                         int argc,
                                         uint8_t expected_count,
                                         uint8_t *dst);

// Разбор строки токов фаз, мА.
static bool parse_feed_plan_current_line(char **argv,
                                         int argc,
                                         uint8_t expected_count,
                                         uint16_t *dst);

// Разбор битовой маски одного ведомого устройства.
static bool parse_feed_plan_mask_line(const char *token,
                                      uint8_t phase_count,
                                      uint32_t *mask_out);
// Разбор команды ручного тестирования PPE-3323.
static void ppe_process_input_command(char **argv, int argc);
// ============================================================================
// Отправка ответа в COM-порт
// ============================================================================

void comport_send_response(const char *fmt, ...)
{
    char tx_buf[256];

    // Формируем строку так же, как printf, только в локальный буфер.
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
    va_end(args);

    // Если строка не сформировалась, ничего не отправляем.
    if (len <= 0)
    {
        return;
    }

    // Если строка не поместилась полностью, отправляем обрезанную версию.
    if (len >= (int)sizeof(tx_buf))
    {
        len = sizeof(tx_buf) - 1;
    }

    // CDC_Transmit_FS может вернуть USBD_BUSY,
    // если USB ещё передаёт предыдущие данные.
    // Поэтому делаем несколько коротких попыток.
    for (uint8_t i = 0; i < 20U; i++)
    {
        if (CDC_Transmit_FS((uint8_t *)tx_buf, (uint16_t)len) == USBD_OK)
        {
            return;
        }

        //HAL_Delay(1);
    }
}

// ============================================================================
// Опрос COM-порта
// ============================================================================
// Проверить очередь USB CDC и обработать все введённые строки
void comport_poll(void)
{
    static char line[COMPORT_LINE_MAX];

    // USBD_CDC_GetLine() возвращает готовую строку из очереди USB CDC.
    // Если пользователь ввёл несколько строк подряд, обрабатываем все.
    while (USBD_CDC_GetLine(line, sizeof(line)) > 0U)
    {
        comport_process_line(line);
    }
}

// Сбросить флаг готовой команды
void comport_clear_command(void)
{
    // main.c вызывает эту функцию после того,
    // как успешно запустил пакет протокола.
    comport_command_ready = 0U;
}

void comport_clear_ppe_command(void)
{
    ppe_command_ready = 0U;
    ppe_command_type = PPE_COMMAND_NONE;

    ppe_command_voltage_v = 0U;
    ppe_command_current_ma = 0U;
}

// ============================================================================
// Разбор одной строки из COM-порта
// ============================================================================
//
// Формат TX-пакета:
//   PACK TX <rom> <size> [address] <func> [data...]
//
// Формат RX-пакета:
//   PACK RX MATCH <size> <address> <func>
//
// Формат SEARCH-пакета:
//   PACK SEARCH
//
// Примеры:
//   PACK TX SKIP 0 OFF
//   PACK TX MATCH 0 A1 10 12 34 OFF
//   PACK TX SKIP 4 ON 11 22 33 44
//   PACK RX MATCH 10 A1 10 12 34 PARAMETERS
//   PACK SEARCH
//
// Числа для байтов можно писать так:
//   A1
//   0xA1
//   161
//
// Но для байтов длиной 1 или 2 символа значение считается HEX:
//   10 -> 0x10
//   22 -> 0x22
//
// Размер данных считается десятичным:
//   4 -> 4 байта
//   10 -> 10 байтов
//   0x10 -> 16 байтов
//

// Обработать одну строку команды
void comport_process_line(char *line)
{
    char *argv[COMPORT_MAX_TOKENS];
    int argc = 0;

    // Разбиваем входную строку на отдельные токены.
    // Разделители: пробел, табуляция, \r, \n.
    char *token = strtok(line, " \t\r\n");

    while ((token != NULL) && (argc < COMPORT_MAX_TOKENS))
    {
        argv[argc++] = token;
        token = strtok(NULL, " \t\r\n");
    }

    // Пустую строку просто игнорируем.
    if (argc == 0)
    {
        return;
    }

    // Если уже начат построчный ввод плана питания,
    // все следующие строки считаются частью плана.
    if (feed_plan_input_step != FEED_PLAN_INPUT_NONE)
    {
        feed_plan_process_input_line(argv, argc);
        return;
    }

    // Справка.
    if (str_eq_ci(argv[0], "HELP") || strcmp(argv[0], "?") == 0)
    {
        print_help();
        return;
    }

    // Статус протокола и последних результатов.
    if (str_eq_ci(argv[0], "STATUS"))
    {
        comport_send_response("Protocol busy: %u\r\n", protocol_is_busy() ? 1U : 0U);
        comport_send_response("Packet ready: %u\r\n", comport_command_ready ? 1U : 0U);
        comport_send_response("Found ROM count: %u\r\n", found_rom_count);
        comport_send_response("Received data count: %u\r\n", received_data_count);
        return;
    }

    // Печать найденных адресов после ROM-search.
    if (str_eq_ci(argv[0], "FOUND"))
    {
        comport_print_found_roms();
        return;
    }

    // Печать данных, принятых мастером от ведомого устройства.
    if (str_eq_ci(argv[0], "RXDATA"))
    {
        comport_print_received_data();
        return;
    }

    // Запуск построчного ввода плана питания.
    // После этой команды пользователь вводит:
    // N, строку напряжений, строку токов, длительность,
    // затем по одной битовой маске на каждое найденное ведомое устройство.
    if (str_eq_ci(argv[0], "PLAN") || str_eq_ci(argv[0], "FEEDPLAN"))
    {
        /*
         * Основной формат: план передаётся одной строкой.
         *
         * PLAN N U1 ... UN I1 ... IN T_ACTIVE_US T_EXTRA_US
         *      MASK_ROM0 ... MASK_ROM[n]
         *
         * Если передано только слово PLAN, старый построчный формат
         * остаётся доступным как резервный вариант.
         */
        if (argc == 1)
        {
            feed_plan_input_start();
            return;
        }

        feed_plan_process_single_line(argv, argc);
        return;
    }
    /*
     * Ручные команды PPE-3323.
     *
     * Они используются только для проверки связи USART1 → MAX3232 → PPE-3323.
     * Автоматическое управление источником из фазы питания добавится позже.
     */
    if (str_eq_ci(argv[0], "PPE"))
    {
        ppe_process_input_command(argv, argc);
        return;
    }
    // Если один пакет уже разобран, но main.c ещё не успел его запустить,
    // новую строку с пакетом принимать нельзя.
    if (comport_command_ready)
    {
        comport_send_response("Error: previous packet is not processed yet\r\n");
        return;
    }

    int index = 0;

    // Слово PACK можно писать, а можно опускать.
    // То есть поддерживаются оба варианта:
    //   PACK TX SKIP 0 OFF
    //   TX SKIP 0 OFF
    if (str_eq_ci(argv[index], "PACK"))
    {
        index++;
    }

    if (index >= argc)
    {
        comport_send_response("Error: missing packet type\r\n");
        return;
    }

    // Разбираем тип пакета: TX, RX или SEARCH.
    uint8_t parsed_type = 0;
    if (!parse_packet_type_token(argv[index], &parsed_type))
    {
        comport_send_response("Error: invalid packet type\r\n");
        return;
    }

    index++;

    // =====================================================================
    // Пакет поиска устройств
    // =====================================================================
    if (parsed_type == PACKET_TYPE_SEARCH)
    {
        // Разрешаем короткий вариант:
        //   PACK SEARCH
        //
        // И более явный вариант:
        //   PACK SEARCH SEARCH
        //   PACK 2 F0
        //
        // Но если после SEARCH что-то указано, это обязательно должно быть
        // ROM_SEARCH_CMD.
        if (index < argc)
        {
            uint8_t parsed_rom = 0;

            if (!parse_rom_token(argv[index], &parsed_rom))
            {
                comport_send_response("Error: invalid ROM command for SEARCH packet\r\n");
                return;
            }

            if (parsed_rom != ROM_SEARCH_CMD)
            {
                comport_send_response("Error: SEARCH packet must use ROM_SEARCH_CMD\r\n");
                return;
            }

            index++;
        }

        // У SEARCH-пакета не должно быть размера, функции, адреса и данных.
        if (index != argc)
        {
            comport_send_response("Error: too many arguments for SEARCH packet\r\n");
            return;
        }

        // Записываем подготовленный пакет в глобальные переменные.
        packet_type = PACKET_TYPE_SEARCH;
        rom_cmd = ROM_SEARCH_CMD;
        data_size = 0U;
        func_cmd = 0U;

        memset(address_arr, 0, sizeof(address_arr));
        memset(data_arr, 0, sizeof(data_arr));

        // Сообщаем main.c, что пакет готов к запуску.
        comport_command_ready = 1U;

        comport_send_response("OK: SEARCH packet prepared\r\n");
        return;
    }

    // Для TX и RX после типа пакета обязательно нужны:
    // ROM-команда, размер данных и функциональная команда.
    // Для MATCH между размером и функцией ещё будет адрес.
    if ((argc - index) < 3)
    {
        comport_send_response("Error: not enough arguments\r\n");
        return;
    }

    // Разбираем ROM-команду.
    uint8_t parsed_rom = 0;
    if (!parse_rom_token(argv[index], &parsed_rom))
    {
        comport_send_response("Error: invalid ROM command\r\n");
        return;
    }

    index++;

    // Разбираем размер данных.
    uint8_t parsed_size = 0;
    if (!parse_size_token(argv[index], &parsed_size))
    {
        comport_send_response("Error: invalid data size\r\n");
        return;
    }

    index++;

    // Проверяем, что размер не выходит за границы буфера.
    if (parsed_size > MAX_RECEIVED_DATA_SIZE)
    {
        comport_send_response("Error: data size is too large\r\n");
        return;
    }

    // =====================================================================
    // Пакет приёма данных мастером
    // =====================================================================
    if (parsed_type == PACKET_TYPE_RX)
    {
        // В текущей реализации приём данных сделан только через MATCH ROM,
        // потому что мастер должен выбрать конкретное ведомое устройство.
        if (parsed_rom != ROM_MATCH_CMD)
        {
            comport_send_response("Error: RX packet must use MATCH ROM\r\n");
            return;
        }

        // Нельзя принимать ноль байтов.
        if (parsed_size == 0U)
        {
            comport_send_response("Error: RX size must be greater than zero\r\n");
            return;
        }

        // Для RX через MATCH ROM обязательно нужен адрес.
        if (!parse_address_tokens(argv, argc, &index))
        {
            comport_send_response("Error: invalid or missing address\r\n");
            return;
        }

        // После адреса должна идти функциональная команда,
        // которая говорит ведомому, какие данные нужно выдать.
        if (index >= argc)
        {
            comport_send_response("Error: missing functional command\r\n");
            return;
        }

        uint8_t parsed_func = 0;
        if (!parse_func_token(argv[index], &parsed_func))
        {
            comport_send_response("Error: invalid functional command\r\n");
            return;
        }

        index++;

        // В RX-пакете после функциональной команды пользовательские данные
        // указывать нельзя, потому что данные будет выдавать ведомое устройство.
        if (index != argc)
        {
            comport_send_response("Error: RX packet must not contain data bytes\r\n");
            return;
        }

        // Записываем разобранные параметры.
        packet_type = PACKET_TYPE_RX;
        rom_cmd = parsed_rom;
        data_size = parsed_size;
        func_cmd = parsed_func;

        memset(data_arr, 0, sizeof(data_arr));

        comport_command_ready = 1U;

        comport_send_response("OK: RX packet prepared\r\n");
        return;
    }

    // =====================================================================
    // Пакет передачи данных мастером
    // =====================================================================
    if (parsed_type == PACKET_TYPE_TX)
    {
        // ROM_SEARCH_CMD нельзя запускать как обычную передачу.
        // Для этого есть отдельный тип пакета SEARCH.
        if (parsed_rom == ROM_SEARCH_CMD)
        {
            comport_send_response("Error: use SEARCH packet type for ROM_SEARCH_CMD\r\n");
            return;
        }

        memset(address_arr, 0, sizeof(address_arr));
        memset(data_arr, 0, sizeof(data_arr));

        // Если ROM-команда требует адрес, считываем его.
        // Сейчас это только MATCH ROM.
        if (rom_requires_address(parsed_rom))
        {
            if (!parse_address_tokens(argv, argc, &index))
            {
                comport_send_response("Error: invalid or missing address\r\n");
                return;
            }
        }

        // После ROM-команды, размера и возможного адреса
        // должна идти функциональная команда.
        if (index >= argc)
        {
            comport_send_response("Error: missing functional command\r\n");
            return;
        }

        uint8_t parsed_func = 0;
        if (!parse_func_token(argv[index], &parsed_func))
        {
            comport_send_response("Error: invalid functional command\r\n");
            return;
        }

        index++;

        // Если размер данных больше нуля,
        // после функциональной команды должно идти ровно parsed_size байтов.
        for (uint8_t i = 0; i < parsed_size; i++)
        {
            if (index >= argc)
            {
                comport_send_response("Error: missing data byte %u\r\n", i);
                return;
            }

            if (!parse_byte_token(argv[index], &data_arr[i]))
            {
                comport_send_response("Error: invalid data byte %u\r\n", i);
                return;
            }

            index++;
        }

        // Если после ожидаемых байтов что-то осталось,
        // значит пользователь ввёл лишние параметры.
        if (index != argc)
        {
            comport_send_response("Error: too many data bytes\r\n");
            return;
        }

        // Записываем разобранные параметры.
        packet_type = PACKET_TYPE_TX;
        rom_cmd = parsed_rom;
        data_size = parsed_size;
        func_cmd = parsed_func;

        comport_command_ready = 1U;

        comport_send_response("OK: TX packet prepared\r\n");
        return;
    }

    // Если сюда дошли, тип пакета не распознан.
    comport_send_response("Error: unknown packet type\r\n");
}

// ============================================================================
// Печать результатов работы протокола
// ============================================================================

void comport_print_found_roms(void)
{
    comport_send_response("Found devices: %u\r\n", found_rom_count);

    // Каждый адрес состоит из 4 байтов.
    for (uint8_t i = 0; i < found_rom_count; i++)
    {
        comport_send_response("ROM[%u]: %02X %02X %02X %02X\r\n",
                              i,
                              found_roms[i][0],
                              found_roms[i][1],
                              found_roms[i][2],
                              found_roms[i][3]);
    }
}

// ========== Вывод параметров питания найденных ведомых устройств ==========
// Параметры берутся из массива power_params[].
// Индекс power_params[i] соответствует адресу found_roms[i].
//
// Для каждого ведомого устройства выводится 5 строк:
// 1. ROM-адрес;
// 2. напряжение;
// 3. ток;
// 4. время заряда;
// 5. время работы от внутреннего накопителя.
void comport_print_power_params(void)
{
    if (found_rom_count == 0U)
    {
        comport_send_response("Power parameters: no devices\r\n");
        return;
    }

    comport_send_response("Power parameters:\r\n");

    for (uint8_t i = 0U; i < found_rom_count; i++)
    {
        // Первая строка – адрес ведомого устройства.
        comport_send_response("DEVICE[%u] ROM: %02X %02X %02X %02X\r\n",
                              i,
                              found_roms[i][0],
                              found_roms[i][1],
                              found_roms[i][2],
                              found_roms[i][3]);

        // Если параметры для этого адреса не были успешно приняты,
        // всё равно выводим 5 строк на устройство, но явно показываем ошибку.
        if (!power_params[i].valid)
        {
            comport_send_response("  Voltage: not received\r\n");
            comport_send_response("  Current: not received\r\n");
            comport_send_response("  Charge time: not received\r\n");
            comport_send_response("  Work time: not received\r\n");
            continue;
        }

        // Внутри power_params[] напряжение и ток хранятся как uint32_t,
        // хотя по линии они приходят одним байтом.
        comport_send_response("  Voltage: %lu V\r\n",
                              (unsigned long)power_params[i].voltage);

        comport_send_response("  Current: %lu mA\r\n",
                              (unsigned long)power_params[i].current);

        // Время заряда и время работы хранятся в микросекундах.
        comport_send_response("  Charge time: %lu us\r\n",
                              (unsigned long)power_params[i].charge_time_us);

        comport_send_response("  Work time: %lu us\r\n",
                              (unsigned long)power_params[i].work_time_us);
    }
}

// ========== Сброс временного состояния ввода плана питания ==========
static void feed_plan_input_reset(void)
{
    feed_plan_input_step = FEED_PLAN_INPUT_NONE;

    feed_plan_input_phase_count = 0U;
    feed_plan_input_duration_us = 0U;
    feed_plan_input_mask_index = 0U;
    feed_plan_input_extra_duration_us = 0U;
    memset(feed_plan_input_voltage, 0, sizeof(feed_plan_input_voltage));
    memset(feed_plan_input_current, 0, sizeof(feed_plan_input_current));
    memset(feed_plan_input_masks, 0, sizeof(feed_plan_input_masks));
}


// ========== Запуск построчного ввода плана питания ==========
static void feed_plan_input_start(void)
{
    // Если устройств ещё нет, принимать план бессмысленно:
    // маски должны соответствовать порядку found_roms[].
    if (found_rom_count == 0U)
    {
        comport_send_response("wrong data\r\n");
        return;
    }

    feed_plan_input_reset();

    // Следующая строка должна содержать количество фаз N.
    feed_plan_input_step = FEED_PLAN_INPUT_PHASE_COUNT;

    comport_send_response("OK: enter feed plan\r\n");
}


// ========== Ошибка ввода плана питания ==========
static void feed_plan_input_fail(void)
{
    // При любой ошибке сбрасываем временное состояние
    // и выводим общее сообщение, без детальной диагностики.
    feed_plan_input_reset();
    comport_send_response("wrong data\r\n");
}

// ========== Разбор плана питания, введённого одной строкой ==========
static void feed_plan_process_single_line(char **argv, int argc)
{
    uint32_t phase_count_value = 0U;
    uint32_t expected_argc = 0U;

    int voltage_index;
    int current_index;
    int active_duration_index;
    int extra_duration_index;
    int masks_index;

    /*
     * План невозможно связать с масками, если поиск устройств
     * ещё не выполнялся или не обнаружил ни одного ведомого.
     */
    if ((argv == NULL) || (argc < 2) || (found_rom_count == 0U))
    {
        feed_plan_input_fail();
        return;
    }

    /*
     * argv[0] содержит PLAN.
     * argv[1] содержит количество фаз.
     */
    if (!parse_uint32_dec_token(argv[1], &phase_count_value))
    {
        feed_plan_input_fail();
        return;
    }

    if ((phase_count_value == 0U) ||
        (phase_count_value > FEED_PLAN_MAX_PHASES))
    {
        feed_plan_input_fail();
        return;
    }

    /*
     * Общее число токенов:
     *
     * PLAN
     * + N
     * + N напряжений
     * + N токов
     * + T_ACTIVE
     * + T_EXTRA
     * + found_rom_count масок.
     */
    expected_argc = 4U +
                    (2U * phase_count_value) +
                    (uint32_t)found_rom_count;

    if ((uint32_t)argc != expected_argc)
    {
        feed_plan_input_fail();
        return;
    }

    /*
     * Очищаем временные массивы и используем их для разбора
     * однострочной команды так же, как в старом построчном режиме.
     */
    feed_plan_input_reset();

    feed_plan_input_phase_count = (uint8_t)phase_count_value;

    voltage_index = 2;
    current_index = voltage_index + (int)feed_plan_input_phase_count;

    active_duration_index =
            current_index + (int)feed_plan_input_phase_count;

    extra_duration_index = active_duration_index + 1;
    masks_index = extra_duration_index + 1;

    /*
     * Разбор напряжений фаз.
     */
    if (!parse_feed_plan_voltage_line(
            &argv[voltage_index],
            feed_plan_input_phase_count,
            feed_plan_input_phase_count,
            feed_plan_input_voltage))
    {
        feed_plan_input_fail();
        return;
    }

    /*
     * Разбор токов фаз.
     */
    if (!parse_feed_plan_current_line(
            &argv[current_index],
            feed_plan_input_phase_count,
            feed_plan_input_phase_count,
            feed_plan_input_current))
    {
        feed_plan_input_fail();
        return;
    }

    /*
     * Разбор активной части фазы.
     */
    if (!parse_uint32_dec_token(
            argv[active_duration_index],
            &feed_plan_input_duration_us))
    {
        feed_plan_input_fail();
        return;
    }

    if (feed_plan_input_duration_us == 0U)
    {
        feed_plan_input_fail();
        return;
    }

    /*
     * Разбор дополнительного времени на передачу команд источнику
     * и установление нового напряжения.
     */
    if (!parse_uint32_dec_token(
            argv[extra_duration_index],
            &feed_plan_input_extra_duration_us))
    {
        feed_plan_input_fail();
        return;
    }

    /*
     * В разработанной программе дополнительное время не может быть
     * нулевым, так как в нём передаются VSET1, ISET1 и OUT1.
     */
    if (feed_plan_input_extra_duration_us == 0U)
    {
        feed_plan_input_fail();
        return;
    }

    /*
     * Проверяем, что полная длительность фазы помещается в uint32_t.
     */
    if (feed_plan_input_extra_duration_us >
        (0xFFFFFFFFUL - feed_plan_input_duration_us))
    {
        feed_plan_input_fail();
        return;
    }

    /*
     * Каждая маска должна содержать ровно N символов 0 или 1.
     * Количество масок должно совпадать с количеством найденных ROM.
     */
    for (uint8_t i = 0U; i < found_rom_count; i++)
    {
        if (!parse_feed_plan_mask_line(
                argv[masks_index + (int)i],
                feed_plan_input_phase_count,
                &feed_plan_input_masks[i]))
        {
            feed_plan_input_fail();
            return;
        }
    }

    /*
         * Сохраняем план в protocol.c.
         */
        if (!feed_plan_store(feed_plan_input_phase_count,
                             feed_plan_input_voltage,
                             feed_plan_input_current,
                             feed_plan_input_duration_us,
                             feed_plan_input_extra_duration_us,
                             feed_plan_input_masks,
                             found_rom_count))
        {
            feed_plan_input_fail();
            return;
        }

        /*
         * Сразу запускаем адресную передачу индивидуальных FEED_PLAN
         * всем найденным ведомым устройствам.
         */
        if (!feed_plan_send_all_async())
        {
            feed_plan_reset();
            feed_plan_input_fail();
            return;
        }

        feed_plan_input_reset();

        comport_send_response("FEED PLAN saved\r\n");
}
// ========== Разбор десятичного uint32_t ==========
// Для плана питания числа вводятся как обычные десятичные значения.
static bool parse_uint32_dec_token(const char *token, uint32_t *value)
{
    if ((token == NULL) || (value == NULL) || (*token == '\0'))
    {
        return false;
    }

    char *endptr = NULL;
    unsigned long parsed = strtoul(token, &endptr, 10);

    if ((endptr == token) || (*endptr != '\0'))
    {
        return false;
    }

    if (parsed > 0xFFFFFFFFUL)
    {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}


// ========== Разбор строки напряжений фаз ==========
// Значения вводятся в вольтах и должны быть не больше 32 В.
static bool parse_feed_plan_voltage_line(char **argv,
                                         int argc,
                                         uint8_t expected_count,
                                         uint8_t *dst)
{
    if ((argv == NULL) || (dst == NULL))
    {
        return false;
    }

    if (argc != expected_count)
    {
        return false;
    }

    for (uint8_t i = 0U; i < expected_count; i++)
    {
        uint32_t value = 0U;

        if (!parse_uint32_dec_token(argv[i], &value))
        {
            return false;
        }

        if (value > POWER_SOURCE_MAX_VOLTAGE_V)
        {
            return false;
        }

        dst[i] = (uint8_t)value;
    }

    return true;
}


// ========== Разбор строки токов фаз ==========
// Значения вводятся в миллиамперах и должны быть не больше 3000 мА.
static bool parse_feed_plan_current_line(char **argv,
                                         int argc,
                                         uint8_t expected_count,
                                         uint16_t *dst)
{
    if ((argv == NULL) || (dst == NULL))
    {
        return false;
    }

    if (argc != expected_count)
    {
        return false;
    }

    for (uint8_t i = 0U; i < expected_count; i++)
    {
        uint32_t value = 0U;

        if (!parse_uint32_dec_token(argv[i], &value))
        {
            return false;
        }

        if (value > POWER_SOURCE_MAX_CURRENT_MA)
        {
            return false;
        }

        dst[i] = (uint16_t)value;
    }

    return true;
}

// ========== Разбор битовой маски одного ведомого устройства ==========
// Длина строки должна быть равна количеству фаз.
// Допускаются только символы '0' и '1'.
static bool parse_feed_plan_mask_line(const char *token,
                                      uint8_t phase_count,
                                      uint32_t *mask_out)
{
    if ((token == NULL) || (mask_out == NULL))
    {
        return false;
    }

    if (strlen(token) != phase_count)
    {
        return false;
    }

    uint32_t mask = 0U;

    for (uint8_t i = 0U; i < phase_count; i++)
    {
        if (token[i] == '1')
        {
            // Бит 0 соответствует первой фазе,
            // бит 1 – второй фазе и т.д.
            mask |= ((uint32_t)1U << i);
        }
        else if (token[i] == '0')
        {
            // В этой фазе ведомое устройство не питается.
        }
        else
        {
            return false;
        }
    }

    *mask_out = mask;
    return true;
}


// ========== Обработка одной строки построчного ввода плана питания ==========
static void feed_plan_process_input_line(char **argv, int argc)
{
    if ((argv == NULL) || (argc == 0))
    {
        feed_plan_input_fail();
        return;
    }

    if (feed_plan_input_step == FEED_PLAN_INPUT_PHASE_COUNT)
    {
        // Строка должна содержать только N.
        if (argc != 1)
        {
            feed_plan_input_fail();
            return;
        }

        uint32_t phase_count = 0U;

        if (!parse_uint32_dec_token(argv[0], &phase_count))
        {
            feed_plan_input_fail();
            return;
        }

        if ((phase_count == 0U) || (phase_count > FEED_PLAN_MAX_PHASES))
        {
            feed_plan_input_fail();
            return;
        }

        feed_plan_input_phase_count = (uint8_t)phase_count;
        feed_plan_input_step = FEED_PLAN_INPUT_VOLTAGES;
        return;
    }

    if (feed_plan_input_step == FEED_PLAN_INPUT_VOLTAGES)
    {
        // Строка напряжений: ровно N чисел.
    	if (!parse_feed_plan_voltage_line(argv,
    	                                  argc,
    	                                  feed_plan_input_phase_count,
    	                                  feed_plan_input_voltage)){
            feed_plan_input_fail();
            return;
        }

        feed_plan_input_step = FEED_PLAN_INPUT_CURRENTS;
        return;
    }

    if (feed_plan_input_step == FEED_PLAN_INPUT_CURRENTS)
    {
        // Строка токов: ровно N чисел.
    	if (!parse_feed_plan_current_line(argv,
    	                                  argc,
    	                                  feed_plan_input_phase_count,
    	                                  feed_plan_input_current))
        {
            feed_plan_input_fail();
            return;
        }

        feed_plan_input_step = FEED_PLAN_INPUT_DURATION;
        return;
    }

    if (feed_plan_input_step == FEED_PLAN_INPUT_DURATION)
    {
        // Строка активной длительности фазы питания: одно число, мкс.
        // Это именно полезное время питания, без времени перестройки источника.
        if (argc != 1)
        {
            feed_plan_input_fail();
            return;
        }

        if (!parse_uint32_dec_token(argv[0], &feed_plan_input_duration_us))
        {
            feed_plan_input_fail();
            return;
        }

        if (feed_plan_input_duration_us == 0U)
        {
            feed_plan_input_fail();
            return;
        }

        // Следующая строка должна содержать дополнительное время
        // на перестройку источника питания.
        feed_plan_input_step = FEED_PLAN_INPUT_EXTRA_DURATION;
        return;
    }

    if (feed_plan_input_step == FEED_PLAN_INPUT_EXTRA_DURATION)
    {
        // Строка дополнительного времени: одно число, мкс.
        // Нулевое значение допускается, если дополнительная задержка не нужна.
        if (argc != 1)
        {
            feed_plan_input_fail();
            return;
        }

        if (!parse_uint32_dec_token(argv[0], &feed_plan_input_extra_duration_us))
        {
            feed_plan_input_fail();
            return;
        }

        // Проверяем, что сумма активного времени и дополнительного времени
        // помещается в uint32_t. Именно эта сумма будет передана ведомому.
        if (feed_plan_input_extra_duration_us >
            (0xFFFFFFFFUL - feed_plan_input_duration_us))
        {
            feed_plan_input_fail();
            return;
        }

        feed_plan_input_mask_index = 0U;
        feed_plan_input_step = FEED_PLAN_INPUT_MASKS;
        return;
    }

    if (feed_plan_input_step == FEED_PLAN_INPUT_MASKS)
    {
        // Каждая строка маски должна быть одним токеном,
        // например 100, 010, 101.
        if (argc != 1)
        {
            feed_plan_input_fail();
            return;
        }

        if (feed_plan_input_mask_index >= found_rom_count)
        {
            feed_plan_input_fail();
            return;
        }

        if (!parse_feed_plan_mask_line(argv[0],
                                       feed_plan_input_phase_count,
                                       &feed_plan_input_masks[feed_plan_input_mask_index]))
        {
            feed_plan_input_fail();
            return;
        }

        feed_plan_input_mask_index++;

        // Если приняли маски для всех найденных ведомых,
        // сохраняем готовый план в protocol.c.
        if (feed_plan_input_mask_index >= found_rom_count)
        {
            if (!feed_plan_store(feed_plan_input_phase_count,
                                 feed_plan_input_voltage,
                                 feed_plan_input_current,
                                 feed_plan_input_duration_us,
                                 feed_plan_input_extra_duration_us,
                                 feed_plan_input_masks,
                                 found_rom_count))
            {
                feed_plan_input_fail();
                return;
            }

            /*
             * План успешно сохранён в памяти мастера.
             * Сразу запускаем передачу индивидуальных масок ведомым устройствам.
             */
            if (!feed_plan_send_all_async())
            {
                feed_plan_reset();
                feed_plan_input_fail();
                return;
            }

            feed_plan_input_reset();
            comport_send_response("FEED PLAN saved\r\n");
            return;
        }

        return;
    }

    // Защита от неизвестного состояния.
    feed_plan_input_fail();
}

void comport_print_received_data(void)
{
    comport_send_response("Received bytes: %u\r\n", received_data_count);

    // Печатаем все байты, которые мастер принял от ведомого устройства.
    for (uint8_t i = 0; i < received_data_count; i++)
    {
        comport_send_response("%02X ", received_data[i]);
    }

    comport_send_response("\r\n");
}

// ========== Разбор команд ручного тестирования PPE-3323 ==========
static void ppe_process_input_command(char **argv, int argc)
{
    uint32_t voltage_v = 0U;
    uint32_t current_ma = 0U;

    if ((argv == NULL) || (argc < 2))
    {
        comport_send_response("Error: PPE command is incomplete\r\n");
        return;
    }

    /*
     * Нельзя подготовить новую команду, пока предыдущая
     * ещё не обработана main.c.
     */
    if ((ppe_command_ready != 0U) ||
        (comport_command_ready != 0U))
    {
        comport_send_response("Error: previous command is not processed yet\r\n");
        return;
    }

    if (str_eq_ci(argv[1], "SET"))
    {
        /*
         * Формат:
         *
         * PPE SET <напряжение_В> <ток_мА>
         *
         * Пример:
         * PPE SET 12 100
         */
        if (argc != 4)
        {
            comport_send_response("Error: use PPE SET <V> <mA>\r\n");
            return;
        }

        if (!parse_uint32_dec_token(argv[2], &voltage_v) ||
            !parse_uint32_dec_token(argv[3], &current_ma))
        {
            comport_send_response("Error: PPE values must be decimal\r\n");
            return;
        }

        if ((voltage_v > POWER_SOURCE_MAX_VOLTAGE_V) ||
            (current_ma > POWER_SOURCE_MAX_CURRENT_MA))
        {
            comport_send_response("Error: PPE value is out of range\r\n");
            return;
        }

        ppe_command_type = PPE_COMMAND_SET;
        ppe_command_voltage_v = (uint8_t)voltage_v;
        ppe_command_current_ma = (uint16_t)current_ma;

        ppe_command_ready = 1U;
        return;
    }

    if (str_eq_ci(argv[1], "ON"))
    {
        if (argc != 2)
        {
            comport_send_response("Error: use PPE ON\r\n");
            return;
        }

        ppe_command_type = PPE_COMMAND_ON;
        ppe_command_ready = 1U;
        return;
    }

    if (str_eq_ci(argv[1], "OFF"))
    {
        if (argc != 2)
        {
            comport_send_response("Error: use PPE OFF\r\n");
            return;
        }

        ppe_command_type = PPE_COMMAND_OFF;
        ppe_command_ready = 1U;
        return;
    }

    if (str_eq_ci(argv[1], "STATUS"))
    {
        if (argc != 2)
        {
            comport_send_response("Error: use PPE STATUS\r\n");
            return;
        }

        ppe_command_type = PPE_COMMAND_STATUS;
        ppe_command_ready = 1U;
        return;
    }

    comport_send_response("Error: unknown PPE command\r\n");
}

// ============================================================================
// Внутренние функции парсера
// ============================================================================

static bool str_eq_ci(const char *a, const char *b)
{
    // Сравнение строк без учёта регистра.
    // Возвращает true, если строки одинаковые.
    while (*a && *b)
    {
        char ca = (char)toupper((unsigned char)*a);
        char cb = (char)toupper((unsigned char)*b);

        if (ca != cb)
        {
            return false;
        }

        a++;
        b++;
    }

    return (*a == '\0') && (*b == '\0');
}


static bool token_has_hex_letters(const char *s)
{
    // Проверяем наличие символов A..F.
    // Если они есть, число точно надо разбирать как HEX.
    while (*s)
    {
        char c = *s;

        if ((c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))
        {
            return true;
        }

        s++;
    }

    return false;
}


static bool token_is_hex_string(const char *s)
{
    // Проверяем, состоит ли строка только из HEX-символов.
    // Используется для адреса вида A1101234.
    if (s == NULL || *s == '\0')
    {
        return false;
    }

    while (*s)
    {
        if (!isxdigit((unsigned char)*s))
        {
            return false;
        }

        s++;
    }

    return true;
}


static bool parse_size_token(const char *token, uint8_t *value)
{
    // Размер данных удобнее воспринимать как десятичное число.
    // Например:
    //   4    -> 4 байта
    //   10   -> 10 байтов
    //   0x10 -> 16 байтов
    if (token == NULL || value == NULL)
    {
        return false;
    }

    char *endptr = NULL;
    int base = 10;

    if ((strlen(token) > 2U) &&
        (token[0] == '0') &&
        (token[1] == 'x' || token[1] == 'X'))
    {
        base = 16;
    }

    unsigned long parsed = strtoul(token, &endptr, base);

    if (endptr == token || *endptr != '\0')
    {
        return false;
    }

    if (parsed > 0xFFU)
    {
        return false;
    }

    *value = (uint8_t)parsed;
    return true;
}


static bool parse_byte_token(const char *token, uint8_t *value)
{
    // Разбор байта протокола.
    //
    // Логика:
    // 1. Если есть префикс 0x, число HEX.
    // 2. Если есть буквы A..F, число HEX.
    // 3. Если длина 1 или 2 символа, считаем это HEX-байтом.
    // 4. Если длина больше 2 и нет HEX-букв, считаем число десятичным.
    //
    // Примеры:
    //   A1   -> 0xA1
    //   10   -> 0x10
    //   0x10 -> 0x10
    //   204  -> 204 decimal -> 0xCC
    if (token == NULL || value == NULL)
    {
        return false;
    }

    char *endptr = NULL;
    int base = 10;
    size_t len = strlen(token);

    if ((len > 2U) &&
        (token[0] == '0') &&
        (token[1] == 'x' || token[1] == 'X'))
    {
        base = 16;
    }
    else if (token_has_hex_letters(token))
    {
        base = 16;
    }
    else if (len <= 2U)
    {
        base = 16;
    }
    else
    {
        base = 10;
    }

    unsigned long parsed = strtoul(token, &endptr, base);

    if (endptr == token || *endptr != '\0')
    {
        return false;
    }

    if (parsed > 0xFFU)
    {
        return false;
    }

    *value = (uint8_t)parsed;
    return true;
}


static bool parse_packet_type_token(const char *token, uint8_t *type_out)
{
    // Разбор типа пакета.
    // Поддерживаются как текстовые, так и числовые обозначения.

    if (str_eq_ci(token, "0") || str_eq_ci(token, "TX") || str_eq_ci(token, "SEND"))
    {
        *type_out = PACKET_TYPE_TX;
        return true;
    }

    if (str_eq_ci(token, "1") ||
        str_eq_ci(token, "RX") ||
        str_eq_ci(token, "RECV") ||
        str_eq_ci(token, "READ"))
    {
        *type_out = PACKET_TYPE_RX;
        return true;
    }

    if (str_eq_ci(token, "2") || str_eq_ci(token, "SEARCH"))
    {
        *type_out = PACKET_TYPE_SEARCH;
        return true;
    }

    return false;
}


static bool parse_rom_token(const char *token, uint8_t *rom_out)
{
    // Разбор ROM-команды.
    // Пользователь может вводить либо имя, либо код команды.

    if (str_eq_ci(token, "SKIP"))
    {
        *rom_out = ROM_SKIP_CMD;
        return true;
    }

    if (str_eq_ci(token, "SEARCH"))
    {
        *rom_out = ROM_SEARCH_CMD;
        return true;
    }

    if (str_eq_ci(token, "READ"))
    {
        *rom_out = ROM_READ_DATA_CMD;
        return true;
    }

    if (str_eq_ci(token, "MATCH"))
    {
        *rom_out = ROM_MATCH_CMD;
        return true;
    }

    if (str_eq_ci(token, "ALARM"))
    {
        *rom_out = ROM_ALARM_CMD;
        return true;
    }

    if (str_eq_ci(token, "RESUME"))
    {
        *rom_out = ROM_RESUME_CMD;
        return true;
    }

    // Если это не имя, пробуем разобрать как байт.
    return parse_byte_token(token, rom_out);
}


static bool parse_func_token(const char *token, uint8_t *func_out)
{
    // Разбор функциональной команды.
    // Пользователь может вводить либо имя, либо числовой код.

    if (str_eq_ci(token, "OFF"))
    {
        *func_out = FUNC_OFF;
        return true;
    }

    if (str_eq_ci(token, "ON"))
    {
        *func_out = FUNC_ON;
        return true;
    }

    if (str_eq_ci(token, "PARAMETERS") || str_eq_ci(token, "PARAM"))
    {
        *func_out = PARAMETERS;
        return true;
    }

    if (str_eq_ci(token, "NEW_DEVICE") || str_eq_ci(token, "NEW"))
    {
        *func_out = NEW_DEVICE;
        return true;
    }

    if (str_eq_ci(token, "FEED_PLAN") || str_eq_ci(token, "FEED"))
    {
        *func_out = FEED_PLAN;
        return true;
    }

    // Если это не имя, пробуем разобрать как байт.
    return parse_byte_token(token, func_out);
}


static bool rom_requires_address(uint8_t rom)
{
    // Сейчас адрес требуется только для MATCH ROM.
    // Если позже добавишь другие ROM-команды с адресом,
    // их нужно будет добавить сюда.
    return (rom == ROM_MATCH_CMD);
}


static bool parse_address_tokens(char **argv, int argc, int *index)
{
    if (argv == NULL || index == NULL)
    {
        return false;
    }

    if (*index >= argc)
    {
        return false;
    }

    // ---------------------------------------------------------------------
    // Вариант 1: адрес одним 32-битным HEX-числом.
    //
    // Примеры:
    //   A1101234
    //   0xA1101234
    //
    // Такой адрес раскладывается в address_arr[0..3] по старшему байту вперёд:
    //   A1 10 12 34
    // ---------------------------------------------------------------------

    const char *token = argv[*index];
    const char *hex_start = token;
    size_t len = strlen(token);

    if ((len > 2U) &&
        (token[0] == '0') &&
        (token[1] == 'x' || token[1] == 'X'))
    {
        hex_start = token + 2;
        len -= 2U;
    }

    if ((len == 8U) && token_is_hex_string(hex_start))
    {
        uint32_t packed = (uint32_t)strtoul(hex_start, NULL, 16);

        address_arr[0] = (uint8_t)((packed >> 24) & 0xFFU);
        address_arr[1] = (uint8_t)((packed >> 16) & 0xFFU);
        address_arr[2] = (uint8_t)((packed >> 8) & 0xFFU);
        address_arr[3] = (uint8_t)(packed & 0xFFU);

        (*index)++;
        return true;
    }

    // ---------------------------------------------------------------------
    // Вариант 2: адрес четырьмя отдельными байтами.
    //
    // Пример:
    //   A1 10 12 34
    // ---------------------------------------------------------------------

    if ((*index + ROM_ID_LEN) > argc)
    {
        return false;
    }

    for (uint8_t i = 0; i < ROM_ID_LEN; i++)
    {
        if (!parse_byte_token(argv[*index], &address_arr[i]))
        {
            return false;
        }

        (*index)++;
    }

    return true;
}


static void print_help(void)
{
    comport_send_response("Commands:\r\n");
    comport_send_response("  PACK TX <rom> <size> [addr] <func> [data...]\r\n");
    comport_send_response("  PACK RX MATCH <size> <addr> <func>\r\n");
    comport_send_response("  PACK SEARCH\r\n");
    comport_send_response("  STATUS\r\n");
    comport_send_response("  FOUND\r\n");
    comport_send_response("  RXDATA\r\n");
    comport_send_response("  PPE SET <V> <mA>\r\n");
    comport_send_response("  PPE ON\r\n");
    comport_send_response("  PPE OFF\r\n");
    comport_send_response("  PPE STATUS\r\n");
    comport_send_response("\r\n");

    comport_send_response("Examples:\r\n");
    comport_send_response("  PACK TX SKIP 0 OFF\r\n");
    comport_send_response("  PACK TX MATCH 0 A1 10 12 34 OFF\r\n");
    comport_send_response("  PACK TX MATCH 0 A1101234 OFF\r\n");
    comport_send_response("  PACK TX SKIP 4 ON 11 22 33 44\r\n");
    comport_send_response("  PACK RX MATCH 10 A1 10 12 34 PARAMETERS\r\n");
    comport_send_response("  PACK SEARCH\r\n");

    comport_send_response("\r\nFeed plan, one-line format:\r\n");
    comport_send_response(
        "  PLAN N U1 ... UN I1 ... IN T_ACTIVE_US T_EXTRA_US "
        "MASK_ROM0 ... MASK_ROMn\r\n");

    comport_send_response(
        "  Example for one device and two phases:\r\n");

    comport_send_response(
        "  PLAN 2 5 8 10 100 50000 250000 10\r\n");

    comport_send_response(
        "  Number of masks must equal Found ROM count.\r\n");
    comport_send_response("  U values: volts, 0...32\r\n");
    comport_send_response("  I values: milliamperes, 0...3000\r\n");


}
