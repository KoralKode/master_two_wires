/*
 * comport.h
 *
 *  Created on: May 2, 2026
 *      Author: stepa
 */

#ifndef INC_COMPORT_H_
#define INC_COMPORT_H_

#include "protocol.h"
#include <stdint.h>
#include <stdbool.h>

// Максимальная длина одной строки команды из COM-порта
#define COMPORT_LINE_MAX     1024U

// Максимальное количество токенов в одной строке
#define COMPORT_MAX_TOKENS   96U

// Типы пакетов
#define PACKET_TYPE_TX       0U   // пакет передачи данных мастером
#define PACKET_TYPE_RX       1U   // пакет приёма данных мастером
#define PACKET_TYPE_SEARCH   2U   // пакет поиска устройств

// ===== Команды ручного тестирования PPE-3323 =====
#define PPE_COMMAND_NONE     0U
#define PPE_COMMAND_SET      1U
#define PPE_COMMAND_ON       2U
#define PPE_COMMAND_OFF      3U
#define PPE_COMMAND_STATUS   4U

// ===== Данные команды, подготовленные парсером COM-порта =====

extern uint8_t rom_cmd;
extern uint8_t data_size;
extern uint8_t func_cmd;

// 0 – передача, 1 – приём, 2 – поиск устройств
extern uint8_t packet_type;

extern uint8_t address_arr[ROM_ID_LEN];
extern uint8_t data_arr[MAX_RECEIVED_DATA_SIZE];

// Флаг готового пакета, разобранного из COM-порта.
// Название оставлено comport_command_ready, чтобы не менять уже написанный код.
extern volatile uint8_t comport_command_ready;

/*
 * Команда PPE-3323, подготовленная парсером COM-порта.
 *
 * Она не является пакетом двухпроводного протокола.
 * main.c передаёт её драйверу ppe3323.c отдельно.
 */
extern volatile uint8_t ppe_command_ready;
extern uint8_t ppe_command_type;
extern uint8_t ppe_command_voltage_v;
extern uint16_t ppe_command_current_ma;

// ===== Функции COM-порта =====

// Проверить очередь USB CDC и обработать все введённые строки
void comport_poll(void);

// Обработать одну строку команды
void comport_process_line(char *line);

// Сбросить флаг готовой команды
void comport_clear_command(void);


// Сбросить подготовленную команду ручного управления PPE-3323.
void comport_clear_ppe_command(void);
// Вывод ответа в COM-порт
void comport_send_response(const char *fmt, ...);

// Служебный вывод результатов
void comport_print_found_roms(void);
// Вывод параметров питания всех найденных ведомых устройств.
// Для каждого адреса выводится 5 строк:
// адрес, напряжение, ток, время заряда, время работы.
void comport_print_power_params(void);
void comport_print_received_data(void);

#endif /* INC_COMPORT_H_ */
