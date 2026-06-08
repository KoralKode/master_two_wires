/*
 * protocol.h
 *
 *  Created on: May 1, 2026
 *      Author: stepa
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// Таймер создаётся CubeMX в main.c, а используется в protocol.c
extern TIM_HandleTypeDef htim2;

// ===== Параметры протокола =====
#define TIMESLOT_US     70U
#define CRC5_POLY       0x05U
#define ROM_ID_LEN      4U
#define POWER_PHASE_US  10000U

#define MAX_FOUND_DEVICES          16U
#define SEARCH_GROUPS_PER_SEGMENT  5U
#define SEARCH_TOTAL_BITS          (ROM_ID_LEN * 8U)

#define MAX_RECEIVED_DATA_SIZE     64U

// ===== ROM-команды =====
#define ROM_SKIP_CMD        0xCCU
#define ROM_SEARCH_CMD      0xF0U
#define ROM_READ_DATA_CMD 0x33
#define ROM_MATCH_CMD       0x55U
#define ROM_ALARM_CMD       0xECU
#define ROM_RESUME_CMD      0x69U
#define ROM_SEARCH_INTERRUPT_CMD 0xE1U // Поиск ведомых устройств, у которых установлен флаг прерывания

// ===== Функциональные команды =====
#define FUNC_OFF    0x01U
#define FUNC_ON     0x02U
#define PARAMETERS  0x03U
#define NEW_DEVICE  0x04U
#define FEED_PLAN   0x05U

// ===== Результаты ROM-search =====
extern uint8_t found_roms[MAX_FOUND_DEVICES][ROM_ID_LEN];
extern uint8_t found_rom_count;

// ===== Результаты поиска устройств с флагом прерывания =====
// Этот массив заполняется командой ROM_SEARCH_INTERRUPT_CMD.
// Он не должен смешиваться с основным массивом found_roms.
extern uint8_t interrupt_roms[MAX_FOUND_DEVICES][ROM_ID_LEN];
extern uint8_t interrupt_rom_count;

// ===== Результаты будущего поиска устройств с флагом аварии =====
// Сейчас обработка alarm ещё не реализована, но массив сразу добавляется,
// чтобы логика хранения результатов была разделена.
extern uint8_t alarm_roms[MAX_FOUND_DEVICES][ROM_ID_LEN];
extern uint8_t alarm_rom_count;

// ===== Результаты приёма данных мастером =====
extern uint8_t received_data[MAX_RECEIVED_DATA_SIZE];
extern uint8_t received_data_count;

// ===== События протокола для вывода пользователю =====
#define PROTOCOL_EVENT_NONE              0U
#define PROTOCOL_EVENT_NEW_DEVICES        1U
#define PROTOCOL_EVENT_INTERRUPT_DEVICES  2U
#define PROTOCOL_EVENT_ALARM_DEVICES       3U

extern volatile uint8_t protocol_event_type;

// Устанавливается, если текущий пользовательский пакет был прерван
// из-за принятого бита alarm.
extern volatile uint8_t protocol_packet_aborted_by_alarm;

void protocol_poll(void);
void protocol_clear_event(void);
// Сброс признака аварийного завершения пользовательского пакета.
// Вызывается из main.c после вывода сообщения пользователю.
void protocol_clear_packet_abort_flag(void);
bool search_interrupt_packet_async(void);
// Запуск поиска ведомых устройств, у которых установлен флаг alarm.
// Результаты поиска записываются в alarm_roms[].
bool search_alarm_packet_async(void);
// ===== Публичные функции библиотеки =====
void protocol_init(void);
bool protocol_is_busy(void);

// Эту функцию вызывает HAL_TIM_PeriodElapsedCallback() из main.c
void protocol_timer_irq_handler(TIM_HandleTypeDef *htim);

bool send_pack_async(uint8_t rom_cmd,
                     uint8_t func_cmd,
                     uint8_t datasize,
                     uint8_t *data,
                     uint8_t *slave_id);

bool receive_pack_async(uint8_t func_cmd,
                        uint8_t receive_size,
                        uint8_t *slave_id);

bool receive_pack_skip_async(uint8_t func_cmd, uint8_t receive_size);

void clear_received_data(void);

void search_rom_packet_reset(void);
bool search_rom_packet_async(void);
bool search_rom_packet_blocking(void);

uint8_t crc5(uint8_t data);
bool check_crc5(uint8_t data, uint8_t received_crc);

#endif /* INC_PROTOCOL_H_ */
