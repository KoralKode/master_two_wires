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

// ===== Функциональные команды =====
#define FUNC_OFF    0x01U
#define FUNC_ON     0x02U
#define PARAMETERS  0x03U
#define NEW_DEVICE  0x04U
#define FEED_PLAN   0x05U

// ===== Результаты ROM-search =====
extern uint8_t found_roms[MAX_FOUND_DEVICES][ROM_ID_LEN];
extern uint8_t found_rom_count;

// ===== Результаты приёма данных мастером =====
extern uint8_t received_data[MAX_RECEIVED_DATA_SIZE];
extern uint8_t received_data_count;

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

void clear_received_data(void);

void search_rom_packet_reset(void);
bool search_rom_packet_async(void);
bool search_rom_packet_blocking(void);

uint8_t crc5(uint8_t data);
bool check_crc5(uint8_t data, uint8_t received_crc);

#endif /* INC_PROTOCOL_H_ */
