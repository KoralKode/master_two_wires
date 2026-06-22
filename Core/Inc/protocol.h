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

// Данные FEED_PLAN:
//   1 байт  – количество фаз питания;
//   4 байта – суммарная длительность фазы питания, мкс;
//   далее   – битовая маска фаз конкретного ведомого.

// Таймер создаётся CubeMX в main.c, а используется в protocol.c
extern TIM_HandleTypeDef htim2;

// ===== Параметры протокола =====
#define TIMESLOT_US     70U
#define CRC5_POLY       0x05U
#define ROM_ID_LEN      4U
/*
 * Короткая имитационная фаза питания.
 *
 * Используется до первого успешного поиска устройств и во время
 * передачи пакетов FEED_PLAN. Она разделяет сегменты протокола,
 * но не подключает к линии реальный источник питания.
 */
#define SIMULATED_POWER_PHASE_US  10000U

#define MAX_FOUND_DEVICES          16U
#define SEARCH_GROUPS_PER_SEGMENT  5U
#define SEARCH_TOTAL_BITS          (ROM_ID_LEN * 8U)

#define MAX_RECEIVED_DATA_SIZE     64U
// ===== Параметры питания ведомого устройства =====
// Ведомое устройство передаёт параметры командой PARAMETERS.
// Формат ответа занимает 10 байт:
//   1 байт  – напряжение;
//   1 байт  – ток;
//   4 байта – время заряда внутреннего накопителя, мкс;
//   4 байта – время работы от внутреннего накопителя, мкс.
#define POWER_PARAMS_DATA_SIZE     10U

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

// ===== Результаты поиска устройств с флагом аварии =====
// Этот массив заполняется командой ROM_ALARM_CMD.
// Основной массив found_roms[] при этом не изменяется.
extern uint8_t alarm_roms[MAX_FOUND_DEVICES][ROM_ID_LEN];
extern uint8_t alarm_rom_count;

// ===== Хранимые параметры питания ведомого устройства =====
// Структура связана с found_roms[] по индексу:
// если адрес ведомого находится в found_roms[i],
// то его параметры питания находятся в power_params[i].
typedef struct
{
    // 1 – параметры для этой ячейки уже успешно приняты.
    // 0 – параметров ещё нет или они были сброшены.
    uint8_t valid;

    // Копия ROM-адреса ведомого устройства.
    // Она нужна, чтобы проверить, что параметры относятся
    // именно к адресу found_roms[i].
    uint8_t rom[ROM_ID_LEN];

    // Напряжение питания.
    // По линии передаётся 1 байт, но в мастере хранится как uint32_t
    // для дальнейших расчётов и унификации с остальными параметрами.
    uint32_t voltage;

    // Ток питания.
    // По линии передаётся 1 байт, но в мастере хранится как uint32_t.
    uint32_t current;

    // Время заряда внутреннего накопителя ведомого устройства, мкс.
    // По линии передаётся 4 байтами.
    uint32_t charge_time_us;

    // Время работы от внутреннего накопителя, мкс.
    // По линии передаётся 4 байтами.
    uint32_t work_time_us;
} power_params_t;


// Массив параметров питания.
// Индекс совпадает с индексом адреса в found_roms[].
extern power_params_t power_params[MAX_FOUND_DEVICES];

// Количество ячеек power_params[], в которых valid == 1.
extern uint8_t power_params_count;

// ===== Результаты приёма данных мастером =====
extern uint8_t received_data[MAX_RECEIVED_DATA_SIZE];
extern uint8_t received_data_count;

// ===== События протокола для вывода пользователю =====
#define PROTOCOL_EVENT_NONE              0U
#define PROTOCOL_EVENT_NEW_DEVICES        1U
#define PROTOCOL_EVENT_INTERRUPT_DEVICES  2U
#define PROTOCOL_EVENT_ALARM_DEVICES       3U
// Параметры питания от найденных ведомых устройств приняты
// и готовы к выводу на ПК.
#define PROTOCOL_EVENT_POWER_PARAMS_READY  4U
// Индивидуальные маски плана питания переданы всем найденным ведомым.
#define PROTOCOL_EVENT_FEED_PLAN_SENT      5U

// ===== Ограничения плана питания =====
// Маска фаз хранится в uint32_t, поэтому максимум – 32 фазы.
#define FEED_PLAN_MAX_PHASES       32U

/*
 * Ограничения первого регулируемого выхода PPE-3323.
 *
 * План питания на данном этапе управляет только OUT1:
 * напряжение от 0 до 32 В;
 * ограничение тока от 0 до 3000 мА.
 */
#define POWER_SOURCE_MAX_VOLTAGE_V    32U
#define POWER_SOURCE_MAX_CURRENT_MA   3000U

// ===== План питания, полученный от ПК =====
// План принимается от ПК, сохраняется в памяти мастера,
// а затем используется для передачи индивидуальных масок ведомым.
typedef struct
{
    // 1 – план питания успешно принят от ПК.
    // 0 – план отсутствует или был сброшен.
    uint8_t valid;

    // Количество фаз питания в цикле.
    uint8_t phase_count;

    // Напряжение каждой фазы.
    // Пользователь вводит значения без единиц измерения.
    uint8_t phase_voltage[FEED_PLAN_MAX_PHASES];

    /*
     * Ток каждой фазы, мА.
     *
     * uint16_t используется потому, что PPE-3323 позволяет задавать
     * ток до 3000 мА, а uint8_t ограничен значением 255.
     */
    uint16_t phase_current[FEED_PLAN_MAX_PHASES];

    // Полезная длительность активного питания одной фазы, мкс.
    // Это время вводится с ПК как основное время питания.
    uint32_t phase_duration_us;

    // Дополнительное время на перестройку источника питания, мкс.
    // Это время также вводится с ПК и одинаково добавляется ко всем фазам.
    uint32_t phase_extra_duration_us;

    // Количество ведомых устройств, для которых приняты маски.
    // Обычно равно found_rom_count на момент ввода плана.
    uint8_t device_count;

    // Маска фаз для каждого ведомого устройства.
    // Индекс соответствует порядку адресов в found_roms[].
    //
    // Для N = 3:
    // строка "100" -> бит 0 = 1;
    // строка "010" -> бит 1 = 1;
    // строка "101" -> биты 0 и 2 = 1.
    uint32_t device_phase_mask[MAX_FOUND_DEVICES];

} feed_plan_data_t;


// План питания, сохранённый в микроконтроллере.
extern feed_plan_data_t feed_plan_data;

// ===== Признаки готовности к реальной фазе питания =====
// Возвращает 1, если в актуальном списке найден хотя бы один ведомый.
// Признак сбрасывается при полном сбросе или новом полном поиске.
bool protocol_search_found_once(void);

// Возвращает 1, если индивидуальные пакеты FEED_PLAN успешно
// переданы всем ведомым устройствам из актуального списка.
bool protocol_feed_plan_sent_ok(void);

// Возвращает 1, если одновременно выполнены оба условия:
// найдено хотя бы одно устройство и FEED_PLAN передан всем ведомым.

bool protocol_real_power_phase_enabled(void);

// Сброс сохранённого плана питания.
void feed_plan_reset(void);


// Запись нового плана питания в переменные микроконтроллера.
// Функция вызывается из comport.c после успешного разбора всех строк.
bool feed_plan_store(uint8_t phase_count,
                     const uint8_t *phase_voltage,
                     const uint16_t *phase_current,
                     uint32_t phase_duration_us,
                     uint32_t phase_extra_duration_us,
                     const uint32_t *device_masks,
                     uint8_t device_count);
// Запуск передачи индивидуальных масок плана питания всем ведомым.
// Для каждого адреса из found_roms[] будет отправлен пакет:
// ROM_MATCH -> адрес -> размер -> FEED_PLAN -> данные.
// Данные:
//   1 байт  – количество фаз питания;
//   4 байта – полная длительность фазы питания, мкс;
//   далее   – битовая маска фаз конкретного ведомого.
bool feed_plan_send_all_async(void);

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
// Запуск последовательного запроса параметров питания
// у всех найденных ведомых устройств из found_roms[].
//
// Функция запускает внутреннюю обработку:
// для каждого адреса будет отправлен RX-пакет MATCH + PARAMETERS,
// затем принятые 10 байт будут сохранены в power_params[].
bool power_params_request_all_async(void);

// Полный сброс массива параметров питания.
void power_params_reset(void);
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
