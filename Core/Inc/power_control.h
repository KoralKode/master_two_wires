#ifndef INC_POWER_CONTROL_H_
#define INC_POWER_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>


/*
 * Инициализация автомата управления источником PPE-3323.
 *
 * Вызывается после ppe3323_init().
 * После запуска будет поставлена в очередь команда OUT0.
 */
void power_control_init(void);


/*
 * Фоновая обработка источника питания.
 *
 * Вызывается из главного цикла main().
 * Здесь формируются и запускаются UART-команды PPE-3323.
 */
void power_control_poll(void);


/*
 * Разрешение автоматического управления источником.
 *
 * Вызывается после успешной передачи FEED_PLAN всем ведомым.
 */
void power_control_arm_new_plan(void);


/*
 * Запрет автоматического питания и запрос выключения OUT1.
 *
 * Функция не использует UART и может вызываться из прерывания TIM2.
 * Сама команда OUT0 будет передана позже из power_control_poll().
 */
void power_control_disable(void);


/*
 * Запрос параметров очередной фазы питания.
 *
 * Вызывается из protocol.c в начале дополнительного времени фазы.
 * Функция только сохраняет напряжение и ток; UART-команды здесь
 * не передаются.
 */
void power_control_request_phase(uint8_t voltage_v,
                                 uint16_t current_ma);


/*
 * Возвращает 1, если:
 * - VSET1 и ISET1 текущей фазы переданы;
 * - OUT1 передан;
 * - нет ошибки UART.
 *
 * Именно этот признак проверяется перед включением PB1.
 */
bool power_control_is_phase_ready(void);


/*
 * Вызывается из protocol.c, если дополнительное время завершилось,
 * но источник ещё не был готов к включению PB1.
 *
 * Ключ остаётся выключенным, автоматическое питание блокируется,
 * а в main() ставится запрос на OUT0.
 */
void power_control_mark_phase_not_ready(void);


/*
 * Возвращает 1, если автоматическое питание разрешено.
 */
bool power_control_is_automatic_mode(void);


/*
 * Возвращает и сбрасывает признак ошибки источника.
 *
 * Используется main.c, чтобы один раз вывести сообщение через COM-порт.
 */
bool power_control_take_fault_report(void);

#endif /* INC_POWER_CONTROL_H_ */
