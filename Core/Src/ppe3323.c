#include "ppe3323.h"
#include "protocol.h"

#include <stdio.h>
#include <string.h>


/*
 * Максимальная длина одной группы команд PPE-3323.
 *
 * Наиболее длинная используемая группа:
 * OCP1;VSET1 32.00;ISET1 3.000\r
 *
 * Буфер 64 байта содержит достаточный запас.
 */
#define PPE3323_TX_BUFFER_SIZE  64U


typedef enum
{
    PPE3323_COMMAND_NONE = 0U,
    PPE3323_COMMAND_SET_PHASE,
    PPE3323_COMMAND_OUTPUT_ON,
    PPE3323_COMMAND_OUTPUT_OFF
} ppe3323_command_t;


/*
 * UART USART1, подключённый к MAX3232.
 */
static UART_HandleTypeDef *ppe3323_uart = NULL;


/*
 * Буфер команды, ожидающей передачи.
 */
static uint8_t ppe3323_tx_buffer[PPE3323_TX_BUFFER_SIZE] = {0U};
static uint8_t ppe3323_tx_length = 0U;


/*
 * Признаки неблокирующей передачи.
 *
 * tx_pending = 1:
 * команда подготовлена, но HAL_UART_Transmit_IT() ещё не вызван.
 *
 * tx_active = 1:
 * передача уже выполняется UART-периферией.
 */
static volatile uint8_t ppe3323_tx_pending = 0U;
static volatile uint8_t ppe3323_tx_active = 0U;


/*
 * Тип команды нужен, чтобы после успешной передачи настройки фазы
 * разрешить отдельную команду OUT1.
 */
static ppe3323_command_t ppe3323_pending_command = PPE3323_COMMAND_NONE;
static volatile ppe3323_command_t ppe3323_active_command =
        PPE3323_COMMAND_NONE;


/*
 * Признак успешной передачи VSET1 и ISET1.
 *
 * Он не означает, что напряжение уже физически установилось.
 * За физическую перестройку позже будет отвечать
 * phase_extra_duration_us.
 */
static volatile uint8_t ppe3323_phase_configuration_valid = 0U;

/*
 * Признак того, что OUT1 был успешно передан по UART.
 *
 * Он используется автоматом питания, чтобы не повторять OUT1
 * перед каждой фазой.
 */
static volatile uint8_t ppe3323_output_on = 0U;

static volatile ppe3323_state_t ppe3323_state =
        PPE3323_STATE_NOT_INITIALIZED;


/*
 * Подготовка одной текстовой команды или группы команд.
 *
 * В PPE-3323 команды группы разделяются символом ';',
 * а вся группа завершается символом '\r'.
 */
static bool ppe3323_queue_text(const char *text,
                               ppe3323_command_t command_type)
{
    size_t length;

    if ((ppe3323_uart == NULL) || (text == NULL))
    {
        return false;
    }

    /*
     * Новый текст нельзя записывать поверх уже ожидающей
     * или выполняемой передачи.
     */
    if (ppe3323_is_busy())
    {
        return false;
    }

    length = strlen(text);

    if ((length == 0U) || (length >= PPE3323_TX_BUFFER_SIZE))
    {
        return false;
    }

    memcpy(ppe3323_tx_buffer, text, length);
    ppe3323_tx_length = (uint8_t)length;

    ppe3323_pending_command = command_type;
    ppe3323_tx_pending = 1U;
    ppe3323_state = PPE3323_STATE_QUEUED;

    return true;
}


void ppe3323_init(UART_HandleTypeDef *uart)
{
    ppe3323_uart = uart;

    memset(ppe3323_tx_buffer, 0, sizeof(ppe3323_tx_buffer));

    ppe3323_tx_length = 0U;
    ppe3323_tx_pending = 0U;
    ppe3323_tx_active = 0U;

    ppe3323_pending_command = PPE3323_COMMAND_NONE;
    ppe3323_active_command = PPE3323_COMMAND_NONE;

    ppe3323_phase_configuration_valid = 0U;

    ppe3323_output_on = 0U;

    if (ppe3323_uart == NULL)
    {
        ppe3323_state = PPE3323_STATE_ERROR;
    }
    else
    {
        ppe3323_state = PPE3323_STATE_IDLE;
    }
}


void ppe3323_poll(void)
{
    HAL_StatusTypeDef status;

    if (ppe3323_uart == NULL)
    {
        return;
    }

    /*
     * Пока нет подготовленной команды либо UART уже занят,
     * запускать ничего не нужно.
     */
    if ((ppe3323_tx_pending == 0U) ||
        (ppe3323_tx_active != 0U))
    {
        return;
    }

    /*
     * Передача запускается в режиме interrupt.
     *
     * HAL_UART_Transmit_IT() сразу возвращает управление main(),
     * поэтому программа не ждёт 100–150 мс окончания передачи команды.
     */
    status = HAL_UART_Transmit_IT(ppe3323_uart,
                                  ppe3323_tx_buffer,
                                  ppe3323_tx_length);

    if (status == HAL_OK)
    {
        ppe3323_active_command = ppe3323_pending_command;

        ppe3323_pending_command = PPE3323_COMMAND_NONE;
        ppe3323_tx_pending = 0U;
        ppe3323_tx_active = 1U;

        ppe3323_state = PPE3323_STATE_TRANSMITTING;
    }
    else if (status != HAL_BUSY)
    {
        /*
         * HAL_UART_Transmit_IT() не принял команду.
         *
         * Неизвестно, какая настройка источника является актуальной,
         * поэтому дальнейшее включение OUT1 запрещается до новой
         * успешной передачи VSET1 и ISET1.
         */
        ppe3323_tx_pending = 0U;
        ppe3323_pending_command = PPE3323_COMMAND_NONE;

        ppe3323_phase_configuration_valid = 0U;

        /*
         * После ошибки неизвестно, в каком состоянии находится OUT1.
         * Поэтому дальше считаем его выключенным.
         */
        ppe3323_output_on = 0U;

        ppe3323_state = PPE3323_STATE_ERROR;

    }
}


bool ppe3323_queue_phase(uint8_t voltage_v, uint16_t current_ma)
{
    char command[PPE3323_TX_BUFFER_SIZE];

    uint16_t current_a = (uint16_t)(current_ma / 1000U);
    uint16_t current_fraction_ma = (uint16_t)(current_ma % 1000U);

    if (voltage_v > POWER_SOURCE_MAX_VOLTAGE_V)
    {
        return false;
    }

    if (current_ma > POWER_SOURCE_MAX_CURRENT_MA)
    {
        return false;
    }

    /*
     * Пример формируемой строки:
     *
     * OCP1;VSET1 12.00;ISET1 0.100\r
     *
     * OCP1 включает защиту по току.
     * VSET1 и ISET1 относятся к первому регулируемому выходу OUT1.
     */
    (void)snprintf(command,
                   sizeof(command),
                   "OCP1;VSET1 %u.00;ISET1 %u.%03u\r",
                   (unsigned int)voltage_v,
                   (unsigned int)current_a,
                   (unsigned int)current_fraction_ma);

    /*
     * Сначала пытаемся поставить новую настройку в очередь.
     *
     * Если очередь занята, предыдущая корректная настройка источника
     * остаётся действительной. Поэтому признак configuration_valid
     * сбрасывается только после успешной постановки новой команды.
     */
    if (!ppe3323_queue_text(command, PPE3323_COMMAND_SET_PHASE))
    {
        return false;
    }

    /*
     * Новая настройка уже ожидает передачи, поэтому старую настройку
     * больше нельзя считать актуальной для следующего включения OUT1.
     */
    ppe3323_phase_configuration_valid = 0U;

    return true;
}


bool ppe3323_queue_output_on(void)
{
    /*
     * Нельзя включить источник, если после запуска программы
     * ещё не была успешно передана настройка U и I.
     */
    if (ppe3323_phase_configuration_valid == 0U)
    {
        return false;
    }

    return ppe3323_queue_text("OUT1\r",
                              PPE3323_COMMAND_OUTPUT_ON);
}


bool ppe3323_queue_output_off(void)
{
    return ppe3323_queue_text("OUT0\r",
                              PPE3323_COMMAND_OUTPUT_OFF);
}


bool ppe3323_is_busy(void)
{
    return ((ppe3323_tx_pending != 0U) ||
            (ppe3323_tx_active != 0U));
}


ppe3323_state_t ppe3323_get_state(void)
{
    return ppe3323_state;
}


const char *ppe3323_state_text(void)
{
    switch (ppe3323_state)
    {
        case PPE3323_STATE_IDLE:
            return "IDLE";

        case PPE3323_STATE_QUEUED:
            return "QUEUED";

        case PPE3323_STATE_TRANSMITTING:
            return "TRANSMITTING";

        case PPE3323_STATE_ERROR:
            return "ERROR";

        case PPE3323_STATE_NOT_INITIALIZED:
        default:
            return "NOT INITIALIZED";
    }
}


bool ppe3323_has_phase_configuration(void)
{
    return (ppe3323_phase_configuration_valid != 0U);
}

bool ppe3323_output_is_on(void)
{
    return (ppe3323_output_on != 0U);
}


bool ppe3323_has_error(void)
{
    return (ppe3323_state == PPE3323_STATE_ERROR);
}

void ppe3323_uart_tx_complete_callback(UART_HandleTypeDef *uart)
{
    if (uart != ppe3323_uart)
    {
        return;
    }

    if (ppe3323_active_command == PPE3323_COMMAND_SET_PHASE)
    {
        /*
         * VSET1 и ISET1 переданы по UART.
         *
         * Физическое установление напряжения выполняется
         * во время T_EXTRA.
         */
        ppe3323_phase_configuration_valid = 1U;
    }
    else if (ppe3323_active_command == PPE3323_COMMAND_OUTPUT_ON)
    {
        /*
         * OUT1 передан источнику.
         */
        ppe3323_output_on = 1U;
    }
    else if (ppe3323_active_command == PPE3323_COMMAND_OUTPUT_OFF)
    {
        /*
         * OUT0 передан источнику.
         */
        ppe3323_output_on = 0U;
    }

    ppe3323_active_command = PPE3323_COMMAND_NONE;
    ppe3323_tx_active = 0U;

    ppe3323_state = PPE3323_STATE_IDLE;

}


void ppe3323_uart_error_callback(UART_HandleTypeDef *uart)
{
    if (uart != ppe3323_uart)
    {
        return;
    }

    /*
     * При ошибке UART неизвестно, какую часть команды получил источник.
     * Поэтому следующая команда OUT1 запрещается до новой успешной
     * передачи параметров U и I.
     */
    ppe3323_tx_pending = 0U;
    ppe3323_tx_active = 0U;

    ppe3323_pending_command = PPE3323_COMMAND_NONE;
    ppe3323_active_command = PPE3323_COMMAND_NONE;

    ppe3323_phase_configuration_valid = 0U;

    /*
     * При ошибке UART состояние OUT1 больше нельзя считать известным.
     */
    ppe3323_output_on = 0U;

    ppe3323_state = PPE3323_STATE_ERROR;
}
