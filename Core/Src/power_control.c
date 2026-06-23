#include "power_control.h"

#include "ppe3323.h"
#include "protocol.h"


/*
 * Запрос очередной фазы записывается из TIM2 IRQ,
 * а обрабатывается из основного цикла main().
 */
static volatile uint8_t requested_voltage_v = 0U;
static volatile uint16_t requested_current_ma = 0U;
static volatile uint8_t phase_request_pending = 0U;


/*
 * Разрешено ли автоматическое управление источником.
 *
 * Признак становится равным 1 только после успешной передачи
 * индивидуального FEED_PLAN всем ведомым.
 */
static volatile uint8_t automatic_mode = 0U;


/*
 * Готовность источника к физическому подключению через PB1.
 */
static volatile uint8_t phase_ready = 0U;


/*
 * Признаки ошибки и необходимости отключить OUT1.
 */
static volatile uint8_t source_fault = 0U;
static volatile uint8_t fault_report_pending = 0U;
static volatile uint8_t shutdown_requested = 0U;

/*
 * Устанавливается из TIM2 IRQ, если закончилось T_EXTRA,
 * но источник не успел подготовиться.
 *
 * Полная обработка ошибки выполняется в power_control_poll(),
 * то есть в основном цикле, а не в прерывании.
 */
static volatile uint8_t phase_not_ready_pending = 0U;

/*
 * OUT0 уже поставлен в очередь и ожидается его завершение.
 */
static uint8_t shutdown_command_queued = 0U;


/*
 * Параметры фазы, которая сейчас подготавливается.
 */
static uint8_t target_voltage_v = 0U;
static uint16_t target_current_ma = 0U;
static uint8_t target_valid = 0U;


/*
 * Ожидается завершение передачи VSET1/ISET1.
 */
static uint8_t phase_configuration_in_progress = 0U;


/*
 * Последняя успешно переданная настройка источника.
 *
 * Она используется, чтобы не передавать VSET1/ISET1 повторно,
 * если соседние фазы имеют одинаковые U и I.
 */
static uint8_t last_voltage_v = 0U;
static uint16_t last_current_ma = 0U;
static uint8_t last_configuration_valid = 0U;


/*
 * Безопасное считывание запроса фазы.
 *
 * Вызывается только из главного цикла main().
 */
static bool power_control_take_phase_request(uint8_t *voltage_v,
                                             uint16_t *current_ma)
{
    uint32_t primask;

    if ((voltage_v == NULL) || (current_ma == NULL))
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (phase_request_pending == 0U)
    {
        if ((primask & 1U) == 0U)
        {
            __enable_irq();
        }

        return false;
    }

    *voltage_v = requested_voltage_v;
    *current_ma = requested_current_ma;

    phase_request_pending = 0U;

    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }

    return true;
}


/*
 * Переход в аварийно безопасное состояние.
 *
 * PB1 отключается отдельно и сразу в protocol.c.
 * Здесь запрещается его повторное включение и ставится запрос OUT0.
 */
static void power_control_set_fault(void)
{
    automatic_mode = 0U;
    phase_ready = 0U;

    phase_request_pending = 0U;
    target_valid = 0U;
    phase_configuration_in_progress = 0U;
    last_configuration_valid = 0U;

    source_fault = 1U;
    fault_report_pending = 1U;

    shutdown_requested = 1U;
}


/*
 * Отправка OUT0 после завершения текущей UART-команды.
 */
static void power_control_process_shutdown(void)
{
    if (ppe3323_is_busy())
    {
        return;
    }

    if (shutdown_command_queued == 0U)
    {
        if (!ppe3323_queue_output_off())
        {
            source_fault = 1U;
            fault_report_pending = 1U;
            return;
        }

        shutdown_command_queued = 1U;
        return;
    }

    /*
     * OUT0 уже был передан, UART освободился.
     */
    shutdown_command_queued = 0U;
    shutdown_requested = 0U;

    phase_ready = 0U;
    target_valid = 0U;
    phase_configuration_in_progress = 0U;
    last_configuration_valid = 0U;
}


void power_control_init(void)
{
    requested_voltage_v = 0U;
    requested_current_ma = 0U;
    phase_request_pending = 0U;

    automatic_mode = 0U;
    phase_ready = 0U;

    source_fault = 0U;
    fault_report_pending = 0U;

    shutdown_requested = 1U;
    shutdown_command_queued = 0U;
    phase_not_ready_pending = 0U;
    target_voltage_v = 0U;
    target_current_ma = 0U;
    target_valid = 0U;

    phase_configuration_in_progress = 0U;

    last_voltage_v = 0U;
    last_current_ma = 0U;
    last_configuration_valid = 0U;
}


/*
 * Вызывается после успешной передачи FEED_PLAN всем ведомым.
 */
void power_control_arm_new_plan(void)
{
    automatic_mode = 1U;
    phase_ready = 0U;

    phase_request_pending = 0U;
    phase_not_ready_pending = 0U;
    target_valid = 0U;
    phase_configuration_in_progress = 0U;

    last_configuration_valid = 0U;

    source_fault = 0U;
    fault_report_pending = 0U;
}


/*
 * Может вызываться из TIM2 IRQ.
 *
 * UART-функции и HAL_Delay() здесь не используются.
 */
void power_control_disable(void)
{
    /*
     * Функция может быть вызвана из TIM2 IRQ.
     *
     * Поэтому здесь изменяются только volatile-признаки, которые
     * безопасно читаются и записываются как отдельные байты.
     *
     * target_valid, phase_configuration_in_progress и
     * last_configuration_valid сбрасываются позже из main()
     * после фактической отправки OUT0.
     */
    automatic_mode = 0U;
    phase_ready = 0U;

    phase_request_pending = 0U;
    phase_not_ready_pending = 0U;

    /*
     * power_control_poll() увидит этот признак и передаст OUT0,
     * когда UART освободится.
     */
    shutdown_requested = 1U;
}


/*
 * Может вызываться из TIM2 IRQ.
 */
void power_control_request_phase(uint8_t voltage_v,
                                 uint16_t current_ma)
{
    if ((automatic_mode == 0U) || (source_fault != 0U))
    {
        return;
    }

    if ((voltage_v > POWER_SOURCE_MAX_VOLTAGE_V) ||
        (current_ma > POWER_SOURCE_MAX_CURRENT_MA))
    {
        /*
         * Защита от повреждённого или некорректного плана.
         *
         * Полная обработка ошибки выполняется позже в main(),
         * поэтому из TIM2 IRQ меняем только volatile-признаки.
         */
        automatic_mode = 0U;
        phase_ready = 0U;
        phase_request_pending = 0U;
        phase_not_ready_pending = 1U;

        return;
    }

    /*
     * До окончания настройки новой фазы PB1 включать нельзя.
     */
    phase_ready = 0U;

    /*
     * Сначала записываются параметры,
     * затем устанавливается признак готового запроса.
     */
    requested_voltage_v = voltage_v;
    requested_current_ma = current_ma;

    __DMB();

    phase_request_pending = 1U;
}


/*
 * Выполняется только в основном цикле main().
 */
void power_control_poll(void)
{
    uint8_t new_voltage_v;
    uint16_t new_current_ma;

    /*
     * Продвигаем неблокирующую передачу UART.
     */
    ppe3323_poll();

    /*
     * T_EXTRA завершилось, но источник не подготовился.
     *
     * Этот флаг выставляется из TIM2 IRQ, а полная обработка аварии
     * выполняется здесь, в основном цикле.
     */
    if (phase_not_ready_pending != 0U)
    {
        phase_not_ready_pending = 0U;
        power_control_set_fault();
    }

    /*
     * Выключение OUT1 всегда имеет приоритет.
     */
    if (shutdown_requested != 0U)
    {
        power_control_process_shutdown();
        return;
    }

    if ((automatic_mode == 0U) || (source_fault != 0U))
    {
        return;
    }

    /*
     * Завершилась передача VSET1/ISET1.
     */
    if (phase_configuration_in_progress != 0U)
    {
        if (ppe3323_is_busy())
        {
            return;
        }

        if (ppe3323_has_error() ||
            !ppe3323_has_phase_configuration())
        {
            power_control_set_fault();
            return;
        }

        last_voltage_v = target_voltage_v;
        last_current_ma = target_current_ma;
        last_configuration_valid = 1U;

        phase_configuration_in_progress = 0U;
    }

    /*
     * Получен запрос очередной фазы от protocol.c.
     */
    if (phase_request_pending != 0U)
    {
        if (ppe3323_is_busy())
        {
            return;
        }

        if (!power_control_take_phase_request(&new_voltage_v,
                                              &new_current_ma))
        {
            return;
        }

        target_voltage_v = new_voltage_v;
        target_current_ma = new_current_ma;
        target_valid = 1U;

        /*
         * При одинаковых U/I не повторяем VSET1/ISET1.
         *
         * Дополнительное время T_EXTRA всё равно остаётся,
         * потому что его отсчитывает protocol.c.
         */
        if ((last_configuration_valid != 0U) &&
            (last_voltage_v == target_voltage_v) &&
            (last_current_ma == target_current_ma) &&
            ppe3323_has_phase_configuration() &&
            !ppe3323_has_error())
        {
            phase_configuration_in_progress = 0U;
        }
        else
        {
            if (!ppe3323_queue_phase(target_voltage_v,
                                     target_current_ma))
            {
                power_control_set_fault();
                return;
            }

            phase_configuration_in_progress = 1U;
            return;
        }
    }

    if (target_valid == 0U)
    {
        return;
    }

    if (ppe3323_is_busy())
    {
        return;
    }

    if (ppe3323_has_error() ||
        !ppe3323_has_phase_configuration())
    {
        power_control_set_fault();
        return;
    }

    /*
     * OUT1 передаётся только после первой настройки источника.
     *
     * Между фазами OUT1 остаётся включённым,
     * а линия отключается и подключается внешним ключом PB1.
     */
    if (!ppe3323_output_is_on())
    {
        if (!ppe3323_queue_output_on())
        {
            power_control_set_fault();
        }

        return;
    }

    /*
     * UART свободен, VSET1/ISET1 переданы, OUT1 включён.
     *
     * Физическое установление напряжения происходит в пределах T_EXTRA.
     */
    phase_ready = 1U;
}


bool power_control_is_phase_ready(void)
{
    return ((automatic_mode != 0U) &&
            (source_fault == 0U) &&
            (phase_ready != 0U));
}


void power_control_mark_phase_not_ready(void)
{
    /*
     * Функция вызывается из TIM2 IRQ.
     *
     * Здесь нельзя выполнять полное изменение состояния автомата,
     * потому что power_control_poll() может одновременно работать
     * с параметрами текущей фазы в основном цикле.
     */
    if (phase_ready == 0U)
    {
        phase_not_ready_pending = 1U;
    }
}


bool power_control_is_automatic_mode(void)
{
    return (automatic_mode != 0U);
}


bool power_control_take_fault_report(void)
{
    uint32_t primask;
    bool result;

    primask = __get_PRIMASK();
    __disable_irq();

    result = (fault_report_pending != 0U);
    fault_report_pending = 0U;

    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }

    return result;
}
