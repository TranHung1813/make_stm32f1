#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#if 1

#include "app_debug.h"
#include "app_cli.h"
#include "app_shell.h"
#include "main.h"
#include "host_data_layer.h"
#include "board_hw.h"

static shell_context_struct m_user_context;
static app_cli_cb_t *m_cb;
static int32_t cli_reset_system(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_flash_mode(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_volume_change(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_test_host_die(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_set_timeout(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_fake_power_event(p_shell_context_t context, int32_t argc, char **argv);

static const shell_command_context_t cli_command_table[] = 
{
    {"reset",           "\treset: reset system\r\n",                            cli_reset_system,                           0},
    {"flash",           "\tflash: turn off power to flash SC20\r\n",            cli_flash_mode,                             0}, 
    {"pot",             "\tpot: POT command '+' '-' 'reset' \r\n",              cli_volume_change,                          2},
    {"test",            "\ttest: Test cmd\r\n",                                 cli_test_host_die,                          1},     
    {"timeout",         "\ttimeout: Set uart/ping timeout\r\n",                 cli_set_timeout,                            2}, 
    {"pwr",            "\tpwr: on/off to fake power lost event\r\n",            cli_fake_power_event,                       1},  
};


void app_cli_poll(uint8_t ch)
{
    app_shell_task(ch);
}


void app_cli_start(app_cli_cb_t *callback)
{
    m_cb = callback;
    app_shell_set_context(&m_user_context);
    app_shell_init(&m_user_context,
                   m_cb->puts,
                   m_cb->printf,
                   ">",
                   true);

    /* Register CLI commands */
    for (int i = 0; i < sizeof(cli_command_table) / sizeof(shell_command_context_t); i++)
    {
        app_shell_register_cmd(&cli_command_table[i]);
    }

    /* Run CLI task */
    app_shell_task(APP_SHELL_INVALID_CHAR);
}

/* Reset System */
static int32_t cli_reset_system(p_shell_context_t context, int32_t argc, char **argv)
{
//    DEBUG_INFO("System reset\r\n");
    NVIC_SystemReset();
    return 0;
}

static int32_t cli_flash_mode(p_shell_context_t context, int32_t argc, char **argv)
{
    host_data_layer_set_power_mode(HOST_POWER_FLASH_MODE);
    return 0;
}

static int32_t cli_volume_change(p_shell_context_t context, int32_t argc, char **argv)
{
    int num = atoi(argv[2]);
    if (strstr(argv[1], "+"))
    {
        board_hw_digital_pot_increase(num);
        APP_DEBUG_INFO("+\r\n");
    }
    else if (strstr(argv[1], "-"))
    {
        board_hw_digital_pot_decrease(num);
        APP_DEBUG_INFO("-\r\n");
    }
    else if (strstr(argv[1], "reset"))
    {
        board_hw_digital_pot_reset();
        APP_DEBUG_INFO("Reset POT\r\n");
    }
    else if (strstr(argv[1], "set"))
    {
        board_hw_digital_pot_set(num);
        APP_DEBUG_INFO("Set POT %u\r\n", num);
    }
    else if (strstr(argv[1], "unmute"))
    {
        board_hw_digital_pot_unmute();
        APP_DEBUG_INFO("Unmute POT\r\n");
    }
    else if (strstr(argv[1], "mute"))
    {
        board_hw_digital_pot_mute();
        APP_DEBUG_INFO("Mute POT\r\n");
    }
    return 0;
}

static int32_t cli_test_host_die(p_shell_context_t context, int32_t argc, char **argv)
{
    if (strstr(argv[1], "uart"))
    {
        APP_DEBUG_INFO("Fake event uart\r\n");
        board_hw_usart_host_control(false);
    }
    else if (strstr(argv[1], "pulse-"))
    {
        APP_DEBUG_INFO("Fake event ping\r\n");
        board_hw_ping_irq_control(false);
    }
    else if (strstr(argv[1], "exit"))
    {
        board_hw_ping_irq_control(true);
        board_hw_usart_host_control(true);
        APP_DEBUG_INFO("Exit test mode\r\n");
    }
    
    return 0;
}

static int32_t cli_set_timeout(p_shell_context_t context, int32_t argc, char **argv)
{
    int timeout = atoi(argv[2]);
    APP_DEBUG_INFO("Timeout %u\r\n", timeout);
    if (strstr(argv[1], "uart"))
    {
        host_data_layer_set_uart_timeout(timeout);
    }
    else if (strstr(argv[1], "ping"))
    {
        host_data_layer_set_ping_timeout(timeout);
    }
    return 0;
}

static int32_t cli_fake_power_event(p_shell_context_t context, int32_t argc, char **argv)
{
    extern uint8_t fake_power_event;
    if (strstr(argv[1], "on"))
    {
        fake_power_event = 0;
        APP_DEBUG_INFO("Power event OK\r\n");
    }
    else if (strstr(argv[1], "off"))
    {
        fake_power_event = 1;
        APP_DEBUG_INFO("Power event ERR\r\n");
    }
    return 0;
}

#endif /* APP_CLI_ENABLE */
