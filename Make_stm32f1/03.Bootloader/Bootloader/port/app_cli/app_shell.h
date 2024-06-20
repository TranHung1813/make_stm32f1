#ifndef _FSL_SHELL_H_
#define _FSL_SHELL_H_

/*!
 * @addtogroup SHELL
 * @{
 */
#include "stdint.h"
#include "stdbool.h"
#include "stdarg.h"

#define APP_SHELL_INVALID_CHAR          0xFF

/*! @brief Macro to set on/off history feature. */
#ifndef SHELL_USE_HISTORY
#define SHELL_USE_HISTORY (0U)
#endif

/*! @brief Macro to set on/off history feature. */
#ifndef SHELL_SEARCH_IN_HIST
#define SHELL_SEARCH_IN_HIST (0U)
#endif

/*! @brief Macro to select method stream. */
#ifndef SHELL_USE_FILE_STREAM
#define SHELL_USE_FILE_STREAM (0U)
#endif

/*! @brief Macro to set on/off auto-complete feature. */
#ifndef SHELL_AUTO_COMPLETE
#define SHELL_AUTO_COMPLETE (0U)
#endif

/*! @brief Macro to set console buffer size. */
#ifndef SHELL_BUFFER_SIZE
#define SHELL_BUFFER_SIZE (256U)
#endif

/*! @brief Macro to set maximum arguments in command. */
#ifndef SHELL_MAX_ARGS
#define SHELL_MAX_ARGS (5U)
#endif

/*! @brief Macro to set maximum count of history commands. */
#ifndef SHELL_HIST_MAX
#define SHELL_HIST_MAX (1U)
#endif

/*! @brief Macro to set maximum count of commands. */
#ifndef SHELL_MAX_CMD
#define SHELL_MAX_CMD (16U)
#endif

/*! @brief Shell user send data callback prototype.*/
typedef void (*send_data_cb_t)(uint8_t *buf, uint32_t len);

/*! @brief Shell user receiver data callback prototype.*/
typedef void (*recv_data_cb_t)(uint8_t *ch);

/*! @brief Shell user printf data prototype.*/
typedef int (*printf_data_t)(const char *msg);
// typedef int (*printf_data_t)(const char * format, va_list arguments);

/*! @brief A type for the handle special key. */
typedef enum _fun_key_status
{
    kSHELL_Normal = 0U,   /*!< Normal key */
    kSHELL_Special = 1U,  /*!< Special key */
    kSHELL_Function = 2U, /*!< Function key */
} fun_key_status_t;

/*! @brief Data structure for Shell environment. */
typedef struct _shell_context_struct
{
    char *prompt;                 /*!< Prompt string */
    enum _fun_key_status stat;    /*!< Special key status */
    char line[SHELL_BUFFER_SIZE]; /*!< Consult buffer */
    uint8_t cmd_num;              /*!< Number of user commands */
    uint8_t l_pos;                /*!< Total line position */
    uint8_t c_pos;                /*!< Current line position */
#if SHELL_USE_FILE_STREAM
    FILE *STDOUT, *STDIN, *STDERR;
#else
    send_data_cb_t send_data_func; /*!< Send data interface operation */
    printf_data_t printf_data_func;
#endif
    uint16_t hist_current;                            /*!< Current history command in hist buff*/
    uint16_t hist_count;                              /*!< Total history command in hist buff*/
    char hist_buf[SHELL_HIST_MAX][SHELL_BUFFER_SIZE]; /*!< History buffer*/
    bool exit;                                        /*!< Exit Flag*/
} shell_context_struct, *p_shell_context_t;

/*! @brief User command function prototype. */
typedef int32_t (*cmd_function_t)(p_shell_context_t context, int32_t argc, char **argv);

/*! @brief User command data structure. */
typedef struct _shell_command_context
{
    const char *pcCommand; /*!< The command that is executed.  For example "help".  It must be all lower case. */
    char *pcHelpString;    /*!< String that describes how to use the command.  It should start with the command itself,
                                    and end with "\r\n".  For example "help: Returns a list of all the commands\r\n". */
    const cmd_function_t pFuncCallBack; /*!< A pointer to the callback function that returns the output generated by the command. */
    int8_t expected_number_of_parameters; /*!< Commands expect a fixed number of parameters, which may be zero. */
} shell_command_context_t;

/*! @brief Structure list command. */
typedef struct _shell_command_context_list
{
    const shell_command_context_t *cmd_list[SHELL_MAX_CMD]; /*!< The command table list */
    uint8_t number_of_cmd_in_list;                             /*!< The total command in list */
} shell_command_context_list_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* _cplusplus */

/*!
 * @name Shell functional Operation
 * @{
 */

/*!
* @brief Enables the clock gate and configure the Shell module according to the configuration structure.
*
* This function must be called before calling all other Shell functions.
* Call operation the Shell commands with user-defined settings.
* The example below shows how to set up the middleware Shell and
* how to call the app_shell_init function by passing in these parameters:
* Example:
* @code
*   shell_context_struct user_context;
*   app_shell_init(&user_context, SendDataFunc, ReceiveDataFunc, "SHELL>> ");
* @endcode
* @param context The pointer to the Shell environment and  runtime states.
* @param send_cb The pointer to call back send data function.
* @param recv_cb The pointer to call back receive data function.
* @param prompt  The string prompt of Shell
* @param loop_back  Loop back character
*/
void app_shell_init(p_shell_context_t context,
                send_data_cb_t send_cb,
                printf_data_t shell_printf,
                char *prompt,
                bool loop_back);

/*!
 * @brief Shell register command.
 * @param   command_context The pointer to the command data structure.
 * @return  -1 if error or 0 if success
 */
int32_t app_shell_register_cmd(const shell_command_context_t *command_context);

/*!
 * @brief Main loop for Shell.
 * Main loop for Shell; After this function is called, Shell begins to initialize the basic variables and starts to
 * work.
 * @param    context The pointer to the Shell environment and  runtime states.
 * @return   This function does not return until Shell command exit was called.
 */
int32_t app_shell_task(uint8_t ch);

/*!
 * @brief Set shell contexr
 */
void app_shell_set_context(p_shell_context_t context);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_SHELL_H_ */
