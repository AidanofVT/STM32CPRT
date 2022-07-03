
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _INTERP_H
#define _INTERP_H


//
// PUBLIC
//
#define MAX_NUM_OF_COMMANDS 128  // Maximum number of commands.
#define MAX_NUM_OF_ARGS 16       // Max number of args including the command itself.
#define MAX_ARG_SIZE 32          // Max string size of arg (command itself is an arg).

typedef enum
{
    INTERP_SUCCESS,
    INTERP_CMD_REGISTER_FAIL,
    INTERP_BAD_ARGC,
    INTERP_ARG_CMD_NOT_VALID,
    INTERP_ARG_UNDER_MIN,
    INTERP_ARG_OVER_MAX,
    INTERP_CMD_FAIL

} INTERP;

typedef struct CMD_TAB_ENT
{
    char                    name[16];   /* Command string name. */
    int                     (*cmdFn)(); /* Procedure to call for command. */
    int                     minArgs;    /* Minimum number of arguments. */
    int                     maxArgs;    /* Maximum number of arguments. */

} CmdTableEntry;


int             IsAlpha(char c);
INTERP          InterpRun(const char* cmdStr);
INTERP          InterpRegister( char *cmdName, int (*proc)(), int minArgs, int maxArgs );
void            RegisterAllCommands();
void            InterpInit();


#endif /* _INTERP_H */
