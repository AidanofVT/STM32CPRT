/*****************************************************************************
*
*               PROPRIETARY NOTICE
*               ----------- ------
*
*  This code is the sole property of Nuubis, Inc. and may
*  not be used, duplicated or distributed without the express written
*  permission of Nuubis, Inc.
*
*  Copyright (C) 2020 Nuubis, Inc.
*
*               ALL RIGHTS RESERVED
*               --- ------ --------
*
*  NAME:        interp.c
*
*  DESCRIPTION: Command interpreter and execution. Functions for receiving,
*               parsing, and executing special commands from the UART port.
*
******************************************************************************
*
*  AUTHOR: S. Reynolds
*
*  DATE AND REVISION HISTORY:
*     ##:  MM/DD/YYYY   Note:
*     01:  05/01/2020   Initial Release
*
*  MODULES INCLUDED:
*
*
*****************************************************************************/

// Include Files
#include "main.h"
#include "interp.h"
#include <string.h>
#include <stdio.h>

// External Data Structures

// Local Constants and Macros

// Module Code

static CmdTableEntry CommandTable[MAX_NUM_OF_COMMANDS];
static uint16_t      currentNumberOfCommands = 0;

static char   cmdArgs[MAX_NUM_OF_ARGS][MAX_ARG_SIZE] = { {0} };
static char * scriptArgv[MAX_NUM_OF_ARGS] = {0};
static int    scriptArgc = 0;

// Code

/*-----------------------------------------------------------------------------
 * Function:    IsAlpha
 *
 * Description: Check to see is a char is alpha numeric.
 *
 * Parameters:  c - Char to be checked.
 *
 * Returns:     -1 if alphanumeric, 0 otherwise.
 *---------------------------------------------------------------------------*/
int IsAlpha(char c)
{
    if (((c >= '0') && (c <= '9')) ||
        ((c >= 'A') && (c <= 'Z')) ||
        ((c >= 'a') && (c <= 'z')))
    {
        return -1;
    }
    else
    {
        return 0;
    }
}


/*----------------------------------------------------------------------------
 * Function:    InterpLookup
 *
 * This function searches the command table for an entry matching the input
 * command name. If a match is found, a pointer to an associated command table
 * entry is returned.
 *
 * Parameters:  cmdName - Pointer to string containing the command name.
 *
 * Returns:     If the command name is recognized, a pointer to an associated
 *              command table entry. If the command name is not recognized,
 *              a NULL pointer is returned.
 *--------------------------------------------------------------------------*/
static CmdTableEntry* InterpLookup(char* cmdName)
{
    uint16_t i;
    CmdTableEntry* pCmdTableEntry = 0;

    // Check if command name exists.
    if (cmdName == (char*)0)
        return NULL;

    // Look in command table for a matching command name.
    for ( i=0; i <= currentNumberOfCommands; i++ )
    {
        if ( !strcmp(CommandTable[i].name, cmdName) )
        {
            pCmdTableEntry = &CommandTable[i];
            return pCmdTableEntry;
        }
    }

    return NULL;
}


/*----------------------------------------------------------------------------
 * Function:    InterpValidateCmd
 *
 * The command validation function. This function performs the command table
 * lookup and validates the number of command parameters.
 *
 * Parameters:  ppCmdTabEnt - User's pointer for this function to place a
 *                            pointer to a command table entry associated with
 *                            the current command.
 *
 * Returns:     An interpreter error code.
 *--------------------------------------------------------------------------*/
static INTERP InterpValidateCmd(char** argv, int argc, CmdTableEntry** ppCmdTabEnt )
{
    CmdTableEntry*  pCmdTableEntry = 0;
    CmdTableEntry** UserPPcmdTabEnt = ppCmdTabEnt;

    *UserPPcmdTabEnt = 0;

    /* Check if command name exists. */
    if ( argv[0] == NULL )
        return INTERP_ARG_CMD_NOT_VALID;

    /* Find the command object associated with the command. */
    pCmdTableEntry = InterpLookup( argv[0] );
    if ( pCmdTableEntry == NULL )
        return INTERP_ARG_CMD_NOT_VALID;

    // Validate the command arguments.
    if ( argc < pCmdTableEntry->minArgs )
        return INTERP_ARG_UNDER_MIN;
    else if ( argc > pCmdTableEntry->maxArgs )
        return INTERP_ARG_OVER_MAX;
    else
    {
        *ppCmdTabEnt = pCmdTableEntry;
        return INTERP_SUCCESS;
    }

    return INTERP_ARG_CMD_NOT_VALID;
}


/*----------------------------------------------------------------------------
 * Function:    InterpExecute
 *
 * This function validates the assembled command from the global command
 * buffer. If the command is valid, its associated function handler is
 * executed.
 *
 * Parameters:  None.
 *
 * Returns:     An interpreter error code.
 *--------------------------------------------------------------------------*/
static INTERP InterpExecute(int argc, char** argv)
{
    CmdTableEntry*       pCmdTabEnt1   = NULL;
    INTERP               interpRes     = INTERP_BAD_ARGC;

    /* Check arguments and lookup command procedure. */
    if (argc > 0)
    {
        /* Check command for validity. */
        if ( (interpRes = InterpValidateCmd(argv, argc, &pCmdTabEnt1)) == INTERP_SUCCESS )
        {
            // Execute the command.
            if ( pCmdTabEnt1->cmdFn(argc, argv) )
            {
            	UartPrintf("Error Executing Command: %s\r\n", argv[0]);
                interpRes = INTERP_CMD_FAIL;
            }
        }
        else
        {
            switch ( interpRes )
            {
                case INTERP_ARG_CMD_NOT_VALID:
                	UartPrintf("Command: %s -- not Valid \r\n", argv[0]);
                    break;

                case INTERP_ARG_UNDER_MIN:
                	UartPrintf("Too few command arguments\r\n");
                    break;

                case INTERP_ARG_OVER_MAX:
                	UartPrintf("Too many command arguments\r\n");
                    break;

                default:
                	UartPrintf("Unknown command failure\r\n");
                	break;
            }
        }
    }

    return interpRes;
}


/*-----------------------------------------------------------------------------
 * Function:     InterpRun
 *
 * Description:  Parse and execute an ASCII command including arguments.
 *
 * Parameters:  cmdStr - A pointer to a command string containing an ASCII command.
 *
 * Returns:     0 if command executed successfully, 1 otherwise.
 *----------------------------------------------------------------------------*/
INTERP InterpRun(const char* cmdStr)
{
    unsigned int    i   = 0;
    const char*     ptr = cmdStr;

    /* Create command tokens. */
    scriptArgc = 0;

    while (*ptr != 0)
    {
        /* Argument is a string. */
        if (*ptr == '"')
        {
            ptr++;
            while (*ptr != '"')
            {
                scriptArgv[scriptArgc][i++] = *ptr++;
            }
        }

        /* Add character to argument. */
        if (IsAlpha(*ptr))
        {
            scriptArgv[scriptArgc][i++] = *ptr;
        }
        else if ((*ptr == '.') ||
                 (*ptr == '_') ||
                 (*ptr == '-') ||
                 (*ptr == '/') ||
                 (*ptr == '+') ||
                 (*ptr == '=') ||
                 (*ptr == '>') ||
                 (*ptr == ':') ||
                 (*ptr == '?'))
        {
            scriptArgv[scriptArgc][i++] = *ptr;
        }
        /* Terminate argument. */
        else if ( i > 0 )
        {
            scriptArgv[scriptArgc][i] = '\0';
            i = 0;
            scriptArgc++;
        }

        /* If at end of cmd str and there were cmd chars for the prev argv, */
        /* terminate the argument. */
        ptr++;
        if ( (*ptr == 0) && (i > 0) )
        {
            scriptArgv[scriptArgc][i] = '\0';
            i = 0;
            scriptArgc++;
        }
    }

    /* Execute the parsed ASCII command. */
    return InterpExecute( scriptArgc, scriptArgv );
}


/*----------------------------------------------------------------------------
 * Function:    InterpRegister
 *
 * This function registers a command function handler linked to a command name.
 *
 * Parameters:  cmdName - Pointer to string containing command name.
 *              proc    - Function pointer of the command handler.
 *              minArgs - The minimum # of args the handler accepts.
 *              maxArgs - The maximum # of args the handler accepts.
 *
 * Returns:     A interpreter error code.
 *--------------------------------------------------------------------------*/
INTERP InterpRegister( char* cmdName, int (*proc)(), int minArgs, int maxArgs )
{
    if ( currentNumberOfCommands < MAX_NUM_OF_COMMANDS )
    {
        strcpy( CommandTable[currentNumberOfCommands].name, cmdName );
        CommandTable[currentNumberOfCommands].cmdFn   = proc;
        CommandTable[currentNumberOfCommands].minArgs = minArgs;
        CommandTable[currentNumberOfCommands].maxArgs = maxArgs;
        currentNumberOfCommands++;
        return ( INTERP_SUCCESS );
    }

    return( INTERP_CMD_REGISTER_FAIL );
}


/*----------------------------------------------------------------------------
 * Function:    InterpInit
 *
 * This function is called to initialize the command interpreter.
 *
 * Parameters:  None.
 *
 * Returns:     None.
 *--------------------------------------------------------------------------*/
void InterpInit()
{
    uint16_t i;

    // Initialize the command argument array.
    for ( i = 0; i < MAX_NUM_OF_ARGS; i++ )
        scriptArgv[i] = &(cmdArgs[i][0]);

    UartPrintf("Interpreter Initialized\r\n");
}
