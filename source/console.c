/*
 * console.c
 *
 *  Created on: Sep 26, 2020
 *      Author: sceaj
 */

#include "fsl_lpuart.h"

int __sys_write(int handle, char *buffer, int size)
{
    if (buffer == 0)
    {
        /* return -1 if error. */
        return -1;
    }

    /* This function only writes to "standard out" and "standard err" for all other file handles it returns failure. */
    if ((handle != 1) && (handle != 2))
    {
        return -1;
    }

    /* Send data. */
    LPUART_WriteBlocking(LPUART1, (const uint8_t*)buffer, (size_t)size);

    return 0;
}

int __sys_readc(void)
{
    return LPUART_ReadByte(LPUART1);
}



