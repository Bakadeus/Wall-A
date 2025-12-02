/*
 * sys_getentropy.c
 *
 *  Created on: Dec 2, 2025
 *      Author: thhoguin
 */


#include <errno.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int _getentropy(void *buffer, size_t length)
{
    errno = ENOSYS;
    return -1;
}

#ifdef __cplusplus
}
#endif
