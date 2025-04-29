#pragma once

#include "../../Middlewares/Third_Party/FreeRTOS/source/CMSIS_RTOS_V2/cmsis_os.h"

/* --- mutexes --- */
/* declare mutexes begin*/
extern osMutexId_t itmMutex;
extern osMutexId_t canBroadcastMutex;
/* declare mutexes end*/

void initMutexes();

/* --- semaphores --- */
/* declare semaphores begin */
/* declare semaphores end */

void initSemphrs();

/* --- queues --- */
/* declare queues begin */
/* declare queues end */

void initQueues();
