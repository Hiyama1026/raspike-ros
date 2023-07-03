// SPDX-License-Identifier: MIT
/*
 * Copyright (c) 2022 Embedded and Real-Time Systems Laboratory,
 *                    Graduate School of Information Science, Nagoya Univ., JAPAN
 */
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "uros_raspike-rt.h"

/*
 *  メインタスク
 */
void
main_task(intptr_t exinf)
{

    while (1) {
        SYSTIM st;
        get_tim(&st);

        dly_tsk(10000);     
    }
}
