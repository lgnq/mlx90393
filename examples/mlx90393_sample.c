/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#include <rtthread.h>
#include "mlx90393.h"

/* Default configuration, please change according to the actual situation, support i2c and spi device name */
#define MLX90393_DEVICE_NAME  "i2c2"


/* Test function */
static int mlx90393_test()
{
    struct mlx90393_device *dev;
    int i;

    /* Initialize mlx90393, The parameter is RT_NULL, means auto probing for i2c*/
    dev = mlx90393_init(MLX90393_DEVICE_NAME, RT_NULL);

    if (dev == RT_NULL)
    {
        rt_kprintf("mlx90393 init failed\n");
        return -1;
    }
    rt_kprintf("mlx90393 init succeed\n");

    for (i = 0; i < 5; i++)
    {
        struct mlx90393_txyz txyz;
        mlx90393_get_txyz_raw(dev, &txyz);
        rt_kprintf("txyz.x = %3d txyz.y = %3d, txyz.z = %3d, txyz.t = %3d\n", txyz.x, txyz.y, txyz.z,txyz.t);
        rt_thread_mdelay(100);
    }

    mlx90393_deinit(dev);

    return 0;
}
MSH_CMD_EXPORT(mlx90393_test, mlx90393 sensor test function);
