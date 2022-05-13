/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#ifndef __SENSOR_MELEXIS_MLX90393_H__
#define __SENSOR_MELEXIS_MLX90393_H__

#include "sensor.h"
#include "mlx90393.h"

#define MLX90393_I2C_ADDRESS                    0x19 // address pin A0/A1 low(GND), default for Melexis MLX90393

#define MLX90393_CTRL_NOP                       0x10
#define MLX90393_CTRL_RESET                     0x11

int rt_hw_mlx90393_init(const char *name, struct rt_sensor_config *cfg);

#endif
