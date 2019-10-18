/*
 * bme_task.h
 *
 *  Created on: Mar 21, 2016
 *      Author: Nicholas Metcalf
 */

#ifndef BME_TASK_H
#define BME_TASK_H

#define BME_TASK_STACK_SIZE   512
#define BME_WAKE_DELAY        5000    // 5 ms

void bme280_task_init(void);
void bme280_task_run(void);

#endif /* BME_TASK_H */
