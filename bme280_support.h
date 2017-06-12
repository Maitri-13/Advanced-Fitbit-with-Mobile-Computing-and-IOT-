/*
 * bme280_support.h
 *
 *  Created on: Apr 8, 2017
 *      Author: richa
 */

#ifndef SRC_BME280_SUPPORT_H_
#define SRC_BME280_SUPPORT_H_

#include "bme280.h"
#include "main.h"
#include "stdint.h"
void bme280_data_readout_template(int32_t v_comp_temp_s32[0],uint32_t v_comp_press_u32[0],uint32_t v_comp_humidity_u32[0]);

#endif /* SRC_BME280_SUPPORT_H_ */
