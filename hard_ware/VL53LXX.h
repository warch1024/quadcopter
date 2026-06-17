#ifndef __VL53LXX_H__
#define __VL53LXX_H__

#include "stm32f10x.h"

#define VL53LXX_DEFAULT_ADDR            0x29

#define VL53LXX_OK                      0x00
#define VL53LXX_ERR_I2C                 0x01
#define VL53LXX_ERR_TIMEOUT             0x02
#define VL53LXX_ERR_ID                  0x03
#define VL53LXX_ERR_INIT                0x04


#define VL53LXX_I2C_TIMEOUT             100000

#define VL53LXX_XSHUT_GPIO              GPIOB
#define VL53LXX_XSHUT_PIN               GPIO_Pin_12

#define VL53LXX_REG_SOFT_RESET                          0x0000
#define VL53LXX_REG_I2C_SLAVE_ADDR                      0x0001
#define VL53LXX_REG_VHV_CONFIG_TIMEOUT_MACROP_BOUND     0x0008
#define VL53LXX_REG_GPIO_HV_MUX_CTRL                    0x0030
#define VL53LXX_REG_GPIO_TIO_HV_STATUS                  0x0031
#define VL53LXX_REG_PHASECAL_CONFIG_TIMEOUT_MACROP      0x004B
#define VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI    0x005E
#define VL53LXX_REG_RANGE_CONFIG_VCSEL_PERIOD_A         0x0060
#define VL53LXX_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI    0x0061
#define VL53LXX_REG_RANGE_CONFIG_VCSEL_PERIOD_B         0x0063
#define VL53LXX_REG_RANGE_CONFIG_VALID_PHASE_HIGH       0x0069
#define VL53LXX_REG_SD_CONFIG_WOI_SD0                   0x0078
#define VL53LXX_REG_SD_CONFIG_INITIAL_PHASE_SD0         0x007A
#define VL53LXX_REG_SYSTEM_INTERRUPT_CLEAR              0x0086
#define VL53LXX_REG_SYSTEM_MODE_START                   0x0087
#define VL53LXX_REG_RESULT_INTERRUPT_STATUS             0x0088
#define VL53LXX_REG_RESULT_RANGE_STATUS                 0x0089
#define VL53LXX_REG_RESULT_FINAL_CROSSTALK_RANGE_MM_SD0 0x0096
#define VL53LXX_REG_RESULT_PEAK_SIGNAL_RATE_MCPS_SD0    0x0098
#define VL53LXX_REG_RESULT_AMBIENT_RATE_MCPS_SD         0x0090
#define VL53LXX_REG_FIRMWARE_SYSTEM_STATUS              0x00E5
#define VL53LXX_REG_IDENTIFICATION_MODEL_ID             0x010F

#define VL53LXX_MODE_STOP                               0x00
#define VL53LXX_MODE_START_RANGING                      0x40

#define VL53LXX_DISTANCEMODE_SHORT                      0x01
#define VL53LXX_DISTANCEMODE_MEDIUM                     0x02
#define VL53LXX_DISTANCEMODE_LONG                       0x03

#define VL53LXX_MODEL_ID_EXPECTED                       0xEACC

#define VL53LXX_OFFSET_MM                               10

typedef struct {
    uint16_t distance_mm;
    uint8_t  range_status;
    uint16_t signal_rate;
    uint16_t ambient_rate;
} vl53lxx_result_t;

uint8_t vl53lxx_init(void);
void vl53lxx_set_timing_budget_ms(uint16_t ms);
void vl53lxx_set_distance_mode(uint8_t mode);
void vl53lxx_start_continuous(void);
void vl53lxx_stop_continuous(void);
uint8_t vl53lxx_check_for_data_ready(uint8_t *is_ready);
uint8_t vl53lxx_read_result(vl53lxx_result_t *result);
void vl53lxx_dump_results(void);

#endif
