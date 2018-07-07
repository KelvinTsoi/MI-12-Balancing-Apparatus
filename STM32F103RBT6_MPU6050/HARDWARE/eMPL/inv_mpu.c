/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu.c
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "delay.h"
#include "usart.h"


#define MPU6050
#define MOTION_DRIVER_TARGET_MSP430

/* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 * reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin)
 * labs(long x)
 * fabsf(float x)
 * min(int a, int b)
 */

#define i2c_write   MPU_Write_Len
#define i2c_read    MPU_Read_Len
#define delay_ms    delay_ms
#define get_ms      mget_ms
#define fabs        fabsf
#define min(a,b) ((a<b)?a:b)

static int set_int_enable(unsigned char enable, int type);

/* Hardware registers needed by driver. */
struct gyro_reg_s
{
  unsigned char who_am_i;
  unsigned char rate_div;
  unsigned char lpf;
  unsigned char prod_id;
  unsigned char user_ctrl;
  unsigned char fifo_en;
  unsigned char gyro_cfg;
  unsigned char accel_cfg;
//    unsigned char accel_cfg2;
//    unsigned char lp_accel_odr;
  unsigned char motion_thr;
  unsigned char motion_dur;
  unsigned char fifo_count_h;
  unsigned char fifo_r_w;
  unsigned char raw_gyro;
  unsigned char raw_accel;
  unsigned char temp;
  unsigned char int_enable;
  unsigned char dmp_int_status;
  unsigned char int_status;
//    unsigned char accel_intel;
  unsigned char pwr_mgmt_1;
  unsigned char pwr_mgmt_2;
  unsigned char int_pin_cfg;
  unsigned char mem_r_w;
  unsigned char accel_offs;
  unsigned char i2c_mst;
  unsigned char bank_sel;
  unsigned char mem_start_addr;
  unsigned char prgm_start_h;
};

/* Information specific to a particular device. */
struct hw_s
{
  unsigned char addr;
  unsigned short max_fifo;
  unsigned char num_reg;
  unsigned short temp_sens;
  short temp_offset;
  unsigned short bank_size;
};

/* When entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * TODO: This is tacky. Fix it.
 */
struct motion_int_cache_s
{
  unsigned short gyro_fsr;
  unsigned char accel_fsr;
  unsigned short lpf;
  unsigned short sample_rate;
  unsigned char sensors_on;
  unsigned char fifo_sensors;
  unsigned char dmp_on;
};

/* Cached chip configuration data.
 * TODO: A lot of these can be handled with a bitmask.
 */
struct chip_cfg_s
{
  /* Matches gyro_cfg >> 3 & 0x03 */
  unsigned char gyro_fsr;
  /* Matches accel_cfg >> 3 & 0x03 */
  unsigned char accel_fsr;
  /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
  unsigned char sensors;
  /* Matches config register. */
  unsigned char lpf;
  unsigned char clk_src;
  /* Sample rate, NOT rate divider. */
  unsigned short sample_rate;
  /* Matches fifo_en register. */
  unsigned char fifo_enable;
  /* Matches int enable register. */
  unsigned char int_enable;
  /* 1 if devices on auxiliary I2C bus appear on the primary. */
  unsigned char bypass_mode;
  /* 1 if half-sensitivity.
   * NOTE: This doesn't belong here, but everything else in hw_s is const,
   * and this allows us to save some precious RAM.
   */
  unsigned char accel_half;
  /* 1 if device in low-power accel-only mode. */
  unsigned char lp_accel_mode;
  /* 1 if interrupts are only triggered on motion events. */
  unsigned char int_motion_only;
  struct motion_int_cache_s cache;
  /* 1 for active low interrupts. */
  unsigned char active_low_int;
  /* 1 for latched interrupts. */
  unsigned char latched_int;
  /* 1 if DMP is enabled. */
  unsigned char dmp_on;
  /* Ensures that DMP will only be loaded once. */
  unsigned char dmp_loaded;
  /* Sampling rate used when DMP is enabled. */
  unsigned short dmp_sample_rate;
};

/* Information for self-test. */
struct test_s
{
  unsigned long gyro_sens;
  unsigned long accel_sens;
  unsigned char reg_rate_div;
  unsigned char reg_lpf;
  unsigned char reg_gyro_fsr;
  unsigned char reg_accel_fsr;
  unsigned short wait_ms;
  unsigned char packet_thresh;
  float min_dps;
  float max_dps;
  float max_gyro_var;
  float min_g;
  float max_g;
  float max_accel_var;
};

/* Gyro driver state variables. */
struct gyro_state_s
{
  const struct gyro_reg_s *reg;
  const struct hw_s *hw;
  struct chip_cfg_s *chip_cfg;
  const struct test_s *test;
};

/* Filter configurations. */
enum lpf_e
{
  INV_FILTER_256HZ_NOLPF2 = 0,
  INV_FILTER_188HZ,
  INV_FILTER_98HZ,
  INV_FILTER_42HZ,
  INV_FILTER_20HZ,
  INV_FILTER_10HZ,
  INV_FILTER_5HZ,
  INV_FILTER_2100HZ_NOLPF,
  NUM_FILTER
};

/* Full scale ranges. */
enum gyro_fsr_e
{
  INV_FSR_250DPS = 0,
  INV_FSR_500DPS,
  INV_FSR_1000DPS,
  INV_FSR_2000DPS,
  NUM_GYRO_FSR
};

/* Full scale ranges. */
enum accel_fsr_e
{
  INV_FSR_2G = 0,
  INV_FSR_4G,
  INV_FSR_8G,
  INV_FSR_16G,
  NUM_ACCEL_FSR
};

/* Clock sources. */
enum clock_sel_e
{
  INV_CLK_INTERNAL = 0,
  INV_CLK_PLL,
  NUM_CLK
};

/* Low-power accel wakeup rates. */
enum lp_accel_rate_e
{
  INV_LPA_1_25HZ,
  INV_LPA_5HZ,
  INV_LPA_20HZ,
  INV_LPA_40HZ
};

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

const struct gyro_reg_s reg[2] =
{
  {
    0x75,  //who_am_i
    0x19,  //rate_div
    0x1A,  //lpf
    0x0C,  //prod_id
    0x6A,  //user_ctrl
    0x23,  //fifo_en
    0x1B,  //gyro_cfg
    0x1C,  //accel_cfg
    0x1F,  // motion_thr
    0x20,  // motion_dur
    0x72,  // fifo_count_h
    0x74,  // fifo_r_w
    0x43,  // raw_gyro
    0x3B,  // raw_accel
    0x41,  // temp
    0x38,  // int_enable
    0x39,  //  dmp_int_status
    0x3A,  //  int_status
    0x6B,  // pwr_mgmt_1
    0x6C,  // pwr_mgmt_2
    0x37,  // int_pin_cfg
    0x6F,  // mem_r_w
    0x06,  // accel_offs
    0x24,  // i2c_mst
    0x6D,  // bank_sel
    0x6E,  // mem_start_addr
    0x70   // prgm_start_h
  },
  {
    0x75,  //who_am_i
    0x19,  //rate_div
    0x1A,  //lpf
    0x0C,  //prod_id
    0x6A,  //user_ctrl
    0x23,  //fifo_en
    0x1B,  //gyro_cfg
    0x1C,  //accel_cfg
    0x1F,  // motion_thr
    0x20,  // motion_dur
    0x72,  // fifo_count_h
    0x74,  // fifo_r_w
    0x43,  // raw_gyro
    0x3B,  // raw_accel
    0x41,  // temp
    0x38,  // int_enable
    0x39,  //  dmp_int_status
    0x3A,  //  int_status
    0x6B,  // pwr_mgmt_1
    0x6C,  // pwr_mgmt_2
    0x37,  // int_pin_cfg
    0x6F,  // mem_r_w
    0x06,  // accel_offs
    0x24,  // i2c_mst
    0x6D,  // bank_sel
    0x6E,  // mem_start_addr
    0x70   // prgm_start_h
  }
};

const struct hw_s hw[2] =
{
  {
    0x68,  //addr
    1024,  //max_fifo
    118,   //num_reg
    340,   //temp_sens
    -521,  //temp_offset
    256    //bank_size
  },
  {
    0x68,	 //addr
    1024,	 //max_fifo
    118,	 //num_reg
    340,	 //temp_sens
    -521,	 //temp_offset
    256	 //bank_size
  }
};

const struct test_s test[2] =
{
  {
    32768 / 250,		 //gyro_sens
    32768 / 16,		 // accel_sens
    0,				 // reg_rate_div
    1,				//	reg_lpf
    0,				 // reg_gyro_fsr
    0x18, 		//	reg_accel_fsr
    50,				//	wait_ms
    5,				//	packet_thresh
    10.0f,			 // min_dps
    105.0f,			 // max_dps
    0.14f,			//	max_gyro_var
    0.3f, 	   //	min_g
    0.95f,		   //	max_g
    0.14f 	   //	max_accel_var
  },
  {
    32768 / 250,		 //gyro_sens
    32768 / 16,		 // accel_sens
    0,				 // reg_rate_div
    1,				//	reg_lpf
    0,				 // reg_gyro_fsr
    0x18, 		//	reg_accel_fsr
    50,				//	wait_ms
    5,				//	packet_thresh
    10.0f,			 // min_dps
    105.0f,			 // max_dps
    0.14f,			//	max_gyro_var
    0.3f, 	   //	min_g
    0.95f,		   //	max_g
    0.14f 	   //	max_accel_var
  }
};


struct chip_cfg_s cf[2] =
{
  {0}, {0}
};


static struct gyro_state_s st =
{
  reg,
  hw,
  cf,
  test
};


#define MAX_PACKET_LENGTH (12)

/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
static int set_int_enable(unsigned char enable, int type)
{
  unsigned char tmp;

  if (st.chip_cfg[type].dmp_on)
  {
    if (enable)
      tmp = BIT_DMP_INT_EN;
    else
      tmp = 0x00;
    if (i2c_write(st.hw[type].addr, st.reg[type].int_enable, 1, &tmp, type))
      return -1;
    st.chip_cfg[type].int_enable = tmp;
  }
  else
  {
    if (!st.chip_cfg[type].sensors)
      return -1;
    if (enable && st.chip_cfg[type].int_enable)
      return 0;
    if (enable)
      tmp = BIT_DATA_RDY_EN;
    else
      tmp = 0x00;
    if (i2c_write(st.hw[type].addr, st.reg[type].int_enable, 1, &tmp, type))
      return -1;
    st.chip_cfg[type].int_enable = tmp;
  }
  return 0;
}

/**
 *  @brief      Register dump for testing.
 *  @return     0 if successful.
 */
int mpu_reg_dump(int type)
{
  unsigned char ii;
  unsigned char data;

  for (ii = 0; ii < st.hw[type].num_reg; ii++)
  {
    if (ii == st.reg[type].fifo_r_w || ii == st.reg[type].mem_r_w)
      continue;
    if (i2c_read(st.hw[type].addr, ii, 1, &data, type))
      return -1;
  }
  return 0;
}

/**
 *  @brief      Read from a single register.
 *  NOTE: The memory and FIFO read/write registers cannot be accessed.
 *  @param[in]  reg     Register address.
 *  @param[out] data    Register data.
 *  @return     0 if successful.
 */
int mpu_read_reg(unsigned char reg, unsigned char *data, int type)
{
  if (reg == st.reg[type].fifo_r_w || reg == st.reg[type].mem_r_w)
    return -1;
  if (reg >= st.hw[type].num_reg)
    return -1;
  return i2c_read(st.hw[type].addr, reg, 1, data, type);
}

/**
 *  @brief      Initialize hardware.
 *  Initial configuration:\n
 *  Gyro FSR: +/- 2000DPS\n
 *  Accel FSR +/- 2G\n
 *  DLPF: 42Hz\n
 *  FIFO rate: 50Hz\n
 *  Clock source: Gyro PLL\n
 *  FIFO: Disabled.\n
 *  Data ready interrupt: Disabled, active low, unlatched.
 *  @param[in]  int_param   Platform-specific parameters to interrupt API.
 *  @return     0 if successful.
 */
int mpu_init(int type)
{
  unsigned char data[6], rev;

  /* Reset device. */
  data[0] = BIT_RESET;
  if (i2c_write(st.hw[type].addr, st.reg[type].pwr_mgmt_1, 1, data, type))
    return -1;
  delay_ms(100);

  /* Wake up chip. */
  data[0] = 0x00;
  if (i2c_write(st.hw[type].addr, st.reg[type].pwr_mgmt_1, 1, data, type))
    return -1;

  /* Check product revision. */
  if (i2c_read(st.hw[type].addr, st.reg[type].accel_offs, 6, data, type))
    return -1;
  rev = ((data[5] & 0x01) << 2) | ((data[3] & 0x01) << 1) |
        (data[1] & 0x01);

  if (rev)
  {
    /* Congrats, these parts are better. */
    if (rev == 1)
      st.chip_cfg[type].accel_half = 1;
    else if (rev == 2)
      st.chip_cfg[type].accel_half = 0;
    else
    {
      return -1;
    }
  }
  else
  {
    if (i2c_read(st.hw[type].addr, st.reg[type].prod_id, 1, data, type))
      return -1;
    rev = data[0] & 0x0F;
    if (!rev)
    {
      return -1;
    }
    else if (rev == 4)
    {
      st.chip_cfg[type].accel_half = 1;
    }
    else
      st.chip_cfg[type].accel_half = 0;
  }

  /* Set to invalid values to ensure no I2C writes are skipped. */
  st.chip_cfg[type].sensors = 0xFF;
  st.chip_cfg[type].gyro_fsr = 0xFF;
  st.chip_cfg[type].accel_fsr = 0xFF;
  st.chip_cfg[type].lpf = 0xFF;
  st.chip_cfg[type].sample_rate = 0xFFFF;
  st.chip_cfg[type].fifo_enable = 0xFF;
  st.chip_cfg[type].bypass_mode = 0xFF;

  /* mpu_set_sensors always preserves this setting. */
  st.chip_cfg[type].clk_src = INV_CLK_PLL;
  /* Handled in next call to mpu_set_bypass. */
  st.chip_cfg[type].active_low_int = 1;
  st.chip_cfg[type].latched_int = 0;
  st.chip_cfg[type].int_motion_only = 0;
  st.chip_cfg[type].lp_accel_mode = 0;
  memset(&st.chip_cfg[type].cache, 0, sizeof(st.chip_cfg[type].cache));
  st.chip_cfg[type].dmp_on = 0;
  st.chip_cfg[type].dmp_loaded = 0;
  st.chip_cfg[type].dmp_sample_rate = 0;

  if (mpu_set_gyro_fsr(2000, type))
    return -1;
  if (mpu_set_accel_fsr(2, type))
    return -1;
  if (mpu_set_lpf(42, type))
    return -1;
  if (mpu_set_sample_rate(50, type))
    return -1;
  if (mpu_configure_fifo(0, type))
    return -1;

  /* Already disabled by setup_compass. */
  if (mpu_set_bypass(0, type))
    return -1;

  mpu_set_sensors(0, type);

  return 0;
}

/**
 *  @brief      Enter low-power accel-only mode.
 *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  \n MPU6050: 1.25Hz, 5Hz, 20Hz, 40Hz
 *  \n MPU6500: 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *  \n If the requested rate is not one listed above, the device will be set to
 *  the next highest rate. Requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e rate.
 *  @param[in]  rate        Minimum sampling rate, or zero to disable LP
 *                          accel mode.
 *  @return     0 if successful.
 */
int mpu_lp_accel_mode(unsigned char rate, int type)
{
  unsigned char tmp[2];

  if (rate > 40)
    return -1;

  if (!rate)
  {
    mpu_set_int_latched(0, type);
    tmp[0] = 0;
    tmp[1] = BIT_STBY_XYZG;
    if (i2c_write(st.hw[type].addr, st.reg[type].pwr_mgmt_1, 2, tmp, type))
      return -1;
    st.chip_cfg[type].lp_accel_mode = 0;
    return 0;
  }
  /* For LP accel, we automatically configure the hardware to produce latched
   * interrupts. In LP accel mode, the hardware cycles into sleep mode before
   * it gets a chance to deassert the interrupt pin; therefore, we shift this
   * responsibility over to the MCU.
   *
   * Any register read will clear the interrupt.
   */
  mpu_set_int_latched(1, type);

  tmp[0] = BIT_LPA_CYCLE;
  if (rate == 1)
  {
    tmp[1] = INV_LPA_1_25HZ;
    mpu_set_lpf(5, type);
  }
  else if (rate <= 5)
  {
    tmp[1] = INV_LPA_5HZ;
    mpu_set_lpf(5, type);
  }
  else if (rate <= 20)
  {
    tmp[1] = INV_LPA_20HZ;
    mpu_set_lpf(10, type);
  }
  else
  {
    tmp[1] = INV_LPA_40HZ;
    mpu_set_lpf(20, type);
  }
  tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG;
  if (i2c_write(st.hw[type].addr, st.reg[type].pwr_mgmt_1, 2, tmp, type))
    return -1;

  st.chip_cfg[type].sensors = INV_XYZ_ACCEL;
  st.chip_cfg[type].clk_src = 0;
  st.chip_cfg[type].lp_accel_mode = 1;
  mpu_configure_fifo(0, type);

  return 0;
}

/**
 *  @brief      Read raw gyro data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_gyro_reg(short * data, unsigned long * timestamp, int type)
{
  unsigned char tmp[6];

  if (!(st.chip_cfg[type].sensors & INV_XYZ_GYRO))
    return -1;

  if (i2c_read(st.hw[type].addr, st.reg[type].raw_gyro, 6, tmp, type))
    return -1;
  data[0] = (tmp[0] << 8) | tmp[1];
  data[1] = (tmp[2] << 8) | tmp[3];
  data[2] = (tmp[4] << 8) | tmp[5];
  if (timestamp)
    get_ms(timestamp);
  return 0;
}

/**
 *  @brief      Read raw accel data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_accel_reg(short * data, unsigned long * timestamp, int type)
{
  unsigned char tmp[6];

  if (!(st.chip_cfg[type].sensors & INV_XYZ_ACCEL))
    return -1;

  if (i2c_read(st.hw[type].addr, st.reg[type].raw_accel, 6, tmp, type))
    return -1;
  data[0] = (tmp[0] << 8) | tmp[1];
  data[1] = (tmp[2] << 8) | tmp[3];
  data[2] = (tmp[4] << 8) | tmp[5];
  if (timestamp)
    get_ms(timestamp);
  return 0;
}

/**
 *  @brief      Read temperature data directly from the registers.
 *  @param[out] data        Data in q16 format.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_temperature(long * data, unsigned long * timestamp, int type)
{
  unsigned char tmp[2];
  short raw;

  if (!(st.chip_cfg[type].sensors))
    return -1;

  if (i2c_read(st.hw[type].addr, st.reg[type].temp, 2, tmp, type))
    return -1;
  raw = (tmp[0] << 8) | tmp[1];
  if (timestamp)
    get_ms(timestamp);

  data[0] = (long)((35 + ((raw - (float)st.hw[type].temp_offset) / st.hw[type].temp_sens)) * 65536L);
  return 0;
}

/**
 *  @brief      Push biases to the accel bias registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values.
 *  @param[in]  accel_bias  New biases.
 *  @return     0 if successful.
 */
int mpu_set_accel_bias(const long * accel_bias, int type)
{
  unsigned char data[6];
  short accel_hw[3];
  short got_accel[3];
  short fg[3];

  if (!accel_bias)
    return -1;
  if (!accel_bias[0] && !accel_bias[1] && !accel_bias[2])
    return 0;

  if (i2c_read(st.hw[type].addr, 3, 3, data, type))
    return -1;
  fg[0] = ((data[0] >> 4) + 8) & 0xf;
  fg[1] = ((data[1] >> 4) + 8) & 0xf;
  fg[2] = ((data[2] >> 4) + 8) & 0xf;

  accel_hw[0] = (short)(accel_bias[0] * 2 / (64 + fg[0]));
  accel_hw[1] = (short)(accel_bias[1] * 2 / (64 + fg[1]));
  accel_hw[2] = (short)(accel_bias[2] * 2 / (64 + fg[2]));

  if (i2c_read(st.hw[type].addr, 0x06, 6, data, type))
    return -1;

  got_accel[0] = ((short)data[0] << 8) | data[1];
  got_accel[1] = ((short)data[2] << 8) | data[3];
  got_accel[2] = ((short)data[4] << 8) | data[5];

  accel_hw[0] += got_accel[0];
  accel_hw[1] += got_accel[1];
  accel_hw[2] += got_accel[2];

  data[0] = (accel_hw[0] >> 8) & 0xff;
  data[1] = (accel_hw[0]) & 0xff;
  data[2] = (accel_hw[1] >> 8) & 0xff;
  data[3] = (accel_hw[1]) & 0xff;
  data[4] = (accel_hw[2] >> 8) & 0xff;
  data[5] = (accel_hw[2]) & 0xff;

  if (i2c_write(st.hw[type].addr, 0x06, 6, data, type))
    return -1;
  return 0;
}

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int mpu_reset_fifo(int type)
{
  unsigned char data;

  if (!(st.chip_cfg[type].sensors))
    return -1;

  data = 0;
  if (i2c_write(st.hw[type].addr, st.reg[type].int_enable, 1, &data, type))
    return -1;
  if (i2c_write(st.hw[type].addr, st.reg[type].fifo_en, 1, &data, type))
    return -1;
  if (i2c_write(st.hw[type].addr, st.reg[type].user_ctrl, 1, &data, type))
    return -1;

  if (st.chip_cfg[type].dmp_on)
  {
    data = BIT_FIFO_RST | BIT_DMP_RST;
    if (i2c_write(st.hw[type].addr, st.reg[type].user_ctrl, 1, &data, type))
      return -1;
    delay_ms(50);
    data = BIT_DMP_EN | BIT_FIFO_EN;
    if (st.chip_cfg[type].sensors & INV_XYZ_COMPASS)
      data |= BIT_AUX_IF_EN;
    if (i2c_write(st.hw[type].addr, st.reg[type].user_ctrl, 1, &data, type))
      return -1;
    if (st.chip_cfg[type].int_enable)
      data = BIT_DMP_INT_EN;
    else
      data = 0;
    if (i2c_write(st.hw[type].addr, st.reg[type].int_enable, 1, &data, type))
      return -1;
    data = 0;
    if (i2c_write(st.hw[type].addr, st.reg[type].fifo_en, 1, &data, type))
      return -1;
  }
  else
  {
    data = BIT_FIFO_RST;
    if (i2c_write(st.hw[type].addr, st.reg[type].user_ctrl, 1, &data, type))
      return -1;
    if (st.chip_cfg[type].bypass_mode || !(st.chip_cfg[type].sensors & INV_XYZ_COMPASS))
      data = BIT_FIFO_EN;
    else
      data = BIT_FIFO_EN | BIT_AUX_IF_EN;
    if (i2c_write(st.hw[type].addr, st.reg[type].user_ctrl, 1, &data, type))
      return -1;
    delay_ms(50);
    if (st.chip_cfg[type].int_enable)
      data = BIT_DATA_RDY_EN;
    else
      data = 0;
    if (i2c_write(st.hw[type].addr, st.reg[type].int_enable, 1, &data, type))
      return -1;
    if (i2c_write(st.hw[type].addr, st.reg[type].fifo_en, 1, &st.chip_cfg[type].fifo_enable, type))
      return -1;
  }
  return 0;
}

/**
 *  @brief      Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_gyro_fsr(unsigned short * fsr, int type)
{
  switch (st.chip_cfg[type].gyro_fsr)
  {
    case INV_FSR_250DPS:
      fsr[0] = 250;
      break;
    case INV_FSR_500DPS:
      fsr[0] = 500;
      break;
    case INV_FSR_1000DPS:
      fsr[0] = 1000;
      break;
    case INV_FSR_2000DPS:
      fsr[0] = 2000;
      break;
    default:
      fsr[0] = 0;
      break;
  }
  return 0;
}

/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_gyro_fsr(unsigned short fsr, int type)
{
  unsigned char data;

  if (!(st.chip_cfg[type].sensors))
    return -1;

  switch (fsr)
  {
    case 250:
      data = INV_FSR_250DPS << 3;
      break;
    case 500:
      data = INV_FSR_500DPS << 3;
      break;
    case 1000:
      data = INV_FSR_1000DPS << 3;
      break;
    case 2000:
      data = INV_FSR_2000DPS << 3;
      break;
    default:
      return -1;
  }

  if (st.chip_cfg[type].gyro_fsr == (data >> 3))
    return 0;
  if (i2c_write(st.hw[type].addr, st.reg[type].gyro_cfg, 1, &data, type))
    return -1;
  st.chip_cfg[type].gyro_fsr = data >> 3;
  return 0;
}

/**
 *  @brief      Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_accel_fsr(unsigned char *fsr, int type)
{
  switch (st.chip_cfg[type].accel_fsr)
  {
    case INV_FSR_2G:
      fsr[0] = 2;
      break;
    case INV_FSR_4G:
      fsr[0] = 4;
      break;
    case INV_FSR_8G:
      fsr[0] = 8;
      break;
    case INV_FSR_16G:
      fsr[0] = 16;
      break;
    default:
      return -1;
  }
  if (st.chip_cfg[type].accel_half)
    fsr[0] <<= 1;
  return 0;
}

/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_accel_fsr(unsigned char fsr, int type)
{
  unsigned char data;

  if (!(st.chip_cfg[type].sensors))
    return -1;

  switch (fsr)
  {
    case 2:
      data = INV_FSR_2G << 3;
      break;
    case 4:
      data = INV_FSR_4G << 3;
      break;
    case 8:
      data = INV_FSR_8G << 3;
      break;
    case 16:
      data = INV_FSR_16G << 3;
      break;
    default:
      return -1;
  }

  if (st.chip_cfg[type].accel_fsr == (data >> 3))
    return 0;
  if (i2c_write(st.hw[type].addr, st.reg[type].accel_cfg, 1, &data, type))
    return -1;
  st.chip_cfg[type].accel_fsr = data >> 3;
  return 0;
}

/**
 *  @brief      Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
int mpu_get_lpf(unsigned short * lpf, int type)
{
  switch (st.chip_cfg[type].lpf)
  {
    case INV_FILTER_188HZ:
      lpf[0] = 188;
      break;
    case INV_FILTER_98HZ:
      lpf[0] = 98;
      break;
    case INV_FILTER_42HZ:
      lpf[0] = 42;
      break;
    case INV_FILTER_20HZ:
      lpf[0] = 20;
      break;
    case INV_FILTER_10HZ:
      lpf[0] = 10;
      break;
    case INV_FILTER_5HZ:
      lpf[0] = 5;
      break;
    case INV_FILTER_256HZ_NOLPF2:
    case INV_FILTER_2100HZ_NOLPF:
    default:
      lpf[0] = 0;
      break;
  }
  return 0;
}

/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
int mpu_set_lpf(unsigned short lpf, int type)
{
  unsigned char data;

  if (!(st.chip_cfg[type].sensors))
    return -1;

  if (lpf >= 188)
    data = INV_FILTER_188HZ;
  else if (lpf >= 98)
    data = INV_FILTER_98HZ;
  else if (lpf >= 42)
    data = INV_FILTER_42HZ;
  else if (lpf >= 20)
    data = INV_FILTER_20HZ;
  else if (lpf >= 10)
    data = INV_FILTER_10HZ;
  else
    data = INV_FILTER_5HZ;

  if (st.chip_cfg[type].lpf == data)
    return 0;
  if (i2c_write(st.hw[type].addr, st.reg[type].lpf, 1, &data, type))
    return -1;
  st.chip_cfg[type].lpf = data;
  return 0;
}

/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_sample_rate(unsigned short * rate, int type)
{
  if (st.chip_cfg[type].dmp_on)
    return -1;
  else
    rate[0] = st.chip_cfg[type].sample_rate;
  return 0;
}

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_sample_rate(unsigned short rate, int type)
{
  unsigned char data;

  if (!(st.chip_cfg[type].sensors))
    return -1;

  if (st.chip_cfg[type].dmp_on)
    return -1;
  else
  {
    if (st.chip_cfg[type].lp_accel_mode)
    {
      if (rate && (rate <= 40))
      {
        /* Just stay in low-power accel mode. */
        mpu_lp_accel_mode(rate, type);
        return 0;
      }
      /* Requested rate exceeds the allowed frequencies in LP accel mode,
       * switch back to full-power mode.
       */
      mpu_lp_accel_mode(0, type);
    }
    if (rate < 4)
      rate = 4;
    else if (rate > 1000)
      rate = 1000;

    data = 1000 / rate - 1;
    if (i2c_write(st.hw[type].addr, st.reg[type].rate_div, 1, &data, type))
      return -1;

    st.chip_cfg[type].sample_rate = 1000 / (1 + data);

    /* Automatically set LPF to 1/2 sampling rate. */
    mpu_set_lpf(st.chip_cfg[type].sample_rate >> 1, type);

    return 0;
  }
}

/**
 *  @brief      Get compass sampling rate.
 *  @param[out] rate    Current compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_compass_sample_rate(unsigned short * rate, int type)
{
  rate[0] = 0;
  return -1;
}

/**
 *  @brief      Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_compass_sample_rate(unsigned short rate, int type)
{
  return -1;
}

/**
 *  @brief      Get gyro sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to dps.
 *  @return     0 if successful.
 */
int mpu_get_gyro_sens(float * sens, int type)
{
  switch (st.chip_cfg[type].gyro_fsr)
  {
    case INV_FSR_250DPS:
      sens[0] = 131.f;
      break;
    case INV_FSR_500DPS:
      sens[0] = 65.5f;
      break;
    case INV_FSR_1000DPS:
      sens[0] = 32.8f;
      break;
    case INV_FSR_2000DPS:
      sens[0] = 16.4f;
      break;
    default:
      return -1;
  }
  return 0;
}

/**
 *  @brief      Get accel sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to g's.
 *  @return     0 if successful.
 */
int mpu_get_accel_sens(unsigned short * sens, int type)
{
  switch (st.chip_cfg[type].accel_fsr)
  {
    case INV_FSR_2G:
      sens[0] = 16384;
      break;
    case INV_FSR_4G:
      sens[0] = 8092;
      break;
    case INV_FSR_8G:
      sens[0] = 4096;
      break;
    case INV_FSR_16G:
      sens[0] = 2048;
      break;
    default:
      return -1;
  }
  if (st.chip_cfg[type].accel_half)
    sens[0] >>= 1;
  return 0;
}

/**
 *  @brief      Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
int mpu_get_fifo_config(unsigned char *sensors, int type)
{
  sensors[0] = st.chip_cfg[type].fifo_enable;
  return 0;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
int mpu_configure_fifo(unsigned char sensors, int type)
{
  unsigned char prev;
  int result = 0;

  /* Compass data isn't going into the FIFO. Stop trying. */
  sensors &= ~INV_XYZ_COMPASS;

  if (st.chip_cfg[type].dmp_on)
    return 0;
  else
  {
    if (!(st.chip_cfg[type].sensors))
      return -1;
    prev = st.chip_cfg[type].fifo_enable;
    st.chip_cfg[type].fifo_enable = sensors & st.chip_cfg[type].sensors;
    if (st.chip_cfg[type].fifo_enable != sensors)
      /* You're not getting what you asked for. Some sensors are
       * asleep.
       */
      result = -1;
    else
      result = 0;
    if (sensors || st.chip_cfg[type].lp_accel_mode)
      set_int_enable(1, type);
    else
      set_int_enable(0, type);
    if (sensors)
    {
      if (mpu_reset_fifo(type))
      {
        st.chip_cfg[type].fifo_enable = prev;
        return -1;
      }
    }
  }

  return result;
}

/**
 *  @brief      Get current power state.
 *  @param[in]  power_on    1 if turned on, 0 if suspended.
 *  @return     0 if successful.
 */
int mpu_get_power_state(unsigned char *power_on, int type)
{
  if (st.chip_cfg[type].sensors)
    power_on[0] = 1;
  else
    power_on[0] = 0;
  return 0;
}

/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int mpu_set_sensors(unsigned char sensors, int type)
{
  unsigned char data;

  if (sensors & INV_XYZ_GYRO)
    data = INV_CLK_PLL;
  else if (sensors)
    data = 0;
  else
    data = BIT_SLEEP;
  if (i2c_write(st.hw[type].addr, st.reg[type].pwr_mgmt_1, 1, &data, type))
  {
    st.chip_cfg[type].sensors = 0;
    return -1;
  }
  st.chip_cfg[type].clk_src = data & ~BIT_SLEEP;

  data = 0;
  if (!(sensors & INV_X_GYRO))
    data |= BIT_STBY_XG;
  if (!(sensors & INV_Y_GYRO))
    data |= BIT_STBY_YG;
  if (!(sensors & INV_Z_GYRO))
    data |= BIT_STBY_ZG;
  if (!(sensors & INV_XYZ_ACCEL))
    data |= BIT_STBY_XYZA;
  if (i2c_write(st.hw[type].addr, st.reg[type].pwr_mgmt_2, 1, &data, type))
  {
    st.chip_cfg[type].sensors = 0;
    return -1;
  }

  if (sensors && (sensors != INV_XYZ_ACCEL))
    /* Latched interrupts only used in LP accel mode. */
    mpu_set_int_latched(0, type);

  st.chip_cfg[type].sensors = sensors;
  st.chip_cfg[type].lp_accel_mode = 0;
  delay_ms(50);
  return 0;
}

/**
 *  @brief      Read the MPU interrupt status registers.
 *  @param[out] status  Mask of interrupt bits.
 *  @return     0 if successful.
 */
int mpu_get_int_status(short * status, int type)
{
  unsigned char tmp[2];
  if (!st.chip_cfg[type].sensors)
    return -1;
  if (i2c_read(st.hw[type].addr, st.reg[type].dmp_int_status, 2, tmp, type))
    return -1;
  status[0] = (tmp[0] << 8) | tmp[1];
  return 0;
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
int mpu_read_fifo(short * gyro, short * accel, unsigned long * timestamp,
                  unsigned char *sensors, unsigned char *more, int type)
{
  /* Assumes maximum packet size is gyro (6) + accel (6). */
  unsigned char data[MAX_PACKET_LENGTH];
  unsigned char packet_size = 0;
  unsigned short fifo_count, index = 0;

  if (st.chip_cfg[type].dmp_on)
    return -1;

  sensors[0] = 0;
  if (!st.chip_cfg[type].sensors)
    return -1;
  if (!st.chip_cfg[type].fifo_enable)
    return -1;

  if (st.chip_cfg[type].fifo_enable & INV_X_GYRO)
    packet_size += 2;
  if (st.chip_cfg[type].fifo_enable & INV_Y_GYRO)
    packet_size += 2;
  if (st.chip_cfg[type].fifo_enable & INV_Z_GYRO)
    packet_size += 2;
  if (st.chip_cfg[type].fifo_enable & INV_XYZ_ACCEL)
    packet_size += 6;

  if (i2c_read(st.hw[type].addr, st.reg[type].fifo_count_h, 2, data, type))
    return -1;
  fifo_count = (data[0] << 8) | data[1];
  if (fifo_count < packet_size)
    return 0;

  if (fifo_count > (st.hw[type].max_fifo >> 1))
  {
    /* FIFO is 50% full, better check overflow bit. */
    if (i2c_read(st.hw[type].addr, st.reg[type].int_status, 1, data, type))
      return -1;
    if (data[0] & BIT_FIFO_OVERFLOW)
    {
      mpu_reset_fifo(type);
      return -2;
    }
  }
  get_ms((unsigned long*)timestamp);

  if (i2c_read(st.hw[type].addr, st.reg[type].fifo_r_w, packet_size, data, type))
    return -1;
  more[0] = fifo_count / packet_size - 1;
  sensors[0] = 0;

  if ((index != packet_size) && st.chip_cfg[type].fifo_enable & INV_XYZ_ACCEL)
  {
    accel[0] = (data[index + 0] << 8) | data[index + 1];
    accel[1] = (data[index + 2] << 8) | data[index + 3];
    accel[2] = (data[index + 4] << 8) | data[index + 5];
    sensors[0] |= INV_XYZ_ACCEL;
    index += 6;
  }
  if ((index != packet_size) && st.chip_cfg[type].fifo_enable & INV_X_GYRO)
  {
    gyro[0] = (data[index + 0] << 8) | data[index + 1];
    sensors[0] |= INV_X_GYRO;
    index += 2;
  }
  if ((index != packet_size) && st.chip_cfg[type].fifo_enable & INV_Y_GYRO)
  {
    gyro[1] = (data[index + 0] << 8) | data[index + 1];
    sensors[0] |= INV_Y_GYRO;
    index += 2;
  }
  if ((index != packet_size) && st.chip_cfg[type].fifo_enable & INV_Z_GYRO)
  {
    gyro[2] = (data[index + 0] << 8) | data[index + 1];
    sensors[0] |= INV_Z_GYRO;
    index += 2;
  }

  return 0;
}

/**
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 */
int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
                         unsigned char *more, int type)
{
  unsigned char tmp[2];
  unsigned short fifo_count;
  if (!st.chip_cfg[type].dmp_on)
    return -1;
  if (!st.chip_cfg[type].sensors)
    return -1;

  if (i2c_read(st.hw[type].addr, st.reg[type].fifo_count_h, 2, tmp, type))
    return -1;
  fifo_count = (tmp[0] << 8) | tmp[1];
  if (fifo_count < length)
  {
    more[0] = 0;
    return -1;
  }
  if (fifo_count > (st.hw[type].max_fifo >> 1))
  {
    /* FIFO is 50% full, better check overflow bit. */
    if (i2c_read(st.hw[type].addr, st.reg[type].int_status, 1, tmp, type))
      return -1;
    if (tmp[0] & BIT_FIFO_OVERFLOW)
    {
      mpu_reset_fifo(type);
      return -2;
    }
  }

  if (i2c_read(st.hw[type].addr, st.reg[type].fifo_r_w, length, data, type))
    return -1;
  more[0] = fifo_count / length - 1;
  return 0;
}

/**
 *  @brief      Set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
int mpu_set_bypass(unsigned char bypass_on, int type)
{
  unsigned char tmp;

  if (st.chip_cfg[type].bypass_mode == bypass_on)
    return 0;

  if (bypass_on)
  {
    if (i2c_read(st.hw[type].addr, st.reg[type].user_ctrl, 1, &tmp, type))
      return -1;
    tmp &= ~BIT_AUX_IF_EN;
    if (i2c_write(st.hw[type].addr, st.reg[type].user_ctrl, 1, &tmp, type))
      return -1;
    delay_ms(3);
    tmp = BIT_BYPASS_EN;
    if (st.chip_cfg[type].active_low_int)
      tmp |= BIT_ACTL;
    if (st.chip_cfg[type].latched_int)
      tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
    if (i2c_write(st.hw[type].addr, st.reg[type].int_pin_cfg, 1, &tmp, type))
      return -1;
  }
  else
  {
    /* Enable I2C master mode if compass is being used. */
    if (i2c_read(st.hw[type].addr, st.reg[type].user_ctrl, 1, &tmp, type))
      return -1;
    if (st.chip_cfg[type].sensors & INV_XYZ_COMPASS)
      tmp |= BIT_AUX_IF_EN;
    else
      tmp &= ~BIT_AUX_IF_EN;
    if (i2c_write(st.hw[type].addr, st.reg[type].user_ctrl, 1, &tmp, type))
      return -1;
    delay_ms(3);
    if (st.chip_cfg[type].active_low_int)
      tmp = BIT_ACTL;
    else
      tmp = 0;
    if (st.chip_cfg[type].latched_int)
      tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
    if (i2c_write(st.hw[type].addr, st.reg[type].int_pin_cfg, 1, &tmp, type))
      return -1;
  }
  st.chip_cfg[type].bypass_mode = bypass_on;
  return 0;
}

/**
 *  @brief      Set interrupt level.
 *  @param[in]  active_low  1 for active low, 0 for active high.
 *  @return     0 if successful.
 */
int mpu_set_int_level(unsigned char active_low, int type)
{
  st.chip_cfg[type].active_low_int = active_low;
  return 0;
}

/**
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int mpu_set_int_latched(unsigned char enable, int type)
{
  unsigned char tmp;
  if (st.chip_cfg[type].latched_int == enable)
    return 0;

  if (enable)
    tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
  else
    tmp = 0;
  if (st.chip_cfg[type].bypass_mode)
    tmp |= BIT_BYPASS_EN;
  if (st.chip_cfg[type].active_low_int)
    tmp |= BIT_ACTL;
  if (i2c_write(st.hw[type].addr, st.reg[type].int_pin_cfg, 1, &tmp, type))
    return -1;
  st.chip_cfg[type].latched_int = enable;
  return 0;
}

static int get_accel_prod_shift(float * st_shift, int type)
{
  unsigned char tmp[4], shift_code[3], ii;

  if (i2c_read(st.hw[type].addr, 0x0D, 4, tmp, type))
    return 0x07;

  shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
  shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
  shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);
  for (ii = 0; ii < 3; ii++)
  {
    if (!shift_code[ii])
    {
      st_shift[ii] = 0.f;
      continue;
    }
    /* Equivalent to..
     * st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
     */
    st_shift[ii] = 0.34f;
    while (--shift_code[ii])
      st_shift[ii] *= 1.034f;
  }
  return 0;
}

static int accel_self_test(long * bias_regular, long * bias_st, int type)
{
  int jj, result = 0;
  float st_shift[3], st_shift_cust, st_shift_var;

  get_accel_prod_shift(st_shift, type);
  for(jj = 0; jj < 3; jj++)
  {
    st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
    if (st_shift[jj])
    {
      st_shift_var = st_shift_cust / st_shift[jj] - 1.f;
      if (fabs(st_shift_var) > test[type].max_accel_var)
        result |= 1 << jj;
    }
    else if ((st_shift_cust < test[type].min_g) ||
             (st_shift_cust > test[type].max_g))
      result |= 1 << jj;
  }

  return result;
}

static int gyro_self_test(long * bias_regular, long * bias_st, int type)
{
  int jj, result = 0;
  unsigned char tmp[3];
  float st_shift, st_shift_cust, st_shift_var;

  if (i2c_read(st.hw[type].addr, 0x0D, 3, tmp, type))
    return 0x07;

  tmp[0] &= 0x1F;
  tmp[1] &= 0x1F;
  tmp[2] &= 0x1F;

  for (jj = 0; jj < 3; jj++)
  {
    st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
    if (tmp[jj])
    {
      st_shift = 3275.f / test[type].gyro_sens;
      while (--tmp[jj])
        st_shift *= 1.046f;
      st_shift_var = st_shift_cust / st_shift - 1.f;
      if (fabs(st_shift_var) > test[type].max_gyro_var)
        result |= 1 << jj;
    }
    else if ((st_shift_cust < test[type].min_dps) ||
             (st_shift_cust > test[type].max_dps))
      result |= 1 << jj;
  }
  return result;
}

static int get_st_biases(long * gyro, long * accel, unsigned char hw_test, int type)
{
  unsigned char data[MAX_PACKET_LENGTH];
  unsigned char packet_count, ii;
  unsigned short fifo_count;

  data[0] = 0x01;
  data[1] = 0;
  if (i2c_write(st.hw[type].addr, st.reg[type].pwr_mgmt_1, 2, data, type))
    return -1;
  delay_ms(200);
  data[0] = 0;
  if (i2c_write(st.hw[type].addr, st.reg[type].int_enable, 1, data, type))
    return -1;
  if (i2c_write(st.hw[type].addr, st.reg[type].fifo_en, 1, data, type))
    return -1;
  if (i2c_write(st.hw[type].addr, st.reg[type].pwr_mgmt_1, 1, data, type))
    return -1;
  if (i2c_write(st.hw[type].addr, st.reg[type].i2c_mst, 1, data, type))
    return -1;
  if (i2c_write(st.hw[type].addr, st.reg[type].user_ctrl, 1, data, type))
    return -1;
  data[0] = BIT_FIFO_RST | BIT_DMP_RST;
  if (i2c_write(st.hw[type].addr, st.reg[type].user_ctrl, 1, data, type))
    return -1;
  delay_ms(15);
  data[0] = st.test[type].reg_lpf;
  if (i2c_write(st.hw[type].addr, st.reg[type].lpf, 1, data, type))
    return -1;
  data[0] = st.test[type].reg_rate_div;
  if (i2c_write(st.hw[type].addr, st.reg[type].rate_div, 1, data, type))
    return -1;
  if (hw_test)
    data[0] = st.test[type].reg_gyro_fsr | 0xE0;
  else
    data[0] = st.test[type].reg_gyro_fsr;
  if (i2c_write(st.hw[type].addr, st.reg[type].gyro_cfg, 1, data, type))
    return -1;

  if (hw_test)
    data[0] = st.test[type].reg_accel_fsr | 0xE0;
  else
    data[0] = test[type].reg_accel_fsr;
  if (i2c_write(st.hw[type].addr, st.reg[type].accel_cfg, 1, data, type))
    return -1;
  if (hw_test)
    delay_ms(200);

  /* Fill FIFO for test.wait_ms milliseconds. */
  data[0] = BIT_FIFO_EN;
  if (i2c_write(st.hw[type].addr, st.reg[type].user_ctrl, 1, data, type))
    return -1;

  data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
  if (i2c_write(st.hw[type].addr, st.reg[type].fifo_en, 1, data, type))
    return -1;
  delay_ms(test[type].wait_ms);
  data[0] = 0;
  if (i2c_write(st.hw[type].addr, st.reg[type].fifo_en, 1, data, type))
    return -1;

  if (i2c_read(st.hw[type].addr, st.reg[type].fifo_count_h, 2, data, type))
    return -1;

  fifo_count = (data[0] << 8) | data[1];
  packet_count = fifo_count / MAX_PACKET_LENGTH;
  gyro[0] = gyro[1] = gyro[2] = 0;
  accel[0] = accel[1] = accel[2] = 0;

  for (ii = 0; ii < packet_count; ii++)
  {
    short accel_cur[3], gyro_cur[3];
    if (i2c_read(st.hw[type].addr, st.reg[type].fifo_r_w, MAX_PACKET_LENGTH, data, type))
      return -1;
    accel_cur[0] = ((short)data[0] << 8) | data[1];
    accel_cur[1] = ((short)data[2] << 8) | data[3];
    accel_cur[2] = ((short)data[4] << 8) | data[5];
    accel[0] += (long)accel_cur[0];
    accel[1] += (long)accel_cur[1];
    accel[2] += (long)accel_cur[2];
    gyro_cur[0] = (((short)data[6] << 8) | data[7]);
    gyro_cur[1] = (((short)data[8] << 8) | data[9]);
    gyro_cur[2] = (((short)data[10] << 8) | data[11]);
    gyro[0] += (long)gyro_cur[0];
    gyro[1] += (long)gyro_cur[1];
    gyro[2] += (long)gyro_cur[2];
  }
#ifdef EMPL_NO_64BIT
  gyro[0] = (long)(((float)gyro[0] * 65536.f) / test[type].gyro_sens / packet_count);
  gyro[1] = (long)(((float)gyro[1] * 65536.f) / test[type].gyro_sens / packet_count);
  gyro[2] = (long)(((float)gyro[2] * 65536.f) / test[type].gyro_sens / packet_count);
  if (has_accel)
  {
    accel[0] = (long)(((float)accel[0] * 65536.f) / test[type].accel_sens /
                      packet_count);
    accel[1] = (long)(((float)accel[1] * 65536.f) / test[type].accel_sens /
                      packet_count);
    accel[2] = (long)(((float)accel[2] * 65536.f) / test[type].accel_sens /
                      packet_count);
    /* Don't remove gravity! */
    accel[2] -= 65536L;
  }
#else
  gyro[0] = (long)(((long long)gyro[0] << 16) / test[type].gyro_sens / packet_count);
  gyro[1] = (long)(((long long)gyro[1] << 16) / test[type].gyro_sens / packet_count);
  gyro[2] = (long)(((long long)gyro[2] << 16) / test[type].gyro_sens / packet_count);
  accel[0] = (long)(((long long)accel[0] << 16) / test[type].accel_sens /
                    packet_count);
  accel[1] = (long)(((long long)accel[1] << 16) / test[type].accel_sens /
                    packet_count);
  accel[2] = (long)(((long long)accel[2] << 16) / test[type].accel_sens /
                    packet_count);
  /* Don't remove gravity! */
  if (accel[2] > 0L)
    accel[2] -= 65536L;
  else
    accel[2] += 65536L;
#endif

  return 0;
}

/**
 *  @brief      Trigger gyro/accel/compass self-test.
 *  On success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. For each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n The mask is defined as follows:
 *  \n Bit 0:   Gyro.
 *  \n Bit 1:   Accel.
 *  \n Bit 2:   Compass.
 *
 *  \n Currently, the hardware self-test is unsupported for MPU6500. However,
 *  this function can still be used to obtain the accel and gyro biases.
 *
 *  \n This function must be called with the device either face-up or face-down
 *  (z-axis is parallel to gravity).
 *  @param[out] gyro        Gyro biases in q16 format.
 *  @param[out] accel       Accel biases (if applicable) in q16 format.
 *  @return     Result mask (see above).
 */
int mpu_run_self_test(long * gyro, long * accel, int type)
{
  const unsigned char tries = 2;
  long gyro_st[3], accel_st[3];
  unsigned char accel_result, gyro_result;
  int ii;
  int result;
  unsigned char accel_fsr, fifo_sensors, sensors_on;
  unsigned short gyro_fsr, sample_rate, lpf;
  unsigned char dmp_was_on;

  if (st.chip_cfg[type].dmp_on)
  {
    mpu_set_dmp_state(0, type);
    dmp_was_on = 1;
  }
  else
    dmp_was_on = 0;

  /* Get initial settings. */
  mpu_get_gyro_fsr(&gyro_fsr, type);
  mpu_get_accel_fsr(&accel_fsr, type);
  mpu_get_lpf(&lpf, type);
  mpu_get_sample_rate(&sample_rate, type);
  sensors_on = st.chip_cfg[type].sensors;
  mpu_get_fifo_config(&fifo_sensors, type);

  /* For older chips, the self-test will be different. */
  for (ii = 0; ii < tries; ii++)
    if (!get_st_biases(gyro, accel, 0, type))
      break;
  if (ii == tries)
  {
    /* If we reach this point, we most likely encountered an I2C error.
     * We'll just report an error for all three sensors.
     */
    result = 0;
    goto restore;
  }
  for (ii = 0; ii < tries; ii++)
    if (!get_st_biases(gyro_st, accel_st, 1, type))
      break;
  if (ii == tries)
  {
    /* Again, probably an I2C error. */
    result = 0;
    goto restore;
  }
  accel_result = accel_self_test(accel, accel_st, type);
  gyro_result = gyro_self_test(gyro, gyro_st, type);

  result = 0;
  if (!gyro_result)
    result |= 0x01;
  if (!accel_result)
    result |= 0x02;

restore:

  /* Set to invalid values to ensure no I2C writes are skipped. */
  st.chip_cfg[type].gyro_fsr = 0xFF;
  st.chip_cfg[type].accel_fsr = 0xFF;
  st.chip_cfg[type].lpf = 0xFF;
  st.chip_cfg[type].sample_rate = 0xFFFF;
  st.chip_cfg[type].sensors = 0xFF;
  st.chip_cfg[type].fifo_enable = 0xFF;
  st.chip_cfg[type].clk_src = INV_CLK_PLL;

  mpu_set_gyro_fsr(gyro_fsr, type);
  mpu_set_accel_fsr(accel_fsr, type);
  mpu_set_lpf(lpf, type);
  mpu_set_sample_rate(sample_rate, type);
  mpu_set_sensors(sensors_on, type);
  mpu_configure_fifo(fifo_sensors, type);

  if (dmp_was_on)
    mpu_set_dmp_state(1, type);

  return result;
}

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
int mpu_write_mem(unsigned short mem_addr, unsigned short length,
                  unsigned char *data, int type)
{
  unsigned char tmp[2];

  if (!data)
    return -1;
  if (!st.chip_cfg[type].sensors)
    return -1;

  tmp[0] = (unsigned char)(mem_addr >> 8);
  tmp[1] = (unsigned char)(mem_addr & 0xFF);

  /* Check bank boundaries. */
  if (tmp[1] + length > st.hw[type].bank_size)
    return -1;

  if (i2c_write(st.hw[type].addr, st.reg[type].bank_sel, 2, tmp, type))
    return -1;
  if (i2c_write(st.hw[type].addr, st.reg[type].mem_r_w, length, data, type))
    return -1;
  return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
int mpu_read_mem(unsigned short mem_addr, unsigned short length,
                 unsigned char *data, int type)
{
  unsigned char tmp[2];

  if (!data)
    return -1;
  if (!st.chip_cfg[type].sensors)
    return -1;

  tmp[0] = (unsigned char)(mem_addr >> 8);
  tmp[1] = (unsigned char)(mem_addr & 0xFF);

  /* Check bank boundaries. */
  if (tmp[1] + length > st.hw[type].bank_size)
    return -1;

  if (i2c_write(st.hw[type].addr, st.reg[type].bank_sel, 2, tmp, type))
    return -1;
  if (i2c_read(st.hw[type].addr, st.reg[type].mem_r_w, length, data, type))
    return -1;
  return 0;
}

/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
                      unsigned short start_addr, unsigned short sample_rate, int type)
{
  unsigned short ii;
  unsigned short this_write;
  /* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
#define LOAD_CHUNK  (16)
  unsigned char cur[LOAD_CHUNK], tmp[2];

  if (st.chip_cfg[type].dmp_loaded)
    /* DMP should only be loaded once. */
    return -1;

  if (!firmware)
    return -1;
  for (ii = 0; ii < length; ii += this_write)
  {
    this_write = min(LOAD_CHUNK, length - ii);
    if (mpu_write_mem(ii, this_write, (unsigned char*)&firmware[ii], type))
      return -1;
    if (mpu_read_mem(ii, this_write, cur, type))
      return -1;
    if (memcmp(firmware + ii, cur, this_write))
      return -2;
  }

  /* Set program start address. */
  tmp[0] = start_addr >> 8;
  tmp[1] = start_addr & 0xFF;
  if (i2c_write(st.hw[type].addr, st.reg[type].prgm_start_h, 2, tmp, type))
    return -1;

  st.chip_cfg[type].dmp_loaded = 1;
  st.chip_cfg[type].dmp_sample_rate = sample_rate;
  return 0;
}

/**
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
 */
int mpu_set_dmp_state(unsigned char enable, int type)
{
  unsigned char tmp;
  if (st.chip_cfg[type].dmp_on == enable)
    return 0;

  if (enable)
  {
    if (!st.chip_cfg[type].dmp_loaded)
      return -1;
    /* Disable data ready interrupt. */
    set_int_enable(0, type);
    /* Disable bypass mode. */
    mpu_set_bypass(0, type);
    /* Keep constant sample rate, FIFO rate controlled by DMP. */
    mpu_set_sample_rate(st.chip_cfg[type].dmp_sample_rate, type);
    /* Remove FIFO elements. */
    tmp = 0;
    i2c_write(st.hw[type].addr, 0x23, 1, &tmp, type);
    st.chip_cfg[type].dmp_on = 1;
    /* Enable DMP interrupt. */
    set_int_enable(1, type);
    mpu_reset_fifo(type);
  }
  else
  {
    /* Disable DMP interrupt. */
    set_int_enable(0, type);
    /* Restore FIFO settings. */
    tmp = st.chip_cfg[type].fifo_enable;
    i2c_write(st.hw[type].addr, 0x23, 1, &tmp, type);
    st.chip_cfg[type].dmp_on = 0;
    mpu_reset_fifo(type);
  }
  return 0;
}

/**
 *  @brief      Get DMP state.
 *  @param[out] enabled 1 if enabled.
 *  @return     0 if successful.
 */
int mpu_get_dmp_state(unsigned char *enabled, int type)
{
  enabled[0] = st.chip_cfg[type].dmp_on;
  return 0;
}


/* This initialization is similar to the one in ak8975.c. */
int setup_compass(int type)
{
  return -1;
}

/**
 *  @brief      Read raw compass data.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_compass_reg(short * data, unsigned long * timestamp, int type)
{
  return -1;
}

/**
 *  @brief      Get the compass full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_compass_fsr(unsigned short * fsr, int type)
{
  return -1;
}

/**
 *  @brief      Enters LP accel motion interrupt mode.
 *  The behavior of this feature is very different between the MPU6050 and the
 *  MPU6500. Each chip's version of this feature is explained below.
 *
 *  \n MPU6050:
 *  \n When this mode is first enabled, the hardware captures a single accel
 *  sample, and subsequent samples are compared with this one to determine if
 *  the device is in motion. Therefore, whenever this "locked" sample needs to
 *  be changed, this function must be called again.
 *
 *  \n The hardware motion threshold can be between 32mg and 8160mg in 32mg
 *  increments.
 *
 *  \n Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 5Hz, 20Hz, 40Hz
 *
 *  \n MPU6500:
 *  \n Unlike the MPU6050 version, the hardware does not "lock in" a reference
 *  sample. The hardware monitors the accel data and detects any large change
 *  over a short period of time.
 *
 *  \n The hardware motion threshold can be between 4mg and 1020mg in 4mg
 *  increments.
 *
 *  \n MPU6500 Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *
 *  \n\n NOTES:
 *  \n The driver will round down @e thresh to the nearest supported value if
 *  an unsupported threshold is selected.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e lpa_freq.
 *  \n The MPU6500 does not support a delay parameter. If this function is used
 *  for the MPU6500, the value passed to @e time will be ignored.
 *  \n To disable this mode, set @e lpa_freq to zero. The driver will restore
 *  the previous configuration.
 *
 *  @param[in]  thresh      Motion threshold in mg.
 *  @param[in]  time        Duration in milliseconds that the accel data must
 *                          exceed @e thresh before motion is reported.
 *  @param[in]  lpa_freq    Minimum sampling rate, or zero to disable.
 *  @return     0 if successful.
 */
int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,
                            unsigned char lpa_freq, int type)
{
  unsigned char data[3];

  if (lpa_freq)
  {
    unsigned char thresh_hw;

    /* TODO: Make these const/#defines. */
    /* 1LSb = 32mg. */
    if (thresh > 8160)
      thresh_hw = 255;
    else if (thresh < 32)
      thresh_hw = 1;
    else
      thresh_hw = thresh >> 5;

    if (!time)
      /* Minimum duration must be 1ms. */
      time = 1;

    if (lpa_freq > 40)
      /* At this point, the chip has not been re-configured, so the
       * function can safely exit.
       */
      return -1;

    if (!st.chip_cfg[type].int_motion_only)
    {
      /* Store current settings for later. */
      if (st.chip_cfg[type].dmp_on)
      {
        mpu_set_dmp_state(0, type);
        st.chip_cfg[type].cache.dmp_on = 1;
      }
      else
        st.chip_cfg[type].cache.dmp_on = 0;
      mpu_get_gyro_fsr(&st.chip_cfg[type].cache.gyro_fsr, type);
      mpu_get_accel_fsr(&st.chip_cfg[type].cache.accel_fsr, type);
      mpu_get_lpf(&st.chip_cfg[type].cache.lpf, type);
      mpu_get_sample_rate(&st.chip_cfg[type].cache.sample_rate, type);
      st.chip_cfg[type].cache.sensors_on = st.chip_cfg[type].sensors;
      mpu_get_fifo_config(&st.chip_cfg[type].cache.fifo_sensors, type);
    }

    /* Disable hardware interrupts for now. */
    set_int_enable(0, type);

    /* Enter full-power accel-only mode. */
    mpu_lp_accel_mode(0, type);

    /* Override current LPF (and HPF) settings to obtain a valid accel
     * reading.
     */
    data[0] = INV_FILTER_256HZ_NOLPF2;
    if (i2c_write(st.hw[type].addr, st.reg[type].lpf, 1, data, type))
      return -1;

    /* NOTE: Digital high pass filter should be configured here. Since this
     * driver doesn't modify those bits anywhere, they should already be
     * cleared by default.
     */

    /* Configure the device to send motion interrupts. */
    /* Enable motion interrupt. */
    data[0] = BIT_MOT_INT_EN;
    if (i2c_write(st.hw[type].addr, st.reg[type].int_enable, 1, data, type))
      goto lp_int_restore;

    /* Set motion interrupt parameters. */
    data[0] = thresh_hw;
    data[1] = time;
    if (i2c_write(st.hw[type].addr, st.reg[type].motion_thr, 2, data, type))
      goto lp_int_restore;

    /* Force hardware to "lock" current accel sample. */
    delay_ms(5);
    data[0] = (st.chip_cfg[type].accel_fsr << 3) | BITS_HPF;
    if (i2c_write(st.hw[type].addr, st.reg[type].accel_cfg, 1, data, type))
      goto lp_int_restore;

    /* Set up LP accel mode. */
    data[0] = BIT_LPA_CYCLE;
    if (lpa_freq == 1)
      data[1] = INV_LPA_1_25HZ;
    else if (lpa_freq <= 5)
      data[1] = INV_LPA_5HZ;
    else if (lpa_freq <= 20)
      data[1] = INV_LPA_20HZ;
    else
      data[1] = INV_LPA_40HZ;
    data[1] = (data[1] << 6) | BIT_STBY_XYZG;
    if (i2c_write(st.hw[type].addr, st.reg[type].pwr_mgmt_1, 2, data, type))
      goto lp_int_restore;

    st.chip_cfg[type].int_motion_only = 1;
    return 0;
  }
  else
  {
    /* Don't "restore" the previous state if no state has been saved. */
    int ii;
    char *cache_ptr = (char*)&st.chip_cfg[type].cache;
    for (ii = 0; ii < sizeof(st.chip_cfg[type].cache); ii++)
    {
      if (cache_ptr[ii] != 0)
        goto lp_int_restore;
    }
    /* If we reach this point, motion interrupt mode hasn't been used yet. */
    return -1;
  }
lp_int_restore:
  /* Set to invalid values to ensure no I2C writes are skipped. */
  st.chip_cfg[type].gyro_fsr = 0xFF;
  st.chip_cfg[type].accel_fsr = 0xFF;
  st.chip_cfg[type].lpf = 0xFF;
  st.chip_cfg[type].sample_rate = 0xFFFF;
  st.chip_cfg[type].sensors = 0xFF;
  st.chip_cfg[type].fifo_enable = 0xFF;
  st.chip_cfg[type].clk_src = INV_CLK_PLL;
  mpu_set_sensors(st.chip_cfg[type].cache.sensors_on, type);
  mpu_set_gyro_fsr(st.chip_cfg[type].cache.gyro_fsr, type);
  mpu_set_accel_fsr(st.chip_cfg[type].cache.accel_fsr, type);
  mpu_set_lpf(st.chip_cfg[type].cache.lpf, type);
  mpu_set_sample_rate(st.chip_cfg[type].cache.sample_rate, type);
  mpu_configure_fifo(st.chip_cfg[type].cache.fifo_sensors, type);

  if (st.chip_cfg[type].cache.dmp_on)
    mpu_set_dmp_state(1, type);

  st.chip_cfg[type].int_motion_only = 0;
  return 0;
}

#define q30  1073741824.0f

static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1
                                         };

u8 run_self_test(int type)
{
  int result;
  long gyro[3], accel[3];
  result = mpu_run_self_test(gyro, accel, type);
  if (result == 0x3)
  {
    /* Test passed. We can trust the gyro data here, so let's push it down
    * to the DMP.
    */
    float sens;
    unsigned short accel_sens;
    mpu_get_gyro_sens(&sens, type);
    gyro[0] = (long)(gyro[0] * sens);
    gyro[1] = (long)(gyro[1] * sens);
    gyro[2] = (long)(gyro[2] * sens);
    dmp_set_gyro_bias(gyro, type);
    mpu_get_accel_sens(&accel_sens, type);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel, type);
    return 0;
  }
  else return 1;
}

unsigned short inv_orientation_matrix_to_scalar(
  const signed char *mtx)
{
  unsigned short scalar;
  /*
     XYZ  010_001_000 Identity Matrix
     XZY  001_010_000
     YXZ  010_000_001
     YZX  000_010_001
     ZXY  001_000_010
     ZYX  000_001_010
   */

  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;


  return scalar;
}

unsigned short inv_row_2_scale(const signed char *row)
{
  unsigned short b;

  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7;      // error
  return b;
}

void mget_ms(unsigned long * time)
{

}

u8 mpu_dmp_init(int type)
{
  u8 res = 0;
  MPU_IIC_Init(type);
  if(mpu_init(type) == 0)
  {
    res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL, type);
    if(res)return 1;
    res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL, type);
    if(res)return 2;
    res = mpu_set_sample_rate(DEFAULT_MPU_HZ, type);
    if(res)return 3;
    res = dmp_load_motion_driver_firmware(type);
    if(res)return 4;
    res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation), type);
    if(res)return 5;
    res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                             DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                             DMP_FEATURE_GYRO_CAL, type);
    if(res)return 6;
    res = dmp_set_fifo_rate(DEFAULT_MPU_HZ, type);
    if(res)return 7;
    res = run_self_test(type);
    if(res)return 8;
    res = mpu_set_dmp_state(1, type);
    if(res)return 9;
  }
  else return 10;
  return 0;
}

u8 mpu_dmp_get_data(float * pitch, float * roll, float * yaw, int type)
{
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  unsigned long sensor_timestamp;
  short gyro[3], accel[3], sensors;
  unsigned char more;
  long quat[4];
  if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more, type))return 1;
  /* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
   * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
  **/
  /*if (sensors & INV_XYZ_GYRO )
  send_packet(PACKET_TYPE_GYRO, gyro);
  if (sensors & INV_XYZ_ACCEL)
  send_packet(PACKET_TYPE_ACCEL, accel); */
  /* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
   * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
  **/
  if(sensors & INV_WXYZ_QUAT)
  {
    q0 = quat[0] / q30;
    q1 = quat[1] / q30;
    q2 = quat[2] / q30;
    q3 = quat[3] / q30;

    *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;	// pitch
    *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, (-2) * q1 * q1 + (- 2) * q2 * q2 + 1) * 57.3;	// roll
    *yaw   = atan2(2 * (q1 * q2 + q0 * q3), (-2) * q2 * q2 + (-2) * q3 * q3 + 1) * 57.3;	//yaw
  }
  else return 2;
  return 0;
}

