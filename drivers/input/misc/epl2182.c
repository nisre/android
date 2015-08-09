/* drivers/i2c/chips/epl2182.c - light and proxmity sensors driver
 * Copyright (C) 2011 ELAN Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <linux/delay.h>
//#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/input/elan_interface.h>
#include <linux/of_gpio.h>
/******************************************************************************
 * configuration
*******************************************************************************/
#define FT_VTG_MIN_UV           2600000
#define FT_VTG_MAX_UV           3300000
#define FT_I2C_VTG_MIN_UV       1800000
#define FT_I2C_VTG_MAX_UV       1800000

#define PS_INTERRUPT_MODE		1		// 0 is polling mode, 1 is interrupt mode
//#define P_SENSOR_LTHD			104		//100
//#define P_SENSOR_HTHD			116		//500
#define P_SENSOR_LTHD                 500             //100
#define P_SENSOR_HTHD                 850             //500

#define DYNAMIC_INTT    1   //1 is enable, 0 is disable

#define LUX_PER_COUNT			440		// 660 = 1.1*0.6*1000
/******************************************************************************
 * configuration
*******************************************************************************/
typedef enum
{
    CMC_MODE_ALS    = 0x00,
    CMC_MODE_PS     = 0x10,
} CMC_MODE;

#define TXBYTES				2
#define RXBYTES				2

#define PACKAGE_SIZE		2
#define I2C_RETRY_COUNT		10
static struct kobject *alsps_kobj;

#define PS_INTT				4 //ELAN Robert modify ePL2182 ps integration time at 2014/5/19
//#define PS_INTT				4 //ELAN Robert modify ePL21822 ps integration time at 2014/5/19

#define ALS_INTT			6	//5-8

static struct mutex sensor_mutex;

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);

#if PS_INTERRUPT_MODE
//#define EPL_INTR_PIN xx
static void epl_sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(epl_sensor_irq_work, epl_sensor_irq_do_work);
#endif

static void polling_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);


#if DYNAMIC_INTT
// TODO: change delay time
#define PS_DELAY 			50
#define ALS_DELAY 			100

static bool irq_caused_by_power_off = false;
int dynamic_intt_idx;
int dynamic_intt_init_idx = 2;	//initial dynamic_intt_idx

//int c_gain = 100; // 10.0/1000 *10000 //default setting
//int c_gain = 37; //ELAN Robert modify ePL2182 lux gain at 2014/5/19
int c_gain = 15; //ELAN Robert modify ePL21822 lux gain at 2014/5/19

uint8_t dynamic_intt_intt;
uint8_t dynamic_intt_gain;
uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;
uint32_t dynamic_intt_max_lux = 8700; //87000;  //8700
uint32_t dynamic_intt_min_lux = 0;      //0.5
uint32_t dynamic_intt_min_unit = 1000;  //1000: float, 10000:integer

static int als_dynamic_intt_intt[] = {EPL_ALS_INTT_4096, EPL_ALS_INTT_256, EPL_ALS_INTT_16, EPL_ALS_INTT_16};
static int als_dynamic_intt_intt_value[] = {4096, 256, 16, 16};
static int als_dynamic_intt_gain[] = {EPL_M_GAIN, EPL_M_GAIN, EPL_M_GAIN, EPL_L_GAIN};
//static int als_dynamic_intt_high_thr[] = {900, 800, 400, 3200};
//static int als_dynamic_intt_low_thr[] = {32, 32, 32, 256};
static int als_dynamic_intt_high_thr[] = {7200, 6400, 3200, 3200*8};
static int als_dynamic_intt_low_thr[] = {320, 320, 320, 320*8};

static int als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_intt_value)/sizeof(int);

#else

#define PS_DELAY			50
#define ALS_DELAY			55

#endif


/* primitive raw data from I2C */
typedef struct _epl_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
    u16 ps_state;
    u16 ps_int_state;
    u16 ps_ch1_raw;
    u16 als_ch1_raw;
    u16 als_lux;
} epl_raw_data;

struct elan_epl_data
{
    struct i2c_client *client;
    struct regulator *vdd;
    struct regulator *vcc_i2c;
    struct input_dev *als_input_dev;
    struct input_dev *ps_input_dev;
    struct workqueue_struct *epl_wq;
//    struct early_suspend early_suspend;  zhujp modify

    int intr_pin;
    int (*power)(int on);

    int ps_opened;
    int als_opened;

    int enable_pflag;
    int enable_lflag;
    int read_flag;
    int irq;

    int ps_threshold_high;
    int ps_threshold_low;
} ;

static DECLARE_WAIT_QUEUE_HEAD(ps_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(ls_waitqueue);


static int ps_data_changed;
static int ls_data_changed;


static struct wake_lock g_ps_wlock;
struct elan_epl_data *epl_data;
static epl_raw_data	gRawData;

static const char ElanPsensorName[] = "proximity";
static const char ElanALsensorName[] = "light";

static int psensor_mode_suspend = 0;

#define LOG_TAG				        "[EPL2182] "
#define LOG_FUN(f)			        printk(KERN_INFO LOG_TAG"%s\n", __FUNCTION__)
#define LOG_INFO(fmt, args...)		printk(KERN_INFO LOG_TAG fmt, ##args)
#define LOG_ERR(fmt, args...)		printk(KERN_ERR  LOG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

/*
//====================I2C write operation===============//
//regaddr: ELAN Register Address.
//bytecount: How many bytes to be written to register via i2c bus.
//txbyte: I2C bus transmit byte(s). Single byte(0X01) transmit only slave address.
//data: setting value.
//
// Example: If you want to write single byte to 0x1D register address, show below
//	      elan_sensor_I2C_Write(client,0x1D,0x01,0X02,0xff);
//
*/
static int elan_sensor_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry,val;
    struct elan_epl_data *epld = epl_data;

    buffer[0] = (regaddr<<3) | bytecount ;
    buffer[1] = data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        val = gpio_get_value(epld->intr_pin);

        LOG_INFO("INTERRUPT GPIO val = %d\n",  val);

        msleep(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
        LOG_ERR(KERN_ERR "i2c write retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    return ret;
}

static int elan_sensor_I2C_Read(struct i2c_client *client)
{
    uint8_t buffer[RXBYTES];
    int ret = 0, i =0;
    int retry,val;
    struct elan_epl_data *epld = epl_data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_recv(client, buffer, RXBYTES);

        if (ret == RXBYTES)
            break;

        val = gpio_get_value(epld->intr_pin);

        LOG_INFO("INTERRUPT GPIO val = %d\n", val);

        msleep(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
        LOG_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    for(i=0; i<PACKAGE_SIZE; i++)
    {
        gRawData.raw_bytes[i] = buffer[i];
    }

    return ret;
}

static void elan_sensor_restart_work(void)
{
    struct elan_epl_data *epld = epl_data;
 //   LOG_FUN();
    cancel_delayed_work(&polling_work);
    queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(10));
}
int init_ps_flag = 0;
int ps_intr_flag = 0;
static void elan_epl_ps_poll_rawdata(void);
static int elan_sensor_psensor_enable(struct elan_epl_data *epld)
{
    int ret;
    uint8_t regdata = 0;
    struct i2c_client *client = epld->client;

 //   LOG_INFO("--- Proximity sensor Enable --- \n");

    //disable_irq(epld->irq);
    ret = elan_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);

    regdata = EPL_SENSING_8_TIME | EPL_PS_MODE | EPL_M_GAIN ;
    regdata = regdata | (PS_INTERRUPT_MODE ? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE);
    ret = elan_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

    regdata = PS_INTT<<4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
    ret = elan_sensor_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

    //set_psensor_intr_threshold(P_SENSOR_LTHD ,P_SENSOR_HTHD);
    set_psensor_intr_threshold(epld->ps_threshold_low ,epld->ps_threshold_high);

    ret = elan_sensor_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
    ret = elan_sensor_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
    msleep(PS_DELAY);

#if PS_INTERRUPT_MODE
    //if(epld->enable_lflag)
    if(PS_INTERRUPT_MODE == 1)
    {
 elan_sensor_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
        elan_sensor_I2C_Read(client);
        gRawData.ps_ch1_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

        elan_sensor_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
        elan_sensor_I2C_Read(client);
        gRawData.ps_state= !((gRawData.raw_bytes[0]&0x04)>>2);

        if(init_ps_flag == 0 || (gRawData.ps_state!= gRawData.ps_int_state))
        {
            init_ps_flag = 1;
            regdata = EPL_SENSING_8_TIME | EPL_PS_MODE | EPL_M_GAIN ;
            regdata = regdata | EPL_S_SENSING_MODE;
            ret = elan_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

            elan_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_FRAME_ENABLE);
            LOG_INFO("%s: >>>>>>EPL_INT_FRAME_ENABLE\r\n", __func__);
            ps_intr_flag = 1;
           // gRawData.ps_int_state = gRawData.ps_state;
            elan_epl_ps_poll_rawdata();
           // elan_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW);
           // LOG_INFO("%s: >>>>>>Direct report ps status\r\n", __func__);

        }
        else
        {
            elan_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW);
//            LOG_INFO("%s: >>>>>>>EPL_INT_ACTIVE_LOW\r\n", __func__);
        }

    }
    else
    {
        elan_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW);
    }
    //enable_irq(epld->irq);
#endif

    if (ret != 0x02)
    {
        LOG_INFO("P-sensor i2c err\n");
    }

    return ret;
}

static int elan_sensor_lsensor_enable(struct elan_epl_data *epld)
{
    int ret;
    uint8_t regdata = 0;
    struct i2c_client *client = epld->client;

   // LOG_INFO("--- ALS sensor Enable --- \n");

    //disable_irq(epld->irq);
    regdata = EPL_INT_DISABLE;
    ret = elan_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02, regdata);

#if DYNAMIC_INTT /*DYNAMIC_INTT*/

    regdata = EPL_S_SENSING_MODE | EPL_SENSING_16_TIME | EPL_ALS_MODE | dynamic_intt_gain;
    ret = elan_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

    regdata = dynamic_intt_intt<<4 | EPL_PST_1_TIME | EPL_8BIT_ADC;
    ret = elan_sensor_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

#else
    regdata = EPL_S_SENSING_MODE | EPL_SENSING_8_TIME | EPL_ALS_MODE | EPL_AUTO_GAIN;
    ret = elan_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

    regdata = ALS_INTT<<4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
    ret = elan_sensor_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);
#endif

    ret = elan_sensor_I2C_Write(client,REG_10,W_SINGLE_BYTE,0x02, EPL_GO_MID);
    ret = elan_sensor_I2C_Write(client,REG_11,W_SINGLE_BYTE,0x02, EPL_GO_LOW);

    ret = elan_sensor_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
    ret = elan_sensor_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);

    msleep(ALS_DELAY);

    if(ret != 0x02)
    {
        LOG_INFO(" ALS-sensor i2c err\n");
    }

    return ret;

}

/*
//====================elan_epl_ps_poll_rawdata===============//
//polling method for proximity sensor detect. Report proximity sensor raw data.
//Report "ABS_DISTANCE" event to HAL layer.
//Variable "value" 0 and 1 to represent which distance from psensor to target(human's face..etc).
//value: 0 represent near.
//value: 1 represent far.
*/
static void elan_epl_ps_poll_rawdata(void)
{
    struct elan_epl_data *epld = epl_data;
    struct i2c_client *client = epld->client;

    elan_sensor_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_LOCK);

    elan_sensor_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
    elan_sensor_I2C_Read(client);
    gRawData.ps_state= !((gRawData.raw_bytes[0]&0x04)>>2);

    elan_sensor_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
    elan_sensor_I2C_Read(client);
    gRawData.ps_ch1_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    elan_sensor_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);

    LOG_INFO("### ps_ch1_raw_data  (%d), value(%d) ###\n\n", gRawData.ps_ch1_raw, gRawData.ps_state);
	ps_data_changed = 1;
    input_report_abs(epld->ps_input_dev, ABS_DISTANCE, gRawData.ps_state);
    input_sync(epld->ps_input_dev);
    epld->ps_input_dev->absinfo[ABS_DISTANCE].value = -1;
}


#if DYNAMIC_INTT ///*DYNAMIC_INTT*/
long raw_convert_to_lux(u16 raw_data)
{
    long lux = 0;

    /* lux = C*count*(4096/intt)*(64/Gain), Gain:H=64, M=8, L=1 */
#if 0
    if(dynamic_intt_gain == EPL_H_GAIN){
        lux = c_gain * raw_data * (4096 / als_dynamic_intt_intt_value[dynamic_intt_idx]) * (64 / 64);
    }
    else if(dynamic_intt_gain == EPL_M_GAIN){
        lux = c_gain * raw_data * (4096 / als_dynamic_intt_intt_value[dynamic_intt_idx]) * (64 / 8);
    }
    else if(dynamic_intt_gain == EPL_L_GAIN){
        lux = c_gain * raw_data * (4096 / als_dynamic_intt_intt_value[dynamic_intt_idx]) * (64 / 1);
    }
#else/* lux = C*count*(4096/intt) for epl2182*/
    lux = c_gain * raw_data * (4096 / als_dynamic_intt_intt_value[dynamic_intt_idx]);
    if(lux >= (dynamic_intt_max_lux*1000)){
        LOG_INFO("raw_convert_to_lux: change max lux\r\n");
        lux = dynamic_intt_max_lux * 1000;
    }
    else if(lux <= (dynamic_intt_min_lux*1000)){
        LOG_INFO("raw_convert_to_lux: change min lux\r\n");
        lux = dynamic_intt_min_lux * 1000;
    }
#endif
    return lux;
}
#endif /*DYNAMIC_INTT*/


static void elan_epl_als_rawdata(void)
{
    struct elan_epl_data *epld = epl_data;
    struct i2c_client *client = epld->client;

#if DYNAMIC_INTT /*DYNAMIC_INTT*/
	uint32_t now_lux, lux_tmp;
	u16 raw;
    int als_com = 0;

    elan_sensor_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
    elan_sensor_I2C_Read(client);
    raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    elan_sensor_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
    elan_sensor_I2C_Read(client);
    als_com = (gRawData.raw_bytes[0]&0x04)>>2;
    //LOG_INFO("dynamic_intt_idx=%d, als_dynamic_intt_intt_value=%d, dynamic_intt_gain=%d, als_raw=%d, als_com=%d\r\n",
    //                                            dynamic_intt_idx, als_dynamic_intt_intt_value[dynamic_intt_idx], dynamic_intt_gain, raw, als_com);

    if(als_com == 1){
        if(dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)){
            lux_tmp = dynamic_intt_max_lux * 1000;
        }
        else{
            raw  = dynamic_intt_high_thr;
			gRawData.als_ch1_raw = dynamic_intt_high_thr;
      		lux_tmp = raw_convert_to_lux(raw);
            dynamic_intt_idx++;
        }
 //       LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>> channel output has saturated! \r\n");

    }
    else{
        if(raw > dynamic_intt_high_thr)
    	{
      		if(dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)){
    	      	lux_tmp = dynamic_intt_max_lux * 1000;
//    	      	LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MAX_LUX\r\n");
      		}
            else{
				raw  = dynamic_intt_high_thr;
				gRawData.als_ch1_raw = dynamic_intt_high_thr;
          		lux_tmp = raw_convert_to_lux(raw);
                dynamic_intt_idx++;
//                LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>change INTT high: %d, raw: %d \r\n", dynamic_intt_idx, raw);
            }
        }
        else if(raw < dynamic_intt_low_thr)
        {
            if(dynamic_intt_idx == 0){
                lux_tmp = dynamic_intt_min_lux * 1000;
 //               LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MIN_LUX\r\n");
            }
            else{
				raw  = dynamic_intt_low_thr;
				gRawData.als_ch1_raw = dynamic_intt_low_thr;
            	lux_tmp = raw_convert_to_lux(raw);
                dynamic_intt_idx--;
 //               LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>change INTT low: %d, raw: %d \r\n", dynamic_intt_idx, raw);
            }
        }
        else
        {
        	lux_tmp = raw_convert_to_lux(raw);
        }
    }

    now_lux = lux_tmp / dynamic_intt_min_unit;
    gRawData.als_lux = now_lux;

    //now_lux = raw_convert_to_lux(gRawData.als_ch1_raw);

   // LOG_INFO("-------------------  ALS raw = %d, now_lux = %d   \r\n",  raw, now_lux);
    ls_data_changed = 1;
    input_report_abs(epld->als_input_dev, ABS_MISC, now_lux);
    input_sync(epld->als_input_dev);

    dynamic_intt_intt = als_dynamic_intt_intt[dynamic_intt_idx];
    dynamic_intt_gain = als_dynamic_intt_gain[dynamic_intt_idx];
    dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
    dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];

#else
    uint32_t lux;

    elan_sensor_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
    elan_sensor_I2C_Read(client);
    gRawData.als_ch1_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    lux =  (gRawData.als_ch1_raw* LUX_PER_COUNT) / 1000 * 15 / 100;
    if(lux>20000)
        lux=20000;

   // LOG_INFO("-------------------  ALS raw = %d, lux = %d\n\n",  gRawData.als_ch1_raw,  lux);
	ls_data_changed = 1;
    input_report_abs(epld->als_input_dev, ABS_MISC, lux);
    input_sync(epld->als_input_dev);
#endif
}

/*
//====================set_psensor_intr_threshold===============//
//low_thd: The value is psensor interrupt low threshold.
//high_thd:	The value is psensor interrupt hihg threshold.
//When psensor rawdata > hihg_threshold, interrupt pin will be pulled low.
//After interrupt occur, psensor rawdata < low_threshold, interrupt pin will be pulled high.
*/
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    int ret = 0;
    struct elan_epl_data *epld = epl_data;
    struct i2c_client *client = epld->client;

    uint8_t high_msb ,high_lsb, low_msb, low_lsb;

    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb  = (uint8_t) (low_thd & 0x00ff);

 //   LOG_INFO("[%s]: L=%d, H=%d \r\n", __func__, low_thd, high_thd);
    elan_sensor_I2C_Write(client,REG_2,W_SINGLE_BYTE,0x02,high_lsb);
    elan_sensor_I2C_Write(client,REG_3,W_SINGLE_BYTE,0x02,high_msb);
    elan_sensor_I2C_Write(client,REG_4,W_SINGLE_BYTE,0x02,low_lsb);
    elan_sensor_I2C_Write(client,REG_5,W_SINGLE_BYTE,0x02,low_msb);

    return ret;
}

#if PS_INTERRUPT_MODE
static void epl_sensor_irq_do_work(struct work_struct *work)
{
    struct elan_epl_data *epld = epl_data;
    struct i2c_client *client = epld->client;
    int mode = 0;
    LOG_FUN();

    if(irq_caused_by_power_off) {
        enable_irq(epld->irq);
        return;
    }

    elan_sensor_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_LOCK);

    elan_sensor_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
    elan_sensor_I2C_Read(client);
    mode = gRawData.raw_bytes[0]&(3<<4);

    // 0x10 is ps mode
    if(mode==CMC_MODE_PS && epld->enable_pflag)
    {
        gRawData.ps_int_state= !((gRawData.raw_bytes[0]&0x04)>>2);
        elan_epl_ps_poll_rawdata();
    }
    else
    {
        LOG_INFO("error: interrupt in als\n");
    }
    elan_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW);
    elan_sensor_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);
    enable_irq(epld->irq);

    if(ps_intr_flag == 1)
    {
        ps_intr_flag = 0;
        elan_sensor_psensor_enable(epld);
//      LOG_INFO("[%s]: ps_intr_flag =%d \r\n", __func__, ps_intr_flag);
    }
}

static irqreturn_t elan_sensor_irq_handler(int irqNo, void *handle)
{
    struct elan_epl_data *epld = (struct elan_epl_data*)handle;

    disable_irq_nosync(epld->irq);
    wake_lock_timeout(&g_ps_wlock, HZ/2);
    queue_work(epld->epl_wq, &epl_sensor_irq_work);

    return IRQ_HANDLED;
}
#endif
int als_suspend_flag = 0;

static void polling_do_work(struct work_struct *work)
{
    struct elan_epl_data *epld = epl_data;
    struct i2c_client *client = epld->client;

    bool isInterleaving = epld->enable_pflag==1 && epld->enable_lflag==1;
    bool isAlsOnly = epld->enable_pflag==0 && epld->enable_lflag==1;
    bool isPsOnly = epld->enable_pflag==1 && epld->enable_lflag==0;

    cancel_delayed_work(&polling_work);
    if (als_suspend_flag == 1)
        return;

   // LOG_INFO("enable_pflag = %d, enable_lflag = %d\n", epld->enable_pflag, epld->enable_lflag);
    if(isInterleaving || isAlsOnly || (isPsOnly && PS_INTERRUPT_MODE==0))
    {
        queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(ALS_DELAY+2*PS_DELAY+30));
    }

    if(isAlsOnly || isInterleaving)
    {
        if(als_suspend_flag == 0)
        {
            elan_sensor_lsensor_enable(epld);
            elan_epl_als_rawdata();
        }
        if(isInterleaving)
        {
            elan_sensor_psensor_enable(epld);
            if(PS_INTERRUPT_MODE == 0)
            {
                elan_epl_ps_poll_rawdata();
            }

        }

    }
    else if(isPsOnly)
    {
        elan_sensor_psensor_enable(epld);

        if(PS_INTERRUPT_MODE)
        {
            // do nothing
        }
        else
        {
            elan_epl_ps_poll_rawdata();
        }
    }
    else
    {
        elan_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
        elan_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,EPL_S_SENSING_MODE);
        cancel_delayed_work(&polling_work);
    }
}

static ssize_t elan_ls_operationmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct elan_epl_data *epld = epl_data;
    LOG_FUN();

    sscanf(buf, "%hu",&mode);
 //   LOG_INFO("==>[operation mode]=%d\n", mode);

    if(mode == 0)
    {
        epld->enable_lflag = 0;
        epld->enable_pflag = 0;
    }
    else if(mode == 1)
    {
        epld->enable_lflag = 1;
        epld->enable_pflag = 0;
    }
    else if(mode == 2)
    {
        epld->enable_lflag = 0;
        epld->enable_pflag = 1;
    }
    else if(mode == 3)
    {
        epld->enable_lflag = 1;
        epld->enable_pflag = 1;
    }
    else
    {
        LOG_INFO("0: none\n1: als only\n2: ps only\n3: interleaving");
    }

    elan_sensor_restart_work();
    return count;
}

static ssize_t elan_ls_operationmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct elan_epl_data *epld = epl_data;
    long *tmp = (long*)buf;
    uint16_t mode =0;
    LOG_FUN();

    if(  epld->enable_pflag==0 &&  epld->enable_lflag==0)
    {
        mode = 0;
    }
    else if(  epld->enable_pflag==0 &&  epld->enable_lflag==1)
    {
        mode = 1;
    }
    else if(  epld->enable_pflag==1 && epld->enable_lflag==0)
    {
        mode = 2;
    }
    else if(  epld->enable_pflag==1 && epld->enable_lflag==1)
    {
        mode = 3;
    }
    tmp[0] = mode;
    return sprintf(buf, "%d \n", mode);
}

static ssize_t elan_ls_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct elan_epl_data *epld = epl_data;
    u16 ch1;

    if(!epl_data)
    {
        LOG_INFO("epl_data is null!!\n");
        return 0;
    }
    elan_sensor_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_LOCK);

    elan_sensor_I2C_Write(epld->client,REG_16,R_TWO_BYTE,0x01,0x00);
    elan_sensor_I2C_Read(epld->client);
    ch1 = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
 //   LOG_INFO("ch1 raw_data = %d\n", ch1);

    elan_sensor_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);

    return sprintf(buf, "%d\n", ch1);
}

static ssize_t epl2182_show_als_data(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct elan_epl_data *obj = epl_data;
        int als_tmp;
        LOG_FUN();

        if(!epl_data)
        {
            LOG_ERR("epl_data is null!!\n");
            return 0;
        }

        if(obj->enable_lflag == 0)
        {
            obj->enable_lflag = 1;
            elan_sensor_restart_work();
            msleep(ALS_DELAY*als_dynamic_intt_intt_num*2);    //dynamic INTT
        }


        als_tmp = gRawData.als_lux;

        return sprintf(buf, "%d\n", als_tmp);
}

static ssize_t epl2182_show_ps_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct elan_epl_data *obj = epl_data;
    bool isPsOnly = obj->enable_pflag==1 && obj->enable_lflag==0;
    bool isInterleaving = obj->enable_pflag==1 && obj->enable_lflag==1;
    bool isAlsOnly = obj->enable_pflag==0 && obj->enable_lflag==1;
    int ps_tmp;
//   LOG_FUN();
//    LOG_INFO("isPsOnly=%d, isInterleaving=%d, isAlsOnly=%d \r\n", isPsOnly, isInterleaving, isAlsOnly);
    if(!epl_data)
    {
        LOG_ERR("epl_data is null!!\n");
        return 0;
    }

    if(isAlsOnly == 1 || isInterleaving == 1)
    {
        obj->enable_pflag = 1;
        elan_sensor_restart_work();
        msleep(ALS_DELAY+2*PS_DELAY+30);
    }

    if(isPsOnly == 1 || obj->enable_pflag == 0)
    {
        if(obj->enable_pflag == 0)
        {
            elan_sensor_psensor_enable(obj);
        }
        obj->enable_pflag = 1;
        isPsOnly = 1;
    }

    if(isPsOnly == 1)
    {
        elan_sensor_I2C_Write(obj->client,REG_16,R_TWO_BYTE,0x01,0x00);
        elan_sensor_I2C_Read(obj->client);
        gRawData.ps_ch1_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
        msleep(5);
    }

    ps_tmp = gRawData.ps_ch1_raw;
//    LOG_INFO("[%s]: gRawData.ps_ch1_raw=%d \r\n", __FUNCTION__, gRawData.ps_ch1_raw);

    return sprintf(buf, "%d\n", ps_tmp);
}

static ssize_t epl2182_show_type(struct device *dev, struct device_attribute *attr, char *buf)
{

    return sprintf(buf, "elan_alsps\n");
}


static ssize_t ps_thres_high_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
  	struct elan_epl_data *epld = epl_data;
	int ret;
//	LOG_FUN();

	ret = sprintf(buf, "%d\n", epld->ps_threshold_high);
	return ret;
}
static ssize_t ps_thres_high_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    struct elan_epl_data *epld = epl_data;
//    LOG_FUN();

    sscanf(buf, "%d",&epld->ps_threshold_high);
    set_psensor_intr_threshold(epld->ps_threshold_low,epld->ps_threshold_high);

    return size;
}
static ssize_t ps_thres_low_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
  	struct elan_epl_data *epld = epl_data;
	int ret;
//	LOG_FUN();

	ret = sprintf(buf, "%d\n", epld->ps_threshold_low);
	return ret;
}
static ssize_t ps_thres_low_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    struct elan_epl_data *epld = epl_data;
//    LOG_FUN();

    sscanf(buf, "%d",&epld->ps_threshold_low);
    set_psensor_intr_threshold(epld->ps_threshold_low,epld->ps_threshold_high);

    return size;
}

#if 0
static DEVICE_ATTR(elan_ls_operationmode, S_IROTH|S_IWOTH, elan_ls_operationmode_show,elan_ls_operationmode_store);
static DEVICE_ATTR(elan_ls_status, S_IROTH|S_IWOTH, elan_ls_status_show,NULL);
static DEVICE_ATTR(als_data, S_IROTH  | S_IWOTH, epl2182_show_als_data, NULL);
static DEVICE_ATTR(ps_data, S_IROTH  | S_IWOTH, epl2182_show_ps_data, NULL);
static DEVICE_ATTR(type, S_IROTH  | S_IWOTH, epl2182_show_type, NULL);
static DEVICE_ATTR(ps_thres_low, S_IROTH  | S_IWOTH, ps_thres_low_show, ps_thres_low_store);
static DEVICE_ATTR(ps_thres_high, S_IROTH  | S_IWOTH, ps_thres_high_show, ps_thres_high_store);
#else

static DEVICE_ATTR(elan_ls_operationmode, 0664, elan_ls_operationmode_show,elan_ls_operationmode_store);
static DEVICE_ATTR(elan_ls_status, 0664, elan_ls_status_show,NULL);
static DEVICE_ATTR(als_data, 0664, epl2182_show_als_data, NULL);
static DEVICE_ATTR(ps_data, 0664, epl2182_show_ps_data, NULL);
static DEVICE_ATTR(type, 0664, epl2182_show_type, NULL);

static struct attribute *elan_alsps_attributes[] =
{
    &dev_attr_elan_ls_operationmode.attr,
    &dev_attr_elan_ls_status.attr,
    &dev_attr_als_data.attr,
    &dev_attr_ps_data.attr,
    &dev_attr_type.attr,
    NULL,
};

static struct attribute_group elan_alsps_attr_group =
{
    .attrs = elan_alsps_attributes,
};



static struct kobj_attribute ps_thres_low=
__ATTR(ps_thres_low, 0664, ps_thres_low_show, ps_thres_low_store);

static struct kobj_attribute ps_thres_high=
__ATTR(ps_thres_high, 0664, ps_thres_high_show, ps_thres_high_store);
#endif

static struct attribute *ets_attributes[] =
{
    &ps_thres_low.attr,
    &ps_thres_high.attr,
    NULL,
};

static struct attribute_group ets_attr_group =
{
    .attrs = ets_attributes,
};

static int create_sysfs_interfaces(struct device *dev)
{
	int err;
//alsps_kobj = kobject_create_and_add("als_ps", &dev->kobj);
	//if(!alsps_kobj)
	//	return -ENOMEM;
	err = sysfs_create_group(&dev->kobj, &ets_attr_group);
	if (err)
		kobject_put(alsps_kobj);
	return 0;
}

static int elan_als_open(struct inode *inode, struct file *file)
{
    struct elan_epl_data *epld = epl_data;

    LOG_FUN();

    if (epld->als_opened)
    {
        return -EBUSY;
    }
    epld->als_opened = 1;

    return 0;
}

static int elan_als_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
    struct elan_epl_data *epld = epl_data;
    int buf[1];
    if(epld->read_flag ==1)
    {
        buf[0] = gRawData.als_ch1_raw;
        if(copy_to_user(buffer, &buf , sizeof(buf)))
            return 0;
        epld->read_flag = 0;
        return 12;
    }
    else
    {
        return 0;
    }
}

static int elan_als_release(struct inode *inode, struct file *file)
{
    struct elan_epl_data *epld = epl_data;

    LOG_FUN();

    epld->als_opened = 0;

    return 0;
}

static long elan_als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int flag;
    unsigned long buf[1];
    struct elan_epl_data *epld = epl_data;

    void __user *argp = (void __user *)arg;

    LOG_INFO("als io ctrl cmd %d\n", _IOC_NR(cmd));

    switch(cmd)
    {
        case ELAN_EPL6800_IOCTL_GET_LFLAG:

            LOG_INFO("elan ambient-light IOCTL Sensor get lflag \n");
            flag = epld->enable_lflag;
            if (copy_to_user(argp, &flag, sizeof(flag)))
                return -EFAULT;

            LOG_INFO("elan ambient-light Sensor get lflag %d\n",flag);
            break;

        case ELAN_EPL6800_IOCTL_ENABLE_LFLAG:

            LOG_INFO("elan ambient-light IOCTL Sensor set lflag \n");
            if (copy_from_user(&flag, argp, 1))
                return -EFAULT;
			LOG_INFO("elan ambient-light Sensor set lflag %d\n",flag);
			if(flag) {
				flag = 1;
			}else {
				flag = 0;
			}
#if DYNAMIC_INTT
			dynamic_intt_idx = dynamic_intt_init_idx;
            dynamic_intt_intt = als_dynamic_intt_intt[dynamic_intt_idx];
            dynamic_intt_gain = als_dynamic_intt_gain[dynamic_intt_idx];
            dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
            dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
#endif
            //if (flag < 0 || flag > 1)
            //    return -EINVAL;

            epld->enable_lflag = flag;
            elan_sensor_restart_work();

            //LOG_INFO("elan ambient-light Sensor set lflag %d\n",flag);
            break;

        case ELAN_EPL6800_IOCTL_GETDATA:
            buf[0] = (unsigned long)gRawData.als_ch1_raw;
            if(copy_to_user(argp, &buf , 2))
                return -EFAULT;

            break;

        default:
            LOG_ERR("invalid cmd %d\n", _IOC_NR(cmd));
            return -EINVAL;
    }

    return 0;

}

static unsigned int elan_als_poll(struct file *fp, poll_table * wait)
{
	if(ls_data_changed){
		//pr_info("%s, ls_data_changed 1st, ls_data = 0x%x\n", __func__, ls_data);
		ls_data_changed = 0;
		//mutex_unlock(&stk3x1x_mutex);
		return POLLIN | POLLRDNORM;
	}
	poll_wait(fp, &ls_waitqueue, wait);
	//mutex_unlock(&stk3x1x_mutex);
	return 0;
}

static struct file_operations elan_als_fops =
{
    .owner = THIS_MODULE,
    .open = elan_als_open,
    .read = elan_als_read,
    .release = elan_als_release,
    .unlocked_ioctl = elan_als_ioctl,
    .poll = elan_als_poll,
};

static struct miscdevice elan_als_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "elan_als",
    .fops = &elan_als_fops
};

static int elan_ps_open(struct inode *inode, struct file *file)
{
    struct elan_epl_data *epld = epl_data;

    LOG_FUN();

    if (epld->ps_opened)
        return -EBUSY;

    epld->ps_opened = 1;

    return 0;
}

static int elan_ps_release(struct inode *inode, struct file *file)
{
    struct elan_epl_data *epld = epl_data;

    LOG_FUN();

    epld->ps_opened = 0;

    psensor_mode_suspend = 0;

    return 0;
}

static long elan_ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int value;
    int flag;
    struct elan_epl_data *epld = epl_data;

    void __user *argp = (void __user *)arg;

    LOG_INFO("ps io ctrl cmd %d\n", _IOC_NR(cmd));

    //ioctl message handle must define by android sensor library (case by case)
    switch(cmd)
    {
        case ELAN_EPL6800_IOCTL_GET_PFLAG:

            LOG_INFO("elan Proximity Sensor IOCTL get pflag \n");
            flag = epld->enable_pflag;
            if (copy_to_user(argp, &flag, sizeof(flag)))
                return -EFAULT;

            LOG_INFO("elan Proximity Sensor get pflag %d\n",flag);
            break;

        case ELAN_EPL6800_IOCTL_ENABLE_PFLAG:
            LOG_INFO("elan Proximity IOCTL Sensor set pflag \n");
            if (copy_from_user(&flag, argp, sizeof(flag)))
                return -EFAULT;
			if(flag) {
				flag = 1;
			}else {
				flag = 0;
			}
            //if (flag < 0 || flag > 1)
            //    return -EINVAL;

            epld->enable_pflag = flag;
            elan_sensor_restart_work();

            LOG_INFO("elan Proximity Sensor set pflag %d\n",flag);
            break;

        case ELAN_EPL6800_IOCTL_GETDATA:

            //value = gRawData.ps_ch1_raw;
            value = gRawData.ps_state;
            if(copy_to_user(argp, &value , 1))
                return -EFAULT;

            LOG_INFO("elan proximity Sensor get data (%d) \n",value);
            break;

        default:
            LOG_ERR("invalid cmd %d\n", _IOC_NR(cmd));
            return -EINVAL;
    }

    return 0;
}

static unsigned int elan_ps_poll(struct file *fp, poll_table * wait)
{
	if(ps_data_changed) {
		ps_data_changed = 0;
		return POLLIN | POLLRDNORM;
	}
	poll_wait(fp, &ps_waitqueue, wait);
	return 0;
}


static struct file_operations elan_ps_fops =
{
    .owner = THIS_MODULE,
    .open = elan_ps_open,
    .release = elan_ps_release,
    .unlocked_ioctl = elan_ps_ioctl,
    .poll		= elan_ps_poll,
};

static struct miscdevice elan_ps_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "elan_ps",
    .fops = &elan_ps_fops
};

static int initial_sensor(struct elan_epl_data *epld)
{
    struct i2c_client *client = epld->client;

    int ret = 0;

    LOG_INFO("initial_sensor enter!\n");

    ret = elan_sensor_I2C_Read(client);

    if(ret < 0)
        return -EINVAL;

    elan_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0x02, EPL_S_SENSING_MODE);
    elan_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
    set_psensor_intr_threshold(P_SENSOR_LTHD , P_SENSOR_HTHD);

    msleep(2);

    epld->enable_lflag = 0;
    epld->enable_pflag = 0;

    return ret;
}

/*----------------------------------------------------------------------------*/
static ssize_t light_enable_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct elan_epl_data *epld  = epl_data;
	printk("%s: ALS_status=%d\n", __func__, epld->enable_lflag);
	return sprintf(buf, "%d\n", epld->enable_lflag);
}

static ssize_t light_enable_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct elan_epl_data *epld = epl_data;

    LOG_INFO("light_enable_store: enable=%s \n", buf);

	if (sysfs_streq(buf, "1")){
		epld->enable_lflag= 1;
    }
	else if (sysfs_streq(buf, "0"))
		epld->enable_lflag= 0;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return 0;
	}
#if DYNAMIC_INTT
		dynamic_intt_idx = dynamic_intt_init_idx;
        dynamic_intt_intt = als_dynamic_intt_intt[dynamic_intt_idx];
        dynamic_intt_gain = als_dynamic_intt_gain[dynamic_intt_idx];
        dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
        dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
#endif
	elan_sensor_restart_work();
	return size;
}

/*----------------------------------------------------------------------------*/
static struct device_attribute dev_attr_light_enable =
__ATTR(enable, S_IRWXUGO,
	   light_enable_show, light_enable_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};
/*----------------------------------------------------------------------------*/

static int lightsensor_setup(struct elan_epl_data *epld)
{
    int err = 0;
    LOG_INFO("lightsensor_setup enter.\n");

    epld->als_input_dev = input_allocate_device();
    if (!epld->als_input_dev)
    {
        LOG_ERR( "could not allocate ls input device\n");
        return -ENOMEM;
    }
    epld->als_input_dev->name = ElanALsensorName;
    set_bit(EV_ABS, epld->als_input_dev->evbit);
    input_set_abs_params(epld->als_input_dev, ABS_MISC, 0, 9, 0, 0);

    err = input_register_device(epld->als_input_dev);
    if (err < 0)
    {
        LOG_ERR("can not register ls input device\n");
        goto err_free_ls_input_device;
    }

    err = misc_register(&elan_als_device);
    if (err < 0)
    {
        LOG_ERR("can not register ls misc device\n");
        goto err_unregister_ls_input_device;
    }

    err = sysfs_create_group(&epld->als_input_dev->dev.kobj, &light_attribute_group);

	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_free_ls_input_device;
	}

    return err;

err_unregister_ls_input_device:
    input_unregister_device(epld->als_input_dev);
err_free_ls_input_device:
    input_free_device(epld->als_input_dev);
    return err;
}

/*----------------------------------------------------------------------------*/
static ssize_t proximity_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct elan_epl_data *epld = epl_data;
	printk("%s: PS status=%d\n", __func__, epld->enable_pflag);
	LOG_INFO("epl2182_setup_psensor enter.\n");
	return sprintf(buf, "%d\n", epld->enable_pflag);
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t proximity_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct elan_epl_data *epld = epl_data;

    LOG_INFO("proximity_enable_store: enable=%s \n", buf);
	if (sysfs_streq(buf, "1")){
		epld->enable_pflag =1;
		init_ps_flag = 0;
    }
	else if (sysfs_streq(buf, "0")) {
	    epld->enable_pflag =0;
            //set_psensor_intr_threshold(0, 70);
        }
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return 0;
	}

	elan_sensor_restart_work();
	return size;
}
/*----------------------------------------------------------------------------*/
static struct device_attribute dev_attr_ps_enable =
__ATTR(enable, S_IRWXUGO,
	   proximity_enable_show, proximity_enable_store);

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_ps_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};
/*----------------------------------------------------------------------------*/

static int psensor_setup(struct elan_epl_data *epld)
{
    int err = 0;
    LOG_INFO("psensor_setup enter.\n");

    epld->ps_input_dev = input_allocate_device();
    if (!epld->ps_input_dev)
    {
        LOG_ERR("could not allocate ps input device\n");
        return -ENOMEM;
    }
    epld->ps_input_dev->name = ElanPsensorName;

    set_bit(EV_ABS, epld->ps_input_dev->evbit);
    input_set_abs_params(epld->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    err = input_register_device(epld->ps_input_dev);
    if (err < 0)
    {
        LOG_ERR("could not register ps input device\n");
        goto err_free_ps_input_device;
    }

    err = misc_register(&elan_ps_device);
    if (err < 0)
    {
        LOG_ERR("could not register ps misc device\n");
        goto err_unregister_ps_input_device;
    }

    err = sysfs_create_group(&epld->ps_input_dev->dev.kobj, &proximity_attribute_group);
	if (err) {
		pr_err("%s: PS could not create sysfs group\n", __func__);
		goto err_free_ps_input_device;
	}

    return err;

err_unregister_ps_input_device:
    input_unregister_device(epld->ps_input_dev);
err_free_ps_input_device:
    input_free_device(epld->ps_input_dev);
    return err;
}

#if PS_INTERRUPT_MODE
static int setup_interrupt(struct elan_epl_data *epld)
{
    struct i2c_client *client = epld->client;

    int err = 0;
  //  msleep(5);
#if 0
    err = gpio_request(S5PV210_GPH0(1), "Elan EPL IRQ");
    if (err)
    {
        LOG_ERR("gpio pin request fail (%d)\n", err);
        goto initial_fail;
    }
    else
    {
        LOG_INFO("----- Samsung gpio config success -----\n");
        s3c_gpio_cfgpin(S5PV210_GPH0(1),S3C_GPIO_SFN(0x0F)/*(S5PV210_GPH0_1_EXT_INT30_1) */);
        s3c_gpio_setpull(S5PV210_GPH0(1),S3C_GPIO_PULL_NONE);
    }

#else
/*********************************
    err = gpio_request(EPL_INTR_PIN, "rpr_irq_gpio");
	if (err < 0) {
		LOG_ERR("gpio %d request failed (%d)\n", EPL_INTR_PIN, err);
		goto initial_fail;
	}

	err = gpio_direction_input(EPL_INTR_PIN);
	if (err < 0) {
		LOG_ERR(
			"fail to set gpio %d as input (%d)\n", EPL_INTR_PIN, err);
		goto fail_free_intr_pin;
	}
**********************************/
       unsigned int irq_gpio;
       unsigned int irq_gpio_flags;
       struct device_node *np = client->dev.of_node;

        irq_gpio = of_get_named_gpio_flags(np, "epl,irq-gpio",
                                0, &irq_gpio_flags);
        epld->intr_pin = irq_gpio;
        if (irq_gpio < 0) {
 //               return initial_fail;
         gpio_free(epld->intr_pin);
        }
        if (gpio_is_valid(irq_gpio)) {
                err = gpio_request(irq_gpio, "epl_irq_gpio");
                if (err) {
                        printk( "irq gpio request failed");
                }

                err = gpio_direction_input(irq_gpio);
                if (err) {
                        printk("set_direction for irq gpio failed\n");
                }
        }
#endif
    err = request_irq(epld->irq,elan_sensor_irq_handler, IRQF_TRIGGER_FALLING ,
                      client->dev.driver->name, epld);
    if(err <0)
    {
        LOG_ERR("request irq pin %d fail for gpio\n",err);
        goto fail_free_intr_pin;
    }
    irq_set_irq_wake(epld->irq, 1);

    return err;

//initial_fail:
fail_free_intr_pin:
    gpio_free(epld->intr_pin);
    return err;
}
#endif

static int elan_power_on(struct elan_epl_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
	}

	return rc;
}

static int elan_power_init(struct elan_epl_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

#ifdef CONFIG_SUSPEND

static int elan_sensor_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct elan_epl_data *epld = epl_data;
//    unsigned long flags;

    if(!epl_data)
    {
        LOG_ERR("null pointer!!\n");
        return 0;
    }
    else
    {
        printk("%s:_________ls_enable %d, ps_enable %d\n", __func__, epld->enable_lflag,epld->enable_pflag);
    }
    if(epld->enable_lflag == 1)
    {
        als_suspend_flag = 1;
        epld->enable_lflag = 0;
    }

    if(epld->enable_pflag)
    {
        //elan_sensor_psensor_enable(epld);
        LOG_INFO("[%s]: ps enable \r\n", __func__);
    } else {
        irq_caused_by_power_off = true;
        elan_sensor_I2C_Write(client,REG_7, W_SINGLE_BYTE, 0x02, EPL_C_P_UP);
        elan_sensor_I2C_Write(client,REG_7, W_SINGLE_BYTE, 0x02, EPL_C_P_DOWN);
        elan_power_on(epld, false);
        irq_set_irq_wake(client->irq, 0);
		disable_irq(client->irq);
    }

    mutex_lock(&sensor_mutex);
    //spin_lock_irqsave(&sensor_mutex.wait_lock, flags);
	cancel_delayed_work_sync(&polling_work);
	//msleep(ALS_DELAY+2*PS_DELAY+30);
	mutex_unlock(&sensor_mutex);
	//spin_unlock_irqrestore(&sensor_mutex.wait_lock, flags);

    return 0;
}
/**************************
static void elan_sensor_early_suspend(struct early_suspend *h)
{
    struct elan_epl_data *epld = epl_data;
    struct i2c_client *client = epld->client;
    LOG_FUN();

    if( epld->enable_pflag==0)
    {
        elan_sensor_I2C_Write(client,REG_7, W_SINGLE_BYTE, 0x02, EPL_C_P_DOWN);
        cancel_delayed_work(&polling_work);
    }

    psensor_mode_suspend = 1;
}
****************      zhujp modify**********/
static int elan_sensor_resume(struct i2c_client *client)
{
    struct elan_epl_data *epld = epl_data;
    unsigned long flags;

    if(!epl_data)
    {
        LOG_ERR("no epl2182 chip!!\n");
        return 0;
    }
    else
    {
        printk("%s:_________ls_enable %d, ps_enable %d\n", __func__, epld->enable_lflag,epld->enable_pflag);
    }
    if(als_suspend_flag == 1)
    {
        als_suspend_flag = 0;
        epld->enable_lflag = 1;
    }

    if (epld->enable_pflag) {
        //mutex_lock(&sensor_mutex);
        spin_lock_irqsave(&sensor_mutex.wait_lock, flags);
        elan_sensor_restart_work();
        //mutex_unlock(&sensor_mutex);
        spin_unlock_irqrestore(&sensor_mutex.wait_lock, flags);
        LOG_INFO("[%s]: ps enable(%d) || als enable(%d) \r\n", __func__, epld->enable_pflag, epld->enable_lflag);
    } else {
        elan_power_on(epld, true);
        initial_sensor(epld);
        enable_irq(client->irq);
		irq_set_irq_wake(client->irq, 1);
		irq_caused_by_power_off = false;
    }

    return 0;
}

/************************************
static void elan_sensor_late_resume(struct early_suspend *h)
{
    struct elan_epl_data *epld = epl_data;
    struct i2c_client *client = epld->client;

    LOG_FUN();

    if(epld->enable_pflag | epld->enable_lflag)
    {
        elan_sensor_I2C_Write(client,REG_7, W_SINGLE_BYTE, 0x02, EPL_C_P_UP);
    }

    if(epld->enable_pflag || epld->enable_lflag)
    {
        elan_sensor_restart_work();
    }
    psensor_mode_suspend = 0;
}
*******************zhujp modify************/
#endif

static int elan_sensor_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    int err = 0;
    struct elan_epl_data *epld ;
    //struct elan_epl_platform_data *pdata;
    static struct platform_device *sensor_dev;
    //struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    LOG_INFO("elan sensor probe enter.\n");

    epld = kzalloc(sizeof(struct elan_epl_data), GFP_KERNEL);
    if (!epld)
        return -ENOMEM;

    epld->client = client;
    i2c_set_clientdata(client, epld);
    err = elan_power_init(epld, true);
    if (err) {
        dev_err(&client->dev, "power init failed");
    }

    err = elan_power_on(epld, true);
    if (err) {
        dev_err(&client->dev, "power on failed");
    }

    if((i2c_smbus_read_byte_data(client, 0x98)) != 0x68)
    {
        LOG_INFO("elan ALS/PS sensor is failed. \n");
        goto i2c_fail;
    }
/*
    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
    {
       // dev_err(&client->dev,"No supported i2c func what we need?!!\n");
        err = -EIO;
        goto i2c_fail;
    }
*/
    LOG_INFO("chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
    LOG_INFO("chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
    LOG_INFO("chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
    LOG_INFO("chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
    LOG_INFO("chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
    LOG_INFO("chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
    LOG_INFO("chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
    LOG_INFO("chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
    LOG_INFO("chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
    LOG_INFO("chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
    LOG_INFO("chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
    LOG_INFO("chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
    LOG_INFO("chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
    LOG_INFO("chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
    LOG_INFO("chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

    epld->client = client;
    epld->irq = client->irq;
    epld->ps_threshold_high = P_SENSOR_HTHD;
    epld->ps_threshold_low = P_SENSOR_LTHD;
    i2c_set_clientdata(client, epld);

    epl_data = epld;

    mutex_init(&sensor_mutex);

    epld->epl_wq = create_singlethread_workqueue("elan_sensor_wq");
    if (!epld->epl_wq)
    {
        LOG_ERR("can't create workqueue\n");
        err = -ENOMEM;
        goto err_create_singlethread_workqueue;
    }

    err = lightsensor_setup(epld);
    if (err < 0)
    {
        LOG_ERR("lightsensor_setup error!!\n");
        goto err_lightsensor_setup;
    }

    err = psensor_setup(epld);
    if (err < 0)
    {
        LOG_ERR("psensor_setup error!!\n");
        goto err_psensor_setup;
    }

    err = initial_sensor(epld);
    if (err < 0)
    {
        LOG_ERR("fail to initial sensor (%d)\n", err);
        goto err_sensor_setup;
    }

#if PS_INTERRUPT_MODE
    err = setup_interrupt(epld);
    if (err < 0)
    {
        LOG_ERR("setup error!\n");
        goto err_sensor_setup;
    }
#endif
/*********************
#ifdef CONFIG_SUSPEND
//    epld->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1; zhujp modify
//    epld->early_suspend.suspend = elan_sensor_early_suspend;
//    epld->early_suspend.resume = elan_sensor_late_resume;
//    register_early_suspend(&epld->early_suspend);
#endif
****************zhujp modify********/
    wake_lock_init(&g_ps_wlock, WAKE_LOCK_SUSPEND, "ps_wakelock");

    sensor_dev = platform_device_register_simple("elan_alsps", -1, NULL, 0);
    if (IS_ERR(sensor_dev))
    {
        printk ("sensor_dev_init: error\n");
        goto err_fail;
    }

    err = sysfs_create_group(&sensor_dev->dev.kobj, &elan_alsps_attr_group);
    if (err !=0)
    {
        dev_err(&client->dev,"%s:create sysfs group error", __func__);
        goto err_fail;
    }

    err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "device EPL8881 sysfs register failed\n");
		goto err_fail;
	}

    LOG_INFO("sensor probe success.\n");

    return err;

err_fail:
    kobject_put(alsps_kobj);
    input_unregister_device(epld->als_input_dev);
    input_unregister_device(epld->ps_input_dev);
    input_free_device(epld->als_input_dev);
    input_free_device(epld->ps_input_dev);
err_lightsensor_setup:
err_psensor_setup:
err_sensor_setup:
    destroy_workqueue(epld->epl_wq);
    misc_deregister(&elan_ps_device);
    misc_deregister(&elan_als_device);
err_create_singlethread_workqueue:
kfree(epld);
i2c_fail:
    elan_power_on(epld, false);
//err_platform_data_null:
//    kfree(epld);
    return err;
}

static int elan_sensor_remove(struct i2c_client *client)
{
    struct elan_epl_data *epld = i2c_get_clientdata(client);

    dev_dbg(&client->dev, "%s: enter.\n", __func__);
    kobject_put(alsps_kobj);
 //   unregister_early_suspend(&epld->early_suspend);   zhujp modify
    input_unregister_device(epld->als_input_dev);
    input_unregister_device(epld->ps_input_dev);
    input_free_device(epld->als_input_dev);
    input_free_device(epld->ps_input_dev);
    misc_deregister(&elan_ps_device);
    misc_deregister(&elan_als_device);
    free_irq(epld->irq,epld);
    destroy_workqueue(epld->epl_wq);
    kfree(epld);
    return 0;
}

static const struct i2c_device_id elan_sensor_id[] =
{
    { "EPL2182", 0 },
    { }
};

static struct of_device_id epl_match_table[] = {
                { .compatible = "epl,epl2182",},
                { },
        };

static struct i2c_driver elan_sensor_driver =
{
    .probe	= elan_sensor_probe,
    .remove	= elan_sensor_remove,
    .id_table	= elan_sensor_id,
    .driver	= {
        .name = "EPL2182",
        .owner = THIS_MODULE,
        .of_match_table =epl_match_table,
    },
#ifdef CONFIG_SUSPEND
    .suspend = elan_sensor_suspend,
    .resume = elan_sensor_resume,
#endif
};

static int __init elan_sensor_init(void)
{
    return i2c_add_driver(&elan_sensor_driver);
}

static void __exit  elan_sensor_exit(void)
{
    i2c_del_driver(&elan_sensor_driver);
}

module_init(elan_sensor_init);
module_exit(elan_sensor_exit);

MODULE_AUTHOR("Renato Pan <renato.pan@eminent-tek.com>");
MODULE_DESCRIPTION("ELAN epl2182 driver");
MODULE_LICENSE("GPL");
