/* drivers/i2c/chips/epl8881.c - light and proxmity sensors driver
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

/** VERSION: 1.02**/

#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <linux/delay.h>
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
#include <linux/of_gpio.h>
#include <linux/i2c/epl8881.h>
#include <linux/kobject.h>
//#define PS_RAW_8BIT
/******************************************************************************
* configuration
*******************************************************************************/
#define PS_POLLING_MODE         	0				// 1 is polling mode, 0 is interrupt mode

#ifdef PS_RAW_8BIT //ices add by 20131230 /*PS_RAW_8BIT*/
    #define PS_LOW_THRESHOLD		13//7 //3500       //psensor Low threshold
    #define PS_HIGH_THRESHOLD		27//13 //7000       //psensor high threshold
#else
    #define PS_LOW_THRESHOLD		15000//2000 //13 //3500       //psensor Low threshold
    #define PS_HIGH_THRESHOLD		20000//5000 //27 //7000       //psensor high threshold
#endif //ices add by 20131230 /*PS_RAW_8BIT*/


#define LUX_PER_COUNT			700        //Lux per count


#define HS_BUFFER_SIZE			200
#define HS_CYCLE				1
static int hs_count=0;
static int start_idx=0;
static int HS_INTT	= EPL_INTT_PS_250;
static struct mutex sensor_mutex;

static int PS_INTT 					= EPL_INTT_PS_40;//EPL_INTT_PS_150; //EPL_INTT_PS_70; //psensor integration time
//static int ALS_INTT 					= EPL_INTT_ALS_100; //EPL_INTT_ALS_500; //ALS register integration time
//static int EPL_INTT_ALS 					= 100; //500; //ALS integration time
static int COLOR_INTT 				= EPL_INTT_CLR_250;


#define PS_DELAY			35//35
#define ALS_DELAY			200//165
#define COLOR_DELAY 			80

#define FT_VTG_MIN_UV           2600000
#define FT_VTG_MAX_UV           3300000
#define FT_I2C_VTG_MIN_UV       1800000
#define FT_I2C_VTG_MAX_UV       1800000

int dynamic_intt_idx;
int dynamic_intt_init_idx = 1;	//Dynamic init idx
uint32_t dynamic_intt_min_unit = 1000;

int c_gain = 900;
uint8_t dynamic_intt_intt;
uint8_t dynamic_intt_gain;
uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;


static int als_dynamic_intt_intt[] = {EPL_INTT_ALS_3000, EPL_INTT_ALS_250, EPL_INTT_ALS_15};
static int als_dynamic_intt_intt_value[] = {3000, 250, 15};
static int als_dynamic_intt_gain[] = {EPL_M_GAIN, EPL_M_GAIN, EPL_M_GAIN};
static int als_dynamic_intt_high_thr[] = {900*64, 850*64, 580*64};
static int als_dynamic_intt_low_thr[] = {60*64, 60*64, 40*64};

static int als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_intt_value)/sizeof(int);

/******************************************************************************
*******************************************************************************/

#define TXBYTES                             2
#define RXBYTES                             2

#define PACKAGE_SIZE 			8
#define I2C_RETRY_COUNT 		10
static struct kobject *alsps_kobj;
typedef struct _epl_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
    u16 ps_state;
    u16 ps_raw;

    u16 als_ch1_raw;

    u16 als_raw;
    u16 als_lux;
    uint32_t ratio;
    u16 hs_data[HS_BUFFER_SIZE];
    u16 color_raws[4];
} epl_raw_data;

struct epl_sensor_priv
{
    struct i2c_client *client;
    struct regulator *vdd;
    struct regulator *vcc_i2c;
    struct input_dev *als_input_dev;
    struct input_dev *ps_input_dev;
    struct input_dev *input_dev;
    struct workqueue_struct *epl_wq;
    struct class *epl_sensor_class;
    struct device *ls_dev;
    struct device *ps_dev;

   unsigned int intr_pin;
    int (*power)(int on);
    int ps_pocket_mode;

    int ps_opened;
    int als_opened;

    int ps_threshold_high;
    int ps_threshold_low;

    int als_threshold_high;
    int als_threshold_low;

    int polling_mode_als;
    int polling_mode_ps;
    int polling_mode_hs;

    int als_suspend;
    int ps_suspend;
    int hs_suspend;
    int clr_suspend;

    int lux_per_count;

    int enable_pflag;
    int enable_lflag;
    int enable_hflag;
    int enable_cflag;

    int read_flag;
    int irq;

//    int als_it;
    int ps_delay;

    int ALS_ADC;
    int ALS_delay;
    int clr_delay;
} ;

static struct platform_device *sensor_dev;
static struct wake_lock g_ps_wlock;
struct epl_sensor_priv *epl_sensor_obj;
static epl_raw_data	gRawData;

static const char ElanPsensorName[]="proximity";
static const char ElanALsensorName[]="lightsensor";

#define LOG_TAG                      "[EPL8881] "
#define LOG_FUN(f)               	 printk(KERN_INFO LOG_TAG"%s\n", __FUNCTION__)
#define LOG_INFO(fmt, args...)    	 printk(KERN_INFO LOG_TAG fmt, ##args)
#define LOG_ERR(fmt, args...)   	 printk(KERN_ERR  LOG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define D(x...) pr_info(x)

static int epl_sensor_set_ps_threshold(uint16_t low_thd, uint16_t high_thd);
static int epl_sensor_setup_interrupt(struct epl_sensor_priv *epld);

static void epl_sensor_eint_work(struct work_struct *work);
static DECLARE_WORK(epl_sensor_irq_work, epl_sensor_eint_work);

static void epl_sensor_polling_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, epl_sensor_polling_work);

/*
//====================I2C write operation===============//
//regaddr: ELAN Register Address.
//bytecount: How many bytes to be written to register via i2c bus.
//txbyte: I2C bus transmit byte(s). Single byte(0X01) transmit only slave address.
//data: setting value.
//
// Example: If you want to write single byte to 0x1D register address, show below
//	      epl_sensor_I2C_Write(client,0x1D,0x01,0X02,0xff);
//
*/
static int epl_sensor_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry,val;
    struct epl_sensor_priv *epld = epl_sensor_obj;

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
/*----------------------------------------------------------------------------*/
static int epl_sensor_I2C_Read(struct i2c_client *client)
{
    uint8_t buffer[RXBYTES];
    int ret = 0, i =0;
    int retry,val;
    struct epl_sensor_priv *epld = epl_sensor_obj;

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
static void epl_sensor_restart_work(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    cancel_delayed_work(&polling_work);
    queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(50));
}
/*-------Color sensor Enable------------------------------------------------------*/
static int epl_sensor_color_enable(struct epl_sensor_priv *epld, int enable)
{
    int ret = 0;
    uint8_t regdata;
    struct i2c_client *client = epld->client;

  //  LOG_INFO("[%s] enable = %d\n", __func__, enable);

    if(enable)
    {
        regdata = EPL_INT_DISABLE;
        ret = epl_sensor_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);

        regdata =  EPL_COLOR_MODE |EPL_10BIT_ADC | EPL_AUTO_GAIN |EPL_S_SENSING_MODE;;
        ret = epl_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

        regdata = COLOR_INTT| EPL_SENSING_8_TIME;
        ret = epl_sensor_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

        ret = epl_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0X02,EPL_GO_MID);
        ret = epl_sensor_I2C_Write(client,REG_10,W_SINGLE_BYTE,0x02,EPL_GO_LOW);

        ret = epl_sensor_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02,EPL_C_RESET);
        ret = epl_sensor_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);

        msleep(epld->clr_delay);
    }

    if(ret<0)
    {
        LOG_ERR("[%s] color enable %d fail\n",__func__,ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}

/*-------healthy sensor Enable------------------------------------------------------*/
static void epl_sensor_hs_enable(struct epl_sensor_priv *epld, bool interrupt, bool full_enable)
{
    int ret;
    uint8_t regdata = 0;
    struct i2c_client *client = epld->client;

    if(full_enable)
    {
        regdata =  EPL_INT_CH1 | (interrupt? EPL_INT_FRAME_ENABLE : EPL_INT_DISABLE );
        ret = epl_sensor_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);

        regdata =  EPL_PS_MODE |EPL_10BIT_ADC | EPL_L_GAIN |EPL_S_SENSING_MODE;
        ret = epl_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

        ret = epl_sensor_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02, 0xc0);

    }

    regdata = HS_INTT | (HS_CYCLE<<5);
    ret = epl_sensor_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);
    ret = epl_sensor_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02,EPL_C_RESET);

    ret = epl_sensor_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);

}
static int epl_sensor_psensor_enable(struct epl_sensor_priv *epld)
{
    int ret;
    uint8_t regdata = 0;
    struct i2c_client *client = epld->client;
    int ps_state;
    ret = epl_sensor_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE | EPL_INT_CH1);
    regdata =  EPL_PS_MODE | EPL_10BIT_ADC | EPL_M_GAIN ;
	
    regdata = regdata | (epld->polling_mode_ps==0 ? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE);	
    ret = epl_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);
	
    regdata = PS_INTT | EPL_SENSING_8_TIME;
    ret = epl_sensor_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

    epl_sensor_set_ps_threshold(epld->ps_threshold_low,epld->ps_threshold_high);
    /*
        Chip Reset Control(2)
            0 : the chip reset (exclusive of I2C registers)
            1 : the chip start to run
    */
    ret = epl_sensor_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02,EPL_C_RESET);
    ret = epl_sensor_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);

    msleep(epld->ps_delay);
    if (epld->polling_mode_ps==0)
    {
        epl_sensor_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
        epl_sensor_I2C_Read(client);
        ps_state= !((gRawData.raw_bytes[0]&0x04)>>2);

        epl_sensor_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
        epl_sensor_I2C_Read(client);
#ifdef PS_RAW_8BIT //ices add by 20131230 /*PS_RAW_8BIT*/
        gRawData.ps_raw = gRawData.raw_bytes[1];  //8bit
#else
        gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0]; //16bit
#endif  //ices end by 20131230 /*PS_RAW_8BIT*/
        if(gRawData.ps_state!= ps_state)
        {
            regdata =  EPL_INT_CH1 | EPL_INT_FRAME_ENABLE;
            epl_sensor_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02,regdata);
        }
        else
        {
            regdata =  EPL_INT_CH1 | EPL_INT_ACTIVE_LOW;
            epl_sensor_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02,regdata);
        }
    }
    if (ret != 0x02)
    {
        LOG_INFO("P-sensor i2c err\n");
    }

    return ret;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int epl_sensor_lsensor_enable(struct epl_sensor_priv *epld)
{
    int ret;
    int regdata = 0;
    struct i2c_client *client = epld->client;

    LOG_INFO("--- ALS sensor Enable --- \n");
    /*
        Interrupt Channel Selection(3-2): ch0(00), ch1(01), ch2(10), ch3(11)
        Interrupt Mode Selection(1-0): binary(00), disable(10), active-low(11)
    */
    regdata = EPL_INT_DISABLE;
    ret = epl_sensor_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);
    
	regdata = EPL_S_SENSING_MODE | EPL_COLOR_MODE | dynamic_intt_gain | EPL_10BIT_ADC;
    ret = epl_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

    regdata = dynamic_intt_intt | EPL_SENSING_16_TIME;
    ret = epl_sensor_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

    regdata = EPL_GO_MID;
    ret = epl_sensor_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02, regdata);

    regdata = EPL_GO_LOW;
    ret = epl_sensor_I2C_Write(client,REG_10,W_SINGLE_BYTE,0x02, regdata);
    /*
        Chip Reset Control(2)
            0 : the chip reset (exclusive of I2C registers)
            1 : the chip start to run
    */
    ret = epl_sensor_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02,EPL_C_RESET);
    ret = epl_sensor_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
    //msleep(ALS_DELAY);
    msleep(epld->ALS_delay);

    if(ret != 0x02)
    {
        LOG_INFO(" ALS-sensor i2c err\n");
    }



    return ret;

}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
int epl_sensor_read_color(struct i2c_client *client)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    uint8_t setting;
    u16 raws[4];

    if(client == NULL)
    {
        LOG_ERR("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
    epl_sensor_I2C_Read(client);
    setting =  (gRawData.raw_bytes[0]>>3&7);
    if(setting!=7)
    {
        LOG_ERR("read color data in wrong mode\n");
    }

    epl_sensor_I2C_Write(epld->client,REG_14,R_TWO_BYTE,0x01,0x00);
    epl_sensor_I2C_Read(epld->client);
    raws[0] = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    epl_sensor_I2C_Write(epld->client,REG_16,R_TWO_BYTE,0x01,0x00);
    epl_sensor_I2C_Read(epld->client);
    raws[1] = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    epl_sensor_I2C_Write(epld->client,REG_18,R_TWO_BYTE,0x01,0x00);
    epl_sensor_I2C_Read(epld->client);
    raws[2] = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    epl_sensor_I2C_Write(epld->client,REG_20,R_TWO_BYTE,0x01,0x00);
    epl_sensor_I2C_Read(epld->client);
    raws[3] = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    // FIX: mid gain and low gain cannot report ff in auton gain
    if((setting>>6)&&1 ==0 && raws[1]==65535)
    {
        LOG_INFO("setting %d, gain %x, ch1 %d\n", setting, setting>>6,  raws[1] );
        LOG_INFO("skip FF in auto gain\n\n");
    }
    else
    {
        gRawData.color_raws[0] = raws[0]>>6;
        gRawData.color_raws[1] = raws[1]>>6;
        gRawData.color_raws[2] = raws[2]>>6;
        gRawData.color_raws[3] = raws[3]>>6;

        //LOG_INFO("read color kgrb %d %d %d %d\n", gRawData.color_raws[0]  , gRawData.color_raws[1]  , gRawData.color_raws[2]  , gRawData.color_raws[3]   );
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
/*
//====================epl_sensor_read_ps===============//
//polling method for proximity sensor detect. Report proximity sensor raw data.
//Report "ABS_DISTANCE" event to HAL layer.
//Variable "value" 0 and 1 to represent which distance from psensor to target(human's face..etc).
//value: 0 represent near.
//value: 1 represent far.
*/
static void epl_sensor_read_ps(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t setting;

    /*read ps state*/
    epl_sensor_I2C_Write(epld->client,REG_13,R_SINGLE_BYTE,0x01,0);
    epl_sensor_I2C_Read(epld->client);
    setting = gRawData.raw_bytes[0];
    if(((setting>>3)&7)!=0x01)
    {
        LOG_ERR("read ps data in wrong mode\n");
    }
    gRawData.ps_state= !((gRawData.raw_bytes[0]&0x04)>>2);

    /*read channel 1 raw data*/
    epl_sensor_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
    epl_sensor_I2C_Read(client);
#ifdef PS_RAW_8BIT //ices add by 20131230 /*PS_RAW_8BIT*/
    gRawData.ps_raw = gRawData.raw_bytes[1];  //8bit
#else
    gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0]; //16bit
#endif  //ices end by 20131230 /*PS_RAW_8BIT*/




    LOG_INFO("### ps_raw_data  (%d), value(%d) ###\n\n", gRawData.ps_raw, gRawData.ps_state);

    input_report_abs(epld->ps_input_dev, ABS_DISTANCE, gRawData.ps_state);
    input_sync(epld->ps_input_dev);
}
/*----------------------------------------------------------------------------*/

uint32_t raw_convert_to_lux(u16 raw_data)
{
	uint32_t lux = 0;
	if ((gRawData.als_ch1_raw * 15 / als_dynamic_intt_intt_value[dynamic_intt_idx]) > 65535)
		gRawData.als_raw = 65535;
	else
		gRawData.als_raw = gRawData.als_ch1_raw * 15 / als_dynamic_intt_intt_value[dynamic_intt_idx];

	lux = c_gain * raw_data * 15 / als_dynamic_intt_intt_value[dynamic_intt_idx];
	LOG_INFO("[%s]: gRawData.als_raw=%d\r\n", __func__, gRawData.als_raw);
	return lux;
}

/*----------------------------------------------------------------------------*/
static void epl_sensor_read_als(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t now_gain;
    uint32_t lux_temp;
    u16 ch1;

    epl_sensor_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
    epl_sensor_I2C_Read(client);
    now_gain = gRawData.raw_bytes[0];
    if(((now_gain>>3)&7)!=(EPL_COLOR_MODE>>4)) //ALS mode 0x00
    {
        LOG_ERR("read als data in wrong mode\n");
    }

    epl_sensor_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
    epl_sensor_I2C_Read(client);
    ch1= (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    LOG_INFO("dynamic_intt_idx=%d, als_dynamic_intt_intt_value=%d, dynamic_intt_gain=%d, ch1 = %d\r\n",
                                    dynamic_intt_idx, als_dynamic_intt_intt_value[dynamic_intt_idx], dynamic_intt_gain, ch1);
    gRawData.als_ch1_raw = ch1;

	if(gRawData.als_ch1_raw > dynamic_intt_high_thr)
	{
		if(dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)){
			gRawData.als_ch1_raw = dynamic_intt_high_thr;
            lux_temp = raw_convert_to_lux(gRawData.als_ch1_raw);
		}else{
            		gRawData.als_ch1_raw = dynamic_intt_high_thr;
			lux_temp = raw_convert_to_lux(gRawData.als_ch1_raw);
			dynamic_intt_idx++;
		}
    }
    else if(gRawData.als_ch1_raw < dynamic_intt_low_thr)
    {
        if(dynamic_intt_idx == 0){
      		lux_temp = raw_convert_to_lux(gRawData.als_ch1_raw);
        }
        else{
            gRawData.als_ch1_raw = dynamic_intt_low_thr;
  		    lux_temp = raw_convert_to_lux(gRawData.als_ch1_raw);
            dynamic_intt_idx--;
        }
    }
    else
    {
        lux_temp = raw_convert_to_lux(gRawData.als_ch1_raw);
    }
    gRawData.als_lux = lux_temp  / dynamic_intt_min_unit;
    LOG_INFO("-------------------  ALS raw = %d, lux = %d  \r\n\n",  gRawData.als_ch1_raw, gRawData.als_lux);

    input_report_abs(epld->als_input_dev, ABS_MISC, gRawData.als_lux);
    input_sync(epld->als_input_dev);

	dynamic_intt_intt = als_dynamic_intt_intt[dynamic_intt_idx];
    dynamic_intt_gain = als_dynamic_intt_gain[dynamic_intt_idx];
    dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
    dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
}
/*----------------------------------------------------------------------------*/

static void epl_sensor_read_hs(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    u16 data;
    int idx = start_idx+hs_count;

    mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Write(client,REG_16,R_EIGHT_BYTE,0x01,0x00);
    epl_sensor_I2C_Read(client);
    data=((gRawData.raw_bytes[1]<<8)|gRawData.raw_bytes[0])>>6;

    if(data>950 && HS_INTT>0)
        HS_INTT--;

    if(idx>=HS_BUFFER_SIZE)
        idx-=HS_BUFFER_SIZE;

    gRawData.hs_data[idx] = data;

    if(hs_count>=HS_BUFFER_SIZE)
    {
        start_idx++;
        if(start_idx>=HS_BUFFER_SIZE)
            start_idx=0;
    }

    hs_count++;
    if(hs_count>=HS_BUFFER_SIZE)
        hs_count=HS_BUFFER_SIZE;
    mutex_unlock(&sensor_mutex);

}

/*
//====================epl_sensor_set_ps_threshold===============//
//low_thd: The value is psensor interrupt low threshold.
//high_thd:	The value is psensor interrupt hihg threshold.
//When psensor rawdata > hihg_threshold, interrupt pin will be pulled low.
//After interrupt occur, psensor rawdata < low_threshold, interrupt pin will be pulled high.
*/
static int epl_sensor_set_ps_threshold(uint16_t low_thd, uint16_t high_thd)

{
    int ret = 0;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_thd_h = 0;
    uint8_t high_thd_l = 0;
    uint8_t low_thd_h = 0;
    uint8_t low_thd_l = 0;
    high_thd_h = (high_thd >> 8) & 0xFF;
    high_thd_l = high_thd & 0xFF;
    low_thd_h = (low_thd >> 8) & 0xFF;
    low_thd_l = low_thd & 0xFF;
#ifdef PS_RAW_8BIT //ices add by 20131230 /*PS_RAW_8BIT*/
    epl_sensor_I2C_Write(client,REG_2,W_SINGLE_BYTE,0x02, high_thd_l);
    epl_sensor_I2C_Write(client,REG_3,W_SINGLE_BYTE,0x02, 0);
    epl_sensor_I2C_Write(client,REG_4,W_SINGLE_BYTE,0x02, low_thd_l);
    epl_sensor_I2C_Write(client,REG_5,W_SINGLE_BYTE,0x02, 0);
#else
    epl_sensor_I2C_Write(client,REG_2,W_SINGLE_BYTE,0x02, high_thd_l);
    epl_sensor_I2C_Write(client,REG_3,W_SINGLE_BYTE,0x02, high_thd_h);
    epl_sensor_I2C_Write(client,REG_4,W_SINGLE_BYTE,0x02, low_thd_l);
    epl_sensor_I2C_Write(client,REG_5,W_SINGLE_BYTE,0x02, low_thd_h);
#endif //ices end by 20131230 /*PS_RAW_8BIT*/
    return ret;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static void epl_sensor_polling_work(struct work_struct *work)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;

    bool enable_ps = epld->enable_pflag==1 && epld->ps_suspend==0;
    bool enable_als = epld->enable_lflag==1 && epld->als_suspend==0;
    bool enable_hs = epld->enable_hflag==1 && epld->hs_suspend==0;
    bool enable_clr = epld->enable_cflag==1 && epld->clr_suspend==0;
    //LOG_INFO("enable_pflag = %d, enable_lflag = %d, enable_cflag = %d\n", enable_ps, enable_als, enable_clr);

    cancel_delayed_work(&polling_work);
    if( (enable_ps&& epld->polling_mode_ps==1) || enable_als==true || enable_clr==true )
    {
 //queue_delayed_work( epld->epl_wq, &polling_work, msecs_to_jiffies(COLOR_DELAY+2*PS_DELAY+30) );
        queue_delayed_work( epld->epl_wq, &polling_work, msecs_to_jiffies(ALS_DELAY+2*PS_DELAY+30) );
    }

    if(enable_hs)
    {
        if (epld->polling_mode_hs==0)
        {
            epl_sensor_hs_enable(epld, true, true);
        }
        else
        {
            epl_sensor_read_hs();
            epl_sensor_hs_enable(epld, false, true);
            queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(10));
        }
    }
    else
    {
        if(enable_als)
        {
#if 1
            /*enable ALS*/
            epl_sensor_lsensor_enable(epld);
            /*read als*/
            epl_sensor_read_als();
		/*report als event*/
	    //input_report_abs(epld->als_input_dev, ABS_MISC, gRawData.als_lux);
	    //input_sync(epld->als_input_dev);
#else
	        epl_sensor_color_enable(epld, 1);
               epl_sensor_read_color(epld->client);
		input_report_abs(epld->als_input_dev, ABS_MISC, gRawData.color_raws[1]);
		input_sync(epld->als_input_dev);
#endif
	    
        }
        if(enable_clr)
        {
            epl_sensor_color_enable(epld, 1);
            epl_sensor_read_color(epld->client);
        }

        if(enable_ps)
        {
            /*enable psensor*/
            epl_sensor_psensor_enable(epld);
            /*reading psensor if ps is polling mode*/
            if(epld->polling_mode_ps==1)
            {
                epl_sensor_read_ps();
            }
        }
    }

    if(enable_als==false && enable_ps==false && enable_hs==false && enable_clr==false)
    {
        cancel_delayed_work(&polling_work);
        LOG_INFO("disable sensor\n");
        epl_sensor_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
        epl_sensor_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02,EPL_C_P_DOWN);
    }
}
static irqreturn_t epl_sensor_eint_func(int irqNo, void *handle)
{
    struct epl_sensor_priv *epld = (struct epl_sensor_priv*)handle;
    disable_irq_nosync(epld->irq);
    queue_work(epld->epl_wq, &epl_sensor_irq_work);

    return IRQ_HANDLED;
}
static void epl_sensor_eint_work(struct work_struct *work)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    int mode = 0;
	LOG_ERR("epl_sensor_eint_work entry!!!\n");
	
	if(epld->ps_suspend) {
		enable_irq(epld->irq);
        return;
	}
	
    if(epld->enable_hflag)
    {
        epl_sensor_read_hs();
        epl_sensor_hs_enable(epld, true, true);
    }

    else  if(epld->enable_pflag)
    {
        /*check  comparator*/
        epl_sensor_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
        epl_sensor_I2C_Read(client);
        mode = (gRawData.raw_bytes[0]>>3)&7;

        if(mode==0x01 && epld->enable_pflag)
        {
            /*read ps*/
            epl_sensor_read_ps();
        }
        else
        {
            LOG_INFO("error: interrupt in ps\n");
        }
        /*enable interrupt mode, active-low*/
        epl_sensor_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02,EPL_INT_CH1 | EPL_INT_ACTIVE_LOW);
        /*data unlock*/
        epl_sensor_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);
    }

    enable_irq(epld->irq);

}
/*----------------------------------------------------------------------------*/
static int epl_sensor_setup_interrupt(struct epl_sensor_priv *epld)
{
	struct i2c_client *client = epld->client;
	int err = 0;
       unsigned int irq_gpio;
       struct device_node *np = client->dev.of_node;
  	LOG_INFO("epl_sensor_setup_interrupt enter.\n");
	if(np)
	{
		irq_gpio = of_get_named_gpio(np, "epl,irq-gpio", 0);
		LOG_INFO("epl_sensor_setup_interrupt enter. intr_pin:%d\n",irq_gpio);
			if (!gpio_is_valid(irq_gpio))
			{
				LOG_INFO("[ERROR!! gpio unvalid \n");
			}
			else
			{
				LOG_INFO("%s no node",__func__);
			}
	}
	else
	{
		LOG_INFO("epl node is not NUll\n");
	}
        epld->intr_pin = irq_gpio;
        if (irq_gpio < 0) {
		gpio_free(epld->intr_pin);
        }
        if (gpio_is_valid(irq_gpio)) {
                err = gpio_request(irq_gpio, "epl_irq_gpio");
                if (err) {
                        LOG_INFO( "irq gpio request failed");
                }

                err = gpio_direction_input(irq_gpio);
                if (err) {
                        LOG_INFO("set_direction for irq gpio failed\n");
                }
        }
	err = request_irq(epld->irq,epl_sensor_eint_func, IRQF_TRIGGER_FALLING,client->dev.driver->name, epld);
	if(err <0)
	{
		LOG_INFO("request irq pin %d fail for gpio\n",err);
		goto fail_free_intr_pin;
	}
	irq_set_irq_wake(epld->irq, 1);
	LOG_INFO("epl_sensor_setup_interrupt end!!\n");
	return err;
	
fail_free_intr_pin:
    gpio_free(epld->intr_pin);
    return err;
}

static int epl_sensor_initial_sensor(struct epl_sensor_priv *epld)
{
    struct i2c_client *client = epld->client;

    int ret = 0;

    LOG_INFO("epl_sensor_initial_sensor enter!\n");

    ret = epl_sensor_I2C_Read(client);

    if(ret < 0)
        return -EINVAL;
    /* Setting single mode and disable interrupt mode */
    epl_sensor_I2C_Write(client,REG_0,W_SINGLE_BYTE,0x02, EPL_S_SENSING_MODE);
    epl_sensor_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, EPL_INT_DISABLE);

    msleep(2);

    epld->enable_lflag = 0;
    epld->enable_pflag = 0;

    return ret;
}
static int epl_sensor_setup_lsensor(struct epl_sensor_priv *epld)
{
    int err = 0;

    LOG_INFO("epl_sensor_setup_lsensor enter.\n");

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
    return err;
	
err_free_ls_input_device:
    input_free_device(epld->als_input_dev);
    return err;
}
/*----------------------------------------------------------------------------*/
static int epl_sensor_setup_psensor(struct epl_sensor_priv *epld)
{
    int err = 0;
    LOG_INFO("epl_sensor_setup_psensor enter.\n");

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
    return err;

err_free_ps_input_device:
    input_free_device(epld->ps_input_dev);
    return err;
}
/*----------------------------------------------------------------------------*/
static int epl_power_on(struct epl_sensor_priv *data, bool on)
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
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
	}

	return rc;
}
static int epl_power_init(struct epl_sensor_priv *data, bool on)
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
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_SUSPEND
static int epl_sensor_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	LOG_FUN();
	if(epld->enable_lflag){
		epld->als_suspend = 1;
		epld->enable_lflag = 0;
	}
	if(epld->enable_pflag)
	{
		epl_sensor_psensor_enable(epld);
	}
	else
	{
		epld->ps_suspend = 1;
		epl_sensor_I2C_Write(client,REG_8, W_SINGLE_BYTE, 0x02, EPL_C_P_UP);
	//	epl_sensor_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
		epl_sensor_I2C_Write(client,REG_8, W_SINGLE_BYTE, 0x02, EPL_C_P_DOWN);
		epl_power_on(epld, false);
	}
	return 0;
}
static int epl_sensor_resume(struct i2c_client *client)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int err;
	LOG_FUN();
	if(epld->als_suspend)
	{
		epld->als_suspend = 0;
		epld->enable_lflag = 1;
	}
	if(epld->enable_pflag){
		epl_sensor_restart_work();
	}else{
		epld->ps_suspend = 0;
		err=epl_power_on(epld, true);
		if (err) {
			pr_info("power on failed\n");
		}
	}
	return 0;
}
#endif
static ssize_t ls_adc_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
	ssize_t len = 0;
	struct epl_sensor_priv *epld = epl_sensor_obj;
	/*Todo: set levels */

	int current_level = 0;

    if(epld->enable_lflag == 0 && epld->enable_pflag == 0){
        epl_sensor_lsensor_enable(epld);
	    /*read als*/
	    epl_sensor_read_als();
    }
    else{
        epld->enable_lflag = 1;
        epl_sensor_restart_work();
    }

	D("%s: ADC = 0x%04X, Level = %d \n", __func__, gRawData.als_lux, current_level);
	len = sprintf(buf, "ADC[0x%04X] => level %d\n", gRawData.als_lux, current_level);

	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_reg(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
    ssize_t len = 0;
    struct i2c_client *client = epl_sensor_obj->client;

    if(!epl_sensor_obj)
    {
        LOG_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

    return len;
}
/*----------------------------------------------------------------------------*/
/*show sensor status ---------------------------------------------------------*/
static ssize_t epl_sensor_show_status(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
    ssize_t len = 0;

    if(!epl_sensor_obj)
    {
        LOG_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "als_lux=%d    \n", gRawData.als_lux);
    len += snprintf(buf+len, PAGE_SIZE-len, "als_ch1_raw=%d   \n", gRawData.als_ch1_raw);
    len += snprintf(buf+len, PAGE_SIZE-len, "ratio=%d    \n",gRawData.ratio);
    len += snprintf(buf+len, PAGE_SIZE-len, "c_gain=%d       \n", c_gain);
    len += snprintf(buf+len, PAGE_SIZE-len, "INTT=%d  \n", als_dynamic_intt_intt_value[dynamic_intt_idx]);
    len += snprintf(buf+len, PAGE_SIZE-len, "ps_state=%d  \n", gRawData.ps_state);
    len += snprintf(buf+len, PAGE_SIZE-len, "ps_raw=%d  \n", gRawData.ps_raw);

    return len;
}
/*store color sensor Intt ---------------------------------------------------------*/
static ssize_t epl_sensor_store_color_int_time(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    if(!epl_sensor_obj)
    {
        LOG_ERR("epl8852_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d", &COLOR_INTT);
    LOG_INFO("color int time is %d\n", COLOR_INTT);
    return size;
}
/*enable color sensor ---------------------------------------------------------*/
static ssize_t epl_sensor_store_color_enable(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int enabled ;
    LOG_FUN();
    if(!epl_sensor_obj)
    {
        LOG_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d", &enabled);
    epld->enable_cflag = enabled;
    epl_sensor_restart_work();
    return size;
}
/*enable color sensor ---------------------------------------------------------*/
static ssize_t epl_sensor_show_color_raws(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
    int i;
    u16 *tmp = (u16*)buf;
    for(i=0; i<4; i++)
    {
        tmp[i]=gRawData.color_raws[i];
    }
    return 8;
}
/*Setting integration time for ALS mode---------------------------------------*/
static ssize_t epl_sensor_store_als_int_time(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    int als_int_time;
    LOG_FUN();
    sscanf(buf, "%d",&als_int_time);
    dynamic_intt_intt = als_int_time;
    return size;
}
/*show ps calibration raw data------------------------------------------------*/
static ssize_t epl_sensor_show_ps_cal_raw(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
	struct epl_sensor_priv *obj = epl_sensor_obj;
	int ret;
	epl_sensor_psensor_enable(obj);
	epl_sensor_I2C_Write(obj->client,REG_6,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE | EPL_INT_CH1);
	epl_sensor_read_ps();
	 ret = sprintf(buf, "%d\n", gRawData.ps_raw );
	return  ret;
}
/*Setting ps integration time ---------------------------------------------------------*/
static ssize_t epl_sensor_store_ps_int_time(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    LOG_FUN();
    sscanf(buf, "%d",&PS_INTT);
    return size;
}
/*Setting ps integration time ---------------------------------------------------------*/
static ssize_t epl_sensor_store_als_adc(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    int adc;
    LOG_FUN();
    sscanf(buf, "%d",&adc);
    epl_sensor_obj->ALS_ADC = (adc << 2);
    if(adc == 1){
        epl_sensor_obj->ALS_delay=80;
    }
    else{
        epl_sensor_obj->ALS_delay=165;
    }
    epl_sensor_restart_work();
    return size;
}
/*Setting ps H/L threshold ------------------------------------------------------------*/
static ssize_t epl_sensor_store_ps_threshold(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();
    sscanf(buf, "%d,%d",&epld->ps_threshold_low, &epld->ps_threshold_high);
    return size;
}
/*Setting ps polling mode(0: interrupt, 1: polling) ----------------------------------*/
static ssize_t epl_sensor_store_ps_polling_mode(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();
    sscanf(buf, "%d",&epld->polling_mode_ps);
    if(epld->polling_mode_ps==0)
        epl_sensor_setup_interrupt(epld);
    return size;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_hs_enable(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    uint16_t mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    LOG_FUN();
    sscanf(buf, "%hu",&mode);
    if(mode)
        obj->enable_hflag=1;
    else
        obj->enable_hflag=0;
    if(mode)
    {
        HS_INTT = 15;
        start_idx=0;
        hs_count=0;
    }
    epl_sensor_restart_work();
    return size;
}
static ssize_t epl_sensor_show_hs_raws(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
    u16 *tmp = (u16*)buf;
    u16 length= hs_count*1;
    int byte_count=2+length*2;
    int i=0;
    int start = 0;

    mutex_lock(&sensor_mutex);
    tmp[0]= length;
    for(i=start; i<length; i++)
        tmp[i+1]=     gRawData.hs_data[i];
    hs_count=0;
    mutex_unlock(&sensor_mutex);

    return byte_count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ps_adc_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
	int ps_adc = 0;
	int ret;
	struct  epl_sensor_priv *epl8881 = epl_sensor_obj;
	if(epl8881->enable_pflag == 0 && epl8881->enable_lflag == 0){
        epl_sensor_psensor_enable(epl8881);
        epl_sensor_read_ps();
	}
	else if (epl8881->enable_pflag == 1 && epl8881->enable_lflag == 0){
        /*read channel 1 raw data*/
        epl_sensor_I2C_Write(epl8881->client,REG_16,R_TWO_BYTE,0x01,0x00);
        epl_sensor_I2C_Read(epl8881->client);
#ifdef PS_RAW_8BIT //ices add by 20131230 /*PS_RAW_8BIT*/
        gRawData.ps_raw = gRawData.raw_bytes[1];  //8bit
#else
        gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0]; //16bit
#endif  //ices end by 20131230 /*PS_RAW_8BIT*/
	}else{
        epl8881->enable_pflag = 1;
	}
	ps_adc = gRawData.ps_raw;
	ret = sprintf(buf, "ADC[0x%02X], ENABLE = %d, epl8881->intr_pin = %d, "
			"ps_pocket_mode = %d\n",
			ps_adc, epl8881->enable_pflag, epl8881->intr_pin, epl8881->ps_pocket_mode);
	return ret;
}
static ssize_t ps_enable_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
	struct  epl_sensor_priv *epl8881 = epl_sensor_obj;
	int ret;
	LOG_FUN();
	ret = sprintf(buf, "%d\n", epl8881->enable_pflag);
	return ret;
}
static ssize_t ps_enable_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
	int ps_en, err;
	struct  epl_sensor_priv *epl8881 = epl_sensor_obj;
	ps_en = -1;
	sscanf(buf, "%d", &ps_en);
	LOG_INFO("[epl8881] %s: ps_en=%d\n", __func__, ps_en);
	if (ps_en != 0 && ps_en != 1
			&& ps_en != 10 && ps_en != 13 && ps_en != 16)
		return -EINVAL;
	if (ps_en && !epl8881->enable_pflag) {
		err = epl_sensor_psensor_enable(epl8881);
		epl8881->enable_pflag = 1;
	} else if (!ps_en && epl8881->enable_pflag) {
		epl8881->enable_pflag = 0;
	}
	epl_sensor_restart_work();
	return size;
}
static ssize_t als_enable_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
	struct  epl_sensor_priv *epl8881 = epl_sensor_obj;
	int ret;
	LOG_FUN();
	ret = sprintf(buf, "%d\n", epl8881->enable_lflag);
	return ret;
}
static ssize_t als_enable_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
	int als_en;
	struct  epl_sensor_priv *epl8881 = epl_sensor_obj;
	als_en = -1;
	sscanf(buf, "%d", &als_en);
	LOG_INFO("[epl8881] %s: als_en=%d\n", __func__, als_en);
	if(als_en != 0 && als_en !=1)
	{
		return -EINVAL;
	}
	epl8881->enable_lflag = als_en;
	epl_sensor_restart_work();
	return size;
}
static ssize_t ps_delay_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
	struct  epl_sensor_priv *epl8881 = epl_sensor_obj;
	int ret;
	LOG_FUN();

	ret = sprintf(buf, "%d\n", epl8881->ps_delay);
	return ret;
}
static ssize_t epl_sensor_store_ps_delay(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();

    sscanf(buf, "%d",&epld->ps_delay);

    return size;
}
static ssize_t als_delay_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
	struct  epl_sensor_priv *epl8881 = epl_sensor_obj;
	int ret;
	LOG_FUN();

	ret = sprintf(buf, "%d\n", epl8881->ALS_delay);
	return ret;
}
static ssize_t epl_sensor_store_als_delay(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();

    sscanf(buf, "%d",&epld->ALS_delay);

    return size;
}
static ssize_t epl_sensor_store_clr_delay(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();

    sscanf(buf, "%d",&epld->clr_delay);

    return size;
}
static ssize_t ps_thres_high_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
  	struct  epl_sensor_priv *epl8881 = epl_sensor_obj;
	int ret;
	LOG_FUN();

	ret = sprintf(buf, "%d\n", epl8881->ps_threshold_high);
	return ret;
}
static ssize_t ps_thres_high_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();

    sscanf(buf, "%d",&epld->ps_threshold_high);
    epl_sensor_set_ps_threshold(epld->ps_threshold_low,epld->ps_threshold_high);

    return size;
}
static ssize_t ps_thres_low_show(struct kobject *kobj,struct kobj_attribute *attr,char *buf)
{
  	struct  epl_sensor_priv *epl8881 = epl_sensor_obj;
	int ret;
	LOG_FUN();

	ret = sprintf(buf, "%d\n", epl8881->ps_threshold_low);
	return ret;
}
static ssize_t ps_thres_low_store(struct kobject *kobj,struct kobj_attribute *attr,const char *buf, size_t size)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();

    sscanf(buf, "%d",&epld->ps_threshold_low);
    epl_sensor_set_ps_threshold(epld->ps_threshold_low,epld->ps_threshold_high);

    return size;
}

static struct kobj_attribute ls_adc =
__ATTR(epl_ls_adc, 0664, ls_adc_show, NULL);
static struct kobj_attribute elan_reg =
__ATTR(epl_elan_reg, 0644, epl_sensor_show_reg, NULL);
static struct kobj_attribute elan_status =
__ATTR(epl_elan_status, S_IROTH|S_IWOTH, epl_sensor_show_status, NULL);
static struct kobj_attribute color_int_time =
__ATTR(epl_color_int_time, S_IROTH|S_IWOTH, NULL, epl_sensor_store_color_int_time);
static struct kobj_attribute color_enable =
__ATTR(epl_color_enable, S_IROTH|S_IWOTH, NULL, epl_sensor_store_color_enable);
static struct kobj_attribute color_raws =
__ATTR(epl_color_raws, S_IROTH|S_IWOTH, epl_sensor_show_color_raws, NULL);
static struct kobj_attribute als_int_time =
__ATTR(epl_als_int_time, 0664, NULL, epl_sensor_store_als_int_time);
static struct kobj_attribute ps_cal_raw =
__ATTR(epl_ps_cal_raw, 0664, epl_sensor_show_ps_cal_raw, NULL);
static struct kobj_attribute ps_int_time =
__ATTR(epl_ps_int_time, 0664, NULL, epl_sensor_store_ps_int_time);
static struct kobj_attribute als_adc =
__ATTR(epl_als_adc, 0664, NULL, epl_sensor_store_als_adc);
static struct kobj_attribute ps_threshold =
__ATTR(epl_ps_threshold, 0664, NULL, epl_sensor_store_ps_threshold);
static struct kobj_attribute ps_polling_mode =
__ATTR(epl_ps_polling_mode, 0664, NULL, epl_sensor_store_ps_polling_mode);
static struct kobj_attribute hs_enable =
__ATTR(epl_hs_enable, 0666, NULL, epl_sensor_store_hs_enable);
static struct kobj_attribute hs_raws =
__ATTR(epl_hs_raws, 0666, epl_sensor_show_hs_raws, NULL);
static struct kobj_attribute ps_adc =
__ATTR(epl_ps_adc, 0664, ps_adc_show, NULL);
static struct kobj_attribute ps_enable=
__ATTR(epl_ps_enable, 0664, ps_enable_show, ps_enable_store);
static struct kobj_attribute ps_thres_high=
__ATTR(epl_ps_thres_high, 0664, ps_thres_high_show, ps_thres_high_store);
static struct kobj_attribute ps_thres_low=
__ATTR(epl_ps_thres_low, 0664, ps_thres_low_show, ps_thres_low_store);
static struct kobj_attribute als_enable=
__ATTR(epl_als_enable, 0664, als_enable_show, als_enable_store);
static struct kobj_attribute ps_delay=
__ATTR(epl_ps_delay, 0664, ps_delay_show, epl_sensor_store_ps_delay);
static struct kobj_attribute als_delay=
__ATTR(epl_als_delay, 0664, als_delay_show, epl_sensor_store_als_delay);
static struct kobj_attribute clr_delay=
__ATTR(epl_clr_delay, 0664, NULL, epl_sensor_store_clr_delay);

static struct attribute *attributes_alsps[] = {
	&color_int_time.attr,
	&color_enable.attr,
	&color_raws.attr,
	&als_int_time.attr,
	&ls_adc.attr,
	&ps_adc.attr,
	&elan_status.attr,
	&elan_reg.attr,
	&ps_int_time.attr,
	&als_adc.attr,
	&ps_threshold.attr,
	&ps_cal_raw.attr,
	&ps_polling_mode.attr,
	&hs_enable.attr,
	&hs_raws.attr,
	&ps_enable.attr,
	&ps_thres_high.attr,
	&ps_thres_low.attr,
	&als_enable.attr,
	&ps_delay.attr,
	&als_delay.attr,
	&clr_delay.attr,
	NULL,
};
static struct attribute_group attr_group_alsps = {
	.attrs = attributes_alsps,
};
static int create_sysfs_interfaces(struct device *dev)
{
	int err;
	alsps_kobj = kobject_create_and_add("als_ps", &dev->kobj);
	if(!alsps_kobj)
		return -ENOMEM;
	err = sysfs_create_group(alsps_kobj, &attr_group_alsps);
	if (err)
		kobject_put(alsps_kobj);
	return 0;
}
static int epl_sensor_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int err = 0;
	struct epl_sensor_priv *epld ;

	LOG_INFO("elan sensor probe enter.\n");

	epld = kzalloc(sizeof(struct epl_sensor_priv), GFP_KERNEL);
	if (!epld)
		return -ENOMEM;
	epld->client = client;
	err = epl_power_init(epld, true);
	if (err) {
		dev_err(&client->dev, "power init failed");
	}
	
	err = epl_power_on(epld, true);
	if (err) {
		dev_err(&client->dev, "power on failed");
	}
	
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        dev_err(&client->dev,"No supported i2c func what we need?!!\n");
        err = -ENOTSUPP;
        goto i2c_fail;
    }
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
    LOG_INFO("chip id REG 0x17 value = %8x\n", i2c_smbus_read_byte_data(client, 0xB8));
	
	mutex_init(&sensor_mutex);
	epld->lux_per_count = LUX_PER_COUNT;	
	epld->irq = client->irq;
	i2c_set_clientdata(client, epld);
		
    epld->ps_threshold_high = PS_HIGH_THRESHOLD;
    epld->ps_threshold_low = PS_LOW_THRESHOLD;
    epld->polling_mode_ps = PS_POLLING_MODE;
    epld->ps_delay = PS_DELAY;
    epld->ALS_delay = ALS_DELAY;
    epld->clr_delay = COLOR_DELAY;
    epl_sensor_obj = epld;

    epld->epl_wq = create_singlethread_workqueue("elan_sensor_wq");
    if (!epld->epl_wq)
    {
        LOG_ERR("can't create workqueue\n");
        err = -ENOMEM;
        goto err_create_singlethread_workqueue;
    }
	/*check chip ID*/	
    if((i2c_smbus_read_byte_data(client, 0xB8)) != 0x88)
    {
        LOG_INFO("elan ALS/PS sensor is failed. \n");
        goto i2c_fail;
    }

    /*setup lightsensor*/
    err = epl_sensor_setup_lsensor(epld);
    if (err < 0)
    {
        LOG_ERR("epl_sensor_setup_lsensor error!!\n");
        goto err_lightsensor_setup;
    }

    /*setup psensor*/
    err = epl_sensor_setup_psensor(epld);
    if (err < 0)
    {
        LOG_ERR("epl_sensor_setup_psensor error!!\n");
        goto err_psensor_setup;
    }

    /*init sensor*/
    err = epl_sensor_initial_sensor(epld);
    if (err < 0)
    {
        LOG_ERR("fail to initial sensor (%d)\n", err);
        goto err_sensor_setup;
    }

    if (epld->polling_mode_ps==0)
    {
        /*setup interrupt*/
        err = epl_sensor_setup_interrupt(epld);
        if (err < 0)
        {
            LOG_ERR("epl_sensor_setup_interrupt error!\n");
            goto err_sensor_setup;
        }
    }
	wake_lock_init(&g_ps_wlock, WAKE_LOCK_SUSPEND, "ps_wakelock");
	sensor_dev = platform_device_register_simple("elan_alsps", -1, NULL, 0);
	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "device EPL8881 sysfs register failed\n");
		goto err_fail;
	}	
    LOG_INFO("sensor probe success.\n");

    return err;
err_fail:
	   kobject_put(alsps_kobj);
err_lightsensor_setup:
err_psensor_setup:
err_sensor_setup:
    destroy_workqueue(epld->epl_wq);
err_create_singlethread_workqueue:
i2c_fail:
    kfree(epld);
    return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int epl_sensor_remove(struct i2c_client *client)
{
    struct epl_sensor_priv *epld = i2c_get_clientdata(client);

    dev_dbg(&client->dev, "%s: enter.\n", __func__);
    kobject_put(alsps_kobj);
    platform_device_unregister(sensor_dev);
    input_unregister_device(epld->als_input_dev);
    input_unregister_device(epld->ps_input_dev);
    input_free_device(epld->als_input_dev);
    input_free_device(epld->ps_input_dev);
    free_irq(epld->irq,epld);
    destroy_workqueue(epld->epl_wq);
    kfree(epld);
    return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl_sensor_id[] =
{
    { "EPL8881", 0 },
    { }
};

static struct of_device_id epl_match_table[] = {
                { .compatible = "epl,epl8881",},
                { },
};

static struct i2c_driver epl_sensor_driver =
{
    .probe	= epl_sensor_probe,
    .remove	= epl_sensor_remove,
    .id_table	= epl_sensor_id,
    .driver	= {
	.name = "EPL8881",
	.owner = THIS_MODULE,
	.of_match_table =epl_match_table,
    },
#ifdef CONFIG_SUSPEND
    .suspend = epl_sensor_suspend,
    .resume = epl_sensor_resume,
#endif
};

static int __init epl_sensor_init(void)
{
    return i2c_add_driver(&epl_sensor_driver);
}

static void __exit  epl_sensor_exit(void)
{
    i2c_del_driver(&epl_sensor_driver);
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
module_init(epl_sensor_init);
module_exit(epl_sensor_exit);
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Renato Pan <renato.pan@eminent-tek.com>");
MODULE_DESCRIPTION("ELAN epl8881 driver");
MODULE_LICENSE("GPL");






