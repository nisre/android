

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include <linux/msm_mdp.h>

#include "../mdss.h"
#include "../mdss_panel.h"
#include "../mdss_dsi.h"
#include "../mdss_debug.h"

#include "lenovo_fb.h"
#include "../mdss_fb.h"

extern int lenovo_lcd_effect_handle(struct mdss_dsi_ctrl_pdata *ctrl_data,struct hal_panel_ctrl_data *hal_ctrl_data);
int lenovo_fb_lcd_effect_handle(struct msm_fb_data_type *mfd,struct hal_panel_ctrl_data *hal_ctrl_data)
{
	int ret = 0;

	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		return -1;
	}
	ret = lenovo_lcd_effect_handle(ctrl_pdata,hal_ctrl_data);
	
	return ret;	
}
#if 0
ssize_t lenovo_fb_get_panel_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret=0;

	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
		
		
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	else ret= snprintf(buf, PAGE_SIZE, "panel name: %s\n",(char*)(&(ctrl_pdata->panel_name[0])));
	return ret;
}

extern int g_lcd_effect_log_on;
ssize_t lenovo_fb_set_lcd_effect_debug_onoff(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)  
{
	unsigned long level;
	if (kstrtoul(buf, 0, &level))  return -EINVAL;
	if(level) g_lcd_effect_log_on = true;
	else g_lcd_effect_log_on = false;
	pr_err("[houdz1]%s:g_lcd_effect_log_on = %d\n",__func__,g_lcd_effect_log_on);
	return count;
}

ssize_t lenovo_fb_get_lcd_effect_debug_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	ret = snprintf(buf, PAGE_SIZE, "%d\n",g_lcd_effect_log_on);
	return ret;
}
#endif

extern int lenovo_get_index_by_name(struct lenovo_lcd_effect *pLcdEffect,char *name);
ssize_t lenovo_fb_get_cabc(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	int index,level;
	struct hal_panel_ctrl_data hal_ctrl_data;
		
	
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	
	memset(&hal_ctrl_data,0,sizeof(struct hal_panel_ctrl_data));
	
	index = lenovo_get_index_by_name(&(ctrl_pdata->lenovoLcdEffect),"CABC");
	pr_err("[houdz1]%s:index = %d\n",__func__,index);
	hal_ctrl_data.id =GET_EFFECT ;
	hal_ctrl_data.index =index ;
	level = lenovo_lcd_effect_handle(ctrl_pdata,&hal_ctrl_data);
	ret = snprintf(buf, PAGE_SIZE, "%d\n",level);
	return ret;
}

ssize_t lenovo_fb_set_cabc(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)  ///
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	int index;
	unsigned long level;
	struct hal_panel_ctrl_data hal_ctrl_data;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	

	if (kstrtoul(buf, 0, &level))  return -EINVAL;
	index = lenovo_get_index_by_name(&(ctrl_pdata->lenovoLcdEffect),"CABC");


	hal_ctrl_data.level = level;
	hal_ctrl_data.id =SET_EFFECT;
	hal_ctrl_data.index =index ;
	ret = lenovo_lcd_effect_handle(ctrl_pdata,&hal_ctrl_data);
	return count;
}

ssize_t lenovo_fb_get_lcd_supported_effect(struct device *dev, struct device_attribute *attr, char *buf)
{

	ssize_t ret = 0,i,count;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct hal_panel_ctrl_data hal_ctrl_data;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	
	hal_ctrl_data.id =GET_EFFECT_NUM;
	count = lenovo_lcd_effect_handle(ctrl_pdata,&hal_ctrl_data);

	for (i = 0; i <count; i++) 
	{
		ret += snprintf(buf + ret, PAGE_SIZE, "%s\n", hal_ctrl_data.panel_data.effect[i].name);
	}

	return ret;
}

ssize_t lenovo_fb_get_lcd_supported_mode(struct device *dev, struct device_attribute *attr, char *buf)
{

	ssize_t ret = 0,i,count;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct hal_panel_ctrl_data hal_ctrl_data;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	
	hal_ctrl_data.id =GET_MODE_NUM;
	count = lenovo_lcd_effect_handle(ctrl_pdata,&hal_ctrl_data);

	for (i = 0; i <count; i++) 
	{
		ret += snprintf(buf + ret, PAGE_SIZE, "%s\n", hal_ctrl_data.panel_data.mode[i].name);
	}

	return ret;
}

ssize_t lenovo_fb_get_ce(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	int index;
	int level;
	struct hal_panel_ctrl_data hal_ctrl_data;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	
	index = lenovo_get_index_by_name(&(ctrl_pdata->lenovoLcdEffect),"SATURATION");

	hal_ctrl_data.id =GET_EFFECT;
	hal_ctrl_data.index =index ;
	level = lenovo_lcd_effect_handle(ctrl_pdata,&hal_ctrl_data);
	
	ret = snprintf(buf, PAGE_SIZE, "%d\n",level);
	return ret;
}

ssize_t lenovo_fb_set_ce(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	int index;
	unsigned long level;
	struct hal_panel_ctrl_data hal_ctrl_data;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	

	if (kstrtoul(buf, 0, &level))  return -EINVAL;
	
	index = lenovo_get_index_by_name(&(ctrl_pdata->lenovoLcdEffect),"SATURATION");

	hal_ctrl_data.id =SET_EFFECT;
	hal_ctrl_data.index =index ;
	hal_ctrl_data.level =level;
	ret = lenovo_lcd_effect_handle(ctrl_pdata,&hal_ctrl_data);
	return count;
}


static int g_effect_index =0;

ssize_t lenovo_fb_get_effect_index(struct device *dev, struct device_attribute *attr, char *buf)
{
	return g_effect_index;
}



ssize_t lenovo_fb_set_effect_index(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned long index;
	if (kstrtoul(buf, 0, &index)) 	return -EINVAL;
	g_effect_index =index;
	return count;
}
ssize_t lenovo_fb_get_effect(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	//unsigned long index;
	int level;
	struct hal_panel_ctrl_data hal_ctrl_data;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	

	//if (kstrtoul(buf, 0, &index)) 	return -EINVAL;
	

	hal_ctrl_data.id =GET_EFFECT;
	hal_ctrl_data.index =g_effect_index ;
	level = lenovo_lcd_effect_handle(ctrl_pdata,&hal_ctrl_data);
	
	ret = snprintf(buf, PAGE_SIZE, "%d\n",level);
	return ret;
}

ssize_t lenovo_fb_set_effect(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	int index,level;
	unsigned long data;
	struct hal_panel_ctrl_data hal_ctrl_data;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	

	if (kstrtoul(buf, 0, &data))  return -EINVAL;
	index = (data >> 4) & 0xf;
	level = data & 0xf;
	

	hal_ctrl_data.id =SET_EFFECT;
	hal_ctrl_data.index =index ;
	hal_ctrl_data.level =level;
	ret = lenovo_lcd_effect_handle(ctrl_pdata,&hal_ctrl_data);
	return count;
}

ssize_t lenovo_fb_get_mode(struct device *dev, struct device_attribute *attr,char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	int level;
	struct hal_panel_ctrl_data hal_ctrl_data;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	
	hal_ctrl_data.id =GET_MODE;
	level = lenovo_lcd_effect_handle(ctrl_pdata,&hal_ctrl_data);

	pr_err("[houdz1]%s:mode =%d,name = %s\n",__func__,level,hal_ctrl_data.panel_data.mode[level].name);
	
	ret = snprintf(buf, PAGE_SIZE, "%s\n", hal_ctrl_data.panel_data.mode[level].name);
	return ret;
}

ssize_t lenovo_fb_set_mode(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	unsigned long data;
	struct hal_panel_ctrl_data hal_ctrl_data;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	if (ctrl_pdata == NULL) 
	{
		pr_err("[houdz1]mdss_panel_set_cabc fail:(ctrl_pdata == NUL\n");
		ret = -1;
	}
	

	if (kstrtoul(buf, 0, &data))  return -EINVAL;

	
	hal_ctrl_data.mode = data; 
	hal_ctrl_data.id =SET_MODE;
	ret= lenovo_lcd_effect_handle(ctrl_pdata,&hal_ctrl_data);
	return count;
}


ssize_t lenovo_fb_set_dimming(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long mode;

	if (kstrtoul(buf, 0, &mode))
		return -EINVAL;

	return count;
}

ssize_t lenovo_fb_get_dimming(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "not support\n");
	return ret;
}

ssize_t lenovo_fb_set_bl_gpio_level(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long mode;

	if (kstrtoul(buf, 0, &mode))
		return -EINVAL;

	return count;
}

ssize_t lenovo_fb_get_bl_gpio_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "not support\n");
	return ret;
}

#if 0

extern lenovo_disp_feature_info_t disp_feature_info[2];
extern lenovo_disp_feature_state_t disp_feature_state[2];
//static BOOL LCM_Feature_set_resume = FALSE;



extern int lcd_effect_set_inversion(struct mdss_dsi_ctrl_pdata *ctrl,int inversion);
static int mdss_fb_set_inversion(struct msm_fb_data_type *mfd,int inversion)
{

	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata ;
			
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	lcd_effect_set_inversion(ctrl_pdata,inversion);
	return 0;
}


extern int lcd_effect_set_cabc(struct mdss_dsi_ctrl_pdata *ctrl,int cabc);
static int mdss_fb_set_cabc(struct msm_fb_data_type *mfd,int cabc)
{

	struct mdss_panel_data *pdata=dev_get_platdata(&mfd->pdev->dev);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata ;
			
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,panel_data);
	lcd_effect_set_cabc(ctrl_pdata,cabc);
	return 0;
}




int mdss_fb_set_lcm_feature_mode(struct msm_fb_data_type *mfd,lenovo_disp_feature_state_t *states)
{
	static unsigned int cabc_mode,inverse_mode/*,gamma_mode,ie_mode*/;


	if((states->cabc_mode>=0)&&(cabc_mode!=states->cabc_mode)){
		cabc_mode = states->cabc_mode;
		 mdss_fb_set_cabc(mfd,cabc_mode);
	}
	if((states->inverse_mode>=0)&&(inverse_mode!=states->inverse_mode)){
		inverse_mode = states->inverse_mode;
		mdss_fb_set_inversion(mfd,inverse_mode);
	}
	#if 0
	if((states->gamma_mode>=0)&&(gamma_mode!=states->gamma_mode)){
		gamma_mode = states->gamma_mode;
		DISP_SetGammaMode(gamma_mode);
	}
	if((states->ie_mode>=0)&&(ie_mode!=states->ie_mode)){
		ie_mode = states->ie_mode;
		DISP_SetIeMode(ie_mode);
	}
	#endif
    return 0; 
}


#endif



