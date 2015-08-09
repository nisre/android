#include "../mdss_panel.h"
#include "../mdss_dsi_cmd.h"
#include "lenovo_lcd_effect.h"
#include "../mdss_dsi.h"


/***********************************
*novatek effect: cabc
************************************/

extern int g_lcd_effect_log_on;


#define LCD_EFFECT_LOG(fmt, arg...) \
       do {                                                    \
           if(g_lcd_effect_log_on) LCD_EFFECT_LOG_PRINT(ANDROID_LOG_ERROR, "LCD_EFFECT1", fmt, ##arg);  \
        }while(0)


extern struct dsi_cmd_desc lcd_inverse_cmd[1];
extern char lcd_inverse[2];
extern int lcd_effect_send_cmd(struct mdss_dsi_ctrl_pdata *ctrl,struct lenovo_lcd_effect_data *pEffectData,int cmd_index);

static int lenovo_show_lcd_param(struct dsi_cmd_desc *cmds, int cmd_cnt)
{
	int i, j;

	printk("======================================= cmds_cnt %d =========================================\n", cmd_cnt);
	for (i = 0; i < cmd_cnt; i++) {
		printk("%2x %2x %2x %2x %2x %2x ", cmds[i].dchdr.dtype,
				cmds[i].dchdr.last,
				cmds[i].dchdr.vc,
				cmds[i].dchdr.ack,
				cmds[i].dchdr.wait,
				cmds[i].dchdr.dlen);
		for (j = 0; j < cmds[i].dchdr.dlen; j++) {
			printk("%2x ", cmds[i].payload[j]);
		}
		printk("\n");
	}
	pr_debug("===========================================================================================\n");
	return 0;
}

extern int lcd_effect_set_cabc_reg(struct mdss_dsi_ctrl_pdata *ctrl,int regData);


int lenovo_set_effect_level_hx8394d(void *pData,int index,int level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_data = pData;
	struct lenovo_lcd_effect *lcdEffect=&(ctrl_data->lenovoLcdEffect);
	struct lenovo_lcd_effect_data *pEffectData =  lcdEffect->pEffectData;
	int maxLevel = pEffectData->max_level;
	int ret=0;
	struct dsi_cmd_desc *custom_mode_cmds,*gamma_cmds,*ce_cmds;
	int i;

	LCD_EFFECT_LOG("%s:index = %d,level=%d\n",__func__,index,level);
	
	if(index > (lcdEffect->effectDataCount)) 
	{
		pr_err("[houdz1]%s:index(%d)>effectDataCount\n",__func__,index);
		return -1;
	}
	pEffectData += index;		
	if(pEffectData->is_support != 1)
	{
		pr_err("[houdz1]%s:the effect(%s) is not support\n",__func__,pEffectData->effect_name);
		return -1;
	}
	maxLevel = pEffectData->max_level;
	if((level <0) ||(level > maxLevel)) 
	{
		pr_err("[houdz1]%s:level(%d) is errort\n",__func__,level);
		return -1;
	}

	custom_mode_cmds = (ctrl_data->custom_mode_cmds).cmds;

	switch(index)	
	{
		case EFFECT_INDEX_CABC:
			lcd_effect_set_cabc_reg(ctrl_data,level);
			break;
		case EFFECT_INDEX_SAT:
			LCD_EFFECT_LOG("[houdz]%s:EFFECT_INDEX_SAT level = %d\n",__func__,level);

			lcd_effect_send_cmd(ctrl_data,pEffectData,level);
			if(custom_mode_cmds != NULL)
			{
				custom_mode_cmds += 7;
				ce_cmds = (pEffectData->cmds)+level*(pEffectData->cmds_cnt);
				for(i=0;i<(pEffectData->cmds_cnt);i++)
				{
					memcpy(custom_mode_cmds,ce_cmds,sizeof(struct dsi_cmd_desc));
					custom_mode_cmds += 1;
					ce_cmds += 1;
				}
				if(g_lcd_effect_log_on) lenovo_show_lcd_param((ctrl_data->custom_mode_cmds).cmds,(ctrl_data->custom_mode_cmds).cmd_cnt);
			}
			break;
		case EFFECT_INDEX_CONTRAST:
			
			break;
		case EFFECT_INDEX_HUE:
			LCD_EFFECT_LOG("[houdz]%s:EFFECT_INDEX_HUE level = %d\n",__func__,level);

			lcd_effect_send_cmd(ctrl_data,pEffectData,level);
			if(custom_mode_cmds != NULL)
			{
				custom_mode_cmds += 1;
				gamma_cmds = (pEffectData->cmds)+level*(pEffectData->cmds_cnt);
				for(i=0;i<(pEffectData->cmds_cnt);i++)
				{
					memcpy(custom_mode_cmds,gamma_cmds,sizeof(struct dsi_cmd_desc));
					custom_mode_cmds += 1;
					gamma_cmds += 1;
				}
				if(g_lcd_effect_log_on) lenovo_show_lcd_param((ctrl_data->custom_mode_cmds).cmds,(ctrl_data->custom_mode_cmds).cmd_cnt);
			}
			break;
		case EFFECT_INDEX_SRE:
			
			break;
		case EFFECT_INDEX_INVERSE:
			if(level) lcd_inverse[0]= 0x21;
			else lcd_inverse[0]=0x20;
			lcd_effect_send_cmd(ctrl_data,pEffectData,0);
			break;
		default:
			pr_err("[houdz1]%s:(index = %d) is not support\n",__func__,index);		
			break;

	}
	return ret;
}






