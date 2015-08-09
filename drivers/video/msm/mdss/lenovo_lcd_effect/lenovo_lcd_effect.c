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

#include <soc/qcom/socinfo.h> //lenovo.sw2 houdz1 add for of_board_is_z2()


#include "lenovo_lcd_effect.h"

#define MAX_SUPPORT_EFFECT_COUNT 10
#define CUSTOM_MODE_INDEX 0

static int g_lcd_curret_effect_level[MAX_SUPPORT_EFFECT_COUNT];
static int g_lcd_curret_mode;

int g_lcd_effect_log_on = false;


#define LCD_EFFECT_LOG(fmt, arg...) \
       do {                                                    \
           if(g_lcd_effect_log_on) LCD_EFFECT_LOG_PRINT(ANDROID_LOG_ERROR, "LCD_EFFECT", fmt, ##arg);  \
        }while(0)


static char lcd_cabc[2] = {0x55,0x00};
struct dsi_cmd_desc cabc_cmd[1] =
{
	{{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(lcd_cabc)},lcd_cabc},
};


static int lcd_get_effect_support(struct lenovo_lcd_effect *lcdEffect,struct hal_panel_data *panel_data)
{
	int i;
	struct lenovo_lcd_effect_data *pEffectData = lcdEffect->pEffectData;

	panel_data->effect_cnt = lcdEffect->effectDataCount;
	for (i = 0; i < panel_data->effect_cnt; i++) 
	{
		
		if(pEffectData->is_support == 1)
		{
			memcpy(panel_data->effect[i].name, pEffectData->effect_name, strlen(pEffectData->effect_name));
			LCD_EFFECT_LOG("%s:support - %s\n",__func__,panel_data->effect[i].name);
		}
		else
		{
			memcpy(panel_data->effect[i].name, "null",5);
			LCD_EFFECT_LOG("%s:support - %s\n",__func__,panel_data->effect[i].name);
		}
		pEffectData ++ ;
	}
	LCD_EFFECT_LOG("%s:GET_EFFECT_NUM = 0x%x\n",__func__,panel_data->effect_cnt);
	return panel_data->effect_cnt;
}


static int lcd_get_effect_max_level(struct lenovo_lcd_effect_data *effectData, int index)
{
	struct lenovo_lcd_effect_data *pEffectData = effectData;
	pEffectData += index;
	LCD_EFFECT_LOG("%s: name: [%s] index: [%d] max_level: [%d]\n", __func__,pEffectData->effect_name, index, pEffectData->max_level);
	return pEffectData->max_level;
}


static int lcd_get_effect_level(struct lenovo_lcd_effect_data *effectData, int index)
{
	struct lenovo_lcd_effect_data *pEffectData = effectData;
	
	if((index <0)||(pEffectData ==NULL)) return -1;
	
	pEffectData +=index;
	LCD_EFFECT_LOG("%s: name: [%s] index: [%d] level: [%d]\n", __func__,pEffectData->effect_name, index, g_lcd_curret_effect_level[index]);
	return g_lcd_curret_effect_level[index];
}

static int lcd_get_mode_support(struct lenovo_lcd_effect *lcdEffect,struct hal_panel_data *panel_data)
{
	int i;
	struct lenovo_lcd_mode_data *pModeData = lcdEffect->pModeData;

	for (i = 0; i < lcdEffect->modeDataCount; i++) 
	{
		if(pModeData->is_support == 1)
		{
			memcpy(panel_data->mode[i].name, pModeData->mode_name, strlen(pModeData->mode_name));
			LCD_EFFECT_LOG("%s:support - %s\n",__func__,panel_data->mode[i].name);
		}
		else 	
		{
			memcpy(panel_data->mode[i].name, "null",5);
			LCD_EFFECT_LOG("%s:support - %s\n",__func__,panel_data->mode[i].name);
		}		
		
		pModeData++;
	}
	panel_data->mode_cnt = lcdEffect->modeDataCount;
	LCD_EFFECT_LOG("%s: mode_cnt=%d\n", __func__,panel_data->mode_cnt);
	return panel_data->mode_cnt;
}


static int lcd_get_mode_level(struct lenovo_lcd_mode_data *modeData,struct hal_panel_data *panel_data)
{
	struct lenovo_lcd_mode_data *pModeData = modeData;

	pModeData += g_lcd_curret_mode;
	LCD_EFFECT_LOG("%s: name: [%s]  mode: [%d]\n", __func__,pModeData->mode_name, g_lcd_curret_mode);
	memcpy(panel_data->mode[g_lcd_curret_mode].name,pModeData->mode_name,sizeof(pModeData->mode_name));
	return g_lcd_curret_mode;
}

static int is_custom_mode(void)
{
	return (g_lcd_curret_mode==CUSTOM_MODE_INDEX);
}


int lenovo_get_index_by_name(struct lenovo_lcd_effect *pLcdEffect,char *name)
{
	struct lenovo_lcd_effect_data *pEffectData = pLcdEffect->pEffectData;
	int effectCount = pLcdEffect->effectDataCount;
	int i;
	for(i=0;i<effectCount;i++)
	{
		if (!strcmp(name,pEffectData->effect_name)) return i;
		pEffectData++;
	}
	return -EINVAL;
}


static void dsi_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}


static int lenovo_lcd_set_mode(struct mdss_dsi_ctrl_pdata *ctrl_data,int mode)
{
	struct dsi_panel_cmds panel_cmds;

	struct lenovo_lcd_effect *lcdEffect=&(ctrl_data->lenovoLcdEffect);
	struct lenovo_lcd_mode_data *pModeData =  lcdEffect->pModeData;

	if(mode >(lcdEffect->modeDataCount)) return -1;
	

	LCD_EFFECT_LOG("%s:mode = %d \n",__func__,mode);
	pModeData += mode;
	if(pModeData->is_support != 1)
	{
		pr_err("[houdz1]%s:the mode(%d) is not support\n",__func__,mode);
		return -1;
	}
	ctrl_data->is_ultra_mode = 0;
	memset(&panel_cmds, 0, sizeof(panel_cmds));
	panel_cmds.cmds =pModeData->cmds;
	panel_cmds.cmd_cnt = pModeData->cmds_cnt;
	if(mode == MODE_INDEX_ULTRA) ctrl_data->is_ultra_mode = 1;
	dsi_cmds_send(ctrl_data,&panel_cmds);
	/*if (is_show_lcd_param)
		show_lcd_param(lcd_cmd.cmd, lcd_cmd.cnt);*/
	g_lcd_curret_mode = mode;
	return 0;
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 dsi_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;
	int ret =0;

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = fxn; /* call back */
	ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

	return ret;
}


int lcd_effect_set_cabc_reg(struct mdss_dsi_ctrl_pdata *ctrl,int regData)
{
	unsigned char read_cabc=0;
	unsigned char timeout=4;
	struct dsi_panel_cmds dsi_cmd;	

	char cabc_mode[2] = {0x55,0x00};
	struct dsi_cmd_desc cabc_mode_cmd = {{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(cabc_mode)},cabc_mode};

		
	cabc_mode[1] = regData;
	LCD_EFFECT_LOG("%s:regData = 0x%x\n",__func__,cabc_mode[1]);
	memset(&dsi_cmd,0,sizeof(struct dsi_panel_cmds));
	dsi_cmd.cmds = &cabc_mode_cmd;
	dsi_cmd.cmd_cnt = 1;
	dsi_cmds_send(ctrl,&dsi_cmd);
	do
	{
		dsi_cmd_read(ctrl,0x56,0x00,NULL,&read_cabc,1); 
		LCD_EFFECT_LOG("read_cabc=0x%x,timeout=%d\n",read_cabc,timeout);
		if(read_cabc == regData) break;
	}while(timeout--);
	if(timeout == 0) return -1;
	return 0;
}




int lcd_effect_send_cmd(struct mdss_dsi_ctrl_pdata *ctrl,struct lenovo_lcd_effect_data *pEffectData,int cmd_index)
{
	struct dsi_panel_cmds dsi_cmd;
	int i;
	struct dsi_ctrl_hdr *pdchdr;
	char *payload;
	struct dsi_cmd_desc  *pdesc;
	
	memset(&dsi_cmd,0,sizeof(struct dsi_panel_cmds));

	dsi_cmd.cmds =  (pEffectData->cmds)+cmd_index*(pEffectData->cmds_cnt);
	dsi_cmd.cmd_cnt =  pEffectData->cmds_cnt;
	dsi_cmd.link_state = DSI_LP_MODE;
	if(g_lcd_effect_log_on)
	{
		LCD_EFFECT_LOG("[houdz]%s: addr=0x%x,cmd_cnt=%d\n",__func__,(unsigned int)(dsi_cmd.cmds),dsi_cmd.cmd_cnt);
		pdesc = dsi_cmd.cmds;
		pdchdr = &(pdesc->dchdr);
		payload = pdesc->payload;
		for(i =0; i<(pdchdr->dlen);i++) LCD_EFFECT_LOG("[houdz]%s:0x%x\n",__func__,*(payload+i));
	}
	dsi_cmds_send(ctrl,&dsi_cmd);
	return 0;
}


int lenovo_lcd_effect_handle(struct mdss_dsi_ctrl_pdata *ctrl_data,struct hal_panel_ctrl_data *hal_ctrl_data)
{
	int ret = 0;
	struct lenovo_lcd_effect *lcdEffect=&(ctrl_data->lenovoLcdEffect);
	struct lenovo_lcd_effect_data *pEffectData =  lcdEffect->pEffectData;
	struct lenovo_lcd_mode_data *pModeData =  lcdEffect->pModeData;
	
	if((lcdEffect == NULL)||(pEffectData ==NULL)||(pModeData == NULL)) return -1;

	LCD_EFFECT_LOG("%s:hal_ctrl_data->id = %d\n",__func__,hal_ctrl_data->id);
	switch(hal_ctrl_data->id)
	{
		case GET_EFFECT_NUM:
			ret = lcd_get_effect_support(lcdEffect,&(hal_ctrl_data->panel_data));
			break;

		case GET_EFFECT_LEVEL:
			ret = lcd_get_effect_max_level(pEffectData,hal_ctrl_data->index);
			break;
		case GET_EFFECT:
			ret = lcd_get_effect_level(pEffectData,hal_ctrl_data->index);
			break;
		case GET_MODE_NUM:
			ret = lcd_get_mode_support(lcdEffect,&(hal_ctrl_data->panel_data));
			break;
		case GET_MODE:
			ret = lcd_get_mode_level(pModeData,&(hal_ctrl_data->panel_data));
			break;
		case SET_EFFECT:
			if (is_custom_mode()||(hal_ctrl_data->index ==EFFECT_INDEX_INVERSE))
			{
				if(lcdEffect->pFuncSetEffect == NULL) ret =-1;
				else
				{
					ret = (*(lcdEffect->pFuncSetEffect))(ctrl_data,hal_ctrl_data->index, hal_ctrl_data->level);
					if(ret == 0) 
					g_lcd_curret_effect_level[hal_ctrl_data->index] = hal_ctrl_data->level;					
				}
			}
			else 
			{
				pModeData += g_lcd_curret_mode;
				pr_err("%s:(%s) can't support change effect\n",__func__,pModeData->mode_name);
				ret = -EINVAL;
			}
			break;
		case SET_MODE:
			ret = lenovo_lcd_set_mode(ctrl_data,hal_ctrl_data->mode);
			break;
		default:
			break;
	}
	return ret;
}

extern int lenovo_lcd_effect_init_for_z2(struct mdss_dsi_ctrl_pdata *ctrl_pdata);

int lenovo_lcd_effect_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	memset(g_lcd_curret_effect_level,0x0,sizeof(g_lcd_curret_effect_level));
	g_lcd_curret_effect_level[EFFECT_INDEX_HUE] =15;
	g_lcd_curret_mode = 2;

	if(of_board_is_z2())
	{
		if(lenovo_lcd_effect_init_for_z2(ctrl_pdata)) return -1;
		LCD_EFFECT_LOG("%s--of_board_is_z2\n",__func__);
		return 0;
	}
	return 0;
}


int lenovo_lcd_effect_reset(struct mdss_dsi_ctrl_pdata *ctrl_data)
{
	struct lenovo_lcd_effect *lcdEffect=&(ctrl_data->lenovoLcdEffect);
	struct lenovo_lcd_effect_data *pEffectData =  lcdEffect->pEffectData;
	int ret,i;
	ret = 0;
	if(lcdEffect->pFuncSetEffect == NULL) ret =-1;
	else if(is_custom_mode())
	{
             for(i=0;i<(lcdEffect->effectDataCount);i++,pEffectData++)
              {
			 if(pEffectData->is_support ==1) 
				ret = (*(lcdEffect->pFuncSetEffect))(ctrl_data,i,g_lcd_curret_effect_level[i] );
              }
	}
	else 
	{
		ret = (*(lcdEffect->pFuncSetEffect))(ctrl_data,EFFECT_INDEX_INVERSE,g_lcd_curret_effect_level[EFFECT_INDEX_INVERSE]);
		ret = lenovo_lcd_set_mode(ctrl_data,g_lcd_curret_mode);
	}
	return ret;
}

char lcd_inverse[2] = {0x20,0x00};
struct dsi_cmd_desc lcd_inverse_cmd[1] = 
{
	{{DTYPE_DCS_WRITE, 1, 0, 0, 1, sizeof(lcd_inverse)},lcd_inverse},
};



