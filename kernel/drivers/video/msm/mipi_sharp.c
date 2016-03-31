/* drivers/video/msm/mipi_sharp.c  (Display Driver)
 *
 * Copyright (C) 2011-2012 SHARP CORPORATION
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mdp.h"
#include "mdp4.h"
#include "mipi_sharp.h"
#include <sharp/shdisp_kerl.h>
#include <sharp/shlcdc_eventlog.h>
#include <linux/fb.h>
#include <linux/hrtimer.h>

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */

#define MIPI_SHARP_RW_MAX_SIZE          SHDISP_LCDDR_BUF_MAX
#define MIPI_SHARP_R_SIZE               10
#define	LCD_RECOVERY_LOOP_MAX           10

enum {
    LCD_RECOVERY_STOP,
    LCD_RECOVERY_ACTIVE,
    NUM_RECOVERY_STATUS
};

static int lcd_recovering = LCD_RECOVERY_STOP;
static int suspended_recovery_flg = 0;
static struct dsi_buf sharp_tx_buf;
static struct dsi_buf sharp_rx_buf;
static DEFINE_MUTEX(ryoma_lpf_register_lock);

static void mipi_sharp_lcd_init_det_info(void);

void msm_lcd_recovery_subscribe(void)
{
    struct shdisp_subscribe subscribe;
    
    if (lcd_recovering != LCD_RECOVERY_STOP) {
        return;
    }
    
    subscribe.irq_type = SHDISP_IRQ_TYPE_DET;
    subscribe.callback = msm_lcd_recovery;
    shdisp_api_event_subscribe(&subscribe);
}

void msm_lcd_recovery_unsubscribe(void)
{
    if (lcd_recovering != LCD_RECOVERY_STOP) {
        return;
    }
    
    shdisp_api_event_unsubscribe(SHDISP_IRQ_TYPE_DET);
}

DEFINE_MUTEX(recovery_lock);
void lock_recovery(void)
{
    mutex_lock(&recovery_lock);
}

void unlock_recovery(void)
{
    mutex_unlock(&recovery_lock);
}

void msm_lcd_recovery(void)
{
	struct msm_fb_data_type *mfd;
	struct fb_info *info = NULL;
	int ret;
	int i;

	suspended_recovery_flg = 0;

	if (!num_registered_fb){
		mipi_sharp_lcd_init_det_info();
		return;
	}
	info = registered_fb[0];
	if (!info){
		mipi_sharp_lcd_init_det_info();
		return;
	}

	
	lock_recovery();
    if(mipi_dsi_get_err_contention_lp1_flag() == 3) {
        SHLCDC_ERRLOG_REC(FB,LCD,MSM_FB_KERL,EEVENTID_FB_RECOVERY_ERR,0);
        MSM_FB_WARNING("msm_lcd_recovery already retry over. return\n");
    	unlock_recovery();
        return;
    }

    mfd = (struct msm_fb_data_type *)info->par;

    if(!mfd->panel_power_on) {
        SHLCDC_ERRLOG_REC(FB,LCD,MSM_FB_KERL,EEVENTID_FB_RECOVERY_ERR,1);
        MSM_FB_WARNING("msm_lcd_recovery already suspend. return\n");
        suspended_recovery_flg = 1;
        unlock_recovery();
        return;
    }

	mipi_sharp_stat_inc(SHDISP_STAT_INC_RECOVERY);

	SHLCDC_EVENTLOG_REC(FB,LCD,MSM_FB_KERL,EEVENTID_FB_RECOVERY,0);
	MSM_FB_WARNING("msm_lcd_recovery start\n");

	lcd_recovering = LCD_RECOVERY_ACTIVE;

	for (i=0; i<LCD_RECOVERY_LOOP_MAX; i++) {

		
		msm_fb_suspend_sub_recovery(mfd);
		
		
		mipi_sharp_delay_us(40000);
	
		
		shdisp_api_do_recovery();

		
		msm_fb_resume_sub_recovery(mfd);

		msm_fb_resume_sub_recovery_phase2(info);

		
		ret = shdisp_api_check_recovery();

		
		if (ret == SHDISP_RESULT_SUCCESS) {
			
			lcd_recovering = LCD_RECOVERY_STOP;
			
			mipi_sharp_lcd_init_det_info();

			SHLCDC_EVENTLOG_REC(FB,LCD,MSM_FB_KERL,EEVENTID_FB_RECOVERY,1);
			MSM_FB_WARNING("msm_lcd_recovery completed\n");

			
			unlock_recovery();
			return;
		}
	}

	
	
	

	lcd_recovering = LCD_RECOVERY_STOP;

    if(shdisp_api_get_mipi_dsi_det_recovery_flag()) {
        mipi_dsi_clr_err_int_mask0_b25();
    }

	SHLCDC_ERRLOG_REC(FB,LCD,MSM_FB_KERL,EEVENTID_FB_RECOVERY_OVER,0);
	MSM_FB_ERR("msm_lcd_recovery retry over\n");

	
	unlock_recovery();

}

void mipi_dsi_det_recovery(void)
{
    shdisp_api_do_mipi_dsi_det_recovery();
}

int mipi_sharp_is_recovery(void)
{
    return lcd_recovering;
}

int mipi_sharp_get_suspended_recovery_info(void)
{
	return suspended_recovery_flg;
}

void mipi_dsi_det_recovery_flag_clr(void)
{
    if(shdisp_api_get_mipi_dsi_det_recovery_flag()) {
        shdisp_api_clr_mipi_dsi_det_recovery_flag();
    }
}

static void mipi_sharp_lcd_init_det_info(void)
{
	if(shdisp_api_get_mipi_dsi_det_recovery_flag()) {
		
		shdisp_api_clr_mipi_dsi_det_recovery_flag();
		
		mipi_dsi_set_err_int_mask0_b25();
	}
}

void mipi_sharp_stat_inc(int mode)
{
    switch (mode) {
    case SHDISP_STAT_INC_ERR_DET:
        mdp4_stat.err_det++;
        break;
    case SHDISP_STAT_INC_RECOVERY:
        mdp4_stat.recovery++;
        break;
    default:
        break;
    }
}

int mipi_sharp_api_cabc_init(void)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    struct fb_info *info = NULL;
    struct msm_fb_panel_data *pdata = NULL;
    int ret = 0;
    
    if (!num_registered_fb){
        pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    info = registered_fb[0];
    if (!info){
        pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mfd = (struct msm_fb_data_type *)info->par;
    if (!mfd) {
        pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    if (!mfd->panel_power_on) {
        pr_err("%s: power off, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode != DSI_CMD_MODE) {
        pr_err("%s: Not command mode, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
    if (!pdata) {
        pr_err("%s: pdata == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_INIT,0);
    mutex_lock(&mfd->dma->ov_mutex);
    
    mipi_dsi_mdp_busy_wait();
    mdp4_dsi_cmd_busy_wait();
    
    ret = pdata->cabc_init(mfd, mipi);
    
    mutex_unlock(&mfd->dma->ov_mutex);
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_INIT_RET,ret);
    
    return ret;
}

int mipi_sharp_api_cabc_indoor_on(void)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    struct fb_info *info = NULL;
    struct msm_fb_panel_data *pdata = NULL;
    int ret = 0;
    
    if (!num_registered_fb){
        pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    info = registered_fb[0];
    if (!info){
        pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mfd = (struct msm_fb_data_type *)info->par;
    if (!mfd) {
        pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    if (!mfd->panel_power_on) {
        pr_err("%s: power off, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode != DSI_CMD_MODE) {
        pr_err("%s: Not command mode, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
    if (!pdata) {
        pr_err("%s: pdata == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_INDOOR_ON,0);
    mutex_lock(&mfd->dma->ov_mutex);
    
    mipi_dsi_mdp_busy_wait();
    mdp4_dsi_cmd_busy_wait();
    
    ret = pdata->cabc_indoor_on(mfd, mipi);
    
    mutex_unlock(&mfd->dma->ov_mutex);
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_INDOOR_ON_RET,ret);
    
    return ret;
}

int mipi_sharp_api_cabc_outdoor_on(int lut_level)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    struct fb_info *info = NULL;
    struct msm_fb_panel_data *pdata = NULL;
    int ret = 0;
    
    if (!num_registered_fb){
        pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    info = registered_fb[0];
    if (!info){
        pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mfd = (struct msm_fb_data_type *)info->par;
    if (!mfd) {
        pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    if (!mfd->panel_power_on) {
        pr_err("%s: power off, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode != DSI_CMD_MODE) {
        pr_err("%s: Not command mode, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
    if (!pdata) {
        pr_err("%s: pdata == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_OUTDOOR_ON,0);
    mutex_lock(&mfd->dma->ov_mutex);
    
    mipi_dsi_mdp_busy_wait();
    mdp4_dsi_cmd_busy_wait();
    
    ret = pdata->cabc_outdoor_on(mfd, mipi, lut_level);
    
    mutex_unlock(&mfd->dma->ov_mutex);
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_OUTDOOR_ON_RET,ret);
    
    return ret;
}

int mipi_sharp_api_cabc_off(int wait_on,int pwm_disable)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    struct fb_info *info = NULL;
    struct msm_fb_panel_data *pdata = NULL;
    int ret = 0;
    
    if (!num_registered_fb){
        pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    info = registered_fb[0];
    if (!info){
        pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mfd = (struct msm_fb_data_type *)info->par;
    if (!mfd) {
        pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    if (!mfd->panel_power_on) {
        pr_err("%s: power off, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode != DSI_CMD_MODE) {
        pr_err("%s: Not command mode, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
    if (!pdata) {
        pr_err("%s: pdata == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_OFF,0);
    mutex_lock(&mfd->dma->ov_mutex);
    
    mipi_dsi_mdp_busy_wait();
    mdp4_dsi_cmd_busy_wait();
    
    ret = pdata->cabc_off(mfd, mipi, wait_on,pwm_disable);
    
    mutex_unlock(&mfd->dma->ov_mutex);
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_OFF_RET,ret);
    
    return ret;
}

int mipi_sharp_api_cabc_outdoor_move(int lut_level)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    struct fb_info *info = NULL;
    struct msm_fb_panel_data *pdata = NULL;
    int ret = 0;
    
    if (!num_registered_fb){
        pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    info = registered_fb[0];
    if (!info){
        pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mfd = (struct msm_fb_data_type *)info->par;
    if (!mfd) {
        pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    if (!mfd->panel_power_on) {
        pr_err("%s: power off, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode != DSI_CMD_MODE) {
        pr_err("%s: Not command mode, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
    if (!pdata) {
        pr_err("%s: pdata == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_OUTDOOR_MODE,0);
    mutex_lock(&mfd->dma->ov_mutex);
    
    mipi_dsi_mdp_busy_wait();
    mdp4_dsi_cmd_busy_wait();
    
    ret = pdata->cabc_outdoor_move(mfd, mipi, lut_level);
    
    mutex_unlock(&mfd->dma->ov_mutex);
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_OUTDOOR_MODE_RET,ret);
    
    return ret;
}

int mipi_sharp_api_set_lpf(unsigned char *param)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    struct fb_info *info = NULL;
    struct msm_fb_panel_data *pdata = NULL;
    int ret = 0;
    
    if (!num_registered_fb){
        pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    info = registered_fb[0];
    if (!info){
        pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mfd = (struct msm_fb_data_type *)info->par;
    if (!mfd) {
        pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    if (!mfd->panel_power_on) {
        pr_err("%s: power off, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode != DSI_CMD_MODE) {
        pr_err("%s: Not command mode, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
    if (!pdata) {
        pr_err("%s: pdata == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    if (!pdata->set_lpf) {
        pr_err("%s: pdata->set_lpf is not support. -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_SET_LPF,0);
    mutex_lock(&ryoma_lpf_register_lock);
    mutex_lock(&mfd->dma->ov_mutex);
    
    mipi_dsi_mdp_busy_wait();
    mdp4_dsi_cmd_busy_wait();

    ret = pdata->set_lpf(mfd, mipi, param);
    
    mutex_unlock(&mfd->dma->ov_mutex);
    mutex_unlock(&ryoma_lpf_register_lock);
    SHLCDC_BUSYLOG_REC(FB,LCD,FBMEM_KERL,EEVENTID_FB_CABC_SET_LPF_RET,ret);
    
    return ret;
}

int mipi_sharp_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    struct fb_info *info = NULL;
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[MIPI_SHARP_RW_MAX_SIZE+1];
    
    if(size > MIPI_SHARP_RW_MAX_SIZE) {
        pr_err("%s: size over, -EINVAL\n", __func__);
        return -EINVAL;
    }
    
    if (!num_registered_fb){
        pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    info = registered_fb[0];
    if (!info){
        pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mfd = (struct msm_fb_data_type *)info->par;
    if (!mfd) {
        pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    if (!mfd->panel_power_on) {
        pr_err("%s: power off, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode != DSI_CMD_MODE) {
        pr_err("%s: Not command mode, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    cmd_buf[0] = addr;
    cmd_buf[1] = 0x00;
    memcpy(&cmd_buf[1], write_data, size);
    
    if((addr >= 0xB0) && (addr != 0xDA) && (addr != 0xDB) && (addr != 0xDC)) {
        if(size == 0) {
            cmd[0].dtype = DTYPE_GEN_WRITE;
        } else if(size == 1) {
            cmd[0].dtype = DTYPE_GEN_WRITE1;
        } else {
            cmd[0].dtype = DTYPE_GEN_LWRITE;
        }
    } else {
        if(size == 0) {
            cmd[0].dtype = DTYPE_DCS_WRITE;
        } else if(size == 1) {
            cmd[0].dtype = DTYPE_DCS_WRITE1;
        } else {
            cmd[0].dtype = DTYPE_DCS_LWRITE;
        }
    }
    
    cmd[0].last = 0x01;
    cmd[0].vc = 0x00;
    cmd[0].ack = 0x00;
    cmd[0].wait = 0x00;
    cmd[0].dlen = size + 1;
    cmd[0].payload = cmd_buf;
    
    mutex_lock(&mfd->dma->ov_mutex);
    
    mipi_dsi_mdp_busy_wait();
    mdp4_dsi_cmd_busy_wait();
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));
    mutex_unlock(&mfd->dma->ov_mutex);
    
    return 0;
}

int mipi_sharp_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    struct fb_info *info = NULL;
    struct dsi_buf *rp, *tp;
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[2+1];
    int i = 0, loop_cnt = 0, calc_size = 0, rx_size = 0;
    
    if((size > MIPI_SHARP_RW_MAX_SIZE) || (size == 0)){
        pr_err("%s: size over, -EINVAL\n", __func__);
        return -EINVAL;
    }
    
    if (!num_registered_fb){
        pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    info = registered_fb[0];
    if (!info){
        pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mfd = (struct msm_fb_data_type *)info->par;
    if (!mfd) {
        pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    if (!mfd->panel_power_on) {
        pr_err("%s: power off, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode != DSI_CMD_MODE) {
        pr_err("%s: Not command mode, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    tp = &sharp_tx_buf;
    rp = &sharp_rx_buf;
    calc_size = size;
    
    if(calc_size % MIPI_SHARP_R_SIZE) {
        loop_cnt = (calc_size / MIPI_SHARP_R_SIZE) + 1;
    } else {
        loop_cnt = (calc_size / MIPI_SHARP_R_SIZE);
    }

    for (i=0; i<loop_cnt; i++) {
        cmd_buf[0] = addr;
        cmd_buf[1] = ((i * MIPI_SHARP_R_SIZE) + 1);
        
        if(calc_size > MIPI_SHARP_R_SIZE){
            rx_size = MIPI_SHARP_R_SIZE;
            calc_size = calc_size - MIPI_SHARP_R_SIZE;
        }else{
            rx_size = calc_size;
        }
        
        if((addr >= 0xB0) && (addr != 0xDA) && (addr != 0xDB) && (addr != 0xDC)) {
            cmd[0].dtype = DTYPE_GEN_READ2;
        } else {
            cmd[0].dtype = DTYPE_DCS_READ;
        }
        
        cmd[0].last = 0x01;
        cmd[0].vc = 0x00;
        cmd[0].ack = 0x01;
        cmd[0].wait = 0x00;
        cmd[0].dlen = 2;
        cmd[0].payload = cmd_buf;
        
        mutex_lock(&mfd->dma->ov_mutex);
        
        mipi_dsi_mdp_busy_wait();
    	mdp4_dsi_cmd_busy_wait();
        
        mipi_dsi_cmds_rx(mfd, tp, rp, cmd, rx_size);
        mutex_unlock(&mfd->dma->ov_mutex);
        
        memcpy(&read_data[i * MIPI_SHARP_R_SIZE], rp->data, rx_size);
    }
    return 0;
}


int mipi_sharp_normal_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    struct fb_info *info = NULL;
    struct dsi_buf *rp, *tp;
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[2+1];
    int i = 0, loop_cnt = 0, calc_size = 0, rx_size = 0;
    
    if((size > MIPI_SHARP_RW_MAX_SIZE) || (size == 0)){
        pr_err("%s: size over, -EINVAL\n", __func__);
        return -EINVAL;
    }
    
    if (!num_registered_fb){
        pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    info = registered_fb[0];
    if (!info){
        pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mfd = (struct msm_fb_data_type *)info->par;
    if (!mfd) {
        pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode != DSI_CMD_MODE) {
        pr_err("%s: Not command mode, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    tp = &sharp_tx_buf;
    rp = &sharp_rx_buf;
    calc_size = size;
    
    if(calc_size % MIPI_SHARP_R_SIZE) {
        loop_cnt = (calc_size / MIPI_SHARP_R_SIZE) + 1;
    } else {
        loop_cnt = (calc_size / MIPI_SHARP_R_SIZE);
    }

    for (i=0; i<loop_cnt; i++) {
        cmd_buf[0] = addr;
        cmd_buf[1] = ((i * MIPI_SHARP_R_SIZE) + 1);
        
        if(calc_size > MIPI_SHARP_R_SIZE){
            rx_size = MIPI_SHARP_R_SIZE;
            calc_size = calc_size - MIPI_SHARP_R_SIZE;
        }else{
            rx_size = calc_size;
        }
        
        if((addr >= 0xB0) && (addr != 0xDA) && (addr != 0xDB) && (addr != 0xDC)) {
            cmd[0].dtype = DTYPE_GEN_READ2;
        } else {
            cmd[0].dtype = DTYPE_DCS_READ;
        }
        
        cmd[0].last = 0x01;
        cmd[0].vc = 0x00;
        cmd[0].ack = 0x01;
        cmd[0].wait = 0x00;
        cmd[0].dlen = 2;
        cmd[0].payload = cmd_buf;
        
        mipi_dsi_mdp_busy_wait();
    	mdp4_dsi_cmd_busy_wait();
        
        mipi_dsi_cmds_rx(mfd, tp, rp, cmd, rx_size);
        
        memcpy(&read_data[i * MIPI_SHARP_R_SIZE], rp->data, rx_size);
    }
    return 0;
}


void mipi_sharp_delay_us(unsigned long usec)
{
    struct timespec tu;
    
    if (usec >= 1000*1000) {
        tu.tv_sec  = usec / 1000000;
        tu.tv_nsec = (usec % 1000000) * 1000;
    }
    else
    {
        tu.tv_sec  = 0;
        tu.tv_nsec = usec * 1000;
    }
    
    hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
    
    return;
}

static int __init mipi_sharp_lcd_init(void)
{
    mipi_dsi_buf_alloc(&sharp_tx_buf, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&sharp_rx_buf, DSI_BUF_SIZE);
    
    return 0;
}
module_init(mipi_sharp_lcd_init);


static void mipi_sharp_lcd_exit(void)
{
    return;
}
module_exit(mipi_sharp_lcd_exit);
