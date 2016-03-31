/* include/sharp/sh_smem.h
 *
 * Copyright (C) 2011 Sharp Corporation
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

#if defined(CONFIG_SHSMEM_CUST_SMARTPHONE)
typedef struct 
{
    unsigned long       sh_filesystem_init;         /* file system innitialize flag */
    unsigned long       sh_hw_revision;             /* hardware revision number */
    unsigned long       sh_model_type;              /* model type information */
    unsigned long       sh_boot_mode;               /* power up mode information */
    unsigned long       sh_softupdate_mode;         /* software update mode  */
    unsigned long       sh_pwr_on_status;           /* power on status information from pmic  */
    unsigned long       shdiag_FlagData;            /* shdiag flag information */
    unsigned short      shdiag_BootMode;            /* shdiag Powerup mode */
    unsigned char       shdiag_FirstBoot;           /* shdiag FirstBoot information */
    unsigned char       rec[3];                     /* reserved */
    unsigned char       shdiag_AdjChashe[16];       /* shdiag Adj chashe information */
    unsigned long       support_FlagData;           /* support flag information */
    int                 sh_sleep_test_mode;
    unsigned char       shsecure_PassPhrase[32];
    unsigned char       bootimg_header[2048];       /* APPSBOOT header information */
    unsigned long       fota_boot_mode;             /* FOTA mode information */
//    unsigned char       pImeiData[12];				/* W-CDMA Imei data  */
    unsigned char       pImeiData[16];				/* W-CDMA Imei data  */
    unsigned char       sh_fwupdateFlag;            /* Update Information */
    unsigned long       sh_boot_key;                /* key(s) ditected OSBL */
    unsigned char       sh_camver[4];               /* Version information */
    unsigned char       sh_touchver[4];             /* Version information */
    unsigned char       sh_miconver[4];             /* Version information */
    unsigned char       shdarea_QRData[128];        /* QRdata */
    unsigned char       sh_swicdev;                 /* USB SWIC exist information */
    unsigned char       shdarea_WlanMacAddress[6];  /* WLAN Mac Address */
    unsigned char       sh_camImgAdjustData[102];   /* Camera Image Adjust Data */
    unsigned char       conf_clrvari[4];            /* Color Variations information */
//    unsigned char       shrmts_data_buf[16384+32];/* Buffer for shrmts */
//    unsigned char       sh_camOtpData[4];           /* 8MCMOS Camera OTPData */
    unsigned char       sh_camOtpData[7152];        /* 8MCMOS Camera OTPData */
    unsigned char       shsys_wait_for_modem_flag;  /* before flash_read wait flag*/
    unsigned char       shusb_softupdate_mode_flag; /* softupdate mode flag */
    unsigned char       shusb_qxdm_ena_flag;        /* QXDM enable flag */
    unsigned char       shusb_usb_charge_ena_flag;  /* USB charge enable flag */
//    unsigned char       shdisp_data_buf[2048];      /* Buffer for shdisp */
    unsigned long       shdisp_data_buf[512];       /* Buffer for shdisp */
    unsigned char       sh_wl_pdadc[868];           /* WLAN characteristics */
    unsigned short      shdiag_TpsBaseLineTbl[700]; /* Touch adjustment */
    int                 shpwr_battery_present;      /* Battery presen */
    int                 shpwr_battery_voltage;      /* Battery voltage */
    int                 shpwr_battery_temperature;  /* Battery temperature */
    int                 shpwr_xo_temperature;       /* XO temperature */
    int                 shpwr_cable_status;         /* Cable status */
    unsigned char       sh_100hflg;                 /* 100 hours test flag */
    unsigned short      shdiag_proxadj[2];          /* Proximity sensor adjust */
    unsigned char       sh_wl_wlancntflg;           /* WLANCNTFLG */
    unsigned char       shtps_fwup_flag;            /* Touch panel firmware update flag */
    int                 shpwr_fuel_data[4];         /* Fuel gauge correction value */
    int                 shpwr_vbat_data[4];         /* Battery A/D converter correction value */
    unsigned char       sh_pvs_flg;                 /* PVS flag */
    unsigned char       shdiag_fullchgflg;          /* Full charge enable flag for Ver.F */
    unsigned char       sh_cache_err_flg;           /* Cache Err flag */
} sharp_smem_common_type;
#elif defined(CONFIG_SHSMEM_CUST_TABLET)
typedef struct 
{
    unsigned long       sh_filesystem_init;         /* file system innitialize flag */
    unsigned long       sh_hw_revision;             /* hardware revision number */
    unsigned long       sh_model_type;              /* model type information */
    unsigned long       sh_boot_mode;               /* power up mode information */
    unsigned long       sh_softupdate_mode;         /* software update mode  */
    unsigned long       sh_pwr_on_status;           /* power on status information from pmic  */
    unsigned long       shdiag_FlagData;            /* shdiag flag information */
    unsigned short      shdiag_BootMode;            /* shdiag Powerup mode */
    unsigned char       shdiag_FirstBoot;           /* shdiag FirstBoot information */
    unsigned char       rec[3];                     /* reserved */
    unsigned char       shdiag_AdjChashe[16];       /* shdiag Adj chashe information */
    unsigned long       support_FlagData;           /* support flag information */
    int                 sh_sleep_test_mode;
    unsigned char       shsecure_PassPhrase[32];
    unsigned char       bootimg_header[2048];       /* APPSBOOT header information */
    unsigned long       fota_boot_mode;             /* FOTA mode information */
//    unsigned char       pImeiData[12];				/* W-CDMA Imei data  */
    unsigned char       pImeiData[16];				/* W-CDMA Imei data  */
    unsigned char       sh_fwupdateFlag;            /* Update Information */
    unsigned long       sh_boot_key;                /* key(s) ditected OSBL */
    unsigned char       sh_camver[4];               /* Version information */
    unsigned char       sh_touchver[4];             /* Version information */
    unsigned char       sh_miconver[4];             /* Version information */
    unsigned char       shdarea_QRData[128];        /* QRdata */
    unsigned char       sh_swicdev;                 /* USB SWIC exist information */
    unsigned char       shdarea_WlanMacAddress[6];  /* WLAN Mac Address */
    unsigned char       sh_camImgAdjustData[102];   /* Camera Image Adjust Data */
    unsigned char       conf_clrvari[4];            /* Color Variations information */
//    unsigned char       shrmts_data_buf[16384+32];/* Buffer for shrmts */
//    unsigned char       sh_camOtpData[4];           /* 8MCMOS Camera OTPData */
    unsigned char       sh_camOtpData[7152];        /* 8MCMOS Camera OTPData */
    unsigned char       shsys_wait_for_modem_flag;  /* before flash_read wait flag*/
    unsigned char       shusb_softupdate_mode_flag; /* softupdate mode flag */
    unsigned char       shusb_qxdm_ena_flag;        /* QXDM enable flag */
    unsigned char       shusb_usb_charge_ena_flag;  /* USB charge enable flag */
//    unsigned char       shdisp_data_buf[2048];      /* Buffer for shdisp */
    unsigned long       shdisp_data_buf[512];       /* Buffer for shdisp */
    unsigned char       sh_wl_pdadc[868];           /* WLAN characteristics */
//    unsigned short      shdiag_TpsBaseLineTbl[700]; /* Touch adjustment */
    unsigned short      shdiag_TpsBaseLineTbl[1000]; /* Touch adjustment */
    unsigned short      shdiag_TpsBaseLineTbl_bk[1000]; /* Touch adjustment(Back side) */
    int                 shpwr_battery_present;      /* Battery presen */
    int                 shpwr_battery_voltage;      /* Battery voltage */
    int                 shpwr_battery_temperature;  /* Battery temperature */
    int                 shpwr_xo_temperature;       /* XO temperature */
    int                 shpwr_cable_status;         /* Cable status */
    unsigned char       sh_100hflg;                 /* 100 hours test flag */
    unsigned short      shdiag_proxadj[2];          /* Proximity sensor adjust */
    unsigned char       sh_wl_wlancntflg;           /* WLANCNTFLG */
    unsigned char       shtps_fwup_flag;            /* Touch panel firmware update flag */
    int                 shpwr_fuel_data[4];         /* Fuel gauge correction value */
    int                 shpwr_vbat_data[4];         /* Battery A/D converter correction value */
    unsigned char       sh_pvs_flg;                 /* PVS flag */
    unsigned char       shdiag_fullchgflg;          /* Full charge enable flag for Ver.F */
} sharp_smem_common_type;
#else
typedef struct 
{
    unsigned long       shdisp_data_buf[512];       /*  Buffer for shdisp */
    unsigned char       shusb_softupdate_mode_flag; /* softupdate mode flag */
    unsigned long       sh_filesystem_init;         /* file system innitialize flag */
    unsigned long       sh_hw_revision;             /* hardware revision number */
    unsigned long       sh_model_type;              /* model type information */
    unsigned long       sh_boot_mode;               /* power up mode information */
    unsigned long       sh_softupdate_mode;         /* software update mode  */
    unsigned long       sh_pwr_on_status;           /* power on status information from pmic  */
    unsigned long       shdiag_FlagData;            /* shdiag flag information */
    unsigned short      shdiag_BootMode;            /* shdiag Powerup mode */
    unsigned char       shdiag_FirstBoot;           /* shdiag FirstBoot information */
    unsigned char       rec[3];                     /* reserved */
    unsigned char       shdiag_AdjChashe[16];       /* shdiag Adj chashe information */
    unsigned long       support_FlagData;           /* support flag information */
    int                 sh_sleep_test_mode;
    unsigned char       shsecure_PassPhrase[32];
    unsigned char       bootimg_header[2048];       /* APPSBOOT header information */
    unsigned long       fota_boot_mode;             /* FOTA mode information */
//    unsigned char       pImeiData[12];				/* W-CDMA Imei data  */
    unsigned char       pImeiData[16];				/* W-CDMA Imei data  */
    unsigned char       sh_fwupdateFlag;            /* Update Information */
    unsigned long       sh_boot_key;				/* key(s) ditected OSBL */
    unsigned char       sh_camver[4];               /* Version information */
    unsigned char       sh_touchver[4];             /* Version information */
    unsigned char       sh_miconver[4];             /* Version information */
    unsigned char       shdarea_QRData[128];		/* QRdata */
    unsigned char       sh_swicdev;                 /* USB SWIC exist information */
    unsigned char       shdarea_WlanMacAddress[6];	/* WLAN Mac Address */
    unsigned char       sh_camImgAdjustData[102];   /* Camera Image Adjust Data */
    unsigned char       conf_clrvari[4];            /* Color Variations information */
//    unsigned char       shrmts_data_buf[16384+32];/* Buffer for shrmts */
//    unsigned char       sh_camOtpData[4];           /* 8MCMOS Camera OTPData */
    unsigned char       sh_camOtpData[7152];        /* 8MCMOS Camera OTPData */
    unsigned char       shsys_wait_for_modem_flag;  /* before flash_read wait flag*/
    unsigned char       shusb_qxdm_ena_flag;        /* QXDM enable flag */
    unsigned char       shusb_usb_charge_ena_flag;  /* USB charge enable flag */
//    unsigned char       shdisp_data_buf[2048];      /* Buffer for shdisp */
    unsigned char       sh_wl_pdadc[868];           /* WLAN characteristics */
//    unsigned short      shdiag_TpsBaseLineTbl[700]; /* Touch adjustment */
    unsigned short      shdiag_TpsBaseLineTbl[1000]; /* Touch adjustment */
    unsigned short      shdiag_TpsBaseLineTbl_bk[1000]; /* Touch adjustment(Back side) */
    int                 shpwr_battery_present;      /* Battery presen */
    int                 shpwr_battery_voltage;      /* Battery voltage */
    int                 shpwr_battery_temperature;  /* Battery temperature */
    int                 shpwr_xo_temperature;       /* XO temperature */
    int                 shpwr_cable_status;         /* Cable status */
    unsigned char       sh_100hflg;                 /* 100 hours test flag */
    unsigned short      shdiag_proxadj[2];          /* Proximity sensor adjust */
    unsigned char       sh_wl_wlancntflg;           /* WLANCNTFLG */
    unsigned char       shtps_fwup_flag;            /* Touch panel firmware update flag */
    int                 shpwr_fuel_data[4];         /* Fuel gauge correction value */
    int                 shpwr_vbat_data[4];         /* Battery A/D converter correction value */
    unsigned char       sh_pvs_flg;                 /* PVS flag */
    unsigned char       shdiag_fullchgflg;          /* Full charge enable flag for Ver.F */
    unsigned char       shdiag_charge_th_high[8];   /* ChageLimitMax */
    unsigned char       shdiag_charge_th_low[8];    /* ChageLimitMin */
} sharp_smem_common_type;

#define SH_SMEM_COMMON_SIZE 153600
#endif

/*=============================================================================

FUNCTION sh_smem_get_common_address

=============================================================================*/
sharp_smem_common_type *sh_smem_get_common_address( void );

/*=============================================================================

FUNCTION sh_smem_get_sleep_power_collapse_disabled_address

=============================================================================*/
unsigned long *sh_smem_get_sleep_power_collapse_disabled_address( void );

/*=============================================================================

FUNCTION sh_smem_get_100hflg

=============================================================================*/
unsigned char sh_smem_get_100hflg( void );

/*=============================================================================

FUNCTION sh_smem_get_softupdate_flg

=============================================================================*/
unsigned char sh_smem_get_softupdate_flg( void );

/*=============================================================================

FUNCTION sh_smem_get_battery_voltage

=============================================================================*/
int sh_smem_get_battery_voltage( void );

/*=============================================================================

FUNCTION sh_smem_get_fota_boot_mode

=============================================================================*/
unsigned long sh_smem_get_fota_boot_mode( void );

/*=============================================================================

FUNCTION sh_smem_get_pvs_flg

=============================================================================*/
unsigned char sh_smem_get_pvs_flg( void );

#ifdef CONFIG_MACH_DECKARD_AS70
/*=============================================================================

FUNCTION sh_smem_get_cache_err_flg

=============================================================================*/
unsigned char sh_smem_get_cache_err_flg( void );
#endif
