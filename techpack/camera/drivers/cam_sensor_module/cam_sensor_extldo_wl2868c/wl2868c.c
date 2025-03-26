/*
 * Copyright (C) 2015 HUAQIN Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>

#include "wl2868c.h"

static int ldo_id = 0;
static int power_reference_counts[] = {0,0,0,0,0,0,0};

static char* camera_name[] = {
	"main_imx582",
	"front_hi846",
	"Uwide_hi846",
	"tele_hi847"
};

static char* camera_power_pin_name[] = {
	"SENSOR_MCLK",
	"SENSOR_VANA",
	"SENSOR_VDIG",
	"SENSOR_VIO",
	"SENSOR_VAF",
	"SENSOR_VAF_PWDM",
	"SENSOR_CUSTOM_REG1",
	"SENSOR_CUSTOM_REG2",
	"SENSOR_RESET",
	"SENSOR_STANDBY",
	"SENSOR_CUSTOM_GPIO1",
	"SENSOR_CUSTOM_GPIO2",
	"SENSOR_VANA1",
	#ifdef CONFIG_CAMERA_EXTLDO_WL2868C
	"SENSOR_WL2868C_VANA",
	"SENSOR_WL2868C_VDIG",
	"SENSOR_WL2868C_VIO",
	"SENSOR_WL2868C_VAF",
	#endif
	"SENSOR_OIS",
	"SENSOR_SEQ_TYPE_MAX"
};

struct i2c_device{
    unsigned short i2c_addr;
    unsigned short chip_addr;
    unsigned short ldoId;
    unsigned short enable_addr;
};

struct i2c_device which_ldo_chip[] = {
    {WL2868C_LDO_I2C_ADDR,  WL2868C_CHIP_REV_ADDR,  CAMERA_LDO_WL2868C,  WL2868C_LDO_EN_ADDR},
    {ET5907_LDO_I2C_ADDR,   ET5907_CHIP_REV_ADDR,   CAMERA_LDO_ET5907,   ET5907_LDO_EN_ADDR},
    {FAN53870_LDO_I2C_ADDR, FAN53870_CHIP_REV_ADDR, CAMERA_LDO_FAN53870, FAN53870_LDO_EN_ADDR},
};

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
#define USE_CONTROL_ENGPIO //finger also use this ldo, it will use pinctrl;path is kernel/msm-5.4/driver/regulator/wl2868c.c
static struct i2c_client *wl2868c_i2c_client;
#ifdef USE_CONTROL_ENGPIO
static struct pinctrl *wl2868c_pctrl; /* static pinctrl instance */
#endif

struct mutex i2c_control_mutex;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int wl2868c_dts_probe(struct platform_device *pdev);
static int wl2868c_dts_remove(struct platform_device *pdev);
static int wl2868c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int wl2868c_i2c_remove(struct i2c_client *client);

/*****************************************************************************
 * Extern Area
 *****************************************************************************/
void wl2868c_set_en_ldo(WL2868C_SELECT ldonum,unsigned int en)
{
    s32 ret=0;
    unsigned int value =0;

    if (NULL == wl2868c_i2c_client) {
            WL2868C_PRINT("[wl2868c] wl2868c_i2c_client is null!!\n");
            return ;
    }

    wl2868c_i2c_client->addr = which_ldo_chip[ldo_id].i2c_addr;
    ret= i2c_smbus_read_byte_data(wl2868c_i2c_client, which_ldo_chip[ldo_id].enable_addr);
    if (ret <0)
    {
        WL2868C_PRINT("[wl2868c] wl2868c_set_en_ldo read error!\n");
        return;
    }

    if(en == 0)
    {
        value = (ret & (~(0x01<<ldonum)));
    }
    else
    {
        if(ldo_id == 0 || which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2868C) {
            value = (ret|(0x01<<ldonum))|0x80;
        }else if ((which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5907)
                || (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_FAN53870)) {
            value = (ret|(0x01<<ldonum));
        }
    }

    i2c_smbus_write_byte_data(wl2868c_i2c_client, which_ldo_chip[ldo_id].enable_addr, value);
    //WL2868C_PRINT("[wl2868c] wl2868c_set_en_ldo enable before:%x after set :%x\n",ret,value);
    return;

}
/** wl2868c
   Voutx=0.496v+LDOX_OUT[6:0]*0.008V LDO1/LDO2
   Voutx=1.504v+LDOX_OUT[7:0]*0.008V LDO3~LDO7
===FAN53870
   Voutx=0.800v+(LDOX_OUT[6:0]-99)*0.008V LDO1/LDO2
   Voutx=1.500v+()LDOX_OUT[7:0]-16)*0.008V LDO3~LDO7
===ET5907
   Voutx=0.600v+LDOX_OUT[7:0]*0.006V LDO1/LDO2
   Voutx=1.200v+LDOX_OUT[7:0]*0.010V LDO3~LDO7
*/
void wl2868c_set_ldo_value(WL2868C_SELECT ldonum,unsigned int value)
{
    unsigned int  Ldo_out =0;
    unsigned char regaddr =0;
    s32 ret =0;

    WL2868C_PRINT("[wl2868c] %s begin\n",__FUNCTION__);

    if (NULL == wl2868c_i2c_client) {
        WL2868C_PRINT("[wl2868c] wl2868c_i2c_client is null!!\n");
        return ;
    }
    if(ldonum >= WL2868C_MAX)
    {
        WL2868C_PRINT("[wl2868c] error ldonum not support!!!\n");
        return;
    }

    switch(ldonum)
    {
        case WL2868C_DVDD1:
        case WL2868C_DVDD2:
            if(ldo_id == 0 || which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2868C) {//WL2868C
                if (value < 496) {
                    WL2868C_PRINT("[WL2868C] error vol!!!\n");
                    goto exit;
                } else {
                    Ldo_out = (value - 496)/8;
                }
            } else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_FAN53870) {//FAN53870
                if (value < 800) {
                    WL2868C_PRINT("[FAN53870] error vol!!!\n");
                    goto exit;
                } else {
                    Ldo_out = (value - 800)/8 + 99;
                }
            } else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5907) {//ET5907
                if (value < 600) {
                    WL2868C_PRINT("[ET5907] error vol!!!\n");
                    goto exit;
                } else {
                    Ldo_out = (value - 600)/6;
                }
            }
            break;
        case WL2868C_AVDD1:
        case WL2868C_AVDD2:
        case WL2868C_VDDAF:
        case WL2868C_VDDOIS:
        case WL2868C_VDDIO:
            if(ldo_id == 0 || which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2868C) {//WL2868C
                if(value < 1504)
                {
                    WL2868C_PRINT("[wl2868c] error vol!!!\n");
                    goto exit;
                }
                else
                {
                    Ldo_out = (value - 1504)/8;
                }
            } else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_FAN53870) {//FAN53870
                if(value < 1500)
                {
                    WL2868C_PRINT("[wl2868c-FAN53870] error vol!!!\n");
                    goto exit;
                }
                else
                {
                    Ldo_out = (value - 1500)/8 + 16;
                }
            }else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5907) {//ET5907
                if(value < 1200)
                {
                    WL2868C_PRINT("[wl2868c-ET5907] error vol!!!\n");
                    goto exit;
                }
                else
                {
                    Ldo_out = (value - 1200)/10;
                }
            }
            break;
        default:
            goto exit;
        break;
    }

    if(ldo_id == 0 || (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2868C)) {
        WL2868C_PRINT("[wl2868c] CAMERA_LDO_WL2868C");
        regaddr = ldonum + WL2868C_LDO1_OUT_ADDR;
    } else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5907) {
        WL2868C_PRINT("[wl2868c] CAMERA_LDO_ET5907");
        regaddr = ldonum + LDO1_OUT_ADDR;
    } else if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_FAN53870) {
        WL2868C_PRINT("[wl2868c] CAMERA_LDO_FAN53870");
        regaddr = ldonum + LDO1_OUT_ADDR;
    }

    i2c_smbus_write_byte_data(wl2868c_i2c_client, regaddr, Ldo_out);
    ret=i2c_smbus_read_byte_data(wl2868c_i2c_client,regaddr);

exit:
    WL2868C_PRINT("[wl2868c] %s exit err\n",__FUNCTION__);

}

static struct wl2868c_ldomap ldolist[] = {
    {CAMERA_INDEX_MAIN, SENSOR_WL2868C_VANA, WL2868C_AVDD1}, //BackMain AVDD
    {CAMERA_INDEX_MAIN, SENSOR_WL2868C_VDIG, WL2868C_DVDD2}, //BackMain DVDD
    {CAMERA_INDEX_MAIN, SENSOR_WL2868C_VIO, WL2868C_VDDIO}, //BackMain IOVDD
    {CAMERA_INDEX_MAIN, SENSOR_WL2868C_VAF, WL2868C_VDDAF}, //BackMain AFVDD

#if defined(CONFIG_ARCH_SONY_MURRAY)
    {CAMERA_INDEX_FRONT, SENSOR_WL2868C_VANA, WL2868C_AVDD1}, //FrontMain AVDD
#else
    {CAMERA_INDEX_FRONT, SENSOR_WL2868C_VANA, WL2868C_VDDOIS}, //FrontMain AVDD
#endif
    {CAMERA_INDEX_FRONT, SENSOR_WL2868C_VDIG, WL2868C_DVDD1}, //FrontMain DVDD
    {CAMERA_INDEX_FRONT, SENSOR_WL2868C_VIO, WL2868C_VDDIO}, //FrontMain IOVDD

    {CAMERA_INDEX_MAIN_WIDE, SENSOR_WL2868C_VIO, WL2868C_VDDIO}, //BackWide IOVDD
    {CAMERA_INDEX_MAIN_WIDE, SENSOR_WL2868C_VANA, WL2868C_AVDD2}, //BackWide AVDD
    {CAMERA_INDEX_MAIN_WIDE, SENSOR_WL2868C_VDIG, WL2868C_DVDD1}, //BackWide DVDD

    {CAMERA_INDEX_MAIN_TELE, SENSOR_WL2868C_VIO, WL2868C_VDDIO}, //BackTele IOVDD
    {CAMERA_INDEX_MAIN_TELE, SENSOR_WL2868C_VAF, WL2868C_VDDAF}, //BackTele AFVDD
    {CAMERA_INDEX_MAIN_TELE, SENSOR_WL2868C_VANA, WL2868C_VDDOIS}, //BackTele AVDD
    {CAMERA_INDEX_MAIN_TELE, SENSOR_WL2868C_VDIG, WL2868C_DVDD1}, //BackTele DVDD

};

int wl2868c_set_ldo_enable(uint32_t sensor_index, enum msm_camera_power_seq_type seq_type, uint32_t min_voltage, uint32_t max_voltage)
{
    WL2868C_SELECT ldonum = WL2868C_NONE;
    unsigned int ldo_vol_value = 0;
    unsigned int i = 0;

    if(sensor_index >= CAMERA_INDEX_MAX_NUM ||
            seq_type >= SENSOR_SEQ_TYPE_MAX ||
            min_voltage < 600000 ||
            max_voltage < 600000 ||
            min_voltage > max_voltage){
        WL2868C_PRINT("[wl2868c] %s invalid parameters!!!\n",__FUNCTION__);
        return -1;
    }

    mutex_lock(&i2c_control_mutex);
    for(i = 0;i < (sizeof(ldolist) / sizeof(ldolist[0]));i++) {
        if(sensor_index == ldolist[i].sensor_index && seq_type == ldolist[i].seq_type) {
            ldonum = ldolist[i].ldo_selected;
            break;
        }
    }

    if(ldonum == WL2868C_NONE) {
        WL2868C_PRINT("[wl2868c] %s ldo setting not found in ldolist!!!\n",__FUNCTION__);
        return -2;
    }

    ldo_vol_value = min_voltage / 1000;
    power_reference_counts[ldonum] += 1;
    if(power_reference_counts[ldonum] > 1){
        WL2868C_PRINT("[wl2868c] %s: %s, LDO_%d poweron already!\n", camera_name[sensor_index],
                                                                     camera_power_pin_name[seq_type],
                                                                     ldonum);
    }else{
        wl2868c_set_ldo_value(ldonum, ldo_vol_value);
        wl2868c_set_en_ldo(ldonum, 1);
        WL2868C_PRINT("[wl2868c] %s: %s, LDO_%d %dmV poweron \n", camera_name[sensor_index],
                                                                  camera_power_pin_name[seq_type],
                                                                  ldonum, ldo_vol_value);
    }
    WL2868C_PRINT("[wl2868c] power counts array:[%d, %d, %d, %d, %d, %d, %d]\n",
                                                power_reference_counts[0],power_reference_counts[1],
                                                power_reference_counts[2],power_reference_counts[3],
                                                power_reference_counts[4],power_reference_counts[5],
                                                power_reference_counts[6]);
    mutex_unlock(&i2c_control_mutex);
    return 0;
}

int wl2868c_set_ldo_disable(uint32_t sensor_index, enum msm_camera_power_seq_type seq_type)
{
    WL2868C_SELECT ldonum = WL2868C_NONE;
    unsigned int i = 0;

    if(sensor_index >= CAMERA_INDEX_MAX_NUM ||
                    seq_type >= SENSOR_SEQ_TYPE_MAX) {
        WL2868C_PRINT("[wl2868c] %s invalid parameters!!!\n",__FUNCTION__);
        return -1;
    }
    mutex_lock(&i2c_control_mutex);
    for(i = 0;i < (sizeof(ldolist) / sizeof(ldolist[0]));i++) {
        if(sensor_index == ldolist[i].sensor_index && seq_type == ldolist[i].seq_type) {
            ldonum = ldolist[i].ldo_selected;
            break;
        }
    }

    if(ldonum == WL2868C_NONE) {
        WL2868C_PRINT("[wl2868c] %s ldo setting not found in ldolist!!!\n",__FUNCTION__);
        return -2;
    }
    power_reference_counts[ldonum] -= 1;
    if(power_reference_counts[ldonum] == 0){
        wl2868c_set_en_ldo(ldonum, 0);
        WL2868C_PRINT("[wl2868c] %s: %s, LDO_%d poweroff \n", camera_name[sensor_index],
                                                              camera_power_pin_name[seq_type],
                                                              ldonum);
    }else{
        WL2868C_PRINT("[wl2868c] %s: %s, LDO_%d will not powerff, still in use!\n", camera_name[sensor_index],
                                                                                   camera_power_pin_name[seq_type],
                                                                                   ldonum);
    }
    WL2868C_PRINT("[wl2868c] power counts array:[%d, %d, %d, %d, %d, %d, %d]\n",
                                                power_reference_counts[0],power_reference_counts[1],
                                                power_reference_counts[2],power_reference_counts[3],
                                                power_reference_counts[4],power_reference_counts[5],
                                                power_reference_counts[6]);
    mutex_unlock(&i2c_control_mutex);
    return 0;
}

/*****************************************************************************
 * Data Structure
 *****************************************************************************/
#ifdef USE_CONTROL_ENGPIO
static const char *wl2868c_state_name[WL2868C_GPIO_STATE_MAX] = {
    "wl2868c_gpio_enp0",
    "wl2868c_gpio_enp1"
};/* DTS state mapping name */
#endif

static const struct of_device_id gpio_of_match[] = {
    { .compatible = "qualcomm,gpio_wl2868c", },
    {},
};

static const struct of_device_id i2c_of_match[] = {
    { .compatible = "qualcomm,i2c_wl2868c", },
    {},
};

static const struct i2c_device_id wl2868c_i2c_id[] = {
    {"WL2868C_I2C", 0},
    {},
};

static struct platform_driver wl2868c_platform_driver = {
    .probe = wl2868c_dts_probe,
    .remove = wl2868c_dts_remove,
    .driver = {
        .name = "WL2868C_DTS",
        .of_match_table = gpio_of_match,
    },
};

static struct i2c_driver wl2868c_i2c_driver = {
/************************************************************
Attention:
Althouh i2c_bus do not use .id_table to match, but it must be defined,
otherwise the probe function will not be executed!
************************************************************/
    .id_table = wl2868c_i2c_id,
    .probe = wl2868c_i2c_probe,
    .remove = wl2868c_i2c_remove,
    .driver = {
        .name = "WL2868C_I2C",
        .of_match_table = i2c_of_match,
    },
};

/*****************************************************************************
 * Function
 *****************************************************************************/
#ifdef USE_CONTROL_ENGPIO
static long wl2868c_set_state(const char *name)
{
    int ret = 0;
    struct pinctrl_state *pState = 0;

    BUG_ON(!wl2868c_pctrl);

    pState = pinctrl_lookup_state(wl2868c_pctrl, name);
    if (IS_ERR(pState)) {
        pr_err("set state '%s' failed\n", name);
        ret = PTR_ERR(pState);
        goto exit;
    }

    /* select state! */
    pinctrl_select_state(wl2868c_pctrl, pState);

exit:
    return ret; /* Good! */
}

void wl2868c_gpio_select_state(WL2868C_GPIO_STATE s)
{
    WL2868C_PRINT("[wl2868c]%s,%d\n",__FUNCTION__,s);

    BUG_ON(!((unsigned int)(s) < (unsigned int)(WL2868C_GPIO_STATE_MAX)));
    wl2868c_set_state(wl2868c_state_name[s]);
}

static long wl2868c_dts_init(struct platform_device *pdev)
{
    int ret = 0;
    struct pinctrl *pctrl;

    /* retrieve */
    pctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(pctrl)) {
        dev_err(&pdev->dev, "Cannot find disp pinctrl!");
        ret = PTR_ERR(pctrl);
        goto exit;
    }

    wl2868c_pctrl = pctrl;

exit:
    return ret;
}
#endif

static int wl2868c_dts_probe(struct platform_device *pdev)
{
#ifdef USE_CONTROL_ENGPIO
    int ret = 0;

    ret = wl2868c_dts_init(pdev);
    if (ret) {
        WL2868C_PRINT("[wl2868c]wl2868c_dts_probe failed\n");
        return ret;
    }
#endif
    WL2868C_PRINT("[wl2868c] wl2868c_dts_probe success\n");

    return 0;
}

static int wl2868c_dts_remove(struct platform_device *pdev)
{
    platform_driver_unregister(&wl2868c_platform_driver);

    return 0;
}

static int wl2868c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i = 0;
    int chipid = 0;

    if (NULL == client) {
        WL2868C_PRINT("[wl2868c] i2c_client is NULL\n");
        return -1;
    }

    for(i = 0; i < (sizeof(which_ldo_chip) / sizeof(which_ldo_chip[0])); i++) {
        client->addr = which_ldo_chip[i].i2c_addr;
        wl2868c_i2c_client = client;
        wl2868c_gpio_select_state(WL2868C_GPIO_STATE_ENP0);

        chipid = i2c_smbus_read_byte_data(wl2868c_i2c_client, which_ldo_chip[i].chip_addr) & 0xff;
        WL2868C_PRINT("[wl2868c]camera_ldo_i2c_probe addr = 0x%x,chipid:vendorid = 0x%x\n", client->addr, chipid);

        if (chipid == which_ldo_chip[i].ldoId) {
             ldo_id = i;
             WL2868C_PRINT("[wl2868c]camera_ldo_i2c_probe, this is %d\n", client->addr, i);
             break;
        }
    }

    client->addr=0x2E;
    mutex_init(&i2c_control_mutex);
    WL2868C_PRINT("[wl2868c]wl2868c_i2c_probe success addr = 0x%x\n", client->addr);
    return 0;
}

static int wl2868c_i2c_remove(struct i2c_client *client)
{
    wl2868c_i2c_client = NULL;
    i2c_unregister_device(client);

    return 0;
}

int wl2868c_init_module(void)
{
        if (platform_driver_register(&wl2868c_platform_driver)) {
                WL2868C_PRINT("[wl2868c]Failed to register wl2868c_platform_driver!\n");
                i2c_del_driver(&wl2868c_i2c_driver);
                return -1;
        }
    WL2868C_PRINT("begin wl2868c initialization");
    if (i2c_add_driver(&wl2868c_i2c_driver)) {
        WL2868C_PRINT("[wl2868c]Failed to register wl2868c_i2c_driver!\n");
        return -1;
    }

    return 0;
}

void wl2868c_exit_module(void)
{
    platform_driver_unregister(&wl2868c_platform_driver);
    i2c_del_driver(&wl2868c_i2c_driver);
}

//module_init(wl2868c_init);
//module_exit(wl2868c_exit);

MODULE_AUTHOR("AmyGuo <guohuiqing@huaqin.com>");
MODULE_DESCRIPTION("EXT CAMERA LDO Driver");
MODULE_LICENSE("GPL");
