/*
 *  vi5300_module.c - Linux kernel modules for VI5300 FlightSense TOF
 *						 sensor
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  Originally developed for Linux 4.x by vendor.
 *  Ported to Raspberry Pi kernel 6.15.0 by Vasyl Dykyj.
 */

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <../drivers/gpio/gpiolib.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/of_gpio.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/time.h>
#include <linux/ktime.h>

#include "vi5300.h"
#include "vi5300_platform.h"
#include "vi5300_firmware.h"
#include "vi5300_api.h"

#define VI5300_IOCTL_PERIOD _IOW('p', 0x01, uint32_t)
#define VI5300_IOCTL_XTALK_CALIB _IOR('p', 0x02, struct VI5300_XTALK_Calib_Data)
#define VI5300_IOCTL_XTALK_CONFIG _IOW('p', 0x03, struct VI5300_XTALK_Config_Data)
#define VI5300_IOCTL_OFFSET_CALIB _IOR('p', 0x04, struct VI5300_OFFSET_Calib_Data)
#define VI5300_IOCTL_OFFSET_CONFIG _IOW('p', 0x05, int16_t)
#define VI5300_IOCTL_REFTOF_CONFIG _IOW('p', 0x0c, int16_t)
#define VI5300_IOCTL_POWER_ON _IO('p', 0x06)
#define VI5300_IOCTL_CHIP_INIT _IO('p', 0x07)
#define VI5300_IOCTL_START _IO('p', 0x08)
#define VI5300_IOCTL_STOP _IO('p', 0x09)
#define VI5300_IOCTL_MZ_DATA _IOR('p', 0x0a, struct VI5300_Measurement_Data)
#define VI5300_IOCTL_POWER_OFF _IO('p', 0x0b)

#define VI5300_DRV_NAME "vi5300"
static int xtalk_mark;
static int offset_mark;

struct vi5300_api_fn_t {
	int32_t (*Power_ON)(VI5300_DEV dev);
	int32_t (*Power_OFF)(VI5300_DEV dev);
	void (*Chip_Register_Init)(VI5300_DEV dev);
	void (*Set_Period)(VI5300_DEV dev, uint32_t period);
	int32_t (*Single_Measure)(VI5300_DEV dev);
	int32_t (*Start_Continuous_Measure)(VI5300_DEV dev);
	int32_t (*Stop_Continuous_Measure)(VI5300_DEV dev);
	int32_t (*Get_Measure_Data)(VI5300_DEV dev);
	int32_t (*Get_Interrupt_State)(VI5300_DEV dev);
	int32_t (*Chip_Init)(VI5300_DEV dev);
	int32_t (*Start_XTalk_Calibration)(VI5300_DEV dev);
	int32_t (*Start_Offset_Calibration)(VI5300_DEV dev);
	int32_t (*Get_XTalk_Parameter)(VI5300_DEV dev);
	int32_t (*Config_XTalk_Parameter)(VI5300_DEV dev);
	int32_t (*Config_RefTof_Parameter)(VI5300_DEV dev);
	void (*Read_ChipID)(VI5300_DEV dev, uint8_t *chipid);
};
static struct vi5300_api_fn_t vi5300_api_func_tbl = {
	.Power_ON = VI5300_Chip_PowerON,
	.Power_OFF = VI5300_Chip_PowerOFF,
	.Chip_Register_Init = VI5300_Chip_Register_Init,
	.Set_Period = VI5300_Set_Period,
	.Single_Measure = VI5300_Single_Measure,
	.Start_Continuous_Measure = VI5300_Start_Continuous_Measure,
	.Stop_Continuous_Measure = VI5300_Stop_Continuous_Measure,
	.Get_Measure_Data = VI5300_Get_Measure_Data,
	.Get_Interrupt_State = VI5300_Get_Interrupt_State,
	.Chip_Init = VI5300_Chip_Init,
	.Start_XTalk_Calibration = VI5300_Start_XTalk_Calibration,
	.Start_Offset_Calibration = VI5300_Start_Offset_Calibration,
	.Get_XTalk_Parameter = VI5300_Get_XTalk_Parameter,
	.Config_XTalk_Parameter = VI5300_Config_XTalk_Parameter,
	.Config_RefTof_Parameter = VI5300_Config_RefTof_Parameter,
	.Read_ChipID = VI5300_Read_ChipID,
};
struct vi5300_api_fn_t *vi5300_func_tbl;

static void vi5300_setupAPIFunctions(void)
{
	vi5300_func_tbl->Power_ON = VI5300_Chip_PowerON;
	vi5300_func_tbl->Power_OFF = VI5300_Chip_PowerOFF;
	vi5300_func_tbl->Chip_Register_Init = VI5300_Chip_Register_Init;
	vi5300_func_tbl->Set_Period = VI5300_Set_Period;
	vi5300_func_tbl->Single_Measure = VI5300_Single_Measure;
	vi5300_func_tbl->Start_Continuous_Measure = VI5300_Start_Continuous_Measure;
	vi5300_func_tbl->Stop_Continuous_Measure = VI5300_Stop_Continuous_Measure;
	vi5300_func_tbl->Get_Measure_Data = VI5300_Get_Measure_Data;
	vi5300_func_tbl->Get_Interrupt_State = VI5300_Get_Interrupt_State;
	vi5300_func_tbl->Chip_Init = VI5300_Chip_Init;
	vi5300_func_tbl->Start_XTalk_Calibration = VI5300_Start_XTalk_Calibration;
	vi5300_func_tbl->Start_Offset_Calibration = VI5300_Start_Offset_Calibration;
	vi5300_func_tbl->Get_XTalk_Parameter = VI5300_Get_XTalk_Parameter;
	vi5300_func_tbl->Config_XTalk_Parameter = VI5300_Config_XTalk_Parameter;
	vi5300_func_tbl->Config_RefTof_Parameter = VI5300_Config_RefTof_Parameter;
	vi5300_func_tbl->Read_ChipID = VI5300_Read_ChipID;
}

static void vi5300_enable_irq(struct  vi5300_data *data)
{
	if(!data)
		return;

	if(data->intr_state == VI5300_INTR_DISABLED)
	{
		data->intr_state = VI5300_INTR_ENABLED;
		enable_irq(data->irq);
	}
}

static void vi5300_disable_irq(struct  vi5300_data *data)
{
	if(!data)
		return;

	if(data->intr_state == VI5300_INTR_ENABLED)
	{
		data->intr_state = VI5300_INTR_DISABLED;
		disable_irq(data->irq);
	}
}

static ssize_t vi5300_chip_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vi5300_data *data = dev_get_drvdata(dev);

	if(NULL != data)
		return scnprintf(buf, PAGE_SIZE, "%u\n", data->chip_enable);

	return -EINVAL;
}

static ssize_t vi5300_chip_enable_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct vi5300_data *data = dev_get_drvdata(dev);
	VI5300_Error Status = VI5300_ERROR_NONE;
	unsigned int val = 0;

	if(NULL != data)
	{
		mutex_lock(&data->work_mutex);
		if(sscanf(buf, "%u\n", &val) != 1)
		{
			mutex_unlock(&data->work_mutex);
			return -1;
		}
		if(val !=0 && val !=1)
		{
			vi5300_errmsg("enable store unvalid value=%u\n", val);
			mutex_unlock(&data->work_mutex);
			return count;
		}
		if(val == 1)
		{
			if(data->chip_enable == 0)
			{
				data->chip_enable = 1;
				vi5300_enable_irq(data);
				Status = vi5300_func_tbl->Power_ON(data);
				ktime_get_real_ts64(&data->start_ts);
			} else {
				vi5300_errmsg("already enabled!!\n");
			}
		} else {
			if(data->chip_enable == 1)
			{
				data->chip_enable = 0;
				vi5300_disable_irq(data);
				data->fwdl_status = 0;
				Status = vi5300_func_tbl->Power_OFF(data);
			}
			else {
				vi5300_errmsg("already disabled!!\n");
			}
		}
		mutex_unlock(&data->work_mutex);
		return Status ? -1 : count;
	}

	return -EPERM;
}

static DEVICE_ATTR(chip_enable, 0664, vi5300_chip_enable_show, vi5300_chip_enable_store);

/* for debug */
static ssize_t vi5300_enable_debug_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vi5300_data *data = dev_get_drvdata(dev);

	if(NULL != data)
		return snprintf(buf, PAGE_SIZE, "%u\n", data->enable_debug);

	return -EINVAL;
}

static ssize_t vi5300_enable_debug_store(struct device *dev,
					struct device_attribute *attr, const
					char *buf, size_t count)
{
	struct vi5300_data *data = dev_get_drvdata(dev);
	unsigned int on = 0;

	if(NULL != data)
	{
		mutex_lock(&data->work_mutex);
		if(sscanf(buf, "%u\n", &on) != 1)
		{
			mutex_unlock(&data->work_mutex);
			return -1;
		}
		if ((on != 0) &&  (on != 1)) {
			vi5300_errmsg("set debug=%d\n", on);
			mutex_unlock(&data->work_mutex);
			return count;
		}
		mutex_unlock(&data->work_mutex);
		data->enable_debug = on;
		return count;
	}

	return -EINVAL;
}

static DEVICE_ATTR(enable_debug, 0664, vi5300_enable_debug_show, vi5300_enable_debug_store);

static ssize_t vi5300_chip_init_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct vi5300_data *data = dev_get_drvdata(dev);
	VI5300_Error Status = VI5300_ERROR_NONE;
	unsigned int val = 0;

	if(NULL != data)
	{
		mutex_lock(&data->work_mutex);
		if(sscanf(buf, "%u\n", &val) != 1)
		{
			mutex_unlock(&data->work_mutex);
			return -1;
		}
		
		if(val)
		{
			Status = vi5300_func_tbl->Chip_Init(data);
			data->fwdl_status = 1;
		} else {
			mutex_unlock(&data->work_mutex);
			return -EINVAL;
		}
		mutex_unlock(&data->work_mutex);
		return Status ? -1 : count;
	}

	return -EPERM; 
}

static DEVICE_ATTR(chip_init, 0220, NULL, vi5300_chip_init_store);

static ssize_t vi5300_period_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vi5300_data *data = dev_get_drvdata(dev);

	if(data != NULL)
		return scnprintf(buf, PAGE_SIZE, "%u\n", data->period);

	return -EINVAL;
}

static ssize_t vi5300_period_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t val = 0;
	struct vi5300_data *data = dev_get_drvdata(dev);

	if(data != NULL)
	{
		mutex_lock(&data->work_mutex);
		if(sscanf(buf, "%u\n", &val) != 1)
		{
			mutex_unlock(&data->work_mutex);
			return -EINVAL;
		}

		data->period = val;
		vi5300_func_tbl->Set_Period(data, data->period);
		mutex_unlock(&data->work_mutex);
		return count;
	}

	return -EPERM;
}

static DEVICE_ATTR(period, 0664, vi5300_period_show, vi5300_period_store);

static ssize_t vi5300_capture_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct vi5300_data *data = dev_get_drvdata(dev);
	VI5300_Error Status = VI5300_ERROR_NONE;
	unsigned int val = 0;

	if(NULL != data)
	{
		mutex_lock(&data->work_mutex);
		if(sscanf(buf, "%u\n", &val) != 1)
		{
			mutex_unlock(&data->work_mutex);
			return -1;
		}
		if(val !=0 && val !=1)
		{
			vi5300_errmsg("capture store unvalid value=%u\n", val);
			mutex_unlock(&data->work_mutex);
			return count;
		}
		if(val == 1)
		{
			Status = vi5300_func_tbl->Start_Continuous_Measure(data);
		} else {
			Status = vi5300_func_tbl->Stop_Continuous_Measure(data);
		}
		mutex_unlock(&data->work_mutex);
		return Status ? -1 : count;
	}

	return -EPERM; 
}

static DEVICE_ATTR(capture, 0220, NULL, vi5300_capture_store);

static ssize_t vi5300_xtalk_calib_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vi5300_data *data = dev_get_drvdata(dev);

	if(data != NULL)
		return scnprintf(buf, PAGE_SIZE, "%d %u %u\n", data->XtalkData.xtalk_cal,
						data->XtalkData.xtalk_peak, data->XtalkData.xtalk_maxratio);

	return -EPERM;
}

static ssize_t vi5300_xtalk_calib_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct vi5300_data *data = dev_get_drvdata(dev);
	VI5300_Error Status = VI5300_ERROR_NONE;
	unsigned int val = 0;

	if(NULL != data)
	{
		mutex_lock(&data->work_mutex);
		if(sscanf(buf, "%u\n", &val) != 1)
		{
			mutex_unlock(&data->work_mutex);
			return -1;
		}
		if(val !=1)
		{
			vi5300_errmsg("xtalk calibration store unvalid value=%u\n", val);
			mutex_unlock(&data->work_mutex);
			return count;
		}
		xtalk_mark = 1;
		Status = vi5300_func_tbl->Start_XTalk_Calibration(data);
		mdelay(600);
		xtalk_mark = 0;
		mutex_unlock(&data->work_mutex);
		return Status ? -1 : count;
	}

	return -EPERM; 
}

static DEVICE_ATTR(xtalk_calib, 0664, vi5300_xtalk_calib_show, vi5300_xtalk_calib_store);

static ssize_t vi5300_offset_calib_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vi5300_data *data = dev_get_drvdata(dev);

	if(data != NULL)
		return scnprintf(buf, PAGE_SIZE, "%d\n", data->OffsetData.offset_cal);

	return -EPERM;
}

static ssize_t vi5300_offset_calib_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct vi5300_data *data = dev_get_drvdata(dev);
	VI5300_Error Status = VI5300_ERROR_NONE;
	unsigned int val = 0;

	if(NULL != data)
	{
		mutex_lock(&data->work_mutex);
		if(sscanf(buf, "%u\n", &val) != 1)
		{
			mutex_unlock(&data->work_mutex);
			return -1;
		}
		if(val !=1)
		{
			vi5300_errmsg("offset calibration store unvalid value=%u\n", val);
			mutex_unlock(&data->work_mutex);
			return count;
		}
		offset_mark = 1;
		Status = vi5300_func_tbl->Start_Offset_Calibration(data);
		offset_mark = 0;
		mutex_unlock(&data->work_mutex);
		return Status ? -1 : count;
	}

	return -EPERM; 
}

static DEVICE_ATTR(offset_calib, 0664, vi5300_offset_calib_show, vi5300_offset_calib_store);

static ssize_t vi5300_xtalk_config_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct vi5300_data *data = dev_get_drvdata(dev);
	VI5300_Error Status = VI5300_ERROR_NONE;
	struct VI5300_XTALK_Config_Data *xtalk_data =
			(struct VI5300_XTALK_Config_Data *)buf;

	if(NULL != data)
	{
		mutex_lock(&data->work_mutex);
		data->XtalkConfig.xtalk_config = xtalk_data->xtalk_config;
		data->XtalkConfig.maxratio = xtalk_data->maxratio;
		Status = vi5300_func_tbl->Config_XTalk_Parameter(data);
		mutex_unlock(&data->work_mutex);
		return Status ? -1 : count;
	}

	return -EPERM; 
}

static DEVICE_ATTR(xtalk_config, 0220, NULL, vi5300_xtalk_config_store);

static ssize_t vi5300_offset_config_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct vi5300_data *data = dev_get_drvdata(dev);
	int val = 0;

	if(NULL != data)
	{
		mutex_lock(&data->work_mutex);
		if(sscanf(buf, "%x\n", &val) != 1)
		{
			mutex_unlock(&data->work_mutex);
			return -1;
		}
		data->offset_config = (int16_t)val;
		mutex_unlock(&data->work_mutex);
		return count;
	}

	return -EPERM;
}

static DEVICE_ATTR(offset_config, 0220, NULL, vi5300_offset_config_store);

static ssize_t vi5300_reftof_config_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct vi5300_data *data = dev_get_drvdata(dev);
	int val = 0;

	if(NULL != data)
	{
		mutex_lock(&data->work_mutex);
		if(sscanf(buf, "%x\n", &val) != 1)
		{
			mutex_unlock(&data->work_mutex);
			return -1;
		}
		data->reftof_config = (int16_t)val;
		mutex_unlock(&data->work_mutex);
		return count;
	}

	return -EPERM;
}

static DEVICE_ATTR(reftof_config, 0220, NULL, vi5300_reftof_config_store);

static ssize_t vi5300_xtalk_data_read(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct vi5300_data *data = dev_get_drvdata(dev);
	void *src = (void *) &(data->XtalkData);
	int rc = 0;

	mutex_lock(&data->work_mutex);
	if (!data->chip_enable) {
		vi5300_errmsg("can't set calib data while disable sensor\n");
		mutex_unlock(&data->work_mutex);
		return -EBUSY;
	}

	if (count > sizeof(struct VI5300_XTALK_Calib_Data))
		count = sizeof(struct VI5300_XTALK_Calib_Data);

	memcpy(buf, src, count);
	data->XtalkConfig.xtalk_config = data->XtalkData.xtalk_cal;
	data->XtalkConfig.maxratio = data->XtalkData.xtalk_maxratio;
	rc = vi5300_func_tbl->Config_XTalk_Parameter(data);
	if (rc) {
		vi5300_errmsg("config xtalk calibration data fail %d", rc);
		mutex_unlock(&data->work_mutex);
		return rc;
	}
	mutex_unlock(&data->work_mutex);

	return count;
}

static ssize_t vi5300_xtalk_data_write(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct vi5300_data *data = dev_get_drvdata(dev);
	struct VI5300_XTALK_Calib_Data *xtalk_data =
				(struct VI5300_XTALK_Calib_Data *)buf;
	int rc = 0;

	mutex_lock(&data->work_mutex);

	if (!data->chip_enable) {
		rc = -EBUSY;
		vi5300_errmsg("can't set calib data while disable sensor\n");
		goto error;
	}

	if (count != sizeof(struct VI5300_XTALK_Calib_Data))
		goto invalid;

	if(data->enable_debug) {
		vi5300_errmsg("xtalk config: %d\n", xtalk_data->xtalk_cal);
		vi5300_errmsg("xtalk maxratio: %d\n", xtalk_data->xtalk_maxratio);
	}

	data->XtalkConfig.xtalk_config = xtalk_data->xtalk_cal;
	data->XtalkConfig.maxratio = xtalk_data->xtalk_maxratio;
	rc = vi5300_func_tbl->Config_XTalk_Parameter(data);
	if (rc) {
		vi5300_errmsg("config xtalk calibration data fail %d", rc);
		goto error;
	}
	mutex_unlock(&data->work_mutex);

	return count;

invalid:
	vi5300_errmsg("invalid syntax");
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static ssize_t vi5300_offset_data_read(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct vi5300_data *data = dev_get_drvdata(dev);
	void *src = (void *) &(data->OffsetData);

	mutex_lock(&data->work_mutex);
	if (!data->chip_enable) {
		vi5300_errmsg("can't set calib data while disable sensor\n");
		mutex_unlock(&data->work_mutex);
		return -EBUSY;
	}

	if (count > sizeof(struct VI5300_OFFSET_Calib_Data))
		count = sizeof(struct VI5300_OFFSET_Calib_Data);

	memcpy(buf, src, count);
	data->offset_config = data->OffsetData.offset_cal;
	mutex_unlock(&data->work_mutex);

	return count;
}

static ssize_t vi5300_offset_data_write(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct vi5300_data *data = dev_get_drvdata(dev);
	struct VI5300_OFFSET_Calib_Data *offset_data = 
				(struct VI5300_OFFSET_Calib_Data *)buf;
	int rc = 0;

	mutex_lock(&data->work_mutex);
	if (!data->chip_enable) {
		rc = -EBUSY;
		vi5300_errmsg("can't set calib data while disable sensor\n");
		goto error;
	}

	if (count != sizeof(struct VI5300_OFFSET_Calib_Data))
		goto invalid;

	if(data->enable_debug)
		vi5300_errmsg("offset config: %d\n", offset_data->offset_cal);

	data->offset_config = offset_data->offset_cal;
	mutex_unlock(&data->work_mutex);
	return count;

invalid:
	vi5300_errmsg("invalid syntax");
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);
	return rc;
}

static ssize_t vi5300_reftof_data_write(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct vi5300_data *data = dev_get_drvdata(dev);
	struct VI5300_OFFSET_Calib_Data *offset_data = 
				(struct VI5300_OFFSET_Calib_Data *)buf;
	int rc = 0;

	mutex_lock(&data->work_mutex);
	if (!data->chip_enable) {
		rc = -EBUSY;
		vi5300_errmsg("can't set calib data while disable sensor\n");
		goto error;
	}

	if (count != sizeof(struct VI5300_OFFSET_Calib_Data))
		goto invalid;

	if(data->enable_debug)
		vi5300_errmsg("reftof config: %d\n", offset_data->ref_tof);

	data->reftof_config = offset_data->ref_tof;
	rc = vi5300_func_tbl->Config_RefTof_Parameter(data);
	if (rc) {
		vi5300_errmsg("config reftof calibration data fail %d", rc);
		goto error;
	}
	mutex_unlock(&data->work_mutex);
	return count;

invalid:
	vi5300_errmsg("invalid syntax");
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);
	return rc;
}

static struct attribute *vi5300_attributes[] = {
	&dev_attr_chip_enable.attr,
	&dev_attr_enable_debug.attr,
	&dev_attr_chip_init.attr,
	&dev_attr_period.attr,
	&dev_attr_capture.attr,
	&dev_attr_xtalk_calib.attr,
	&dev_attr_offset_calib.attr,
	&dev_attr_xtalk_config.attr,
	&dev_attr_offset_config.attr,
	&dev_attr_reftof_config.attr,
	NULL,
};

static const struct attribute_group vi5300_attr_group = {
	.name = NULL,
	.attrs = vi5300_attributes,
};

static struct bin_attribute vi5300_xtalk_data_attr = {
	.attr = {
		.name = "xtalk_calib_data",
		.mode = 0664/*S_IWUGO | S_IRUGO*/,
	},
	.size = sizeof(struct VI5300_XTALK_Calib_Data),
	.read = vi5300_xtalk_data_read,
	.write = vi5300_xtalk_data_write,
};

static struct bin_attribute vi5300_offset_data_attr = {
	.attr = {
		.name = "offset_calib_data",
		.mode = 0664/*S_IWUGO | S_IRUGO*/,
	},
	.size = sizeof(struct VI5300_OFFSET_Calib_Data),
	.read = vi5300_offset_data_read,
	.write = vi5300_offset_data_write,
};

static struct bin_attribute vi5300_reftof_data_attr = {
	.attr = {
		.name = "reftof_calib_data",
		.mode = 0222/*S_IWUGO*/,
	},
	.size = sizeof(struct VI5300_OFFSET_Calib_Data),
	.write = vi5300_reftof_data_write,
};

static irqreturn_t vi5300_irq_handler(int vec, void *info)
{
	struct vi5300_data *data = (struct vi5300_data *)info;
	VI5300_Error Status = VI5300_ERROR_NONE;

	if(!data || !data->fwdl_status)
		return IRQ_HANDLED;

	if (data->irq == vec)
	{
		if(xtalk_mark)
		{
			Status = vi5300_func_tbl->Get_XTalk_Parameter(data);
			if(Status != VI5300_ERROR_NONE)
				vi5300_errmsg("%d : Status = %d\n" , __LINE__, Status);
		}

		if(!xtalk_mark && !offset_mark)
		{
			Status = vi5300_func_tbl->Get_Measure_Data(data);
			if(Status != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d : Status = %d\n" , __LINE__, Status);
				return IRQ_HANDLED;
			}
			input_report_abs(data->input_dev, ABS_HAT0Y, data->Rangedata.timeUSec);
			input_report_abs(data->input_dev, ABS_HAT1X, data->Rangedata.RangeTof);
			input_report_abs(data->input_dev, ABS_HAT1Y, data->Rangedata.RangeNoise);
			input_report_abs(data->input_dev, ABS_BRAKE, data->Rangedata.RangePeak);
			input_report_abs(data->input_dev, ABS_TILT_X, data->Rangedata.RangeConfidence);
			input_report_abs(data->input_dev, ABS_WHEEL, data->Rangedata.RangeStatus);
			input_report_abs(data->input_dev, ABS_TILT_Y, data->Rangedata.RangeCGcount);
			input_report_abs(data->input_dev, ABS_HAT3X, data->Rangedata.RangeIntegralTimes);
			input_sync(data->input_dev);
		}
	}
	return IRQ_HANDLED;
}

static int vi5300_open(struct inode *inode, struct file *file)
{
	vi5300_errmsg("open!\n");
	return 0;
}

static long vi5300_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long rc=0;
	void __user *argp = (void __user *)arg;
	struct vi5300_data *data = container_of(file->private_data,
		struct vi5300_data, miscdev);

	if(!data)
		return -EFAULT;

	switch (cmd) {
		case VI5300_IOCTL_POWER_ON:
			mutex_lock(&data->work_mutex);
			vi5300_enable_irq(data);
			rc = vi5300_func_tbl->Power_ON(data);
			if(rc != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d, CHIP POWER ON FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EIO;
			}
			ktime_get_real_ts64(&data->start_ts);
			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_CHIP_INIT:
			mutex_lock(&data->work_mutex);
			rc = vi5300_func_tbl->Chip_Init(data);
			if(rc != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d, CHIP INIT FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EIO;
			}
			data->fwdl_status = 1;
			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_PERIOD:
			mutex_lock(&data->work_mutex);
			if (copy_from_user(&(data->period), (uint32_t *)argp, sizeof(uint32_t)))
			{
				vi5300_errmsg("%d, GET PERIOD DATA FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EFAULT;
			}
			if(data->enable_debug)
				vi5300_errmsg("period setting: %d\n", data->period);
			vi5300_func_tbl->Set_Period(data, data->period);
			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_XTALK_CALIB:
			mutex_lock(&data->work_mutex);
			xtalk_mark = 1;
			rc = vi5300_func_tbl->Start_XTalk_Calibration(data);
			if(rc != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d, PERFORM XTALK CALIB FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EINVAL;
			}
			msleep(800);
			if (copy_to_user(( struct VI5300_XTALK_Calib_Data *)argp, &(data->XtalkData),
				sizeof( struct VI5300_XTALK_Calib_Data)))
			{
				vi5300_errmsg("%d, COPY XTALK CALIB DATA FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EFAULT;
			}
			xtalk_mark = 0;
			data->XtalkConfig.xtalk_config = data->XtalkData.xtalk_cal;
			data->XtalkConfig.maxratio = data->XtalkData.xtalk_maxratio;
			rc = vi5300_func_tbl->Config_XTalk_Parameter(data);
			if(rc != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d, AFTER CALIBRATION,CONFIG XTALK PARAMETER FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EINVAL;
			}
			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_XTALK_CONFIG:
			mutex_lock(&data->work_mutex);
			if (copy_from_user(&(data->XtalkConfig), (struct VI5300_XTALK_Config_Data *)argp, sizeof(struct VI5300_XTALK_Config_Data)))
			{
				vi5300_errmsg("%d, GET XTALK CALIB DATA FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EFAULT;
			}

			if(data->enable_debug)
			{
				vi5300_errmsg("xtalk config: %d\n", data->XtalkConfig.xtalk_config);
				vi5300_errmsg("xtalk maxratio: %d\n", data->XtalkConfig.maxratio);
			}

			rc = vi5300_func_tbl->Config_XTalk_Parameter(data);
			if(rc != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d, CONFIG XTALK PARAMETER FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EINVAL;
			}
			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_OFFSET_CALIB:
			mutex_lock(&data->work_mutex);
			offset_mark = 1;
			rc = vi5300_func_tbl->Start_Offset_Calibration(data);
			if(rc != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d, PERFORM OFFSET CALIB FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EINVAL;
			}
			if (copy_to_user(( struct VI5300_OFFSET_Calib_Data *)argp, &(data->OffsetData),
				sizeof( struct VI5300_OFFSET_Calib_Data)))
			{
				vi5300_errmsg("%d, COPY OFFSET CALIB DATA FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EFAULT;
			}
			offset_mark = 0;
			data->offset_config = data->OffsetData.offset_cal;
			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_OFFSET_CONFIG:
			mutex_lock(&data->work_mutex);
			if (copy_from_user(&(data->offset_config), (int16_t *)argp, sizeof(int16_t)))
			{
				vi5300_errmsg("%d, GET OFFSET CALIB DATA FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EFAULT;
			}

			if(data->enable_debug)
				vi5300_errmsg("offset config: %d\n", data->offset_config);

			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_REFTOF_CONFIG:
			mutex_lock(&data->work_mutex);
			if (copy_from_user(&(data->reftof_config), (int16_t *)argp, sizeof(int16_t)))
			{
				vi5300_errmsg("%d, GET REFTOF CALIB DATA FAIL\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EINVAL;
			}

			if(data->enable_debug)
				vi5300_errmsg("reftof config: %d\n", data->reftof_config);

			rc = vi5300_func_tbl->Config_RefTof_Parameter(data);
			if(rc != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d, CONFIG REFTOF DATA FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EFAULT;
			}
			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_START:
			mutex_lock(&data->work_mutex);
			rc = vi5300_func_tbl->Start_Continuous_Measure(data);
			if(rc != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d, CHIP START RANGE FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EIO;
			}
			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_MZ_DATA:
			mutex_lock(&data->work_mutex);
			if (copy_to_user((struct VI5300_Measurement_Data *)argp, &(data->Rangedata),
				sizeof(struct VI5300_Measurement_Data)))
			{
				vi5300_errmsg("%d, COPY CONTINUOUS DATA FAIL\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EFAULT;
			}
			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_STOP:
			mutex_lock(&data->work_mutex);
			rc = vi5300_func_tbl->Stop_Continuous_Measure(data);
			if(rc != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d, CHIP STOP RANGE FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EIO;
			}
			mutex_unlock(&data->work_mutex);
			break;
		case VI5300_IOCTL_POWER_OFF:
			mutex_lock(&data->work_mutex);
			vi5300_disable_irq(data);
			data->fwdl_status = 0;
			rc = vi5300_func_tbl->Power_OFF(data);
			if(rc != VI5300_ERROR_NONE)
			{
				vi5300_errmsg("%d, CHIP POWER OFF FAILED\n", __LINE__);
				mutex_unlock(&data->work_mutex);
				return -EIO;
			}
			mutex_unlock(&data->work_mutex);
			break;
		default:
			rc = -EFAULT;
	}

	return rc;
}

static int vi5300_release(struct inode *inode, struct file *file)
{
	vi5300_errmsg("release!\n");
	return 0;
}

static const struct file_operations vi5300_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vi5300_ioctl,
	.open = vi5300_open,
	.release = vi5300_release,
};

static int vi5300_parse_dt(struct device_node *np, struct  vi5300_data *data)
{
	struct gpio_desc *irq_gpio_desc;
	struct gpio_desc *xshut_gpio_desc;

	if (!data || !np)
		return -EINVAL;

	
	irq_gpio_desc = devm_gpiod_get_optional(data->dev, "irq", GPIOD_IN);
	if (irq_gpio_desc == NULL) {
		vi5300_errmsg("VI5300 Parse DT -> failed to get irq gpio, irq_gpio_desc is NULL\n");
		return -ENODEV;
	}
	if (IS_ERR(irq_gpio_desc)) {
		vi5300_errmsg("VI5300 Parse DT -> failed to get irq gpio: %lu\n", PTR_ERR(irq_gpio_desc));
		return PTR_ERR(irq_gpio_desc);
	}

	data->irq_gpio = desc_to_gpio(irq_gpio_desc);
	vi5300_infomsg("VI5300 Parse DT, Interrupt GPIO: %d\n", data->irq_gpio);

	xshut_gpio_desc = devm_gpiod_get_optional(data->dev, "xshut", GPIOD_IN);
	if (xshut_gpio_desc == NULL) {
		vi5300_errmsg("VI5300 Parse DT -> failed to get xshut gpio, xshut_gpio_desc is NULL\n");
		return -ENODEV;
	}
	if (IS_ERR(xshut_gpio_desc)) {
		vi5300_errmsg("VI5300 Parse DT -> failed to get xshut gpio: %lu\n", PTR_ERR(xshut_gpio_desc));
		return PTR_ERR(xshut_gpio_desc);
	}

	data->xshut_gpio = desc_to_gpio(xshut_gpio_desc);
	vi5300_infomsg("VI5300 Parse DT, XSHUT GPIO: %d\n", data->xshut_gpio);

	return  0;
}

static int vi5300_setup(struct  vi5300_data *data)
{
	int rc=0;
	int irq = 0;
	uint8_t buf = 0;

	if (!data)
		return -EINVAL;

	if (!gpio_is_valid(data->irq_gpio) || !gpio_is_valid(data->xshut_gpio))
		return -ENODEV;

	gpio_request(data->xshut_gpio, "vi5300 xshut gpio");
	gpio_request(data->irq_gpio, "vi5300 irq gpio");
	gpio_direction_input(data->irq_gpio);
	irq = gpio_to_irq(data->irq_gpio);
	if(irq<0)
	{
		vi5300_errmsg("fail to map GPIO: %d to INT: %d\n", data->irq_gpio, irq);
		rc = -EINVAL;
		goto exit_free_gpio;
	} else {
		vi5300_dbgmsg("request irq: %d\n", irq);
		rc = request_threaded_irq(irq,  NULL, vi5300_irq_handler,
			IRQF_TRIGGER_FALLING |IRQF_ONESHOT, "vi5300_interrupt", (void *)data);
		if (rc) {
			vi5300_errmsg("%s(%d), Could not allocate VI5300_INT ! result:%d\n",__FUNCTION__, __LINE__, rc);
			goto exit_free_irq;
		}
	}
	data->irq = irq;
	data->intr_state = VI5300_INTR_DISABLED;
	disable_irq(data->irq);
	data->fwdl_status = 0;
	vi5300_func_tbl = &vi5300_api_func_tbl;
	vi5300_setupAPIFunctions();
	vi5300_func_tbl->Power_ON(data);
	vi5300_func_tbl->Chip_Register_Init(data);
	vi5300_read_byte(data, VI5300_REG_DEV_ADDR, &buf);
	vi5300_func_tbl->Power_OFF(data);
	if(buf != VI5300_CHIP_ADDR)
	{
		vi5300_errmsg("VI5300 I2C Transfer Failed, ChipAddr = 0x%x\n", buf);
		rc = -EFAULT;
		goto exit_free_irq;
	}
	vi5300_infomsg("VI5300 I2C Transfer Successfully, ChipAddr = 0x%x\n", buf);

	data->input_dev = input_allocate_device();
	if (data->input_dev == NULL) {
		vi5300_errmsg("Error allocating input_dev.\n");
		goto input_dev_alloc_err;
	}
	data->input_dev->name = "vi5300";
	data->input_dev->id.bustype = BUS_I2C;
	input_set_drvdata(data->input_dev, data);
	set_bit(EV_ABS, data->input_dev->evbit);
	input_set_abs_params(data->input_dev, ABS_HAT3X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(data->input_dev, ABS_HAT0Y, 0, 0xffffffff, 0, 0);
	input_set_abs_params(data->input_dev, ABS_HAT1X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(data->input_dev, ABS_HAT1Y, 0, 0xffffffff, 0, 0);
	input_set_abs_params(data->input_dev, ABS_BRAKE, 0, 0xffff, 0, 0);
	input_set_abs_params(data->input_dev, ABS_TILT_X, 0, 0xffff, 0, 0);
	input_set_abs_params(data->input_dev, ABS_WHEEL, 0, 0xffff, 0, 0);
	input_set_abs_params(data->input_dev, ABS_TILT_Y, 0, 0xffff, 0, 0);
	rc = input_register_device(data->input_dev);
	if(rc) {
		vi5300_errmsg("Error registering input_dev.\n");
		goto input_reg_err;
	}
	rc = sysfs_create_group(&data->input_dev->dev.kobj, &vi5300_attr_group);
	if (rc) {
		vi5300_errmsg("Error creating sysfs attribute group.\n");
		goto sysfs_create_group_err;
	}
	rc = sysfs_create_bin_file(&data->input_dev->dev.kobj, &vi5300_xtalk_data_attr);
	if (rc) {
		rc = -ENOMEM;
		vi5300_errmsg("%d error:%d\n", __LINE__, rc);
		goto sysfs_create_bin_err1;
	}
	rc = sysfs_create_bin_file(&data->input_dev->dev.kobj, &vi5300_offset_data_attr);
	if (rc) {
		rc = -ENOMEM;
		vi5300_errmsg("%d error:%d\n", __LINE__, rc);
		goto sysfs_create_bin_err2;
	}
	rc = sysfs_create_bin_file(&data->input_dev->dev.kobj, &vi5300_reftof_data_attr);
	if (rc) {
		rc = -ENOMEM;
		vi5300_errmsg("%d error:%d\n", __LINE__, rc);
		goto sysfs_create_bin_err3;
	}

	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	data->miscdev.name = "vi5300";
	data->miscdev.fops = &vi5300_fops;
	if (misc_register(&data->miscdev) != 0)
	{
		vi5300_errmsg("Could not register misc. dev for VI5300 Sensor\n");
		rc = -ENOMEM;
		goto misc_register_err;
	}
	data->period = 30;
	data->XtalkConfig.xtalk_config = 0;
	data->XtalkConfig.maxratio = 0;
	data->offset_config = 0;
	data->enable_debug = 0;
	data->Rangedata.RangeStatus = 255;

	return 0;

misc_register_err:
	sysfs_remove_bin_file(&data->input_dev->dev.kobj,
		&vi5300_reftof_data_attr);
sysfs_create_bin_err3:
	sysfs_remove_bin_file(&data->input_dev->dev.kobj,
		&vi5300_offset_data_attr);
sysfs_create_bin_err2:
	sysfs_remove_bin_file(&data->input_dev->dev.kobj,
		&vi5300_xtalk_data_attr);
sysfs_create_bin_err1:
	sysfs_remove_group(&data->input_dev->dev.kobj,
		&vi5300_attr_group);
sysfs_create_group_err:
	input_unregister_device(data->input_dev);
input_reg_err:
	input_free_device(data->input_dev);
input_dev_alloc_err:
exit_free_irq:
	free_irq(irq, data);
exit_free_gpio:
	gpio_free(data->xshut_gpio);
	gpio_free(data->irq_gpio);
	return rc;
}

static int vi5300_probe(struct i2c_client *client)
{
	struct vi5300_data *vi5300_data = NULL;
	struct device *dev = &client->dev;
	struct device_node *node;
	int ret  = 0;

	vi5300_infomsg("Try to probe VI5300 sensor!\n");
	vi5300_data = kzalloc(sizeof(struct vi5300_data), GFP_KERNEL);
	if(!vi5300_data)
	{
		vi5300_errmsg("devm_kzalloc error\n");
		return -ENOMEM;
	}
	if (!dev->of_node)
	{
		vi5300_errmsg("VI5300 Error dev->of_node = NULL\n");
		kfree(vi5300_data);
		return -EINVAL;
	}
	/* setup device data */
	vi5300_infomsg("VI5300 Try to setup!!!\n");
	vi5300_data->dev_name = dev_name(&client->dev);
	vi5300_data->client = client;
	vi5300_data->dev = dev;
	node = dev->of_node;
	i2c_set_clientdata(client, vi5300_data);
	mutex_init(&vi5300_data->work_mutex);
	ret = vi5300_parse_dt(node, vi5300_data);
	if(ret) {
		vi5300_errmsg("VI5300 Parse DT Failed\n");
		goto exit_error;
	}
	ret = vi5300_setup(vi5300_data);
	if(ret) {
		vi5300_errmsg("VI5300 Setup Failed\n");
		goto exit_error;
	}
	vi5300_infomsg("Probe OK for dToF sensor VI5300!\n");
	return 0;

exit_error:
	vi5300_errmsg("VI5300 Error label!!!\n");
	mutex_destroy(&vi5300_data->work_mutex);
	i2c_set_clientdata(client, NULL);
	kfree(vi5300_data);
	return ret;

}

static void vi5300_remove(struct i2c_client *client)
{
	struct vi5300_data *data = i2c_get_clientdata(client);

	if(data->input_dev)
	{
		vi5300_dbgmsg("to unregister sysfs dev\n");
		sysfs_remove_group(&data->input_dev->dev.kobj,
			&vi5300_attr_group);
		sysfs_remove_bin_file(&data->input_dev->dev.kobj,
			&vi5300_xtalk_data_attr);
		sysfs_remove_bin_file(&data->input_dev->dev.kobj,
			&vi5300_offset_data_attr);
		sysfs_remove_bin_file(&data->input_dev->dev.kobj,
			&vi5300_reftof_data_attr);
		vi5300_dbgmsg("to unregister input dev\n");
		input_unregister_device(data->input_dev);
	}
	if (!IS_ERR(data->miscdev.this_device) &&
			data->miscdev.this_device != NULL) {
		vi5300_dbgmsg("to unregister misc dev\n");
		misc_deregister(&data->miscdev);
	}
	if(data->xshut_gpio)
	{
		gpio_direction_output(data->xshut_gpio, 0);
		gpio_free(data->xshut_gpio);
	}
	if(data->irq_gpio)
	{
		free_irq(data->irq, data);
		gpio_free(data->irq_gpio);
	}
	i2c_set_clientdata(client, NULL);
	mutex_destroy(&data->work_mutex);
	kfree(data);
	//return 0;
}
static const struct i2c_device_id vi5300_id[] = {
	{ VI5300_DRV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, vi5300_id);

static const struct of_device_id vi5300_dt_match[] = {
	{.compatible = "evisionics,vi5300",},
	{},
};
MODULE_DEVICE_TABLE(of, vi5300_dt_match);

struct i2c_driver vi5300_driver = {
	.driver  = {
		.name = VI5300_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vi5300_dt_match,
	},
	.probe = vi5300_probe,
	.remove = vi5300_remove,
	.id_table = vi5300_id,
};

static int __init vi5300_init(void)
{
	return i2c_add_driver(&vi5300_driver);
}

static void  __exit vi5300_exit(void)
{
	i2c_del_driver(&vi5300_driver);
}

module_init(vi5300_init);
module_exit(vi5300_exit);

MODULE_AUTHOR("William.li<william.li@vidar.ai>");
MODULE_AUTHOR("Vasyl Dykyj");
MODULE_DESCRIPTION("VI5300 FlightSense TOF  sensor Driver");
MODULE_LICENSE("GPL");
