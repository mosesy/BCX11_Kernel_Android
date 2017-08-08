/* drivers/input/misc/cm32181.c - cm32181 light sensor driver
 *
 * Copyright (C) 2014 Capella Microsystems, Inc.
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
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/cm32181.h>
#include <linux/fs.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/acpi.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_ALS                   0x01

#define CALIBRATION_FILE_PATH	"/data/cal_data"
#define CHANGE_SENSITIVITY 5 // in percent

static int record_init_fail = 0;
static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

struct cm32181_info {
	struct class *cm32181_class;
	struct device *ls_dev;

	struct input_dev *ls_input_dev;

	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int intr_pin;
	int als_enable;
	int als_enabled_before_suspend;	

	int irq;

	int (*power)(int, uint8_t); /* power to the chip */
	
	uint32_t als_resolution; // represented using a fixed 10(-5) notation
	uint32_t cal_data; // represented using a fixed 10(-5) notation	

	struct wake_lock ls_wake_lock;
	int lightsensor_opened;
	uint8_t ALS_cmd_address; // could be 0x48 or 0x10, specified in platform data and is determined by pin ADDR configuration

	uint32_t current_lux_level;
	uint16_t current_adc;

	uint16_t is_cmd;
	uint8_t record_clear_int_fail;
};

struct cm32181_info *lp_info;
int enable_log = 0;
bool cal_data_retrieved = false;
static struct mutex als_enable_mutex, als_disable_mutex;
static struct mutex CM32181_control_mutex;
static int lightsensor_enable(struct cm32181_info *lpi);
static int lightsensor_disable(struct cm32181_info *lpi);
static int initial_cm32181(struct cm32181_info *lpi);

static int control_and_report(struct cm32181_info *lpi, uint8_t mode, uint8_t cmd_enable);

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
  uint8_t subaddr[1];
  
  subaddr[0] = cmd;	
	struct cm32181_info *lpi = lp_info;

	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = subaddr,
		 },
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++)
	{

		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 2) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[LS][CM32181 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
				__func__, slaveAddr, lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT)
	{
		printk(KERN_ERR "[PS_ERR][CM32181 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm32181_info *lpi = lp_info;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++)
	{
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[LS][CM32181 error] %s, i2c err, slaveAddr 0x%x, value 0x%x, ISR gpio%d  = %d, record_init_fail %d\n",
				__func__, slaveAddr, txData[0], lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT)
	{
		printk(KERN_ERR "[PS_ERR][CM32181 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm32181_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0)
	{
		pr_err(
			"[PS_ERR][CM32181 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM32181] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int _cm32181_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[CM32181] %s: _cm32181_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);	
	
	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0)
	{
		pr_err("[PS_ERR][CM32181 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ls_adc_value(uint16_t *als_step, bool resume)
{
	struct cm32181_info *lpi = lp_info;
	int ret = 0;

	if (als_step == NULL)
		return -EFAULT;

	/* Read ALS data: */
	ret = _cm32181_I2C_Read_Word(lpi->ALS_cmd_address, ALS_DATA, als_step);
	if (ret < 0)
	{
		pr_err(
			"[LS][CM32181 error]%s: _cm32181_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	}

	return ret;
}

static int set_lsensor_range(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct cm32181_info *lpi = lp_info;

	_cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_HW, high_thd);
	_cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_LW, low_thd);

	return ret;
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct cm32181_info *lpi = lp_info;
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, 0);

	enable_irq(lpi->irq);
}

static irqreturn_t cm32181_irq_handler(int irq, void *data)
{
	struct cm32181_info *lpi = data;

	disable_irq_nosync(lpi->irq);
	if (enable_log)
		D("[LS][CM32181] %s\n", __func__);

	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	struct cm32181_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static int lightsensor_get_cal_data(struct cm32181_info *lpi)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		err = PTR_ERR(cal_filp);
		if (err != -ENOENT)
			pr_err("%s: Can't open calibration data file\n", __func__);
		set_fs(old_fs);
		return err;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint32_t), &cal_filp->f_pos);
	if (err != sizeof(uint32_t))
	{
		pr_err("%s: Can't read the calibration data from file\n", __func__);
		err = -EIO;
	}

	pr_info("%s: cal_data = %d\n",
		__func__, lpi->cal_data);

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return err;
}

static int lightsensor_enable(struct cm32181_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&als_enable_mutex);
	D("[LS][CM32181] %s\n", __func__);

	if (lpi->als_enable)
	{
		D("[LS][CM32181] %s: already enabled\n", __func__);
		ret = 0;
	}
	else
	{
		if (!cal_data_retrieved)
		{
			/* get calibration data */		
			ret = lightsensor_get_cal_data(lpi);
			if (ret < 0 && ret != -ENOENT)
			{
				pr_err("%s: lightsensor_get_cal_data() failed\n",
					__func__);
			}
			else
			{
				cal_data_retrieved = true;
			}
		}	
	
		ret = control_and_report(lpi, CONTROL_ALS, 1);
	}
	
	mutex_unlock(&als_enable_mutex);
	return ret;
}

static int lightsensor_disable(struct cm32181_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&als_disable_mutex);
	D("[LS][CM32181] %s\n", __func__);

	if (lpi->als_enable == 0)
	{
		D("[LS][CM32181] %s: already disabled\n", __func__);
		ret = 0;
	}
	else
	{
		ret = control_and_report(lpi, CONTROL_ALS, 0);
	}
	
	mutex_unlock(&als_disable_mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm32181_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM32181] %s\n", __func__);
	if (lpi->lightsensor_opened)
	{
		pr_err("[LS][CM32181 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm32181_info *lpi = lp_info;

	D("[LS][CM32181] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm32181_info *lpi = lp_info;

	/*D("[CM32181] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

	switch (cmd)
	{
		case LIGHTSENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg))
			{
				rc = -EFAULT;
				break;
			}
			D("[LS][CM32181] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
				__func__, val);
			rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
			break;
		case LIGHTSENSOR_IOCTL_GET_ENABLED:
			val = lpi->als_enable;
			D("[LS][CM32181] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
				__func__, val);
			rc = put_user(val, (unsigned long __user *)arg);
			break;
		default:
			pr_err("[LS][CM32181 error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm32181_info *lpi = lp_info;
	uint16_t adc_value = 0;
	uint32_t lux_level;
	
	if (lpi->als_enable)
	{
		get_ls_adc_value(&adc_value, 0);
		lux_level = (uint32_t)div64_u64((uint64_t)adc_value * lpi->als_resolution * lpi->cal_data, (uint64_t)100000 * 100000);	
		lpi->current_lux_level = lux_level;
		lpi->current_adc = adc_value;
	}	

	D("[LS][CM32181] %s: ADC = 0x%04X, Lux Level = %d \n",
		__func__, lpi->current_adc, lpi->current_lux_level);
	ret = sprintf(buf, "ADC[0x%04X] => lux level %d\n",
		lpi->current_adc, lpi->current_lux_level);

	return ret;
}

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm32181_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm32181_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto)
	{
		ret = lightsensor_enable(lpi);
	}
	else
	{
		ret = lightsensor_disable(lpi);
	}

	D("[LS][CM32181] %s: lpi->als_enable = %d, ls_auto=%d\n",
		__func__, lpi->als_enable, ls_auto);

	if (ret < 0)
		pr_err(
		"[LS][CM32181 error]%s: set auto light sensor fail\n",
		__func__);

	return count;
}

static uint8_t ALS_CONF = 0;
static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ALS_CONF = %x\n", ALS_CONF);
}

static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm32181_info *lpi = lp_info;
	int value = 0;
	sscanf(buf, "0x%x", &value);

	ALS_CONF = value;
	printk(KERN_INFO "[LS]set ALS_CONF = %x\n", ALS_CONF);
	_cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, ALS_CONF);
	return count;
}

static ssize_t ls_cal_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm32181_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "%d\n", lpi->cal_data);

	return ret;
}

static ssize_t ls_cal_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint32_t new_cal_data = 0;
	struct cm32181_info *lpi = lp_info;	
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	int err = 0;

	sscanf(buf, "%d", &new_cal_data);
	if (new_cal_data != 0)
	{
		lpi->cal_data = new_cal_data;
	}
	else  // reset calibration data
	{
		lpi->cal_data = 2000000;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint32_t), &cal_filp->f_pos);
	if (err != sizeof(uint32_t))
	{
		pr_err("%s: Can't write the calibration data to file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return count;
}

static struct device_attribute dev_attr_light_enable =
__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP, ls_enable_show, ls_enable_store);

static struct device_attribute dev_attr_light_conf =
__ATTR(conf, S_IRUGO | S_IWUSR | S_IWGRP, ls_conf_show, ls_conf_store);

static struct device_attribute dev_attr_light_adc =
__ATTR(adc, S_IRUGO | S_IWUSR | S_IWGRP, ls_adc_show, NULL);

static struct device_attribute dev_attr_light_cal_data =
__ATTR(cali, S_IRUGO | S_IWUSR | S_IWGRP, ls_cal_data_show, ls_cal_data_store);

static struct attribute *light_sysfs_attrs[] = {
&dev_attr_light_enable.attr,
&dev_attr_light_conf.attr,
&dev_attr_light_adc.attr,
&dev_attr_light_cal_data.attr,
NULL
};

static struct attribute_group light_attribute_group = {
.attrs = light_sysfs_attrs,
};

static int lightsensor_setup(struct cm32181_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev)
	{
		pr_err(
			"[LS][CM32181 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "cm32181-ls";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0)
	{
		pr_err("[LS][CM32181 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0)
	{
		pr_err("[LS][CM32181 error]%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int initial_cm32181(struct cm32181_info *lpi)
{
	int val, ret, fail_counter = 0;
	uint16_t status = 0;

	val = gpio_get_value(lpi->intr_pin);
	D("[LS][CM32181] %s, INTERRUPT GPIO val = %d\n", __func__, val);

check_interrupt_gpio:
	if (fail_counter >= 10)
	{
		D("[LS][CM32181] %s, initial fail_counter = %d\n", __func__, fail_counter);
		if (record_init_fail == 0)
			record_init_fail = 1;
		return -ENOMEM;/*If devices without cm32181 chip and did not probe driver*/
	}
	lpi->is_cmd = lpi->is_cmd | CM32181_ALS_SD; 
	ret = _cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->is_cmd );
	if ((ret < 0) && (fail_counter < 10))
	{
		fail_counter++;
		val = gpio_get_value(lpi->intr_pin);
		if (val == 0)
		{
			D("[LS][CM32181] %s, interrupt GPIO val = %d, , inital fail_counter %d\n",
				__func__, val, fail_counter);
 			ret = _cm32181_I2C_Read_Word(lpi->ALS_cmd_address, ALS_STATUS, &status);
 			D("[LS][CM32181] %s, status register = 0x%x, ret %d\n",
				__func__, status, ret);
 		}
		goto	check_interrupt_gpio;
	}

	return 0;
}

static int cm32181_setup(struct cm32181_info *lpi)
{
	int ret = 0;

	als_power(1);
	msleep(5);
	ret = gpio_request(lpi->intr_pin, "gpio_cm32181_intr");
	if (ret < 0)
	{
		pr_err("[LS][CM32181 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0)
	{
		pr_err(
			"[LS][CM32181 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	ret = initial_cm32181(lpi);
	if (ret < 0)
	{
		pr_err(
			"[LS_ERR][CM32181 error]%s: fail to initial cm32181 (%d)\n",
			__func__, ret);
		goto fail_free_intr_pin;
	}
	
	ret = request_any_context_irq(lpi->irq,
			cm32181_irq_handler,
			IRQF_TRIGGER_FALLING,
			"cm32181",
			lpi);
	if (ret < 0)
	{
		pr_err(
			"[LS][CM32181 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}


static int cm32181_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm32181_info *lpi;
	struct cm32181_platform_data *pdata;

	D("[LS][CM32181] %s\n", __func__);

	lpi = kzalloc(sizeof(struct cm32181_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	/*D("[CM32181] %s: client->irq = %d\n", __func__, client->irq);*/

	lpi->i2c_client = client;

	pdata = client->dev.platform_data;
	if (!pdata)
	{
		pr_err("[LS][CM32181 error]%s: Assign platform_data error!!\n",
			__func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}

	lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);
	
	lpi->intr_pin = pdata->intr;
	lpi->power = pdata->power;
	
	lpi->ALS_cmd_address = pdata->ALS_slave_address;

	lpi->is_cmd  = pdata->is_cmd;
	
	lpi->record_clear_int_fail = 0;
	
	if (pdata->is_cmd == 0)
	{
		lpi->is_cmd  = CM32181_ALS_SM_2 | CM32181_ALS_IT_200MS | CM32181_ALS_PERS_1;
	}

	lp_info = lpi;

	mutex_init(&CM32181_control_mutex);

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0)
	{
		pr_err("[LS][CM32181 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	lpi->als_resolution = 2200;
	lpi->cal_data = 2000000;

	lpi->lp_wq = create_singlethread_workqueue("cm32181_wq");
	if (!lpi->lp_wq)
	{
		pr_err("[LS][CM32181 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}
	wake_lock_init(&(lpi->ls_wake_lock), WAKE_LOCK_SUSPEND, "ls_wake_lock");

	ret = cm32181_setup(lpi);
	if (ret < 0)
	{
		pr_err("[PS_ERR][CM32181 error]%s: cm32181_setup error!\n", __func__);
		goto err_cm32181_setup;
	}

	lpi->cm32181_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->cm32181_class))
	{
		ret = PTR_ERR(lpi->cm32181_class);
		lpi->cm32181_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm32181_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev)))
	{
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = sysfs_create_group(&lpi->ls_input_dev->dev.kobj,
	&light_attribute_group);
	if (ret)
	{
		pr_err("[LS][CM32181 error]%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

	lpi->als_enable = 0;
	lpi->als_enabled_before_suspend = 0;

	D("[LS][CM32181] %s: Probe success!\n", __func__);
	
	return ret;

err_sysfs_create_group_light:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->cm32181_class);
err_create_class:
err_cm32181_setup:
	destroy_workqueue(lpi->lp_wq);
	wake_lock_destroy(&(lpi->ls_wake_lock));

	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
err_create_singlethread_workqueue:
	mutex_destroy(&CM32181_control_mutex);
	misc_deregister(&lightsensor_misc);
err_lightsensor_setup:
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
err_platform_data_null:
	kfree(lpi);
	return ret;
}
   
static int control_and_report( struct cm32181_info *lpi, uint8_t mode, uint8_t cmd_enable ) {
	int ret = 0;
	uint16_t adc_value = 0;	
	int val;
	int fail_counter = 0;
	uint16_t status = 0;	
	uint16_t low_thd;
	uint32_t high_thd;
	uint32_t lux_level;
	
	mutex_lock(&CM32181_control_mutex);

	while(1)
	{
		val = gpio_get_value(lpi->intr_pin);  
		D("[CM32181] %s, interrupt GPIO val = %d, fail_counter %d\n",
			__func__, val, fail_counter);
      	
		val = gpio_get_value(lpi->intr_pin);
		if (val == 0)
		{
			ret = _cm32181_I2C_Read_Word(lpi->ALS_cmd_address, ALS_STATUS, &status);
			D("[CM32181] %s, interrupt GPIO val = %d, status register = 0x%x, ret %d\n",
				__func__, val, status, ret);	  
		}

		lpi->is_cmd &= ~CM32181_ALS_INT_EN;
		ret = _cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->is_cmd);
		if (ret == 0)
		{
			break;
		}
		else
		{	
			fail_counter++;
			val = gpio_get_value(lpi->intr_pin);
			D("[CM32181] %s, interrupt GPIO val = %d, , inital fail_counter %d\n",
				__func__, val, fail_counter);		
		}	
		if (fail_counter >= 10)
		{
			D("[CM32181] %s, clear INT fail_counter = %d\n", __func__, fail_counter);
			if (lpi->record_clear_int_fail == 0)
				lpi->record_clear_int_fail = 1;
			ret = -ENOMEM;
			goto error_clear_interrupt;
		}
	}

	if (mode == CONTROL_ALS)
	{
		if (cmd_enable)
		{
			lpi->is_cmd &= ~CM32181_ALS_SD;      
		}
		else
		{
			lpi->is_cmd |= CM32181_ALS_SD;
		}
		_cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->is_cmd);
		lpi->als_enable=cmd_enable;
	}
  
	if (lpi->als_enable)
	{
		if (mode == CONTROL_ALS)
		{
			set_lsensor_range(65535, 0);		
		
			input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
			input_sync(lpi->ls_input_dev);
		}
		else
		{
			get_ls_adc_value(&adc_value, 0);
			lux_level = (uint32_t)div64_u64((uint64_t)adc_value * lpi->als_resolution * lpi->cal_data, (uint64_t)100000 * 100000);		
	  
			D("[LS][CM32181] %s: raw adc = 0x%04X\n",
				__func__, adc_value);

			// set interrupt high/low threshold
			low_thd = (uint16_t)((uint32_t)adc_value * (100 - CHANGE_SENSITIVITY) / 100);
			high_thd = (uint32_t)adc_value * (100 + CHANGE_SENSITIVITY) / 100;
			if (high_thd > 65535)
			{
				high_thd = 65535;
			}
			ret = set_lsensor_range(low_thd, (uint16_t)high_thd);	  

			D("[CM32181] %s: ADC=0x%04X, Lux Level=%d, l_thd = 0x%x, h_thd = 0x%x \n",
					__func__, adc_value, lux_level, low_thd, high_thd);
			lpi->current_lux_level = lux_level;
			lpi->current_adc = adc_value;    
			input_report_abs(lpi->ls_input_dev, ABS_MISC, lux_level);
			input_sync(lpi->ls_input_dev);
		}
		
		lpi->is_cmd |= CM32181_ALS_INT_EN;		
	}
 
	ret = _cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->is_cmd);
	if (ret == 0)
	{
		D("[CM32181] %s, re-enable INT OK\n", __func__);
	}
	else
	{
		D("[CM32181] %s, re-enable INT FAIL\n", __func__);
	}

error_clear_interrupt:
	mutex_unlock(&CM32181_control_mutex);
	return ret;
}

#ifdef CONFIG_PM
static int cm32181_suspend(struct input_dev *dev)
{
	struct cm32181_info *lpi = lp_info;

	D("[LS][CM32181] %s\n", __func__);

	lpi->als_enabled_before_suspend = lpi->als_enable;
	if (lpi->als_enable)
		lightsensor_disable(lpi);

	return 0;
}

static int cm32181_resume(struct input_dev *dev)
{
	struct cm32181_info *lpi = lp_info;

	D("[LS][CM32181] %s\n", __func__);

	if (lpi->als_enabled_before_suspend)
		lightsensor_enable(lpi);

	return 0;
}
#else
#define cm32181_suspend NULL
#define cm32181_resume NULL
#endif /* CONFIG_PM */


static const struct i2c_device_id cm32181_i2c_id[] = {
	{CM32181_I2C_NAME, 0},
	{}
};

static struct i2c_driver cm32181_driver = {
	.id_table = cm32181_i2c_id,
	.probe = cm32181_probe,
	.suspend = cm32181_suspend,
	.resume = cm32181_resume,
	.driver = {
		.name = CM32181_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init cm32181_init(void)
{
	return i2c_add_driver(&cm32181_driver);
}

static void __exit cm32181_exit(void)
{
	i2c_del_driver(&cm32181_driver);
}

static struct cm32181_platform_data cm32181_pdata = {
        .intr = CM32181_INT_N,
        .power = NULL,
        .ALS_slave_address = CM32181_ADDR,
        .is_cmd = CM32181_ALS_SM_2 | CM32181_ALS_IT_200MS | CM32181_ALS_PERS_1,
};

static struct i2c_board_info i2c_CM32181_devices[] = {
	{
		I2C_BOARD_INFO(CM32181_I2C_NAME, CM32181_ADDR),
		.platform_data = &cm32181_pdata,
	},
};

static int __init cm32181_i2c_init(void)
{
	i2c_CM32181_devices[0].irq = gpio_to_irq(CM32181_INT_N);

    D("[LS][CM32181] %s register i2c_CM32181_devices\n", __func__);
	i2c_register_board_info(3, &i2c_CM32181_devices[0], ARRAY_SIZE(i2c_CM32181_devices));
	return 0;
}

fs_initcall(cm32181_i2c_init);

module_init(cm32181_init);
module_exit(cm32181_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM32181 Light Sensor Driver");
MODULE_AUTHOR("Capella Microsystems");
