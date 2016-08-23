#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>

#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>

#include "CwMcuSensor.h"
#include "CwMcuSensor_input.h"

// what?
#define QueueSystemInfoMsgSize		30
#define QueueWarningMsgSize		30

struct CWMCU_T {

	/* Input device */
	struct input_dev *input_acc;
        struct input_dev *input_gyro;
        struct input_dev *input_mag;
	struct input_dev *input_fusion;

	struct platform_device *virmouse_dev;

	/* SPI device register */
	struct spi_device *spi;

	/* N5 power init */
	struct regulator *vdd;
	struct regulator *vcc_spi;

	/* work queue */
	struct workqueue_struct *driver_wq;
	struct work_struct work;
	struct delayed_work	delay_work;

	/* MCU information */
	struct CWMCU_SENSORS_INFO sensors_info[HANDLE_ID_END][SENSORS_ID_END];
	SensorsInit_T	hw_info[DRIVER_ID_END];
	RegInformation *pReadRegInfo;
	RegInformation *pWriteRegInfo;
	u8 m_cReadRegCount;
	u8 m_cWriteRegCount;
	uint8_t initial_hw_config;

	/* MCU mode & kernel status(suspend/resume) */
	int mcu_mode;
	uint8_t kernel_status;

	/* enable & batch list */
	uint32_t enabled_list[HANDLE_ID_END];
	uint32_t interrupt_status;
    uint8_t calibratordata[DRIVER_ID_END][30];
    uint8_t calibratorUpdate[DRIVER_ID_END];

	/* power status */
	volatile	uint32_t power_on_list;

	/* Calibrator status */
	int cal_cmd;
	int cal_type;
	int cal_id;
	
	/* gpio define */
	int irq_gpio;
	int wakeup_gpio;
	int cs_gpio;
	int boot_gpio;
	int reset_gpio;

	/* debug flag */
	uint32_t debug_log;

	/* data of Node command */
	int cmd;
	uint32_t addr;
	int len;
	int firmware_update_status;
	int cw_i2c_rw;	/* r = 0 , w = 1 */
	int cw_i2c_len;
	uint8_t cw_i2c_data[300];

	/* IIO */
	s32 iio_data[6];
	struct iio_dev *indio_dev;
	//struct irq_work iio_irq_work;
	struct iio_trigger  *trig;
	atomic_t pseudo_irq_enable;

	/* device of sysfs class  */
	struct class *sensor_class;
	struct device *sensor_dev;

	/* work delay */
	atomic_t delay;

	/* int supend_flag; */

    int wq_polling_time;
	
	/* mutex lock */
#ifdef CWMCU_MUTEX
	struct mutex mutex_lock;
	struct mutex mutex_lock_spi;
    struct mutex mutex_wakeup_gpio;
#endif

	/* MCU queue size */
	unsigned char loge_buff[QueueSystemInfoMsgSize*2];
	unsigned char loge_buff_count;

	unsigned char logw_buff[QueueWarningMsgSize*2];
	unsigned char logw_buff_count;

	/* MCU information show for debug */
	int mcu_status;
	int mcu_init_count;

	/* voice recognition info */
	uint8_t voice_index;
	uint8_t voice_mode;
	uint8_t voice_command;
	uint32_t voice_package;
	
	/* voice trigger to search */
	uint16_t voice_data_length;
	uint8_t voice_stop_streaming;

};

static int CWMCU_SPI_READ(struct spi_device *spidevice, u8 *data, u8 len)
{
	struct spi_message m;
	struct spi_transfer t;
	int rc = 0;

	spi_message_init(&m);

	memset(&t, 0, sizeof(t));

	t.tx_buf = 0;
	t.rx_buf = data;
	t.len = len;
	
	//if bits_per_word and speed_hz are 0, then default is used from spi device
	//t.bits_per_word = 8;
	//t.speed_hz = (&mcu)->spi->max_speed_hz;

	spi_message_add_tail(&t, &m);

	rc = spi_sync(spidevice, &m);
	if(rc < 0) printk("spi sync error rc : %d\n", rc);

	return 0;
}

static int cwstm_parse_dt(struct device *dev, struct CWMCU_T *sensor)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	ret = of_get_named_gpio(np, "cwstm,irq-gpio", 0);
	if(ret < 0)
	{
		printk("failed to get \"cwstm,irq-gpio\"\n");
		goto err;
	}
	sensor->irq_gpio = ret;

err:
	return ret;
	
}

static int CWMCU_SPI_WRITE(struct spi_device *spidevice, u8 *data, u8 len)
{
	struct spi_message m;
	struct spi_transfer t;
	int rc = 0;

	spi_message_init(&m);

	memset(&t, 0, sizeof(t));

	t.rx_buf = 0;	
	t.tx_buf = data;
	t.len = len;

	//if bits_per_word and speed_hz are 0, then default is used from spi device
        /*t.bits_per_word = 8;
        t.speed_hz = sensor->spi->max_speed_hz;*/

	spi_message_add_tail(&t, &m);

	rc = spi_sync(spidevice, &m);
	if(rc < 0) printk("spi sync error rc : %d\n", rc);

	return 0;
}

static ssize_t active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("active_show do nothing\n");
	return 0;
}

static ssize_t active_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	
	printk("do nothing");

	return count;
	
}

static struct device_attribute attributes[] = {
        __ATTR(enable, 0660,  active_show, active_set),
};

static int create_sysfs_interfaces(struct CWMCU_T *mcu_data)
{
	int i;
	int res=0;

	mcu_data->sensor_class = class_create(THIS_MODULE, "cywee_sensorhub");
	if (IS_ERR(mcu_data->sensor_class)) printk("sensor_class error\n");

	mcu_data->sensor_dev = device_create(mcu_data->sensor_class, NULL, 0, "%s", "sensor_hub");
	if (IS_ERR(mcu_data->sensor_dev)) {
		res = PTR_ERR(mcu_data->sensor_dev);
		printk("sensor_dev error\n");
	}
	 //res = dev_set_drvdata(mcu_data->sensor_dev, mcu_data);
	//if (res) printk("dev_set_drvdata error\n");

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		 if (device_create_file(mcu_data->sensor_dev, attributes + i))
			printk("device_create_file error\n");

	return 0;
}

static void report_acc_values(u8 *read_buf, struct CWMCU_T *mcu)
{
	int reset_value = 0xFFFF0000;
	int temp = 0, temp_x=0, temp_y=0, temp_z=0;

	temp = (read_buf[8] << 8) | (read_buf[7]);
	temp = (temp << 8) | ( read_buf[6]);
	temp = (temp << 8) | ( read_buf[5]);
	temp_x = temp;
	temp = 0;

	temp = (read_buf[12] << 8) | (read_buf[11]);
	temp = (temp << 8) | (read_buf[10]);
        temp = (temp << 8) | (read_buf[9]);
	temp_y = temp;
	temp = 0;

	temp = (read_buf[16] << 8) | (read_buf[15]);
        temp = (temp << 8) | (read_buf[14]);
        temp = (temp << 8) | (read_buf[13]);
        temp_z= temp;
        temp = 0;


	input_report_abs(mcu->input_acc, ABS_X, temp_x);
	input_report_abs(mcu->input_acc, ABS_Y, temp_y);
	input_report_abs(mcu->input_acc, ABS_Z, temp_z);
	input_sync(mcu->input_acc);

	input_report_abs(mcu->input_acc, ABS_X, reset_value);
	input_report_abs(mcu->input_acc, ABS_Y, reset_value);
	input_report_abs(mcu->input_acc, ABS_Z, reset_value);
	input_sync(mcu->input_acc);

}

static void report_gyro_values(u8 *read_buf, struct CWMCU_T *mcu)
{
	int reset_value = 0xFFFF0000;
        int temp = 0, temp_x=0, temp_y=0, temp_z=0;

	temp = (read_buf[20] << 8) | (read_buf[19]);
        temp = (temp << 8) | ( read_buf[18]);
        temp = (temp << 8) | ( read_buf[17]);
        temp_x = temp;
        temp = 0;

        temp = (read_buf[24] << 8) | (read_buf[23]);
        temp = (temp << 8) | (read_buf[22]);
        temp = (temp << 8) | (read_buf[21]);
        temp_y = temp;
        temp = 0;

        temp = (read_buf[28] << 8) | (read_buf[27]);
        temp = (temp << 8) | (read_buf[26]);
        temp = (temp << 8) | (read_buf[25]);
        temp_z = temp;
        temp = 0;

	input_report_abs(mcu->input_gyro, ABS_X, temp_x);
        input_report_abs(mcu->input_gyro, ABS_Y, temp_y);
        input_report_abs(mcu->input_gyro, ABS_Z, temp_x);
        input_sync(mcu->input_gyro);

        input_report_abs(mcu->input_gyro, ABS_X, reset_value);
        input_report_abs(mcu->input_gyro, ABS_Y, reset_value);
        input_report_abs(mcu->input_gyro, ABS_Z, reset_value);
        input_sync(mcu->input_gyro);

}

static void report_fusion_values(u8 *read_buf, struct CWMCU_T *mcu)
{

	int reset_value = 0xFFFF0000;
	int temp = 0, temp_x = 0, temp_y = 0, temp_z = 0;

	temp = (read_buf[32] << 8) | (read_buf[31]);
	temp = (temp << 8) | ( read_buf[30]);
	temp = (temp << 8) | ( read_buf[29]);
	temp_x = temp;
	temp = 0;

	temp = (read_buf[36] << 8) | (read_buf[35]);
	temp = (temp << 8) | (read_buf[34]);
	temp = (temp << 8) | (read_buf[33]);
	temp_y = temp;
	temp = 0;

	temp = (read_buf[40] << 8) | (read_buf[39]);
	temp = (temp << 8) | (read_buf[38]);
	temp = (temp << 8) | (read_buf[37]);
	temp_z = temp;
	temp = 0;

	input_report_abs(mcu->input_fusion, ABS_X, temp_x);
	input_report_abs(mcu->input_fusion, ABS_Y, temp_y);
	input_report_abs(mcu->input_fusion, ABS_Z, temp_z);
	input_sync(mcu->input_fusion);

	input_report_abs(mcu->input_fusion, ABS_X, reset_value);
        input_report_abs(mcu->input_fusion, ABS_Y, reset_value);
        input_report_abs(mcu->input_fusion, ABS_Z, reset_value);
        input_sync(mcu->input_fusion);

}



static irqreturn_t CWMCU_interrupt_thread(int irq, void *data)
{
	struct CWMCU_T *sensor = data;
	u8 tx_addr[1]={0};
	u8 read_buf[64]={0};
	u8 checksum = 0;
	int i = 0;
	tx_addr[0] = 0x03;

	CWMCU_SPI_WRITE(sensor->spi, tx_addr, 1);
	udelay(100);
	CWMCU_SPI_READ(sensor->spi, read_buf, 64);

	// calculate CRC checksum
	for(i=0; i<63 ; i++)
		checksum = checksum + read_buf[i];

	if( (checksum == read_buf[63]) && (read_buf[0] == 'C') )
	{
		report_acc_values(read_buf, sensor);
		report_gyro_values(read_buf, sensor);
		report_fusion_values(read_buf, sensor);
	}


	return IRQ_HANDLED;

}

static int cywee_acc_input_init(struct CWMCU_T *sensor)
{
	int err = 0;

	sensor->input_acc = input_allocate_device();
        if (!sensor->input_acc) {
                printk(KERN_ERR "Failed to allocate acc input device\n");
                err = -ENOMEM;
                goto failed;
        }

	sensor->input_acc->name = ACC_NAME;
	sensor->input_acc->dev.parent = &sensor->spi->dev;

	set_bit(EV_ABS, sensor->input_acc->evbit);
	set_bit(ABS_X, sensor->input_acc->absbit);
        set_bit(ABS_Y, sensor->input_acc->absbit);
	set_bit(ABS_Z, sensor->input_acc->absbit);

	input_set_abs_params(sensor->input_acc, ABS_X, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_Y, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_Z, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_RX, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_RY, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_RZ, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_THROTTLE, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_RUDDER, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_WHEEL, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_HAT0X, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_HAT1X, -ACC_MAX, ACC_MAX, 0, 0);
        input_set_abs_params(sensor->input_acc, ABS_HAT2X, -ACC_MAX, ACC_MAX, 0, 0);

	err = input_register_device(sensor->input_acc);
        if (err) {
                pr_err("Failed to register acc input device\n");
                input_free_device(sensor->input_acc);
                sensor->input_acc = NULL;
                goto failed;
        }

        return 0;

failed:
	return err;
}

static int cywee_gyro_input_init(struct CWMCU_T *sensor)
{
	int err = 0;

        sensor->input_gyro = input_allocate_device();
        if (!sensor->input_gyro) {
                printk(KERN_ERR "Failed to allocate gyro input device\n");
                err = -ENOMEM;
                goto failed;
        }

        sensor->input_gyro->name = GYRO_NAME;
	sensor->input_gyro->dev.parent = &sensor->spi->dev;

        set_bit(EV_ABS, sensor->input_gyro->evbit);
        set_bit(ABS_X, sensor->input_gyro->absbit);
        set_bit(ABS_Y, sensor->input_gyro->absbit);
        set_bit(ABS_Z, sensor->input_gyro->absbit);

        input_set_abs_params(sensor->input_gyro, ABS_X, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_Y, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_Z, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_RX, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_RY, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_RZ, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_THROTTLE, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_RUDDER, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_WHEEL, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_HAT0X, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_HAT1X, GYRO_MIN, GYRO_MAX, 0, 0);
        input_set_abs_params(sensor->input_gyro, ABS_HAT2X, GYRO_MIN, GYRO_MAX, 0, 0);


        err = input_register_device(sensor->input_gyro);
        if (err) {
                pr_err("Failed to register gyro input device\n");
                input_free_device(sensor->input_gyro);
                sensor->input_gyro = NULL;
                goto failed;
        }

        return 0;

failed:
        return err;

}

static int cywee_fusion_input_init(struct CWMCU_T *sensor)
{
	int err = 0;

	sensor->input_fusion = input_allocate_device();
        if (!sensor->input_fusion) {
                printk(KERN_ERR "Failed to allocate fusion2 input device\n");
                err = -ENOMEM;
                goto failed;
        }

	sensor->input_fusion->name = FUSION_RC1_NAME;
	sensor->input_fusion->dev.parent = &sensor->spi->dev;

	set_bit(EV_ABS, sensor->input_fusion->evbit);
	set_bit(ABS_X, sensor->input_fusion->absbit);
        set_bit(ABS_Y, sensor->input_fusion->absbit);
        set_bit(ABS_Z, sensor->input_fusion->absbit);

	input_set_abs_params(sensor->input_fusion, ABS_X, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_Y, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_Z, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_RX, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_RY, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_RZ, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_THROTTLE, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_RUDDER, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_WHEEL, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_HAT0X, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_HAT1X, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_HAT2X, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_HAT0Y, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_HAT1Y, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_HAT2Y, -4096, 4096, 0, 0);

        input_set_abs_params(sensor->input_fusion, ABS_HAT3X, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_HAT3Y, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_GAS, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_BRAKE, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_PRESSURE, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_DISTANCE, -4096, 4096, 0, 0);

        input_set_abs_params(sensor->input_fusion, ABS_TILT_X, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_TILT_Y, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_TOOL_WIDTH, -4096, 4096, 0, 0);

        input_set_abs_params(sensor->input_fusion, ABS_VOLUME, -4096, 4096, 0, 0);
        input_set_abs_params(sensor->input_fusion, ABS_MISC, -4096, 4096, 0, 0);



        err = input_register_device(sensor->input_fusion);
        if (err) {
                pr_err("Failed to register fusion input device\n");
                input_free_device(sensor->input_fusion);
                sensor->input_fusion = NULL;
                goto failed;
        }

        return 0;

failed:
        return err;

}

static int /*__devinit*/ CWMCU_spi_probe(struct spi_device *spi)
{
	struct CWMCU_T *mcu;
	uint32_t max_speed;
	int cpha, cpol, cs_high = 0;
	int cs = 0, error = 0, spi_lsb_first = 0;

	spi->bits_per_word = 8;

	mcu = kzalloc(sizeof(struct CWMCU_T), GFP_KERNEL);

	mcu->spi = spi;

	cs = spi->chip_select;

	//judge spidev work mode
	cpha = (spi->mode & SPI_CPHA) ? 1:0;
	cpol = (spi->mode & SPI_CPOL) ? 1:0;
	
	//judge spidev's cs is low or high active  low:0 high:1
	cs_high = (spi->mode & SPI_CS_HIGH) ? 1:0;
	spi_lsb_first = (spi->mode & SPI_LSB_FIRST ) ? 1:0;
	max_speed = spi->max_speed_hz;

	printk("-CWMCU- SPI info: cs [%d] CPHA [%d] CPOL [%d] CS_HIGH [%d], max_speed [%d], spi_lsb_first[%d]\n"
				, cs, cpha, cpol, cs_high, max_speed, spi_lsb_first);


	error = create_sysfs_interfaces(mcu);
	if (error) printk("create sysfs error\n");

	// create input device node "/dev/input/event*"
	cywee_acc_input_init(mcu);
	cywee_gyro_input_init(mcu);
	cywee_fusion_input_init(mcu);

	error = cwstm_parse_dt(&spi->dev,mcu);
	if(error < 0)
	{
		printk("failed to parse device tree\n");
	}

	gpio_request(mcu->irq_gpio, "cwstm,irq-gpio");

	mcu->spi->irq = gpio_to_irq(mcu->irq_gpio);

	if (mcu->spi->irq > 0)
	{
		error = request_threaded_irq(mcu->spi->irq, NULL, CWMCU_interrupt_thread, IRQF_TRIGGER_RISING | IRQF_ONESHOT, "cwmcu", mcu);
		if (error < 0) printk("request irq %d failed\n", mcu->spi->irq);
	}


	return 0;

//err_parse_dt:

}

static int CWMCU_spi_remove(struct spi_device *spi)
{
	spi = NULL;
	return 0;
}

static struct of_device_id cwstm_match_table[] = {
    { .compatible = "cwstm",},
	{ },
};

/*static const struct dev_pm_ops CWMCU_pm_ops = {
	.suspend = CWMCU_suspend,
	.resume = CWMCU_resume
};*/

static const struct spi_device_id CWMCU_id[] = {
	{ CWMCU_SPI_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(spi, CWMCU_id);

static struct spi_driver CWMCU_spi_driver = {
	.driver = {
		.name = CWMCU_SPI_NAME,
		.owner = THIS_MODULE,
		//.pm = &CWMCU_pm_ops,
		.of_match_table = cwstm_match_table,
	},
	.probe    = CWMCU_spi_probe,
	.remove   = /*__devexit_p*/CWMCU_spi_remove,
	.id_table = CWMCU_id,
};

static int __init CWMCU_init(void){
	printk("%s:%s:(init)\n",LOG_TAG_KERNEL ,__FUNCTION__);
	return spi_register_driver(&CWMCU_spi_driver);
}

static void __exit CWMCU_exit(void){
	spi_unregister_driver(&CWMCU_spi_driver);
}

module_init(CWMCU_init);
module_exit(CWMCU_exit);

MODULE_DESCRIPTION("CWMCU SPI Bus Driver");
MODULE_AUTHOR("CyWee Group Ltd.");
MODULE_LICENSE("GPL");
