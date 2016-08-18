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

// what?
#define QueueSystemInfoMsgSize		30
#define QueueWarningMsgSize		30

struct CWMCU_T {
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

} mcu;

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
	u8 data[64]={0};
        u8 rx_addr[1]={0};
	int i = 0;

	rx_addr[0] = 0x01;
	
	CWMCU_SPI_WRITE((&mcu)->spi,rx_addr, 1);
	udelay(100);
	CWMCU_SPI_READ((&mcu)->spi,data, 64);

	for(i=0;i<64;i++)
		printk("%x ",data[i]);
	printk("\n");

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

static irqreturn_t CWMCU_interrupt_thread(int irq, void *data)
{
	u8 tx_addr[1]={0};
	u8 read_buf[64]={0};

	tx_addr[0] = 0x01;

	CWMCU_SPI_WRITE((&mcu)->spi, tx_addr, 1);
	udelay(100);
	CWMCU_SPI_READ((&mcu)->spi, read_buf, 64);

	return IRQ_HANDLED;

}

static int /*__devinit*/ CWMCU_spi_probe(struct spi_device *spi)
{
	//struct CWMCU_T mcu;	
	uint32_t max_speed;
	int cpha, cpol, cs_high = 0;
	int cs = 0, error = 0, spi_lsb_first = 0;

	(&mcu)->spi = spi;

	spi->bits_per_word = 8;
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


	 error = create_sysfs_interfaces(&mcu);
	 if (error) printk("create sysfs error\n");

	error = cwstm_parse_dt(&spi->dev,&mcu);
	if(error < 0)
	{
		printk("failed to parse device tree\n");
		//goto err_parse_dt;
	}

	gpio_request((&mcu)->irq_gpio, "cwstm,irq-gpio");

	(&mcu)->spi->irq = gpio_to_irq((&mcu)->irq_gpio);

	if ((&mcu)->spi->irq > 0)
	{
		error = request_threaded_irq((&mcu)->spi->irq, NULL, CWMCU_interrupt_thread, IRQF_TRIGGER_RISING | IRQF_ONESHOT, "cwmcu", &mcu);
		if (error < 0) printk("request irq %d failed\n", (&mcu)->spi->irq);
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
