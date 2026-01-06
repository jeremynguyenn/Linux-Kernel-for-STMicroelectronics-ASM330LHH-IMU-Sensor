/*
 * ASM330LHH IMU Driver
 * Author: Hardy Yu
 * Date: Mar 8, 2025
 * Description: An IIO-based driver for the ASM330LHH 6-DOF sensor using SPI.
 * Enhanced with FIFO, interrupt, scaling, and sampling frequency support.
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include "asm330lhh_reg.h"


/* ---------------------- Meta Information ---------------------- */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hardy Yu");
MODULE_DESCRIPTION("An IMU driver for ASM330LHH 6-DOF sensor");


/* ---------------------- Device Definitions ---------------------- */
#define SLAVE_DEVICE_NAME	"my_asm330"
#define ASM330_IIO_BUFF_SIZE 4096  // Example buffer size

/* Scaling tables similar to st_lsm6dsx */
static const struct asm330_fs_table {
    int val;
    int reg_val;
    int scale;  // in micro units (e.g., ug for accel, udps for gyro)
} asm330_acc_fs_table[] = {
    {  2, ASM330LHH_2g,  61000},  // ug/LSB
    {  4, ASM330LHH_4g, 122000},
    {  8, ASM330LHH_8g, 244000},
    { 16, ASM330LHH_16g,488000},
};

static const struct asm330_fs_table asm330_gyro_fs_table[] = {
    {  125, ASM330LHH_125dps,   4375000},  // udps/LSB
    {  250, ASM330LHH_250dps,   8750000},
    {  500, ASM330LHH_500dps,  17500000},
    { 1000, ASM330LHH_1000dps, 35000000},
    { 2000, ASM330LHH_2000dps, 70000000},
    { 4000, ASM330LHH_4000dps,140000000},
};

static const struct asm330_odr_table {
    int val;  // Hz * 1000 for milliHz
    int reg_val;
} asm330_acc_odr_table[] = {
    {    0, ASM330LHH_XL_ODR_OFF},
    {12500, ASM330LHH_XL_ODR_12Hz5},
    {26000, ASM330LHH_XL_ODR_26Hz},
    {52000, ASM330LHH_XL_ODR_52Hz},
    {104000, ASM330LHH_XL_ODR_104Hz},
    {208000, ASM330LHH_XL_ODR_208Hz},
    {417000, ASM330LHH_XL_ODR_417Hz},
    {833000, ASM330LHH_XL_ODR_833Hz},
    {1667000, ASM330LHH_XL_ODR_1667Hz},
    {3333000, ASM330LHH_XL_ODR_3333Hz},
    {6667000, ASM330LHH_XL_ODR_6667Hz},
};

static const struct asm330_odr_table asm330_gyro_odr_table[] = {
    {    0, ASM330LHH_GY_ODR_OFF},
    {12500, ASM330LHH_GY_ODR_12Hz5},
    {26000, ASM330LHH_GY_ODR_26Hz},
    {52000, ASM330LHH_GY_ODR_52Hz},
    {104000, ASM330LHH_GY_ODR_104Hz},
    {208000, ASM330LHH_GY_ODR_208Hz},
    {417000, ASM330LHH_GY_ODR_417Hz},
    {833000, ASM330LHH_GY_ODR_833Hz},
    {1667000, ASM330LHH_GY_ODR_1667Hz},
    {3333000, ASM330LHH_GY_ODR_3333Hz},
    {6667000, ASM330LHH_GY_ODR_6667Hz},
};

/* ---------------------- Data Structures ---------------------- */
struct asm330_iio_t {
    struct spi_device *asm330_spi;
    struct iio_trigger *trig;
    struct mutex mtx;
    stmdev_ctx_t asm330_ctx;
    asm330lhh_status_reg_t status_reg;
    uint32_t    timestamp;
    int16_t     accel[3];
    int16_t     gyro[3];
    int irq;
    uint8_t watermark;
    uint8_t fifo_enabled;
    int acc_fs_idx;
    int gyro_fs_idx;
    int acc_odr_idx;
    int gyro_odr_idx;
};

/* ---------------------- SPI Communication ---------------------- */
/**
 * @brief Write to ASM330 via SPI.
 * @param handle Pointer to SPI device.
 * @param reg Register address.
 * @param bufp Buffer containing data to write.
 * @param len Number of bytes to write.
 * @return 0 on success, -1 on failure.
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    struct spi_transfer xfer[2];
    struct spi_message msg;
    struct spi_device *spi = (struct spi_device *)handle;
    
    spi_message_init(&msg);

    // First transfer: Register address
    memset(&xfer[0], 0, sizeof(xfer[0]));
    xfer[0].tx_buf = &reg;
    xfer[0].len = 1;
    spi_message_add_tail(&xfer[0], &msg);

    // Second transfer: Data payload
    memset(&xfer[1], 0, sizeof(xfer[1]));
    xfer[1].tx_buf = bufp;
    xfer[1].len = len;
    spi_message_add_tail(&xfer[1], &msg);

    // Perform SPI transaction
    if (spi_sync(spi, &msg)) {
        dev_err(&spi->dev, "SPI write failed\n");
        return -1;
    }
    return 0;
}


/**
 * @brief Read from ASM330 via SPI.
 * @param handle Pointer to SPI device.
 * @param reg Register address.
 * @param bufp Buffer to store received data.
 * @param len Number of bytes to read.
 * @return 0 on success, -1 on failure.
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    struct spi_transfer xfer;
    struct spi_message msg;
    struct spi_device *spi = (struct spi_device *)handle;
    
    uint8_t txrx_buffer[len + 1]; // First byte for register address, rest for received data
    memset(txrx_buffer, 0, sizeof(txrx_buffer));

    txrx_buffer[0] = reg | 0x80; // Set read bit

    spi_message_init(&msg);

    memset(&xfer, 0, sizeof(xfer));
    xfer.tx_buf = txrx_buffer; // Send register address, receive response in same transaction
    xfer.rx_buf = txrx_buffer;
    xfer.len = len + 1; // One byte for register, rest for received data
    spi_message_add_tail(&xfer, &msg);

    if (spi_sync(spi, &msg)) {
        dev_err(&spi->dev, "SPI read failed\n");
        return -1;
    }

    // Copy received data into bufp (skip first byte which was the sent register)
    memcpy(bufp, &txrx_buffer[1], len);

    return 0;
}

/* ---------------------- IIO Read/Write Functions ---------------------- */
static int asm330_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
                           int *val, int *val2, long mask) {
    struct asm330_iio_t *data = iio_priv(indio_dev);
    int ret = 0;

    switch (mask) {
    case IIO_CHAN_INFO_RAW:
        mutex_lock(&data->mtx);
        asm330lhh_status_reg_get(&data->asm330_ctx, &data->status_reg);
        if (data->status_reg.xlda) {
            asm330lhh_acceleration_raw_get(&data->asm330_ctx, data->accel);
        }
        if (data->status_reg.gda) {
            asm330lhh_angular_rate_raw_get(&data->asm330_ctx, data->gyro);
        }
        mutex_unlock(&data->mtx);
        switch (chan->scan_index){
            case 0: *val = data->accel[0]; break;
            case 1: *val = data->accel[1]; break;
            case 2: *val = data->accel[2]; break;
            case 3: *val = data->gyro[0]; break;
            case 4: *val = data->gyro[1]; break;
            case 5: *val = data->gyro[2]; break;
            default: return -EINVAL;
        }
        return IIO_VAL_INT;

    case IIO_CHAN_INFO_SCALE:
        if (chan->type == IIO_ACCEL) {
            *val = 0;
            *val2 = asm330_acc_fs_table[data->acc_fs_idx].scale;
            ret = IIO_VAL_INT_PLUS_MICRO;
        } else if (chan->type == IIO_ANGL_VEL) {
            *val = 0;
            *val2 = asm330_gyro_fs_table[data->gyro_fs_idx].scale;
            ret = IIO_VAL_INT_PLUS_MICRO;
        } else {
            ret = -EINVAL;
        }
        return ret;

    case IIO_CHAN_INFO_SAMP_FREQ:
        if (chan->type == IIO_ACCEL) {
            *val = asm330_acc_odr_table[data->acc_odr_idx].val / 1000;
            *val2 = (asm330_acc_odr_table[data->acc_odr_idx].val % 1000) * 1000;
            ret = IIO_VAL_INT_PLUS_MICRO;
        } else if (chan->type == IIO_ANGL_VEL) {
            *val = asm330_gyro_odr_table[data->gyro_odr_idx].val / 1000;
            *val2 = (asm330_gyro_odr_table[data->gyro_odr_idx].val % 1000) * 1000;
            ret = IIO_VAL_INT_PLUS_MICRO;
        } else {
            ret = -EINVAL;
        }
        return ret;

    default:
        return -EINVAL;
    }
}

static int asm330_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
                            int val, int val2, long mask) {
    struct asm330_iio_t *data = iio_priv(indio_dev);
    int ret = 0, i;

    switch (mask) {
    case IIO_CHAN_INFO_SCALE:
        if (chan->type == IIO_ACCEL) {
            for (i = 0; i < ARRAY_SIZE(asm330_acc_fs_table); i++) {
                if (asm330_acc_fs_table[i].scale == val2) {
                    ret = asm330lhh_xl_full_scale_set(&data->asm330_ctx, asm330_acc_fs_table[i].reg_val);
                    if (ret == 0) data->acc_fs_idx = i;
                    break;
                }
            }
        } else if (chan->type == IIO_ANGL_VEL) {
            for (i = 0; i < ARRAY_SIZE(asm330_gyro_fs_table); i++) {
                if (asm330_gyro_fs_table[i].scale == val2) {
                    ret = asm330lhh_gy_full_scale_set(&data->asm330_ctx, asm330_gyro_fs_table[i].reg_val);
                    if (ret == 0) data->gyro_fs_idx = i;
                    break;
                }
            }
        } else {
            ret = -EINVAL;
        }
        return ret;

    case IIO_CHAN_INFO_SAMP_FREQ:
        val = val * 1000 + val2 / 1000;  // to milliHz
        if (chan->type == IIO_ACCEL) {
            for (i = 0; i < ARRAY_SIZE(asm330_acc_odr_table); i++) {
                if (asm330_acc_odr_table[i].val == val) {
                    ret = asm330lhh_xl_data_rate_set(&data->asm330_ctx, asm330_acc_odr_table[i].reg_val);
                    if (ret == 0) data->acc_odr_idx = i;
                    break;
                }
            }
        } else if (chan->type == IIO_ANGL_VEL) {
            for (i = 0; i < ARRAY_SIZE(asm330_gyro_odr_table); i++) {
                if (asm330_gyro_odr_table[i].val == val) {
                    ret = asm330lhh_gy_data_rate_set(&data->asm330_ctx, asm330_gyro_odr_table[i].reg_val);
                    if (ret == 0) data->gyro_odr_idx = i;
                    break;
                }
            }
        } else {
            ret = -EINVAL;
        }
        return ret;

    default:
        return -EINVAL;
    }
}

/* ---------------------- IRQ Handler ---------------------- */
static irqreturn_t asm330_handler(int irq, void *private)
{
    struct iio_dev *indio_dev = private;
    struct asm330_iio_t *data = iio_priv(indio_dev);
    int16_t buf[8];  // accel[3], gyro[3], timestamp[2]
    uint16_t num_samples, fifo_level;
    int i, ret;

    ret = asm330lhh_fifo_data_level_get(&data->asm330_ctx, &fifo_level);
    if (ret < 0)
        return IRQ_NONE;

    num_samples = fifo_level / 14;  // Assuming 14 bytes per sample (accel 6 + gyro 6 + ts 2)

    for (i = 0; i < num_samples; i++) {
        // Read FIFO data (simplified; adjust based on actual FIFO read)
        ret = asm330lhh_read_reg(&data->asm330_ctx, ASM330LHH_FIFO_DATA_OUT_L, (uint8_t *)buf, 14);
        if (ret < 0)
            break;

        iio_push_to_buffers_with_timestamp(indio_dev, buf, iio_get_time_ns(indio_dev));
    }

    return IRQ_HANDLED;
}

static irqreturn_t asm330_trigger_handler(int irq, void *p)
{
    return asm330_handler(irq, ((struct iio_poll_func *)p)->indio_dev);
}

/* ---------------------- Sensor Configuration ---------------------- */
static int asm330_configure_sensor(stmdev_ctx_t *asm330_ctx, struct spi_device *spi, struct asm330_iio_t *data) {
    dev_info(&spi->dev, "Probing device\n");
    
    asm330_ctx->write_reg = platform_write;
    asm330_ctx->read_reg = platform_read;
    asm330_ctx->handle = spi;
    uint8_t asm330_wai;
    asm330lhh_device_id_get(asm330_ctx, &asm330_wai);
    dev_info(&spi->dev, "Read ASM330 WHOAMI reg: 0x%x\n", asm330_wai);
    uint8_t rst;
    asm330lhh_reset_set(asm330_ctx, PROPERTY_ENABLE);
    asm330lhh_reset_get(asm330_ctx, &rst);
    dev_info(&spi->dev, "Read ASM330 RESET reg: 0x%x\n", rst);

    // Configure IMU
	asm330lhh_device_conf_set(asm330_ctx, PROPERTY_ENABLE);
	asm330lhh_block_data_update_set(asm330_ctx, PROPERTY_ENABLE);
	asm330lhh_xl_data_rate_set(asm330_ctx, ASM330LHH_XL_ODR_417Hz);
	asm330lhh_gy_data_rate_set(asm330_ctx, ASM330LHH_GY_ODR_417Hz);
	asm330lhh_xl_full_scale_set(asm330_ctx, ASM330LHH_2g);
	asm330lhh_gy_full_scale_set(asm330_ctx, ASM330LHH_2000dps);
	asm330lhh_timestamp_set(asm330_ctx, PROPERTY_ENABLE);

    /* Configure filtering */
	asm330lhh_xl_hp_path_on_out_set(asm330_ctx, ASM330LHH_LP_ODR_DIV_100);
	asm330lhh_xl_filter_lp2_set(asm330_ctx, PROPERTY_ENABLE);

    /* FIFO configuration */
    asm330lhh_fifo_mode_set(asm330_ctx, ASM330LHH_STREAM_MODE);
    asm330lhh_fifo_xl_batch_set(asm330_ctx, ASM330LHH_XL_BATCHED_AT_417Hz);
    asm330lhh_fifo_gy_batch_set(asm330_ctx, ASM330LHH_GY_BATCHED_AT_417Hz);
    asm330lhh_fifo_watermark_set(asm330_ctx, data->watermark = 128);  // Example watermark
    asm330lhh_fifo_stop_on_wtm_set(asm330_ctx, PROPERTY_ENABLE);

    /* Interrupt configuration */
    asm330lhh_int_notification_set(asm330_ctx, ASM330LHH_ALL_INT_PULSED);
    asm330lhh_pin_int1_route_set(asm330_ctx, (asm330lhh_emb_fsm_int1_a_t){ .int1_fifo_th = 1 });

    data->fifo_enabled = 1;
    data->acc_fs_idx = 0;  // Default 2g
    data->gyro_fs_idx = 4; // Default 2000dps
    data->acc_odr_idx = 6; // 417Hz
    data->gyro_odr_idx = 6;

    return 0;
}

/* ---------------------- IIO Channel Specification ---------------------- */
static const struct iio_chan_spec asm_channels[] = {
	{
		.type = IIO_ACCEL,
		.modified = 1,
        .channel2 = IIO_MOD_X,
        .scan_index = 0,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_LE,
        },
	}, 
    {
        .type = IIO_ACCEL,
		.modified = 1,
        .channel2 = IIO_MOD_Y,
        .scan_index = 1,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_LE,
        },
    },
    {
        .type = IIO_ACCEL,
		.modified = 1,
        .channel2 = IIO_MOD_Z,
        .scan_index = 2,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_LE,
        },
    },
    {
        .type = IIO_ANGL_VEL,
		.modified = 1,
        .channel2 = IIO_MOD_X,
        .scan_index = 3,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_LE,
        },
    },
    {
        .type = IIO_ANGL_VEL,
		.modified = 1,
        .channel2 = IIO_MOD_Y,
        .scan_index = 4,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_LE,
        },
    },
    {
        .type = IIO_ANGL_VEL,
		.modified = 1,
        .channel2 = IIO_MOD_Z,
        .scan_index = 5,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_LE,
        },
    },
    IIO_CHAN_SOFT_TIMESTAMP(6),
};

static const struct iio_info asm330_iio_info = {
	.read_raw = asm330_read_raw,
    .write_raw = asm330_write_raw,
};

/* ---------------------- Trigger Ops ---------------------- */
static const struct iio_trigger_ops asm330_trigger_ops = {
    // Add if needed
};

/* ---------------------- SPI Probe Function ---------------------- */
static int asm330_probe(struct spi_device *spi) {
    /* Setting up IIO dev */
    struct iio_dev *indio_dev;
    struct asm330_iio_t *data;
    int ret;

	dev_info(&spi->dev, "Now I am in the Probe function!\n");

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(struct asm330_iio_t));
	if(!indio_dev) {
		dev_err(&spi->dev, "Error! Out of memory\n");
		return -ENOMEM;
	}

    data = iio_priv(indio_dev);

	indio_dev->name = "asm330_iio";
	indio_dev->info = &asm330_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	indio_dev->channels = asm_channels;
	indio_dev->num_channels = ARRAY_SIZE(asm_channels);

    mutex_init(&data->mtx);

	spi_set_drvdata(spi, indio_dev);

    /* Configure sensor */
    ret = asm330_configure_sensor(&data->asm330_ctx, spi, data);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to configure sensor\n");
        return ret;
    }

    /* Setup trigger */
    data->trig = devm_iio_trigger_alloc(&spi->dev, "%s-dev%d",
                                        indio_dev->name, iio_device_id(indio_dev));
    if (!data->trig)
        return -ENOMEM;

    data->trig->ops = &asm330_trigger_ops;
    iio_trigger_set_drvdata(data->trig, indio_dev);
    ret = iio_trigger_register(data->trig);
    if (ret)
        return ret;

    indio_dev->trig = data->trig;

    /* Setup buffer */
    ret = devm_iio_kfifo_buffer_setup(&spi->dev, indio_dev,
                                      &iio_pollfunc_store_time,
                                      asm330_trigger_handler);
    if (ret)
        return ret;

    /* Get IRQ from DT */
    data->irq = spi->irq;
    if (data->irq > 0) {
        ret = devm_request_threaded_irq(&spi->dev, data->irq, NULL,
                                        asm330_handler,
                                        IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
                                        "asm330", indio_dev);
        if (ret)
            return ret;
    }

    /* Runtime PM */
    pm_runtime_enable(&spi->dev);

    ret = iio_device_register(indio_dev);
    if (ret < 0 ){
        dev_err(&spi->dev, "Unable to register iio device for asm330\n");
        return ret;
    }

    return 0; // Return 0 on success
}

static void asm330_remove(struct spi_device *spi) {
    struct iio_dev * indio_dev = spi_get_drvdata(spi);
    struct asm330_iio_t *data = iio_priv(indio_dev);

    pm_runtime_disable(&spi->dev);
    iio_trigger_unregister(data->trig);
    iio_device_unregister(indio_dev);
}

/* Runtime PM ops */
static int asm330_runtime_suspend(struct device *dev)
{
    struct iio_dev *indio_dev = spi_get_drvdata(to_spi_device(dev));
    struct asm330_iio_t *data = iio_priv(indio_dev);

    asm330lhh_xl_data_rate_set(&data->asm330_ctx, ASM330LHH_XL_ODR_OFF);
    asm330lhh_gy_data_rate_set(&data->asm330_ctx, ASM330LHH_GY_ODR_OFF);

    return 0;
}

static int asm330_runtime_resume(struct device *dev)
{
    struct iio_dev *indio_dev = spi_get_drvdata(to_spi_device(dev));
    struct asm330_iio_t *data = iio_priv(indio_dev);

    asm330lhh_xl_data_rate_set(&data->asm330_ctx, asm330_acc_odr_table[data->acc_odr_idx].reg_val);
    asm330lhh_gy_data_rate_set(&data->asm330_ctx, asm330_gyro_odr_table[data->gyro_odr_idx].reg_val);

    return 0;
}

static const struct dev_pm_ops asm330_pm_ops = {
    SET_RUNTIME_PM_OPS(asm330_runtime_suspend, asm330_runtime_resume, NULL)
};

/* ---------------------- SPI Configurations ---------------------- */
static const struct spi_device_id spi_asm330_ids[] = {
    { SLAVE_DEVICE_NAME, 0},
    {},
};

MODULE_DEVICE_TABLE(spi, spi_asm330_ids);

static struct of_device_id of_asm_ids[] = {
	{
		.compatible = "my_asm330",
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_asm_ids);

static struct spi_driver asm330_driver = {
	.driver = {
        .name = SLAVE_DEVICE_NAME,
        .of_match_table = of_asm_ids,
        .pm = &asm330_pm_ops,
    },
    .probe = asm330_probe,
    .remove = asm330_remove,
    .id_table = spi_asm330_ids,
};

module_spi_driver(asm330_driver)