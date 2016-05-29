/*
 * PCIe driver for TBS cards based on ECP3 FPGA
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#include "tbspcie.h"

#include "tas2101.h"
#include "av201x.h"

#include "mxl5xx.h"

#include "si2168.h"
#include "si2157.h"

#include "stv0910.h"
#include "stv6120.h"

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static bool enable_msi = true;
module_param(enable_msi, bool, 0444);
MODULE_PARM_DESC(enable_msi,
		"enable the use of an msi interrupt if available");

static void tbs_pcie_gpio_write(struct tbs_dev *dev, int gpio, int pin, int set)
{
	uint32_t value;

	value = tbs_read(TBS_GPIO_BASE, TBS_GPIO_DATA(gpio));
	if (set)
		value |= 1 << pin;
	else
		value &= ~(1 << pin);
	tbs_write(TBS_GPIO_BASE, TBS_GPIO_DATA(gpio), value);
}

#define TBS_PCIE_PAGE_SIZE	4194304
//#define TBS_PCIE_DMA_TOTAL	770048
//#define TBS_PCIE_CELL_SIZE	96256
#define TBS_PCIE_DMA_TOTAL	385024
#define TBS_PCIE_CELL_SIZE	48128


static void tbsecp3_init_hw(struct tbs_dev *dev)
{
	int i;
	u32 dma_addr_offset;
	u32 dma_base;

	/* i2c */
	for (i = 0; i < dev->info->i2c_masters; i++) {
		tbs_write(dev->i2c_bus[i].base, TBS_I2C_SPEED, 9);
		tbs_read(dev->i2c_bus[i].base, TBS_I2C_CTRL);
		tbs_write(TBS_INT_BASE, TBS_INT_I2C_MASK(dev->i2c_bus[i].nr), 1);
	}

	/* dma */
	for (i = 0; i < dev->info->adapters; i++) {
		dma_addr_offset = dev->mem_addr_phys + (TBS_PCIE_DMA_TOTAL + 256) * i;

		if (i < 4)
			dma_base = TBS_DMA_BASE(i);
		else
			dma_base = TBS_DMA2_BASE(i);

		tbs_write(dma_base, TBS_DMA_START, 0);
		tbs_write(dma_base, TBS_DMA_ADDR_HIGH, 0);
		tbs_write(dma_base, TBS_DMA_ADDR_LOW, dma_addr_offset);
		tbs_write(dma_base, TBS_DMA_SIZE, TBS_PCIE_DMA_TOTAL);
		tbs_write(dma_base, TBS_DMA_CELL_SIZE, TBS_PCIE_CELL_SIZE);
	}

	/* enable interrupts */
	tbs_write(TBS_INT_BASE, TBS_INT_ENABLE, 1);
}


static int tbs_i2c_init(struct tbs_dev *dev)
{
	int i, ret = 0;

	/* I2C Defaults / setup */
	for (i = 0; i < dev->info->i2c_masters; i++) {
		dev->i2c_bus[i].nr = dev->info->i2c_config[i].nr;
		dev->i2c_bus[i].base = TBS_I2C_BASE(dev->i2c_bus[i].nr);
		dev->i2c_bus[i].dev = dev;
		ret = tbs_i2c_register(&dev->i2c_bus[i]);
		if (ret)
			break;
	}

	if (ret) {
		do {
			tbs_i2c_unregister(&dev->i2c_bus[i]);
		} while (i-- > 0);
	}

	return ret;
}

static void tbs_i2c_exit(struct tbs_dev *dev)
{
	int i;
	for (i = 0; i < dev->info->i2c_masters; i++)
		tbs_i2c_unregister(&dev->i2c_bus[i]);
}


static int tbs_pcie_dma_init(struct tbs_dev *dev)
{
	dma_addr_t dma_addr;

	dev->mem_addr_virt = pci_alloc_consistent(dev->pci_dev,
					TBS_PCIE_PAGE_SIZE, &dma_addr);
	if (!dev->mem_addr_virt) {
		dev_err(&dev->pci_dev->dev, "dma memory alloc failed\n");
		return -ENOMEM;
	}

	memset(dev->mem_addr_virt, 0, TBS_PCIE_PAGE_SIZE);
	dev->mem_addr_phys = dma_addr;
	return 0;
}

static void tbs_pcie_dma_free(struct tbs_dev *dev)
{
	if (dev->mem_addr_virt != NULL) {
		pci_free_consistent(dev->pci_dev, TBS_PCIE_PAGE_SIZE,
				dev->mem_addr_virt, dev->mem_addr_phys);
		dev->mem_addr_virt = NULL;
	}
}

static void tbs_pcie_dma_start(struct tbs_adapter *adapter)
{
	struct tbs_dev *dev = adapter->dev;

	spin_lock_irq(&adapter->adap_lock);

	adapter->buffer = 0;
	adapter->sync_offset = 0;

	if (adapter->tsin < 4) {
		tbs_read(TBS_DMA_BASE(adapter->tsin), TBS_DMA_STATUS);

		tbs_write(TBS_INT_BASE, TBS_DMA_MASK(adapter->tsin), 1); 
		tbs_write(TBS_DMA_BASE(adapter->tsin), TBS_DMA_START, 1);
	} else {
		tbs_read(TBS_DMA2_BASE(adapter->tsin), TBS_DMA_STATUS);

		tbs_write(TBS_INT_BASE, TBS_DMA2_MASK(adapter->tsin), 1);
		tbs_write(TBS_DMA2_BASE(adapter->tsin), TBS_DMA_START, 1);
	}


	adapter->active = 1;

	spin_unlock_irq(&adapter->adap_lock);
}

static void tbs_pcie_dma_stop(struct tbs_adapter *adapter)
{
	struct tbs_dev *dev = adapter->dev;

	spin_lock_irq(&adapter->adap_lock);
	if (adapter->tsin < 4) {
		tbs_read(TBS_DMA_BASE(adapter->tsin), TBS_DMA_STATUS);

		tbs_write(TBS_INT_BASE, TBS_DMA_MASK(adapter->tsin), 0);
		tbs_write(TBS_DMA_BASE(adapter->tsin), TBS_DMA_START, 0);
	} else {
		tbs_read(TBS_DMA2_BASE(adapter->tsin), TBS_DMA_STATUS);

		tbs_write(TBS_INT_BASE, TBS_DMA2_MASK(adapter->tsin), 0);
		tbs_write(TBS_DMA2_BASE(adapter->tsin), TBS_DMA_START, 0);
	}
	adapter->active = 0;
	spin_unlock_irq(&adapter->adap_lock);
}

static void adapter_tasklet(unsigned long adap)
{
	struct tbs_adapter *adapter = (struct tbs_adapter *) adap;
	struct tbs_dev *dev = adapter->dev;
	u8* data;
	u32 active_buffer;
	int i = 0;

	if (adapter->tsin < 4)
		adapter->buffer = tbs_read(TBS_DMA_BASE(adapter->tsin), TBS_DMA_STATUS);
	else
		adapter->buffer = tbs_read(TBS_DMA2_BASE(adapter->tsin), TBS_DMA_STATUS);
	adapter->buffer += 6;

	active_buffer = adapter->buffer;

	spin_lock(&adapter->adap_lock);

	data = (u8*)dev->mem_addr_virt + (adapter->tsin)*(TBS_PCIE_DMA_TOTAL + 256) +
						TBS_PCIE_CELL_SIZE*(active_buffer & 0x07);

	if ((adapter->sync_offset == 0) || (*(data+adapter->sync_offset) != 0x47)) {
		for(i = 0; i < 256; i++)
			if ((*(data + i) == 0x47) && (*(data + i + 188) == 0x47) &&
		 					(*(data + i + 2*188) == 0x47)) {
				adapter->sync_offset = i;
				break;
			}
	}

	data += adapter->sync_offset;

	/* copy from cell0 sync byte offset to cell7 */
	if ((active_buffer & 0x07) == 0x07)
		memcpy( (u8*) dev->mem_addr_virt +
				(adapter->tsin)*(TBS_PCIE_DMA_TOTAL + 256) +
				TBS_PCIE_DMA_TOTAL,
			(u8*) dev->mem_addr_virt + 
				(adapter->tsin)*(TBS_PCIE_DMA_TOTAL + 256),
			adapter->sync_offset);

	if ((dev->mem_addr_virt) && (adapter->active))
		dvb_dmx_swfilter_packets(&adapter->demux, data, adapter->buffer_size / 188);

	spin_unlock(&adapter->adap_lock);
}

static irqreturn_t tbs_irq_handler(int irq, void *dev_id)
{
	struct tbs_dev *dev = (struct tbs_dev *) dev_id;
	struct tbs_i2c *i2c;
	int i, in;
	u32 stat = tbs_read(TBS_INT_BASE, TBS_INT_STATUS);

	tbs_write(TBS_INT_BASE, TBS_INT_STATUS, stat);
	if (stat & 0x00000ff0) {
		/* dma */
		for (i = 0; i < dev->info->adapters; i++) {
			in = dev->info->adap_config[i].ts_in;
			//printk("i=%d, in=%d", i, in);
			if (stat & TBS_INT_DMA_FLAG(in)) {
				tasklet_schedule(&dev->tbs_pcie_adap[i].tasklet);
				//printk(" X");
			}
			//printk(" | ");
		}
		//printk("\n");
	}
	if (stat & 0x0000000f) {
		/* i2c */
		for (i = 0; i < dev->info->i2c_masters; i++) {
			i2c = &dev->i2c_bus[i];
			if (stat & TBS_INT_I2C_FLAG(i2c->nr)) {
				i2c->done = 1;
				wake_up(&i2c->wq);
			}
		}
	}

	tbs_write(TBS_INT_BASE, TBS_INT_ENABLE, 1);
	return IRQ_HANDLED;
}

static int start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct tbs_adapter *adapter = dvbdmx->priv;

	if (!adapter->feeds)
		tbs_pcie_dma_start(adapter);

	return ++adapter->feeds;
}

static int stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct tbs_adapter *adapter = dvbdmx->priv;

	if (--adapter->feeds)
		return adapter->feeds;

	tbs_pcie_dma_stop(adapter);

	return 0;
}

static void tbsecp3_gpio_set_pin(struct tbs_dev *dev, int pin, int state)
{
	u32 tmp;
	u32 bank = (pin >> 3) & ~3;
	u32 bit = pin % 32;

	tmp = tbs_read(TBS_GPIO_BASE, bank);
	if (state)
		tmp |= 1 << bit;
	else
		tmp &= ~(1 << bit);
	//printk("gpio bank=0x%02x val=0x%04x\n", bank, tmp);
	tbs_write(TBS_GPIO_BASE, bank, tmp);
}

static void reset_demod(struct tbs_adapter *adapter)
{
	struct tbs_dev *dev = adapter->dev;
	struct gpio_cfg *cfg = &adapter->cfg->gpio;
	int rst;

	if (cfg->demod_reset_lvl == 0)
		return;

	if (cfg->demod_reset_lvl == TBSECP3_GPIODEF_LOW)
		rst = 0;
	else
		rst = 1;

	tbsecp3_gpio_set_pin(dev, cfg->demod_reset_pin, rst);
	usleep_range(10000, 20000);

	tbsecp3_gpio_set_pin(dev, cfg->demod_reset_pin, 1 - rst);
	usleep_range(50000, 100000);
	
	dev_info(&dev->pci_dev->dev, "Reset adapter %d, pin %d-%d\n", adapter->count, cfg->demod_reset_pin >> 5, cfg->demod_reset_pin & 0x1F);	
}

#define TBS6904_LNBPWR_PIN	(2)

static void tbs6904_lnb_power(struct dvb_frontend *fe,
	int gpio, int onoff)
{
	struct i2c_adapter *adapter = tas2101_get_i2c_adapter(fe, 0);
        struct tbs_i2c *i2c = i2c_get_adapdata(adapter);
	struct tbs_dev *dev = i2c->dev;

	/* lnb power, active low */
	if (onoff)
		tbs_pcie_gpio_write(dev, gpio, TBS6904_LNBPWR_PIN, 0);
	else
		tbs_pcie_gpio_write(dev, gpio, TBS6904_LNBPWR_PIN, 1);
}

static void tbs6904_lnb0_power(struct dvb_frontend *fe, int onoff)
{
	tbs6904_lnb_power(fe, 0, onoff);
}

static void tbs6904_lnb1_power(struct dvb_frontend *fe, int onoff)
{
	tbs6904_lnb_power(fe, 1, onoff);
}

static void tbs6904_lnb2_power(struct dvb_frontend *fe, int onoff)
{
	tbs6904_lnb_power(fe, 2, onoff);
}

static void tbs6904_lnb3_power(struct dvb_frontend *fe, int onoff)
{
	tbs6904_lnb_power(fe, 3, onoff);
}

static struct tas2101_config tbs6904_demod_cfg[] = {
	{
		.i2c_address   = 0x60,
		.id            = ID_TAS2101,
		.lnb_power     = tbs6904_lnb0_power,
		.init          = {0xb0, 0x32, 0x81, 0x57, 0x64, 0x9a, 0x33}, // 0xb1
		.init2         = 0,
	},
	{
		.i2c_address   = 0x68,
		.id            = ID_TAS2101,
		.lnb_power     = tbs6904_lnb1_power,
		.init          = {0xb0, 0x32, 0x81, 0x57, 0x64, 0x9a, 0x33},
		.init2         = 0,
	},
	{
		.i2c_address   = 0x60,
		.id            = ID_TAS2101,
		.lnb_power     = tbs6904_lnb2_power,
		.init          = {0xb0, 0x32, 0x81, 0x57, 0x64, 0x9a, 0x33},
		.init2         = 0,
	},
	{
		.i2c_address   = 0x68,
		.id            = ID_TAS2101,
		.lnb_power     = tbs6904_lnb3_power,
		.init          = {0xb0, 0x32, 0x81, 0x57, 0x64, 0x9a, 0x33},
		.init2         = 0,
	}
};


static struct av201x_config tbs6904_av201x_cfg = {
	.i2c_address = 0x63,
	.id          = ID_AV2012,
	.xtal_freq   = 27000,		/* kHz */
};


static int max_set_voltage(struct i2c_adapter *i2c,
		enum fe_sec_voltage voltage, u8 rf_in)
{
	struct tbs_i2c *i2c_adap = i2c_get_adapdata(i2c);
	struct tbs_dev *dev = i2c_adap->dev;

	u32 val, reg;

	//printk("set voltage on %u = %d\n", rf_in, voltage);
	
	if (rf_in > 3)
		return -EINVAL;

	reg = rf_in * 4;
	val = tbs_read(TBS_GPIO_BASE, reg) & ~4;

	switch (voltage) {
	case SEC_VOLTAGE_13:
		val &= ~2;
		break;
	case SEC_VOLTAGE_18:
		val |= 2;
		break;
	case SEC_VOLTAGE_OFF:
	default:
		val |= 4;
		break;
	}

	tbs_write(TBS_GPIO_BASE, reg, val);
	return 0;
}

static int max_send_master_cmd(struct dvb_frontend *fe, struct dvb_diseqc_master_cmd *cmd)
{
	//printk("send master cmd\n");
	return 0;
}
static int max_send_burst(struct dvb_frontend *fe, enum fe_sec_mini_cmd burst)
{
	//printk("send burst: %d\n", burst);
	return 0;
}

static struct mxl5xx_cfg tbs6909_mxl5xx_cfg = {
	.adr		= 0x60,
	.type		= 0x01,
	.clk		= 24000000,
	.cap		= 12,
	.fw_read	= NULL,

	.set_voltage	= max_set_voltage,
};

static void tbsecp3_remove_i2c_clients(struct tbs_adapter *adapter)
{
	struct i2c_client *client_demod, *client_tuner;

	/* remove tuner I2C client */
	client_tuner = adapter->i2c_client_tuner;
	if (client_tuner) {
		module_put(client_tuner->dev.driver->owner);
		i2c_unregister_device(client_tuner);
		adapter->i2c_client_tuner = NULL;
	}

	/* remove demodulator I2C client */
	client_demod = adapter->i2c_client_demod;
	if (client_demod) {
		module_put(client_demod->dev.driver->owner);
		i2c_unregister_device(client_demod);
		adapter->i2c_client_demod = NULL;
	}
}

static struct stv0910_cfg tbs_stv0910_config = {
	.adr      = 0x68,
	.parallel = 1,
	.rptlvl   = 4,
	.clk      = 30000000,
	.dual_tuner = 1,
};

static struct stv6120_config tbs_stv6120_0_config = {
	.addr			= 0x60,
	.refclk			= 30000000,
	.clk_div		= 2,
	.tuner			= 1,
	.bbgain			= 6,
};

static struct stv6120_config tbs_stv6120_1_config = {
	.addr			= 0x60,
	.refclk			= 30000000,
	.clk_div		= 2,
	.tuner			= 0,
	.bbgain			= 6,
};

static int tbsecp3_frontend_attach(struct tbs_adapter *adapter)
{
	struct tbs_dev *dev = adapter->dev;
	struct pci_dev *pci = dev->pci_dev;

	struct si2168_config si2168_config;
	struct si2157_config si2157_config;

	//struct av201x_config av201x_config;

	struct i2c_board_info info;
	struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct i2c_client *client_demod, *client_tuner;


	adapter->fe = NULL;
	adapter->i2c_client_demod = NULL;
	adapter->i2c_client_tuner = NULL;

	reset_demod(adapter);

	switch (pci->subsystem_vendor) {
	case 0x6205:
		/* attach demod */
		memset(&si2168_config, 0, sizeof(si2168_config));
		si2168_config.i2c_adapter = &i2c;
		si2168_config.fe = &adapter->fe;
		si2168_config.ts_mode = SI2168_TS_PARALLEL;
		si2168_config.ts_clock_gapped = true;

		memset(&info, 0, sizeof(struct i2c_board_info));
		strlcpy(info.type, "si2168", I2C_NAME_SIZE);
		info.addr = 0x64;
		info.platform_data = &si2168_config;
		request_module(info.type);
		client_demod = i2c_new_device(i2c, &info);
		if (client_demod == NULL ||
				client_demod->dev.driver == NULL)
			goto frontend_atach_fail;
		if (!try_module_get(client_demod->dev.driver->owner)) {
			i2c_unregister_device(client_demod);
			goto frontend_atach_fail;
		}
		adapter->i2c_client_demod = client_demod;

		/* attach tuner */
		memset(&si2157_config, 0, sizeof(si2157_config));
		si2157_config.fe = adapter->fe;
		si2157_config.if_port = 1;

		memset(&info, 0, sizeof(struct i2c_board_info));
		strlcpy(info.type, "si2157", I2C_NAME_SIZE);
		info.addr = 0x60;
		info.platform_data = &si2157_config;
		request_module(info.type);
		client_tuner = i2c_new_device(i2c, &info);
		if (client_tuner == NULL ||
				client_tuner->dev.driver == NULL)
			goto frontend_atach_fail;

		if (!try_module_get(client_tuner->dev.driver->owner)) {
			i2c_unregister_device(client_tuner);
			goto frontend_atach_fail;
		}
		adapter->i2c_client_tuner = client_tuner;
		break;
	case 0x6903:
		adapter->fe = dvb_attach(stv0910_attach, i2c,
				&tbs_stv0910_config, adapter->count & 1);
		if (adapter->fe == NULL)
			goto frontend_atach_fail;

		if (dvb_attach(stv6120_attach, adapter->fe,
				adapter->count & 1 ? &tbs_stv6120_1_config : &tbs_stv6120_0_config,i2c) == NULL) {
			pr_err("No STV6120 found !\n");
			dvb_frontend_detach(adapter->fe);
			adapter->fe = NULL;
			dev_err(&dev->pci_dev->dev,
				"TBS_PCIE frontend %d tuner attach failed\n",
				adapter->count);
			goto frontend_atach_fail;
		}

		break;
	case 0x6905:
	case 0x6908:
		adapter->fe = dvb_attach(stv0910_attach, i2c,
				&tbs_stv0910_config, adapter->count & 1);
		if (adapter->fe == NULL)
			goto frontend_atach_fail;

		if (dvb_attach(stv6120_attach, adapter->fe,
				adapter->count & 1 ? &tbs_stv6120_1_config : &tbs_stv6120_0_config,i2c) == NULL) {
			pr_err("No STV6120 found !\n");
			dvb_frontend_detach(adapter->fe);
			adapter->fe = NULL;
			dev_err(&dev->pci_dev->dev,
				"TBS_PCIE frontend %d tuner attach failed\n",
				adapter->count);
			goto frontend_atach_fail;
		}

		break;
	case 0x6902:
	case 0x6904:
		adapter->fe = dvb_attach(tas2101_attach, &tbs6904_demod_cfg[adapter->count], i2c);
		if (adapter->fe == NULL)
			goto frontend_atach_fail;

		if (dvb_attach(av201x_attach, adapter->fe, &tbs6904_av201x_cfg,
				tas2101_get_i2c_adapter(adapter->fe, 2)) == NULL) {
			dvb_frontend_detach(adapter->fe);
			adapter->fe = NULL;
			dev_err(&dev->pci_dev->dev,
				"TBS_PCIE frontend %d tuner attach failed\n",
				adapter->count);
			goto frontend_atach_fail;
		}


#if 0
		/* attach tuner */
		memset(&av201x_config, 0, sizeof(av201x_config));
		av201x_config.fe = adapter->fe;
		av201x_config.xtal_freq = 27000,

		memset(&info, 0, sizeof(struct i2c_board_info));
		strlcpy(info.type, "av2012", I2C_NAME_SIZE);
		info.addr = 0x63;
		info.platform_data = &av201x_config;
		request_module(info.type);
		client_tuner = i2c_new_device(i2c, &info);
		printk("cli_tu=%p\n", client_tuner);
		if (client_tuner != NULL)
			printk("cli_tu.drv=%p\n", client_tuner->dev.driver);
		if (client_tuner == NULL ||
				client_tuner->dev.driver == NULL) {
			printk("client tunner fail 1\n");
			goto frontend_atach_fail;
		}

		if (!try_module_get(client_tuner->dev.driver->owner)) {
			i2c_unregister_device(client_tuner);
			printk("client tunner fail 2\n");
			goto frontend_atach_fail;
		}
		adapter->i2c_client_tuner = client_tuner;
#endif
		break;
	case 0x6909:
/*
		tmp = tbs_read(TBS_GPIO_BASE, 0x20);
		printk("RD 0x20 = %x\n", tmp);
		tbs_write(TBS_GPIO_BASE, 0x20, tmp & 0xfffe);
		tmp = tbs_read(TBS_GPIO_BASE, 0x20);
		printk("RD 0x20 = %x\n", tmp);

		tmp = tbs_read(TBS_GPIO_BASE, 0x24);
		printk("RD 0x24 = %x\n", tmp);
		tbs_write(TBS_GPIO_BASE, 0x24, tmp & 0xfffc);
		tmp = tbs_read(TBS_GPIO_BASE, 0x24);
		printk("RD 0x24 = %x\n", tmp);
*/

		adapter->fe = dvb_attach(mxl5xx_attach, i2c,
				&tbs6909_mxl5xx_cfg, adapter->count);
		if (adapter->fe == NULL)
			goto frontend_atach_fail;

		adapter->fe->ops.diseqc_send_master_cmd = max_send_master_cmd;
		adapter->fe->ops.diseqc_send_burst = max_send_burst;

		break;
	default:
		printk("unknonw card\n");
		return -ENODEV;
		break;
	}

	return 0;

frontend_atach_fail:
	tbsecp3_remove_i2c_clients(adapter);
	if (adapter->fe != NULL)
		dvb_frontend_detach(adapter->fe);
	adapter->fe = NULL;
	dev_err(&dev->pci_dev->dev, "TBSECP3 frontend %d attach failed\n",
		adapter->count);

	return -ENODEV;
}


static void tbs_dvb_exit(struct tbs_adapter *adapter)
{
	//struct tbs_dev *dev = adapter->dev;
	struct dvb_adapter *adap = &adapter->dvb_adapter;
	struct dvb_demux *dvbdemux = &adapter->demux;

	if (adapter->fe) {
		dvb_unregister_frontend(adapter->fe);
		dvb_frontend_detach(adapter->fe);
		adapter->fe = NULL;
	}
	dvb_net_release(&adapter->dvbnet);
	dvbdemux->dmx.close(&dvbdemux->dmx);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, &adapter->fe_mem);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, &adapter->fe_hw);
	dvb_dmxdev_release(&adapter->dmxdev);
	dvb_dmx_release(&adapter->demux);
	dvb_unregister_adapter(adap);
}

static int tbs_dvb_init(struct tbs_adapter *adapter)
{
	struct tbs_dev *dev = adapter->dev;
	struct dvb_adapter *adap = &adapter->dvb_adapter;
	struct dvb_demux *dvbdemux = &adapter->demux;
	struct dmxdev *dmxdev;
	struct dmx_frontend *fe_hw;
	struct dmx_frontend *fe_mem;
	int ret;

	ret = dvb_register_adapter(adap, "TBSECP3 DVB Adapter",
					THIS_MODULE,
					&adapter->dev->pci_dev->dev,
					adapter_nr);
	if (ret < 0) {
		dev_err(&dev->pci_dev->dev, "error registering adapter\n");
		return -ENODEV;
	}

	adap->priv = adapter;
	dvbdemux->priv = adapter;
	dvbdemux->filternum = 256;
	dvbdemux->feednum = 256;
	dvbdemux->start_feed = start_feed;
	dvbdemux->stop_feed = stop_feed;
	dvbdemux->write_to_decoder = NULL;
	dvbdemux->dmx.capabilities = (DMX_TS_FILTERING |
				      DMX_SECTION_FILTERING |
				      DMX_MEMORY_BASED_FILTERING);

	ret = dvb_dmx_init(dvbdemux);
	if (ret < 0) {
		printk("dvb_dmx_init failed, ERROR=%d", ret);
		goto err0;
	}

	dmxdev = &adapter->dmxdev;

	dmxdev->filternum = 256;
	dmxdev->demux = &dvbdemux->dmx;
	dmxdev->capabilities = 0;

	ret = dvb_dmxdev_init(dmxdev, adap);
	if (ret < 0) {
		printk("dvb_dmxdev_init failed, ERROR=%d", ret);
		goto err1;
	}

	fe_hw = &adapter->fe_hw;
	fe_mem = &adapter->fe_mem;

	fe_hw->source = DMX_FRONTEND_0;
	ret = dvbdemux->dmx.add_frontend(&dvbdemux->dmx, fe_hw);
	if ( ret < 0) {
		printk("dvb_dmx_init failed, ERROR=%d", ret);
		goto err2;
	}

	fe_mem->source = DMX_MEMORY_FE;
	ret = dvbdemux->dmx.add_frontend(&dvbdemux->dmx, fe_mem);
	if (ret  < 0) {
		printk("dvb_dmx_init failed, ERROR=%d", ret);
		goto err3;
	}

	ret = dvbdemux->dmx.connect_frontend(&dvbdemux->dmx, fe_hw);
	if (ret < 0) {
		printk("dvb_dmx_init failed, ERROR=%d", ret);
		goto err4;
	}

	ret = dvb_net_init(adap, &adapter->dvbnet, adapter->dmxdev.demux);
	if (ret < 0) {
		printk("dvb_net_init failed, ERROR=%d", ret);
		goto err5;
	}

	tbsecp3_frontend_attach(adapter);
	if (adapter->fe == NULL) {
		printk("TBS frontend attach failed\n");
		ret = -ENODEV;
		goto err6;
	}

	ret = dvb_register_frontend(adap, adapter->fe);
	if (ret < 0) {
		printk("TBS PCIE register frontend failed\n");
		goto err7;
	}

	return ret;

err7:
	dvb_frontend_detach(adapter->fe);
err6:
	dvb_net_release(&adapter->dvbnet);
err5:
	dvbdemux->dmx.close(&dvbdemux->dmx);
err4:
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, fe_mem);
err3:
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, fe_hw);
err2:
	dvb_dmxdev_release(dmxdev);
err1:
	dvb_dmx_release(dvbdemux);
err0:
	dvb_unregister_adapter(adap);
	return ret;
}

static int tbs_adapters_attach(struct tbs_dev *dev)
{
	struct tbs_adapter *adapter;
	int i, ret = 0;

	for (i = 0; i < dev->info->adapters; i++) {
		adapter = &dev->tbs_pcie_adap[i];
		adapter->cfg = &dev->info->adap_config[i];

		ret = tbs_dvb_init(adapter);

		if (ret < 0) {
			printk(KERN_ERR "TBS PCIE Adapter%d attach failed\n",
				adapter->count);
			adapter->count = -1;
		}
	}

	return 0;
}

static void tbs_adapters_detach(struct tbs_dev *dev)
{
	struct tbs_adapter *adapter;
	int i;

	for (i = 0; i < dev->info->adapters; i++) {
		adapter = &dev->tbs_pcie_adap[i];

		/* attach has failed, nothing to do */
		if (adapter->count == -1)
			continue;

		tbsecp3_remove_i2c_clients(adapter);
		tbs_dvb_exit(adapter);
	}
}

static void tbs_adapters_init(struct tbs_dev *dev)
{
	struct tbs_adapter *tbs_adap;
	int i;

	for (i = 0; i < dev->info->adapters; i++) {
		tbs_adap = &dev->tbs_pcie_adap[i];
		tbs_adap->dev = dev;
		tbs_adap->count = i;
		tbs_adap->tsin = dev->info->adap_config[i].ts_in;
		tbs_adap->i2c = &dev->i2c_bus[dev->info->adap_config[i].i2c_master_nr];

		tbs_adap->buffer_size = TBS_PCIE_CELL_SIZE;

		tasklet_init(&tbs_adap->tasklet, adapter_tasklet, (unsigned long) tbs_adap);
		spin_lock_init(&tbs_adap->adap_lock);
	}
}

static void tbs_adapters_release(struct tbs_dev *dev)
{
	struct tbs_adapter *tbs_adap;
	int i;

	for (i = 0; i < dev->info->adapters; i++) {
		tbs_adap = &dev->tbs_pcie_adap[i];
		tbs_adap->dev = dev;
		tasklet_kill(&tbs_adap->tasklet);
	}
}


static bool tbsecp3_enable_msi(struct pci_dev *pci_dev, struct tbs_dev *dev)
{
	int err;

	if (!enable_msi) {
		dev_warn(&dev->pci_dev->dev,
			"MSI disabled by module parameter 'enable_msi'\n");
		return false;
	}

	err = pci_enable_msi(pci_dev);
	if (err) {
		dev_err(&dev->pci_dev->dev,
			"Failed to enable MSI interrupt."
			" Falling back to a shared IRQ\n");
		return false;
	}

	/* no error - so request an msi interrupt */
	err = request_irq(pci_dev->irq, tbs_irq_handler, 0,
				"tbspcie", dev);
	if (err) {
		/* fall back to legacy interrupt */
		dev_err(&dev->pci_dev->dev,
			"Failed to get an MSI interrupt."
			" Falling back to a shared IRQ\n");
		pci_disable_msi(pci_dev);
		return false;
	}
	return true;
}


static int tbsecp3_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct tbs_dev *dev;
	int ret = -ENODEV;

	if (pci_enable_device(pdev) < 0)
		return -ENODEV;

	dev  = kzalloc(sizeof(struct tbs_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err0;
	}

	dev->pci_dev = pdev;
	pci_set_drvdata(pdev, dev);

	dev->info = (struct tbs_cfg_info *) id->driver_data;
	dev_info(&dev->pci_dev->dev, "%s\n", dev->info->name);

	dev->lmmio = ioremap(pci_resource_start(dev->pci_dev, 0),
				pci_resource_len(dev->pci_dev, 0));
	if (!dev->lmmio) {
		ret = -ENOMEM;
		goto err1;
	}

	tbs_write(TBS_INT_BASE, TBS_INT_ENABLE, 0);
	tbs_write(TBS_INT_BASE, TBS_INT_STATUS, 0xff);

	tbs_adapters_init(dev);

	/* dma */
	ret = tbs_pcie_dma_init(dev);
	if (ret < 0)
		goto err2;

	/* i2c */
	ret = tbs_i2c_init(dev);
	if (ret < 0)
		goto err3;

	/* interrupts */
	if (tbsecp3_enable_msi(pdev, dev)) {
		dev->msi = true;
	} else {
		ret = request_irq(dev->pci_dev->irq, tbs_irq_handler,
					IRQF_SHARED, "tbspcie", dev);
		if (ret < 0) {
			dev_err(&dev->pci_dev->dev, "%s: can't get IRQ %d\n",
				dev->info->name, pdev->irq);
			goto err4;
		}
		dev->msi = false;
	}
	/* hw */
	tbsecp3_init_hw(dev);

	ret = tbs_adapters_attach(dev);
	if (ret < 0)
		goto err5;

	dev_info(&dev->pci_dev->dev, "%s ready\n", dev->info->name);
	return 0;

err5:
	tbs_adapters_detach(dev);
	tbs_adapters_release(dev);

	tbs_write(TBS_INT_BASE, TBS_INT_ENABLE, 0);
	free_irq(dev->pci_dev->irq, dev);
	if (dev->msi) {
		pci_disable_msi(pdev);
		dev->msi = false;
	}
err4:
	tbs_i2c_exit(dev);
err3:
	tbs_pcie_dma_free(dev);
err2:
	iounmap(dev->lmmio);
err1:
	pci_set_drvdata(pdev, NULL);
	kfree(dev);
err0:
	pci_disable_device(pdev);
	dev_err(&dev->pci_dev->dev, "%s: probe error\n", dev->info->name);
	return ret;
}

static void tbsecp3_remove(struct pci_dev *pdev)
{
	struct tbs_dev *dev = pci_get_drvdata(pdev);

	/* disable interrupts */
	tbs_write(TBS_INT_BASE, TBS_INT_ENABLE, 0); 
	free_irq(dev->pci_dev->irq, dev);
	if (dev->msi) {
		pci_disable_msi(dev->pci_dev);
		dev->msi = false;
	}
	tbs_adapters_detach(dev);
	tbs_adapters_release(dev);
	tbs_pcie_dma_free(dev);
	tbs_i2c_exit(dev);
	iounmap(dev->lmmio);
	pci_set_drvdata(pdev, NULL);
	pci_disable_device(pdev);
	kfree(dev);
}

static int tbsecp3_resume(struct pci_dev *pdev)
{
	struct tbs_dev *dev = pci_get_drvdata(pdev);

	/* re-init registers */
	tbsecp3_init_hw(dev);

	return 0;
}

/* PCI IDs */
#define TBSECP3_ID(_subvend, _driverdata_idx) { \
	.vendor = TBSECP3_VID, .device = TBSECP3_PID, \
	.subvendor = _subvend, .subdevice = PCI_ANY_ID, \
	.driver_data = (unsigned long)&tbs_ecp3_boards[_driverdata_idx] }

static const struct pci_device_id tbsecp3_id_table[] = {
	TBSECP3_ID(0x6205, TBSECP3_BOARD_TBS6205),
	TBSECP3_ID(0x6903, TBSECP3_BOARD_TBS6903),
	TBSECP3_ID(0x6904, TBSECP3_BOARD_TBS6904),
	TBSECP3_ID(0x6905, TBSECP3_BOARD_TBS6905),
	TBSECP3_ID(0x6908, TBSECP3_BOARD_TBS6908),
	TBSECP3_ID(0x6909, TBSECP3_BOARD_TBS6909),
	{0}
};
MODULE_DEVICE_TABLE(pci, tbsecp3_id_table);

static struct pci_driver tbspcie_driver = {
	.name = "TBSECP3 driver",
	.id_table = tbsecp3_id_table,
	.probe    = tbsecp3_probe,
	.remove   = tbsecp3_remove,
	.resume   = tbsecp3_resume,
	.suspend  = NULL,
};

module_pci_driver(tbspcie_driver);

MODULE_AUTHOR("Luis Alves <ljalvs@gmail.com>");
MODULE_DESCRIPTION("TBSECP3 driver");
MODULE_LICENSE("GPL");
