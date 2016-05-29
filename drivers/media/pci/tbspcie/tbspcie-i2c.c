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



union tbs_i2c_ctrl {
	struct {
		u32 ctrl;
		u32 data;
	} raw;
	struct {
		u8 size:4;
		u8 saddr:2;
		u8 stop:1;
		u8 start:1;
		u8 read:1;
		u8 addr:7;
		u8 buf[6];
	} bits;
};

static int i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msg, int num)
{
	struct tbs_i2c *bus = i2c_get_adapdata(adapter);
	struct tbs_dev *dev = bus->dev;
	union tbs_i2c_ctrl i2c_ctrl;
	int i, j, retval;
	u16 len, remaining, xfer_max;
	u8 *b;

#if 1
	if (msg[0].addr == 1) {
		
		switch (msg[0].buf[0]) {
		case 0:
			printk("status=0x%x\n", tbs_read(bus->base, TBS_I2C_CTRL));
			break;
		case 1:
			printk("data=0x%08x\n", tbs_read(bus->base, TBS_I2C_DATA));
			break;
		case 2:
			printk("speed=0x%02x\n", tbs_read(bus->base, TBS_I2C_SPEED));
			break;
		case 3:
			tbs_write(bus->base, TBS_I2C_SPEED, (u32) msg[0].buf[1]);
			break;
			
		default:
			break;
		}
		return 1;
	}

	if ((msg[0].addr == 0) && (msg[0].len == 3)) {
		u8 pin = msg[0].buf[1];
		u8 bank = msg[0].buf[0];
		u8 state = msg[0].buf[2];
		u32 tmp;
		printk("gpio ctrl: bank 0x%02x pin 0x%02x state=%d", bank, pin, state);

		tmp = tbs_read(TBS_GPIO_BASE, bank);
		if (state)
			tmp |= 1 << pin;
		else
			tmp &= ~(1 << pin);
		printk(" val=0x%04x\n", tmp);
		tbs_write(TBS_GPIO_BASE, bank, tmp);
		return 1;
	}
#endif
	mutex_lock(&bus->i2c_lock);
	for (i = 0; i < num; i++) {

		b = msg[i].buf;
		remaining = msg[i].len;

		i2c_ctrl.raw.ctrl = 0;
		i2c_ctrl.bits.start = 1;
		i2c_ctrl.bits.addr = msg[i].addr;
		
		if (msg[i].flags & I2C_M_RD) {
			i2c_ctrl.bits.read = 1;
			xfer_max = 4;
		} else {
			xfer_max = 6;
		}

		do {
			if (remaining <= xfer_max)
				i2c_ctrl.bits.stop = 1;

			len = remaining > xfer_max ? xfer_max : remaining;
			i2c_ctrl.bits.size = len;

			//j = tbs_read(bus->base, TBS_I2C_CTRL);
			//if (j & 2) {
				/* bus is busy */
			//	dev_err(&dev->pci_dev->dev, "i2c bus busy\n");
			//	retval = -EIO;
			//	goto i2c_xfer_exit;
			//}

			if (!(msg[i].flags & I2C_M_RD)) {
				for (j = 0; j < len; j++)
					i2c_ctrl.bits.buf[j] = *b++;
				tbs_write(bus->base, TBS_I2C_DATA, i2c_ctrl.raw.data);
			}
			bus->done = 0;
			tbs_write(bus->base, TBS_I2C_CTRL, i2c_ctrl.raw.ctrl);
			retval = wait_event_timeout(bus->wq, bus->done == 1, HZ);
			if (retval == 0) {
				dev_err(&dev->pci_dev->dev, "i2c xfer timeout\n");
				retval = -EIO;
				goto i2c_xfer_exit;
			}

			j = tbs_read(bus->base, TBS_I2C_CTRL);
			if (j & 0x04) {
				dev_err(&dev->pci_dev->dev, "i2c nack (%x)\n", j);
				retval = -EIO;
				goto i2c_xfer_exit;
			}

			if (msg[i].flags & I2C_M_RD) {
				i2c_ctrl.raw.data = tbs_read(bus->base, TBS_I2C_DATA);
				memcpy(b, &i2c_ctrl.raw.data, len);
				b += len;
			}
			
			i2c_ctrl.bits.start = 0;
			remaining -= len;
		} while (remaining);

	}
	retval = num;
i2c_xfer_exit:
	mutex_unlock(&bus->i2c_lock);
	return retval;
}

static u32 i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL;
}

struct i2c_algorithm tbs_i2c_algo_template = {
	.master_xfer   = i2c_xfer,
	.functionality = i2c_functionality,
};

/* ----------------------------------------------------------------------- */

static struct i2c_adapter tbs_i2c_adap_template = {
	.name              = "tbspcie",
	.owner             = THIS_MODULE,
	.algo              = &tbs_i2c_algo_template,
};

static struct i2c_client tbspcie_i2c_client_template = {
	.name	= "tbspcie internal",
};

int tbs_i2c_register(struct tbs_i2c *bus)
{
	struct tbs_dev *dev = bus->dev;

	bus->i2c_adap = tbs_i2c_adap_template;
	bus->i2c_adap.dev.parent = &dev->pci_dev->dev;

	bus->i2c_client = tbspcie_i2c_client_template;
	bus->i2c_client.adapter = &bus->i2c_adap;

	strlcpy(bus->i2c_adap.name, bus->dev->info->name,
		sizeof(bus->i2c_adap.name));

	init_waitqueue_head(&bus->wq);
	mutex_init(&bus->i2c_lock);

	i2c_set_adapdata(&bus->i2c_adap, bus);
	return i2c_add_adapter(&bus->i2c_adap);
}

void tbs_i2c_unregister(struct tbs_i2c *bus)
{
	i2c_del_adapter(&bus->i2c_adap);
}

