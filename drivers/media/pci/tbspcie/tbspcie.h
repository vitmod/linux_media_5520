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

#ifndef _TBS_PCIE_H_
#define _TBS_PCIE_H_

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include "demux.h"
#include "dmxdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"
#include "dvb_net.h"
#include "dvbdev.h"
#include "tbspcie-regs.h"


#define TBSECP3_VID		0x544d
#define TBSECP3_PID		0x6178


#define TBSECP3_MAX_ADAPTERS	(8)
#define TBSECP3_MAX_I2C_BUS	(4)

#define TBSECP3_BOARD_TBS6205 	0x6205
#define TBSECP3_BOARD_TBS6902	0x6902
#define TBSECP3_BOARD_TBS6903	0x6903
#define TBSECP3_BOARD_TBS6904	0x6904
#define TBSECP3_BOARD_TBS6905	0x6905
#define TBSECP3_BOARD_TBS6908	0x6908
#define TBSECP3_BOARD_TBS6909	0x6909


enum tbs_i2c_bus_nr {
	TBS_I2C_BUS_0 = 0,
	TBS_I2C_BUS_1,
	TBS_I2C_BUS_2,
	TBS_I2C_BUS_3,
};


struct tbs_dev;
struct tbs_adapter;


#define TBSECP3_GPIODEF_NONE	(0)
#define TBSECP3_GPIODEF_HIGH	(1)
#define TBSECP3_GPIODEF_LOW	(2)


struct tbs_ecp3_subid {
	u16     subvendor;
	u16     subdevice;
	u32     card;
};

struct gpio_cfg {
	u8 voltage_onoff_pin;
	u8 voltage_onoff_lvl;

	u8 voltage_1318_pin;
	u8 voltage_1318_lvl;

	u8 demod_reset_pin;
	u8 demod_reset_lvl;
};


struct tbs_adap_config {
        u32		ts_in;
	u8		i2c_master_nr;
	u32		i2c_speed;


	struct gpio_cfg gpio;
};

struct tbs_i2c_info {
	enum tbs_i2c_bus_nr nr;
	u32 mode;
};

struct tbs_cfg_info {
	char *name;

	int i2c_masters;
	struct tbs_i2c_info i2c_config[4];

	//int (*frontend_attach)(struct tbs_adapter *adapter, int type);

	int adapters;
	struct tbs_adap_config adap_config[8];
};

struct tbs_i2c {
	struct tbs_dev		*dev;
	enum tbs_i2c_bus_nr	nr;
	u32			base;

	struct i2c_adapter	i2c_adap;
	struct i2c_client	i2c_client;

	struct mutex		i2c_lock;
	wait_queue_head_t	wq;

	bool			done;
};

struct tbs_adapter {
	struct tbs_dev		*dev;
	struct tbs_i2c		*i2c;

	struct i2c_client	*i2c_client_demod;
	struct i2c_client	*i2c_client_tuner;

	struct tasklet_struct	tasklet;
	spinlock_t		adap_lock;
	int			active;

	u32			buffer_size;
	u32			buffer;
	u8			sync_offset;

	struct dvb_adapter	dvb_adapter;
	struct dvb_frontend	*fe;
	struct dvb_demux	demux;
	struct dmxdev		dmxdev;
	struct dvb_net		dvbnet;
	struct dmx_frontend	fe_hw;
	struct dmx_frontend	fe_mem;

	int			feeds;
	int			count;
	int			tsin;

	struct tbs_adap_config *cfg;
};





struct tbs_dev {
	struct tbs_cfg_info	*info;

	/* pcie */
	struct pci_dev		*pci_dev;
	void __iomem		*lmmio;

	/* ts port */
	struct tbs_adapter	tbs_pcie_adap[TBSECP3_MAX_ADAPTERS];

	/* i2c */
	struct tbs_i2c		i2c_bus[TBSECP3_MAX_I2C_BUS];

	bool msi;

	dma_addr_t		mem_addr_phys;
	__le32			*mem_addr_virt;
};

#define tbs_read(_b, _o)	readl(dev->lmmio + (_b + _o))
#define tbs_write(_b, _o, _v)	writel((_v), dev->lmmio + (_b + _o))


int tbs_i2c_register(struct tbs_i2c *bus);
void tbs_i2c_unregister(struct tbs_i2c *bus);

extern struct tbs_cfg_info tbs_ecp3_boards[];

#endif
