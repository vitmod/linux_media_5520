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


struct tbs_cfg_info tbs_ecp3_boards[] = {
	[TBSECP3_BOARD_TBS6205] = {
		.name		= "TurboSight TBS 6205 (Quad DVB-T/C)",
		/* i2c info */
		.i2c_masters	= 4,
		.i2c_config	= {
			{ .nr = 0 },
			{ .nr = 1 },
			{ .nr = 2 },
			{ .nr = 3 }
		},
		/* ts ports */
		.adapters	= 4,
		.adap_config	= {
			{
				/* port 0 */
				.ts_in = 0,
				.i2c_master_nr = 0,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(0, 0),
			}, 
			{
				/* port 1 */
				.ts_in = 1,
				.i2c_master_nr = 1,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(2, 0),
			},
			{
				/* port 2 */
				.ts_in = 2,
				.i2c_master_nr = 2,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(1, 0),
			},
			{
				/* port 3 */
				.ts_in = 3,
				.i2c_master_nr = 3,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(3, 0),
			}
		}
	},
	[TBSECP3_BOARD_TBS6902] = {
		.name			= "TurboSight TBS 6903 (Dual DVB-S/S2)",
		.i2c_masters		= 2,
		.i2c_config	= {
			{ .nr = 2 },
			{ .nr = 3 }
		},
		.adapters		= 2,
		.adap_config		= {
			{
				/* adapter 0 */
				.ts_in = 2,
				.i2c_master_nr = 0,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(2, 0),
			}, 
			{
				/* adapter 1 */
				.ts_in = 3,
				.i2c_master_nr = 1,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(3, 0),
			},
		}
	},
	[TBSECP3_BOARD_TBS6903] = {
		.name			= "TurboSight TBS 6903 (Dual DVB-S/S2)",
		.i2c_masters		= 2,
		.i2c_config	= {
			{ .nr = 2 },
			{ .nr = 3 }
		},
		.adapters		= 2,
		.adap_config		= {
			{
				/* adapter 0 */
				.ts_in = 3,
				.i2c_master_nr = 1,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(3, 0),
				.gpio.voltage_1318_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_1318_pin = TBSECP3_GPIO_PIN(3, 1),
				.gpio.voltage_onoff_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_onoff_pin = TBSECP3_GPIO_PIN(3, 2),
			}, 
			{
				/* adapter 1 */
				.ts_in = 2,
				.i2c_master_nr = 1,
				.gpio.voltage_1318_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_1318_pin = TBSECP3_GPIO_PIN(2, 1),
				.gpio.voltage_onoff_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_onoff_pin = TBSECP3_GPIO_PIN(2, 2),
			},
		}
	},
	[TBSECP3_BOARD_TBS6904] = {
		.name			= "TurboSight TBS 6904 (Quad DVB-S/S2)",
		.i2c_masters		= 4,
		.i2c_config	= {
			{ .nr = 0 },
			{ .nr = 1 },
			{ .nr = 2 },
			{ .nr = 3 }
		},
		.adapters		= 4,
		.adap_config		= {
			{
				/* adapter 0 */
				.ts_in = 0,
				.i2c_master_nr = 0,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(0, 0),
			}, 
			{
				/* adapter 1 */
				.ts_in = 1,
				.i2c_master_nr = 1,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(1, 0),
			},
			{
				/* adapter 2 */
				.ts_in = 2,
				.i2c_master_nr = 2,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(2, 0),
			},
			{
				/* adapter 3 */
				.ts_in = 3,
				.i2c_master_nr = 3,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(3, 0),
			}
		}
	},
	[TBSECP3_BOARD_TBS6905] = {
		.name			= "TurboSight TBS 6905 (Quad DVB-S/S2)",
		.i2c_masters		= 2,
		.i2c_config	= {
			{ .nr = 1 },
			{ .nr = 3 }
		},
		.adapters		= 4,
		.adap_config		= {
			{
				/* adapter 0 */
				.ts_in = 1,
				.i2c_master_nr = 0,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(1, 0),
				.gpio.voltage_1318_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_1318_pin = TBSECP3_GPIO_PIN(1, 1),
				.gpio.voltage_onoff_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_onoff_pin = TBSECP3_GPIO_PIN(1, 2),
			}, 
			{
				/* adapter 1 */
				.ts_in = 0,
				.i2c_master_nr = 0,
				.gpio.voltage_1318_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_1318_pin = TBSECP3_GPIO_PIN(0, 1),
				.gpio.voltage_onoff_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_onoff_pin = TBSECP3_GPIO_PIN(0, 2),
			},
			{
				/* adapter 2 */
				.ts_in = 3,
				.i2c_master_nr = 1,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(3, 0),
				.gpio.voltage_1318_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_1318_pin = TBSECP3_GPIO_PIN(3, 1),
				.gpio.voltage_onoff_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_onoff_pin = TBSECP3_GPIO_PIN(3, 2),
			}, 
			{
				/* adapter 3 */
				.ts_in = 2,
				.i2c_master_nr = 1,
				.gpio.voltage_1318_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_1318_pin = TBSECP3_GPIO_PIN(2, 1),
				.gpio.voltage_onoff_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_onoff_pin = TBSECP3_GPIO_PIN(2, 2),
			},
		}
	},
	[TBSECP3_BOARD_TBS6908] = {
		.name			= "TurboSight TBS 6908 (Quad DVB-S/S2)",
		.i2c_masters		= 2,
		.i2c_config	= {
			{ .nr = 1 },
			{ .nr = 3 }
		},
		.adapters		= 4,
		.adap_config		= {
			{
				/* adapter 0 */
				.ts_in = 1,
				.i2c_master_nr = 0,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(1, 0),
				.gpio.voltage_1318_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_1318_pin = TBSECP3_GPIO_PIN(1, 1),
				.gpio.voltage_onoff_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_onoff_pin = TBSECP3_GPIO_PIN(1, 2),
			}, 
			{
				/* adapter 1 */
				.ts_in = 0,
				.i2c_master_nr = 0,
				.gpio.voltage_1318_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_1318_pin = TBSECP3_GPIO_PIN(0, 1),
				.gpio.voltage_onoff_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_onoff_pin = TBSECP3_GPIO_PIN(0, 2),
			},
			{
				/* adapter 2 */
				.ts_in = 3,
				.i2c_master_nr = 1,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(3, 0),
				.gpio.voltage_1318_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_1318_pin = TBSECP3_GPIO_PIN(3, 1),
				.gpio.voltage_onoff_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_onoff_pin = TBSECP3_GPIO_PIN(3, 2),
			}, 
			{
				/* adapter 3 */
				.ts_in = 2,
				.i2c_master_nr = 1,
				.gpio.voltage_1318_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_1318_pin = TBSECP3_GPIO_PIN(2, 1),
				.gpio.voltage_onoff_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.voltage_onoff_pin = TBSECP3_GPIO_PIN(2, 2),
			},
		}
	},
	[TBSECP3_BOARD_TBS6909] = {
		.name			= "TurboSight TBS 6909 (Octa DVB-S/S2)",
		.i2c_masters		= 1,
		.i2c_config		= {
			{ .nr = 3, .mode = 39 },
		},
		.adapters		= 8,
		.adap_config		= {
			{
				/* adapter 0 */
				.ts_in = 3,
				.i2c_master_nr = 0,
				.gpio.demod_reset_lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset_pin = TBSECP3_GPIO_PIN(0, 0),
			}, 
			{
				/* adapter 1 */
				.ts_in = 2,
				.i2c_master_nr = 0,
			},
			{
				/* adapter 2 */
				.ts_in = 1,
				.i2c_master_nr = 0,
			},
			{
				/* adapter 3 */
				.ts_in = 0,
				.i2c_master_nr = 0,
			},
			{
				/* adapter 4 */
				.ts_in = 7,
				.i2c_master_nr = 0,
			},
			{
				/* adapter 5 */
				.ts_in = 6,
				.i2c_master_nr = 0,
			},
			{
				/* adapter 6 */
				.ts_in = 5,
				.i2c_master_nr = 0,
			},
			{
				/* adapter 7 */
				.ts_in = 4,
				.i2c_master_nr = 0,
			}
		}
	},

};

