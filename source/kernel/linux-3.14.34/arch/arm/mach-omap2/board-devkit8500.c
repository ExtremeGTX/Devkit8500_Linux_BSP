/*
 * linux/arch/arm/mach-omap2/board-devkit8500.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/pm_opp.h>
#include <linux/cpu.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>
#include <linux/usb/phy.h>
#include <linux/usb/usb_phy_gen_xceiv.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>
#include <linux/platform_data/mtd-nand-omap2.h>
#include <linux/dm9000.h>

#include "id.h"
#include "common.h"
#include "omap_device.h"
#include "gpmc.h"
#include "soc.h"
#include "mux.h"
#include "hsmmc.h"
#include "pm.h"
#include "board-flash.h"
#include "common-board-devices.h"

#define OMAP3_DEVKIT8500_TS_GPIO       27
#define	NAND_CS	0

static unsigned DVI_PUP_GPIO;

static int board_keymap[] = {
      KEY(0, 0, KEY_A),
      KEY(0, 1, KEY_B),
      KEY(0, 2, KEY_C),
      KEY(0, 3, KEY_D),
      KEY(0, 4, KEY_E),
      KEY(0, 5, KEY_F),

      KEY(1, 0, KEY_G),
      KEY(1, 1, KEY_H),
      KEY(1, 2, KEY_I),
      KEY(1, 3, KEY_J),
      KEY(1, 4, KEY_K),
      KEY(1, 5, KEY_L),

      KEY(2, 0, KEY_M),
      KEY(2, 1, KEY_N),
      KEY(2, 2, KEY_O),
      KEY(2, 3, KEY_P),
      KEY(2, 4, KEY_Q),
      KEY(2, 5, KEY_R),

      KEY(3, 0, KEY_S),
      KEY(3, 1, KEY_U),
      KEY(3, 2, KEY_V),
      KEY(3, 3, KEY_W),
      KEY(3, 4, KEY_X),
      KEY(3, 5, KEY_Y),

      KEY(4, 0, KEY_Z),
      KEY(4, 1, KEY_1),
      KEY(4, 2, KEY_2),
      KEY(4, 3, KEY_3),
      KEY(4, 4, KEY_4),
      KEY(4, 5, KEY_5),

      KEY(5, 0, KEY_6),
      KEY(5, 1, KEY_7),
      KEY(5, 2, KEY_8),
      KEY(5, 3, KEY_9),
      KEY(5, 4, KEY_0),
      KEY(5, 5, KEY_MINUS)
};

static struct matrix_keymap_data board_map_data = {
      .keymap		= board_keymap,
      .keymap_size	= ARRAY_SIZE(board_keymap),
};
static struct gpio_led gpio_leds[] = {
        {
                .name                   = "sys_led",
                .default_trigger        = "heartbeat",
                .gpio                   = 186,
                .active_low             = true,
        },
        {
                .name                   = "user_ledb",
                .gpio                   = -EINVAL,      /* gets replaced */
                .active_low             = true,
        },
        {
                .name                   = "user_led1",
                .gpio                   = -EINVAL,      /* gets replaced */
                .active_low             = true,
        },
        {
                .name                   = "user_led2",
                .gpio                   = -EINVAL,      /* gets replaced */
                .active_low             = true,
        },

};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
        {
                .code                   = KEY_F1, /*KEY_MENU*/
                .gpio                   = 26,
                .desc                   = "menu",
                .active_low             = true,
        },
        {
                .code                   = KEY_ESC, /*KEY_BACK*/
                .gpio                   = 29,
                .desc                   = "back",
                .active_low             = true,
        }
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

#define OMAP_DM9000_GPIO_IRQ	25
#define OMAP_DM9000_BASE	0x2c000000

static struct resource omap_dm9000_resources[] = {
	[0] = {
		.start		= OMAP_DM9000_BASE,
		.end		= (OMAP_DM9000_BASE + 0x4 - 1),
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= (OMAP_DM9000_BASE + 0x400),
		.end		= (OMAP_DM9000_BASE + 0x400 + 0x4 - 1),
		.flags		= IORESOURCE_MEM,
	},
	[2] = {
		.flags		= IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
	},
};

static struct dm9000_plat_data omap_dm9000_platdata = {
	.flags = DM9000_PLATF_16BITONLY,
};

static struct platform_device omap_dm9000_dev = {
	.name = "dm9000",
	.id = -1,
	.num_resources	= ARRAY_SIZE(omap_dm9000_resources),
	.resource	= omap_dm9000_resources,
	.dev = {
		.platform_data = &omap_dm9000_platdata,
	},
};

static void __init omap_dm9000_init(void)
{
	unsigned char *eth_addr = omap_dm9000_platdata.dev_addr;
	struct omap_die_id odi;
	int ret;

	ret = gpio_request_one(OMAP_DM9000_GPIO_IRQ, GPIOF_IN, "dm9000 irq");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for dm9000 IRQ\n",
			OMAP_DM9000_GPIO_IRQ);
		return;
	}

	omap_dm9000_resources[2].start = gpio_to_irq(OMAP_DM9000_GPIO_IRQ);

	/* init the mac address using DIE id */
	omap_get_die_id(&odi);

	eth_addr[0] = 0x02; /* locally administered */
	eth_addr[1] = odi.id_1 & 0xff;
	eth_addr[2] = (odi.id_0 & 0xff000000) >> 24;
	eth_addr[3] = (odi.id_0 & 0x00ff0000) >> 16;
	eth_addr[4] = (odi.id_0 & 0x0000ff00) >> 8;
	eth_addr[5] = (odi.id_0 & 0x000000ff);
}


static struct connector_dvi_platform_data devkit8500_dvi_connector_pdata = {
	.name                   = "dvi",
	.source                 = "tfp410.0",
	.i2c_bus_num            = -1,
};

static struct platform_device devkit8500_dvi_connector_device = {
	.name                   = "connector-dvi",
	.id                     = 0,
	.dev.platform_data      = &devkit8500_dvi_connector_pdata,
};

static struct encoder_tfp410_platform_data devkit8500_tfp410_pdata = {
	.name                   = "tfp410.0",
	.source                 = "dpi.0",
	.data_lines             = 24,
	.power_down_gpio        = -1,
};

static struct platform_device devkit8500_tfp410_device = {
	.name                   = "tfp410",
	.id                     = 0,
	.dev.platform_data      = &devkit8500_tfp410_pdata,
};

static struct connector_atv_platform_data devkit8500_tv_pdata = {
	.name = "tv",
	.source = "venc.0",
	.connector_type = OMAP_DSS_VENC_TYPE_SVIDEO,
	.invert_polarity = false,
};

static struct platform_device devkit8500_tv_connector_device = {
	.name                   = "connector-analog-tv",
	.id                     = 0,
	.dev.platform_data      = &devkit8500_tv_pdata,
};

static struct omap_dss_board_info devkit8500_dss_data = {
	.default_display_name = "dvi",
};

#include "sdram-micron-mt46h32m32lf-6.h"

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -1,
		.deferred	= true,
	},
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp    = -1,
		.gpio_cd    = -EINVAL,
		.deferred	= true,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply devkit8500_vmmc1_supply[] = {
		REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0")
};

static struct regulator_consumer_supply devkit8500_vsim_supply[] = {
		REGULATOR_SUPPLY("vmmc_aux", "omap_hsmmc.0")
};

static struct regulator_consumer_supply devkit8500_vmmc2_supply[] = {
		REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1")
};


static int devkit8500_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	omap_hsmmc_late_init(mmc);

	/* Regulators Linkage Done in _twldata */

	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
        gpio_leds[1].gpio = gpio + TWL4030_GPIO_MAX + 1;
        gpio_leds[2].gpio = gpio + 2;
        gpio_leds[3].gpio = gpio + 15;

        DVI_PUP_GPIO = gpio + 7;
/*
        if (gpio_request_one(DVI_PUP_GPIO, GPIOF_OUT_INIT_LOW, "DVI_PUP"))
        	printk(KERN_ERR "DVI: unable to configure DVI_PUP %d\n", DVI_PUP_GPIO);
        else
        	devkit8500_tfp410_pdata.power_down_gpio = DVI_PUP_GPIO;
*/
        devkit8500_tfp410_pdata.power_down_gpio = DVI_PUP_GPIO;
        /* Register HDMI Transmitter */
		platform_device_register(&devkit8500_tfp410_device);
        /* Register DVI Connector */
		platform_device_register(&devkit8500_dvi_connector_device);
        /* Register TV Connector */
        platform_device_register(&devkit8500_tv_connector_device);

	return 0;
}

static struct twl4030_gpio_platform_data devkit8500_gpio_data = {
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= devkit8500_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data devkit8500_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(devkit8500_vmmc1_supply),
	.consumer_supplies	= devkit8500_vmmc1_supply,
};

/* VMMC2 for MMC2 pins CMD, CLK, DAT0..DAT3 (max 100 mA) */
static struct regulator_init_data devkit8500_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(devkit8500_vmmc2_supply),
	.consumer_supplies	= devkit8500_vmmc2_supply,
};


/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data devkit8500_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(devkit8500_vsim_supply),
	.consumer_supplies	= devkit8500_vsim_supply,
};


static struct twl4030_keypad_data devkit8500_kp_data = {
      .keymap_data      = &board_map_data,
      .rows       	= 6,
      .cols       	= 6,
      .rep        	= 1,
};


static struct twl4030_platform_data devkit8500_twldata = {
	/* platform_data for children goes here */
	.keypad		= &devkit8500_kp_data,
	.gpio		= &devkit8500_gpio_data,
	.vmmc1		= &devkit8500_vmmc1,
	.vmmc2		= &devkit8500_vmmc2,
	.vsim		= &devkit8500_vsim,
};

/* TO BE CHECKED*/
static int __init omap3_devkit8500_i2c_init(void)
{
	omap3_pmic_get_config(&devkit8500_twldata,
				TWL_COMMON_PDATA_USB | TWL_COMMON_PDATA_MADC |
				TWL_COMMON_PDATA_AUDIO,
				TWL_COMMON_REGULATOR_VDAC | TWL_COMMON_REGULATOR_VPLL2);

	devkit8500_twldata.vdac->constraints.apply_uV = true;
//	devkit8500_twldata.usb->usb_mode = T2_USB_MODE_ULPI;

	omap3_pmic_init("twl4030", &devkit8500_twldata);


	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}


static struct platform_device *omap3_devkit8500_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
	&omap_dm9000_dev,
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct mtd_partition devkit8500_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct usbhs_omap_platform_data usbhs_bdata __initdata = {
		.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
};

static struct usbhs_phy_data phy_data[] = {
	{
		.port = 2,
		.reset_gpio = 147,  //GPIO_30: Mux Config says pin used as "NRESWARM" check datasheet
		.vcc_gpio = -1,		/* NO VCC GPIO */
		.vcc_polarity = 1,	/* No Polarity */
	},
};

static void __init omap3_devkit8500_init(void)
{
    omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
    omap_hsmmc_init(mmc); //HS MMC
    omap3_devkit8500_i2c_init();  //require recheck for omap3_devkit8500_i2c_init()

    /*[OMAP_DISPLAY DISABLED]*/
    omap_display_init(&devkit8500_dss_data);


    /*
     * Platform Devices Registeration
     * Leds, Keys, ....
     */
	platform_add_devices(omap3_devkit8500_devices,
			ARRAY_SIZE(omap3_devkit8500_devices));



    /* [TOUCH SCREEN CONTROLLER] */
	//omap_ads7846_init(2, OMAP3_DEVKIT8500_TS_GPIO, 0xA, NULL);    /* in datasheet SPI2_CS0 */

	/* [SERIAL PORTS] */
    omap_serial_init();

    /* [RAM Initialization] */
    omap_sdrc_init(mt46h32m32lf6_sdrc_params,mt46h32m32lf6_sdrc_params);



    /* [NAND Init] */
    board_nand_init(devkit8500_nand_partitions,
    			ARRAY_SIZE(devkit8500_nand_partitions), NAND_CS,
    			NAND_BUSWIDTH_16, NULL);

    omap_twl4030_audio_init("omap3devkit8500", NULL);


    /* [USB HOST] */
    usbhs_init_phys(phy_data, ARRAY_SIZE(phy_data));
    usbhs_init(&usbhs_bdata);

    /* [USB OTG] */
    usb_bind_phy("musb-hdrc.0.auto", 0, "twl4030_usb");
    usb_musb_init(NULL);

    /* Ensure SDRC pins are mux'd for self-refresh */
    omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
    omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

    /* [ETHERNET] */
    omap_dm9000_init();


}



MACHINE_START(DEVKIT8500, "OMAP3 Devkit8500")
	/* Maintainer: Syed Mohammed Khasim - http://beagleboard.org */
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3630_init_early /*omap3_init_early*/,
	.init_irq	= omap3_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= omap3_devkit8500_init,
	.init_late	= omap3630_init_late /*omap3_init_late*/,
	.init_time	= omap3_secure_sync32k_timer_init,
	.restart	= omap3xxx_restart,
MACHINE_END
