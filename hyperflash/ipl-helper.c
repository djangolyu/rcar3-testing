// SPDX-License-Identifier: GPL-2.0
/*
 * To help select IPL.
 * It updates the sector #0 of HyperFlash memory of R-Car H3 StarterKit.
 * It only supports R-Car H3 StarterKit version 3 (WS3.0).
 *
 * If you have a HyperFlash MTD driver, you don't need. Throw it away.
 *
 * It should a platform class driver but not use the platform class to avoid
 * dependency on Devicetree.
 * It's very hacky and cryptic. Never do it like this.
 * And also it's too buggy but I don't have time to fix them.
 *	- Return values must be handled.
 *
 * Copyright (C) Djang Lyu 2020 <lyu2816@gmail.com>
 *
 * Base on:
 *	- drivers/mtd/rpc_hyperflash.c
 *		Copyright (C) 2016 Renesas Electronics Corporation
 *		Copyright (C) 2016 Cogent Embedded, Inc.
 *	- github:renesas/flash_writer
 *		Copyright (C) 2016 Renesas Electronics Corporation
 */

//#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <asm/io.h>

#define RPCIF_BASE		0xEE200000
#define RPCIF_SIZE		0x8100
#define RPCIF_FLASH_BASE	0x08000000
#define RPCIF_FLASH_SIZE	0x04000000

#define RPCIF_WRBUF_BASE	0xEE208000
#define RPCIF_WRBUF_SIZE	(256) /* bytes */


#define _CMNCR		0x0000
#define _SSLDR		0x0004
#define _DRCR		0x000C
#define _DRCMR		0x0010
#define _DREAR		0x0014
#define _DROPR		0x0018
#define _DRENR		0x001C
#define _SMCR		0x0020
#define _SMCMR		0x0024
#define _SMADR		0x0028
#define _SMOPR		0x002C
#define _SMENR		0x0030
#define _SMRDR0		0x0038
#define _SMRDR1		0x003C
#define _SMWDR0		0x0040
#define _SMWDR1		0x0044
#define _CMNSR		0x0048
#define _DRDMCR		0x0058
#define _DRDRENR	0x005C
#define _SMDMCR		0x0060
#define _SMDRENR	0x0064
#define _PHYCNT		0x007C
#define _OFFSET1	0x0080
#define _OFFSET2	0x0084
#define _PHYINT		0x0088

#define _CFI_UNLOCK1		0x555
#define _CFI_UNLOCK2		0x2AA

#define _CFI_CMD_UNLOCK_START	0xAA
#define _CFI_CMD_UNLOCK_ACK	0x55
#define _CFI_CMD_RESET		0xF0
#define _CFI_CMD_READ_STATUS	0x70
#define _CFI_CMD_READ_ID	0x90
#define _CFI_CMD_WRITE		0xA0
#define _CFI_CMD_ERASE_START	0x80
#define _CFI_CMD_ERASE_SECTOR	0x30


/* Driver data for Hyperbus flash memory */
struct hf_drvdata {
	struct device *dev;
	void __iomem *io_base;
	void __iomem *flash_base;
	struct resource *rpc_res;
	struct resource *flash_res;
	struct regmap *regmap;
};

static struct device *hflash_dev;

static const struct regmap_range rpcif_volatile_ranges[] = {
	regmap_reg_range(_SMRDR0, _SMRDR1),
	regmap_reg_range(_SMWDR0, _SMWDR1),
	regmap_reg_range(_CMNSR, _CMNSR),
};

static const struct regmap_access_table rpcif_volatile_table = {
	.yes_ranges	= rpcif_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(rpcif_volatile_ranges),
};

static const struct regmap_config rpcif_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.fast_io	= true,
	.max_register	= _PHYINT,
	.volatile_table	= &rpcif_volatile_table,
};


static int wait_msg_xfer_end(struct hf_drvdata *dd)
{
	u32 sts;

	return regmap_read_poll_timeout(dd->regmap, _CMNSR, sts,
					sts & BIT(0), 0,
					USEC_PER_SEC);
}

static void disable_write_protection(void)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);

	/* WPVAL (0:RPC_WP#=H(Protect Disable),1:RPC_WP#=L(Protect Enable)) */
	regmap_update_bits(dd->regmap, _PHYCNT, BIT(0), 0);
	//dev_dbg(dd->dev, "PHYINT = 0x%08x\n", ioread32(dd->io_base + _PHYINT));
}

/* Issue a flash command */
static int issue_command(uint32_t addr, u16 cmd)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	int ret;

	/* Set RPC-IF to manual mode, TEND bit must be 1 */
	ret = wait_msg_xfer_end(dd);
	if (ret) {
		dev_dbg(dd->dev, "Failed to issue a command, errno = %d", ret);
		return ret;
	} else {
		regmap_write(dd->regmap, _PHYCNT, 0x80038263);
		regmap_write(dd->regmap, _CMNCR, 0x81FFF301);
	}

	/*
	 * Create Command-Address bits of Hyperbus
	 * SL26K{L,S} doesn't care address space
	 */
	/* CA47-45 = 000 -> Write, memory space, wrapped Burst */
	regmap_write(dd->regmap, _SMCMR, 0x00000000);
	/* CA15-3(Reserved) = all 0 */
	regmap_write(dd->regmap, _SMOPR, 0x00000000);

	/* There is no latency between Command-Address and Data */
	regmap_write(dd->regmap, _SMDMCR, 0x00000000);

	/* Must be;
	 * bit14-12 HYPE =101:Hyperflash mode
	 * bit8 ADDRE  = 1 : Address DDR transfer
	 * bit0 SPIDRE = 1 : DATA DDR transfer
	 */
	regmap_write(dd->regmap, _SMDRENR, 0x00005101);

	/* Set up data being sent */
	regmap_write(dd->regmap, _SMWDR1, 0x00000000);
	regmap_write(dd->regmap, _SMWDR0, cpu_to_be16(cmd) << 16);
	regmap_write(dd->regmap, _SMADR, addr);

	/*
	 * Configure SoC for transmission
	 */
	/*
	 * bit31-30 CDB[1:0]   =   10 : 4bit width command
	 * bit25-24 ADB[1:0]   =   10 : 4bit width address
	 * bit17-16 SPIDB[1:0] =   10 : 4bit width transfer data
	 * bit15    DME        =    0 : dummy cycle disable
	 * bit14    CDE        =    1 : Command enable
	 * bit12    OCDE       =    1 : Option Command enable
	 * bit11-8  ADE[3:0]   = 0100 : ADR[23:0] output (24 Bit Address)
	 * bit7-4   OPDE[3:0]  = 0000 : Option data disable
	 * bit3-0   SPIDE[3:0] = 1000 : 16bit transfer
	 */
	regmap_write(dd->regmap, _SMENR, 0xA2225408);

	/*
	 * bit2     SPIRE      = 0 : Data read disable
	 * bit1     SPIWE      = 1 : Data write enable
	 * bit0     SPIE       = 1 : SPI transfer start
	 */
	regmap_write(dd->regmap, _SMCR, 0x00000003);

	return wait_msg_xfer_end(dd);
}

static int __read_register_data(uint32_t addr, uint32_t *buf, uint32_t byte_count)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	int ret;

	/*
	 * bit31  CAL         =  1 : PHY calibration
	 * bit1-0 PHYMEM[1:0] = 11 : HyperFlash
	 */
	regmap_write(dd->regmap, _PHYCNT, 0x80000263);
	/*
	 * bit31  MD       =  1 : Manual mode
	 * bit1-0 BSZ[1:0] = 01 : QSPI Flash x 2 or HyperFlash
	 */
	regmap_write(dd->regmap, _CMNCR, 0x81FFF301);

	/*
	 * bit23-21 CMD[7:5] = 100 : CA47-45 = 100 =>
	 *                                      Read/memory space/WrrapedBrst
	 */
	regmap_write(dd->regmap, _SMCMR, 0x00800000);
	/* ByteAddress(8bit) => WordAddress(16bit) */
	regmap_write(dd->regmap, _SMADR, addr>>1);
	/* CA15-3(Reserved) = all 0 */
	regmap_write(dd->regmap, _SMOPR, 0x00000000);
	/* 15 cycle dummy wait */
	regmap_write(dd->regmap, _SMDMCR, 0x0000000E);
	/*
	 * bit8 ADDRE  = 1 : Address DDR transfer
	 * bit0 SPIDRE = 1 : DATA DDR transfer
	 */
	regmap_write(dd->regmap, _SMDRENR, 0x00005101);
	switch (byte_count) {
	case 2: /* Read 2 bytes */
		/* bit3-0   SPIDE[3:0] = 1000 : 16bit transfer*/
		regmap_write(dd->regmap, _SMENR, 0xA222D408);
		break;
	case 4: /* Read 4 bytes */
		/* bit3-0   SPIDE[3:0] = 1100 : 32bit transfer */
		regmap_write(dd->regmap, _SMENR, 0xA222D40C);
		break;
	case 8:
		/*
		* bit31-30 CDB[1:0]   =   10 : 4bit width command
		* bit25-24 ADB[1:0]   =   10 : 4bit width address
		* bit17-16 SPIDB[1:0] =   10 : 4bit width transfer data
		* bit15    DME        =    1 : dummy cycle enable
		* bit14    CDE        =    1 : Command enable
		* bit12    OCDE       =    1 : Option Command enable
		* bit11-8  ADE[3:0]   = 0100 : ADR[23:0]output(24Bit Address)
		* bit7-4   OPDE[3:0]  = 0000 : Option data disable
		* bit3-0   SPIDE[3:0] = 1111 : 64bit transfer
		*/
		regmap_write(dd->regmap, _SMENR, 0xA222D40F);
		break;
	default:
		dev_err(dd->dev, "Byte count is wrong. byte_count = %u\n",
			byte_count);
		break;
	}

	/*
	 * bit2     SPIRE      = 1 : Data read enable
	 * bit1     SPIWE      = 0 : Data write disable
	 * bit0     SPIE       = 1 : SPI transfer start
	 */
	regmap_write(dd->regmap, _SMCR, 0x00000005);

	ret = wait_msg_xfer_end(dd);
	if (ret) {
		pr_debug("[%s]: error, timedout\n", __func__);
		return ret;
	}

	switch (byte_count) {
	case 2:
		/* Read data[31:0] */
		regmap_read(dd->regmap, _SMRDR0, buf);
		*buf = (*buf >> 16) & 0xFFFF;
		break;
	case 4:
		/* Read data[31:0] */
		regmap_read(dd->regmap, _SMRDR0, buf);
		break;
	case 8:
		/*
		 * Big Endian
		 * addr...addr+4
		 * SMRDR0 SMRDR1
		 * 63..32 31...0
		 */
		/*  Read data[63:32] */
		regmap_read(dd->regmap, _SMRDR0, buf+1);
		/* Read data[31:0] */
		regmap_read(dd->regmap, _SMRDR1, buf);
		break;
	}

	return 0;
}

static int __set_address_map_mode_on_read(void)
{
	/* Reset / ASO Exit */
	return issue_command(0x0, 0xF0);
}

static int __reset_device(void)
{
	/* Reset / ASO Exit */
	return issue_command(0x0, 0xF0);
}

static int __read_device_status(uint32_t *buf)
{
	uint32_t status;
	uint32_t ret;
	int iterations = 1000;

	while (iterations-- > 0) {
		ret = issue_command(0x555, 0x70);
		if (ret)
			return -EIO;
		ret = __read_register_data(0x0, &status, 2);
		if (ret)
			return -EIO;
		status = be16_to_cpu(status); /* Buggy ? */
		if (buf != NULL)
			*buf = status;
		if (status & BIT(7))
			break;
		usleep_range(1000, 2000);
	}

	if ((status & BIT(7)) != BIT(7)) {
		__reset_device();
		return -ETIMEDOUT;
	}

	if (status & (BIT(5) | BIT(4))) {
		__reset_device();
		return -EIO;
	}

	return 0;
}

static int __read_device_id(u32 *buf)
{
	u32 tmpbuf[2];
	u32 addr;
	int ret;

	ret = issue_command(0x555, 0xAA);
	if (ret) return -1;
	ret = issue_command(0x2AA, 0x55);
	if (ret) return -1;
	ret = issue_command(0x555, 0x90);
	if (ret) return -1;

	for (addr = 0; addr < 16; addr += 8) {
		ret = __read_register_data(addr, tmpbuf, 8);
		if (addr == 0) {
			*buf = (be16_to_cpu((tmpbuf[0] & 0xFFFF0000) >> 16) << 16) |
				be16_to_cpu(tmpbuf[0] & 0x0000FFFF);
		}
	}

	__set_address_map_mode_on_read();
	return 0;
}

static int __attribute__ ((unused)) send_data_of_buffer(u32 addr)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	u32 tmp;

	/* bit9   RCF         =  1 : Clear the read cache */
	regmap_write(dd->regmap, _DRCR, 0x01FF0301);

	regmap_write(dd->regmap, _PHYCNT, 0x80038277);
	/* Set up the buffer of RPC-IF */

	regmap_write(dd->regmap, _CMNCR, 0x81FF0301);

	/*
	 * SMCMR[23:21] = CMD[7:5] = CA[47:45] = 000
	 *			---> Write/memory space/WrrapedBrst
	 */
	regmap_write(dd->regmap, _SMCMR, 0x00000000);
	regmap_write(dd->regmap, _SMOPR, 0x00000000); /* CA15-3 = 0 */
	regmap_write(dd->regmap, _SMADR, 0x00000000);

	regmap_write(dd->regmap, _SMDRENR,0x00005101);
	/* bit3-0 SPIDE[3:0] = 1100 : 32bit transfer */
	regmap_write(dd->regmap, _SMENR, 0xA222540F);

	regmap_read(dd->regmap, _CMNSR, &tmp);
	regmap_write(dd->regmap, _SMCR, 0x00000003);
	regmap_read(dd->regmap, _CMNSR, &tmp);

	return wait_msg_xfer_end(dd);

	regmap_write(dd->regmap, _PHYCNT, 0x00038273);
	regmap_write(dd->regmap, _DRCR, 0x01FF0301);

	return 0;
}

static int __erase_sector(uint32_t sector_addr)
{
	issue_command(0x555 + (sector_addr >> 1), 0xAA);
	issue_command(0x2AA + (sector_addr >> 1), 0x55);
	issue_command(0x555 + (sector_addr >> 1), 0x80);
	issue_command(0x555 + (sector_addr >> 1), 0xAA);
	issue_command(0x2AA + (sector_addr >> 1), 0x55);
	issue_command(sector_addr >> 1, 0x30);

	return __read_device_status(NULL);
}

static int reset_rpc(struct hf_drvdata *dd)
{
	void __iomem *io_cpg;

	io_cpg = ioremap_nocache(0xe6150000, 0x1000);
	if (io_cpg == NULL) {
		dev_err(dd->dev, "Failed to map CPG controller\n");
		return -ENOMEM;
	}

	iowrite32((u32)(~BIT(17)), io_cpg + 0x0000); /* CPGWPR = 0x0000 */
	iowrite32(BIT(17), io_cpg + 0x0924); /* SRCR9 = 0x0924 */
	/* Wait 20 us */
	usleep_range(20, 1000);
	iowrite32((u32)(~BIT(17)), io_cpg + 0x0000); /* CPGWPR = 0x0000 */
	iowrite32(BIT(17), io_cpg + 0x0964); /* SRSTCLR9 = 0x0964 */
	iounmap(io_cpg);
	/* Wait 40 us */
	usleep_range(40, 1000);

	return 0;
}

static int setup_rpc_clock(struct hf_drvdata *dd, int clkmhz)
{
	void __iomem *io_base;
	u32 value;

	io_base = ioremap_nocache(0xe6150000, 0x1000);
	if (clkmhz == 40) {
		value = 0x00000017; /* For writing */
	} else {
		value = 0x00000013; /* Default is 80 MHz, datasheet 8-67 */
	}
	iowrite32(~value, io_base + 0x900);
	iowrite32(value, io_base + 0x238);

	value = ioread32(io_base + 0x238);
	iounmap(io_base);

	dev_dbg(dd->dev, "RPCCKCR = 0x%08x\n", value);

	return 0;
}

static int check_ssl_delay(struct device *dev)
{
	u32 value;
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);

	regmap_read(dd->regmap, _SSLDR, &value);
	if (value != 0x400) {
		dev_dbg(dev, "Setting up SSL delay.\n");
		regmap_write(dd->regmap, _SSLDR, 0x400);
	}
	regmap_read(dd->regmap, _SSLDR, &value);
	dev_dbg(dev, "SSLDR = 0x%08x\n", value);
	return 0;
}

static int power_on_rpc_module(void)
{
	void __iomem *map;
	u32 value;

	map = ioremap_nocache(0xe61509a0, 4);
	if (!map) {
		pr_info("Failed to map module control registers.\n");
		return -ENOMEM;
	}

	value = ioread32(map); /* Read MSTPSR9 */
	iounmap(map);
	if (value & BIT(17)) {
		pr_err("RPC moudle is OFF.\n");
		/* FIXME:
		 * RPC-IF module normally have been turned on by TF-A.
		 * Add the code to turn on RPC moudle jusr in case.
		 */
		return 1;
	} else {
		pr_info("RPC moudle is ON.\n");
		return 0;
	}
}

static void __attribute__ ((unused)) dump_flash_memory(void)
{
	void __iomem *io_base;
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	void __iomem *base = dd->io_base;

	iowrite32(0x00078263, base + _PHYCNT);
	iowrite32(0x80078263, base + _PHYCNT);
	iowrite32(0x01FFF301, base + _CMNCR);
	iowrite32(0x001F0100, base + _DRCR);
	iowrite32(0x00A00000, base + _DRCMR);
	iowrite32(0xA222D400, base + _DRENR);
	iowrite32(0x0000000E, base + _DRDMCR);
	iowrite32(0x00005101, base + _DRDRENR);
	iowrite32(0x21511144, base + _OFFSET1);
	iowrite32(0x07070002, base + _PHYINT);

	io_base = ioremap_nocache(0x08000000, PAGE_SIZE);
	if (io_base == NULL) {
		pr_err("Failed to map the flash memory region.\n");
		return;
	}

	pr_debug("0x0000: 0x%08x 0x%08x 0x%08x 0x%08x\n",
		ioread32(io_base + 0), ioread32(io_base + 4),
		ioread32(io_base + 8), ioread32(io_base + 12));
	iounmap(io_base);
}

static int __attribute__ ((unused)) __send_data(u32 addr, u32 data)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	//u32 tmp;

	regmap_write(dd->regmap, _PHYCNT, 0x80038263);
	regmap_write(dd->regmap, _CMNCR, 0x81FF0301);

	regmap_write(dd->regmap, _SMCMR, 0x00000000);
	regmap_write(dd->regmap, _SMOPR, 0x00000000); /* CA15-3 = 0 */
	regmap_write(dd->regmap, _SMADR, addr);

	regmap_write(dd->regmap, _SMWDR0, data);

	regmap_write(dd->regmap, _SMDRENR,0x00005101);
	/* bit3-0 SPIDE[3:0] = 1100 : 32bit transfer */
	regmap_write(dd->regmap, _SMENR, 0xA222540C);

	regmap_write(dd->regmap, _SMCR, 0x00000003);

	return wait_msg_xfer_end(dd);
}

static int program_word(u32 addr, u32 data)
{
	int ret;

	disable_write_protection();

	issue_command(0x555 + ((addr & 0xFFFC0000) >> 1), 0xAA);
	issue_command(0x2AA + ((addr & 0xFFFC0000) >> 1), 0x55);
	issue_command(0x555 + ((addr & 0xFFFC0000) >> 1), 0xA0);

	ret = __send_data(addr>>1, data);
	if (ret) {
		pr_debug("[%s] Failed to send data.\n", __func__);
		return ret;
	}

	return __read_device_status(NULL);
}

static int __attribute__ ((unused)) drvctrl_init(void)
{
	void __iomem *io_base;
	int i;

	io_base = ioremap_nocache(0xe6060000, 0x1000);
	if (io_base == NULL) {
		pr_err("Failed to map ...\n");
		return -ENOMEM;
	}

#if 0 /* Don't care */
	iowrite32(~0x33333333, io_base + 0x0000);
	iowrite32(0x33333333, io_base + 0x0300);
	iowrite32(~0x33333337, io_base + 0x0000);
	iowrite32(0x33333337, io_base + 0x0304);
#endif
	for (i = 0x300; i < 0x308; i += 4) { /* DRV range: 0x300 ~ 0x360 */
		pr_debug("DRVCTRL%d: 0x%08x\n", i >> 2, ioread32(io_base + i));
	}

#if 0 /* Don't care */
	iowrite32(~0x00000FFF, io_base + 0x0000);
	iowrite32(0x00000FFF, io_base + 0x0400);
	iowrite32(~0x00005FBF, io_base + 0x0000);
	iowrite32(0x00005FBF, io_base + 0x0440);
#endif
	pr_debug("PUEN0: 0x%08x (Expected: 0x00000FFF)\n",
		ioread32(io_base + 0x400));
	pr_debug("PUD0: 0x%08x (Expected: 0x00005FBF)\n",
		ioread32(io_base + 0x440));

	iounmap(io_base);

	return 0;
}

static ssize_t status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	u32 status = 0xba;
	__read_device_status(&status);
	return sprintf(buf, "0x%02x\n", status);
}

static ssize_t bootparam_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t written;
	u32 value;

	__set_address_map_mode_on_read();

	if (__read_register_data(0x0, &value, 4))
		return sprintf(buf, "INVALID\n");
	written = sprintf(buf, "0x0000_0000: 0x%08x\n", value);
	__read_register_data(0x0d54, &value, 4);
	written += sprintf(buf+written, "0x0000_0D54: 0x%08x\n", value);
	__read_register_data(0x0e64, &value, 4);
	written += sprintf(buf+written, "0x0000_0e64: 0x%08x\n", value);
	__read_register_data(0x1154, &value, 4);
	written += sprintf(buf+written, "0x0000_1154: 0x%08x\n", value);
	__read_register_data(0x1264, &value, 4);
	written += sprintf(buf+written, "0x0000_1264: 0x%08x\n", value);

	return written;
}

static ssize_t ipl_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	u32 val;
	int ret;
	__set_address_map_mode_on_read();
	ret = __read_register_data(0x0, &val, 4);
	if (ret)
		return sprintf(buf, "INVALID\n");
	if (val & 0x1)
		return sprintf(buf, "B\n");
	return sprintf(buf, "A\n");
}

static ssize_t ipl_store(struct device *dev,
				struct device_attribute *addr,
				const char *buf,
				size_t count)
{
	int ret;
	u32 faked_addr = 0x04000000;
	__set_address_map_mode_on_read();
	ret = __erase_sector(0x0 + faked_addr);
	if (ret)
		goto error;
	if (!strncmp(buf, "A", 1)) {
		ret = program_word(0x0 + faked_addr, 0x4);
		if (ret)
			goto error;
	} else if (!strncmp(buf, "B", 1)) {
		ret = program_word(0x0 + faked_addr, 0x5);
		if (ret)
			goto error;
	} else {
		dev_dbg(dev, "Invalid arguments: A or B\n");
		return count;
	}
	ret = program_word(0x0d54 + faked_addr, 0xe6304000);
	if (ret)
		goto error;
	ret = program_word(0x0e64 + faked_addr, 0x0000aa00);
	if (ret)
		goto error;
	ret = program_word(0x1154 + faked_addr, 0xe6304000);
	if (ret)
		goto error;
	ret = program_word(0x1264 + faked_addr, 0x0000aa00);
	if (ret)
		goto error;

	return count;

error:
	dev_err(dev, "Failed to update the bootparam (%d).\n",
		ret);
	return count;
}

#ifdef DEBUG
static u32 __sector_addr = 0x300000;
static ssize_t debug_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	ssize_t written = 0;
	u32 dyb;
	u32 val;

	written = sprintf(buf, "Sector Address: 0x%08x\n", __sector_addr);

	/* read dyb */
	issue_command(0x555, 0xAA);
	issue_command(0x2AA, 0x55);
	issue_command(0x555, 0xE0);
	__read_register_data(__sector_addr, &dyb, 4);
	issue_command(0x0, 0xF0);
	written += sprintf(buf+written, "DYB = 0x%08x (Expected = 0xff)\n", dyb);

	/* read SA Protection status */
	issue_command(0x555, 0xAA);
	issue_command(0x2AA, 0x55);
	issue_command(0x555, 0xC0);
	issue_command(0x0, 0x60);
	__read_register_data(__sector_addr, &val, 4);
	issue_command(0x0, 0xF0);
	written += sprintf(buf+written, "SA Protection Status = 0x%08x"
			" (Expected = b xxxx xxxx xxxx x111)\n", val);

	/* Evaluate erase status */
	issue_command(0x555, 0xD0);
	__read_device_status(&val);
	written += sprintf(buf+written, "Device Status = 0x%08x"
			" (Expected = b 1xxx xxx1)\n", val);

	__set_address_map_mode_on_read();
	__read_register_data(__sector_addr, &val, 4);
	written += sprintf(buf+written, "0x%08x: 0x%08x\n", __sector_addr, val);

	return written;
}

static ssize_t debug_store(struct device *dev,
				struct device_attribute *addr,
				const char *buf,
				size_t count)
{
	if (!strncmp(buf, "erase", 5)) {
		pr_debug("Erasing 0x%08x\n", __sector_addr);
		__erase_sector(__sector_addr);
	} else if (!strncmp(buf, "write", 5)) {
		pr_debug("Programming a word at 0x%08x\n", __sector_addr);
		program_word(__sector_addr, 0xfaeaae5c);
	} else if (sscanf(buf, "%x", &__sector_addr) == -1)
		return -EINVAL;

	return count;
}
#endif

static DEVICE_ATTR_RO(bootparam);
static DEVICE_ATTR(ipl, 0660, ipl_show, ipl_store);
static DEVICE_ATTR_RO(status);
#ifdef DEBUG
static DEVICE_ATTR(debug, 0660, debug_show, debug_store);
#endif

static struct attribute *hflash_attrs[] = {
	&dev_attr_bootparam.attr,
	&dev_attr_ipl.attr,
	&dev_attr_status.attr,
#ifdef DEBUG
	&dev_attr_debug.attr,
#endif
	NULL
};

static const struct attribute_group hflash_attr_group = {
	.attrs = hflash_attrs,
	.name = "hflash",
};

static int __init hflash_init(void)
{
	struct hf_drvdata *dd;
	struct resource *res;
	struct device *dev;
	u32 device_id = 0;
	int ret;

	/* Hardware configuration */
#if 0
	drvctrl_init(); /* Hardware configuration might be OK. */
#endif

	dev = root_device_register("badrpcif");
	if (IS_ERR(dev)) {
		return PTR_ERR(dev);
	}
	hflash_dev = dev;
	dev_dbg(dev, "Initializing RPC-IF ...\n");

	dd = devm_kzalloc(hflash_dev, sizeof(struct hf_drvdata), GFP_KERNEL);
	if (!dd) {
		return -ENOMEM;
	}
	dd->dev = dev;

	res = request_mem_region(RPCIF_BASE, RPCIF_SIZE, "rpc-if");
	if (res == NULL) {
		pr_err("Failed to aquire the memory region.\n");
		return -ENODEV;
	}

	dd->rpc_res = res;
	dd->io_base = ioremap_nocache(res->start, resource_size(res));
	if (!dd->io_base) {
		return -ENOMEM;
	}
	dev_info(dev, "RPF-IF mapped to 0x%p\n", dd->io_base);

	dev_set_drvdata(hflash_dev, dd);

	dd->regmap = devm_regmap_init_mmio(hflash_dev, dd->io_base,
						&rpcif_regmap_config);
	if (IS_ERR(dd->regmap)) {
		dev_err(dev, "Failed to init regmap\n");
		return PTR_ERR(dd->regmap);
	}

	res = request_mem_region(RPCIF_FLASH_BASE, RPCIF_FLASH_SIZE, "rpc-mmio");
	if (res == NULL) {
		pr_err("Failed to aquire the memory region.\n");
		return -ENODEV;
	}
	dd->flash_res = res;
	dd->flash_base = ioremap_nocache(res->start, resource_size(res));
	if (!dd->flash_base) {
		return -ENOMEM;
	}
	dev_info(dev, "HyperFlash mapped to 0x%p\n", dd->io_base);


	power_on_rpc_module();
	setup_rpc_clock(dd, 80); /* 80 MHz */
	reset_rpc(dd);
	check_ssl_delay(hflash_dev);

	/* Dump RPC-IF status */
#if 0
	regmap_update_bits(dd->regmap, _PHYINT, BIT(1), 0);
	regmap_write(dd->regmap, _OFFSET1, 0x31511144);
	regmap_write(dd->regmap, _OFFSET2, 0x431);
#endif
	dev_dbg(dev, "    PHYINT = 0x%08x\n", ioread32(dd->io_base + _PHYINT));
	dev_dbg(dev, "PHYOFFSET1 = 0x%08x\n", ioread32(dd->io_base + _OFFSET1));
	dev_dbg(dev, "PHYOFFSET2 = 0x%08x\n", ioread32(dd->io_base + _OFFSET2));

	ret = sysfs_create_group(&dev->kobj, &hflash_attr_group);
	if (ret) {
		if (dd->io_base) {
			iounmap(dd->io_base);
		}
		release_mem_region(RPCIF_BASE, RPCIF_SIZE);
		root_device_unregister(hflash_dev);
		return ret;
	}

	__read_device_id(&device_id);
	dev_info(dev, "DEVICE ID: 0x%08x\n", device_id);

	return 0;
}

static void __exit hflash_exit(void)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);

	dev_dbg(dd->dev, "Removing the device ...\n");
	if (dd->io_base)
		iounmap(dd->io_base);
	if (dd->flash_base)
		iounmap(dd->flash_base);
	sysfs_remove_group(&hflash_dev->kobj, &hflash_attr_group);
	release_mem_region(dd->flash_res->start, resource_size(dd->flash_res));
	release_mem_region(dd->rpc_res->start, resource_size(dd->rpc_res));
	if (hflash_dev)
		root_device_unregister(hflash_dev);
	pr_debug("Bye!\n");
}

module_init(hflash_init);
module_exit(hflash_exit);

MODULE_DESCRIPTION("Handling the boot parameters of R-Car 3rd Gen.");
MODULE_AUTHOR("Djang Lyu");
MODULE_LICENSE("GPL v2");
