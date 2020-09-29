// SPDX-License-Identifier: GPL-2.0
/*
 * To help select IPL
 *
 * Simple module, not using platform class.
 * It's very hacky code. Never do it.
 *
 * Copyright (C) Djang Lyu 2020 <lyu2816@gmail.com>
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <asm/io.h>

#define RPCIF_BASE	0xEE200000UL
#define RPCIF_SIZE	0x200UL

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

#define	HYPER_FL_UNLOCK1_ADD		0x555
#define	HYPER_FL_UNLOCK1_DATA		0xAA
#define	HYPER_FL_UNLOCK2_ADD		0x2AA
#define	HYPER_FL_UNLOCK2_DATA		0x55
#define	HYPER_FL_UNLOCK3_ADD		0x555

#define	HYPER_FL_RESET_COM		(0xF0)
#define	HYPER_FL_RD_STATUS_COM		(0x70)
#define	HYPER_FL_ID_ENTRY_COM		(0x90)
#define	HYPER_FL_WORD_PROGRAM_COM	(0xA0)
#define	HYPER_FL_ERASE_1ST_COM		(0x80)
#define	HYPER_FL_SECTOR_ERASE_COM	(0x30)
#define	HYPER_FL_CHIP_ERASE_COM		(0x10)

/* Driver data for Hyperbus flash memory */
struct hf_drvdata {
	struct device *dev;
	void __iomem *io_base;
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
	void __iomem *base = dd->io_base;
	u32 value;

	value = ioread32(base + _PHYINT);
	value &= ~BIT(1);
	iowrite32(value, base + _PHYINT);
	pr_debug("PHYINT = 0x%08x\n", ioread32(base + _PHYINT));
}

static int setup_manual_mode(struct hf_drvdata *dd)
{
	regmap_write(dd->regmap, _PHYCNT, 0x80038263);
	regmap_write(dd->regmap, _CMNCR, 0x81FFF301);

	if (wait_msg_xfer_end(dd)) {
		dev_warn(dd->dev, "Trasmission Timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

/* Issue a flash command */
/* data 16 bits, hyperbus: write to register space, CA[47:45] = b010 */
static int issue_command(uint32_t addr, u16 cmd)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);

#if 1
	//regmap_write(dd->regmap, _PHYCNT, 0x80038263);
	regmap_write(dd->regmap, _PHYCNT, 0x80000263);
	regmap_write(dd->regmap, _CMNCR, 0x81FFF301);
#else
	setup_manual_mode(dd);
#endif

	//regmap_write(dd->regmap, _SMCMR, 0x00400000); /* set CA47-45 */
	/* CA47-45 = 000 -> Write, memory space, wrapped Burst */
	regmap_write(dd->regmap, _SMCMR, 0x00000000); /* set CA47-45 */
	/*
	 * CA15-3(Reserved) = all 0
	 */
	regmap_write(dd->regmap, _SMOPR, 0x00000000);
	regmap_write(dd->regmap, _SMADR, addr);
	/*
	 * bit14-12 HYPE =101:Hyperflash mode
	 * bit8 ADDRE  = 1 : Address DDR transfer
	 * bit0 SPIDRE = 1 : DATA DDR transfer
	 */
	regmap_write(dd->regmap, _SMDRENR, 0x00005101);
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

	//regmap_write(dd->regmap, _SMWDR1, 0x00000000);
	regmap_write(dd->regmap, _SMWDR0, cpu_to_be16(cmd) << 16);

	/*
	 * bit2     SPIRE      = 0 : Data read disable
	 * bit1     SPIWE      = 1 : Data write enable
	 * bit0     SPIE       = 1 : SPI transfer start
	 */
	regmap_write(dd->regmap, _SMCR, 0x00000003);
	pr_debug("\t\t[ADR:0x%08x, CMD:0x%08x] CMNSR = 0x%08x",
			//addr, cpu_to_be32(cmd),
			addr, cpu_to_be16(cmd) << 16,
			ioread32(dd->io_base + _CMNSR));

	return wait_msg_xfer_end(dd);
}

/* Data were get from the memory space. addr is word, */
static int read_4bytes(u32 addr, u32 *buf)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	u32 value;
	int ret;

	regmap_write(dd->regmap, _PHYCNT, 0x00038263);
	regmap_write(dd->regmap, _PHYCNT, 0x80038263);
	regmap_write(dd->regmap, _CMNCR, 0x81FFF301);
	regmap_write(dd->regmap, _SMCMR, 0x00800000);
	/* word addressing, a word is 16 bits */
	regmap_write(dd->regmap, _SMADR, addr>>1); /* Word addressing */
	regmap_write(dd->regmap, _SMOPR, 0x00000000);
	regmap_write(dd->regmap, _SMDMCR, 0x0000000E);
	regmap_write(dd->regmap, _SMDRENR, 0x00005101);
	regmap_write(dd->regmap, _SMENR, 0xA222D40C);
	regmap_write(dd->regmap, _SMCR, 0x00000005);

	ret = wait_msg_xfer_end(dd);
	if (ret)
		return ret;

	regmap_read(dd->regmap, _SMRDR0, &value);
	*buf = value;
	return 0;
}

/* hyperbus, read reg: CA[47:45] = b100 */
static uint16_t read_2bytes_mem(uint32_t addr)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	u32 value;

	setup_manual_mode(dd);
	regmap_write(dd->regmap, _SMCMR, 0x00800000);
	/* word addressing, a word is 16 bits */
	regmap_write(dd->regmap, _SMADR, addr>>1);
	regmap_write(dd->regmap, _SMOPR, 0x00000000);
	regmap_write(dd->regmap, _SMDMCR, 0x0000000E);
	regmap_write(dd->regmap, _SMDRENR, 0x00005101);
	regmap_write(dd->regmap, _SMENR, 0xA222D408);
	regmap_write(dd->regmap, _SMCR, 0x00000005);

	wait_msg_xfer_end(dd);

	/*
	 * Big Endian
	 * addr...addr+4
	 * SMRDR0 SMRDR1
	 * 63..32 31...0
	 */
	regmap_read(dd->regmap, _SMRDR0, &value);
	//return be32_to_cpu(value) & 0xFFFF;
	return (value >> 16) & 0xFFFF;
}

/* hyperbus, read data from register space: CA[47:45] = b110 */
static u16 read_2bytes_reg(uint32_t addr)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	u32 value;

#if 0
	regmap_write(dd->regmap, _PHYCNT, 0x80038263);
	regmap_write(dd->regmap, _CMNCR, 0x81FFF301);
#else
	setup_manual_mode(dd);
#endif

	regmap_write(dd->regmap, _SMCMR, 0x00c00000);
	/* word addressing, a word is 16 bits */
	regmap_write(dd->regmap, _SMADR, addr);
	regmap_write(dd->regmap, _SMOPR, 0x00000000);
	regmap_write(dd->regmap, _SMDMCR, 0x0000000E);
	regmap_write(dd->regmap, _SMDRENR, 0x00005101);
	regmap_write(dd->regmap, _SMENR, 0xA222D408);
	regmap_write(dd->regmap, _SMCR, 0x00000005);

	wait_msg_xfer_end(dd);

	/*
	 * Big Endian
	 * addr...addr+4
	 * SMRDR0 SMRDR1
	 * 63..32 31...0
	 */
	regmap_read(dd->regmap, _SMRDR0, &value);
	return be16_to_cpu((value >> 16) & 0xFFFF);
}

static uint32_t read_8bytes(uint32_t addr, uint32_t *buf)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	void __iomem *base = dd->io_base;

	iowrite32(0x00038263, base + _PHYCNT);
	iowrite32(0x80038263, base + _PHYCNT);
	iowrite32(0x81FFF301, base + _CMNCR);
	iowrite32(0x00800000, base + _SMCMR);
	iowrite32((addr>>1), base + _SMADR);
	iowrite32(0x00000000, base + _SMOPR);
	iowrite32(0x0000000E, base + _SMDMCR);
	iowrite32(0x00005101, base + _SMDRENR);
	iowrite32(0xA222D40F, base + _SMENR);
	iowrite32(0x00000005, base + _SMCR);

	//wait_for_end_of_tx();
	wait_msg_xfer_end(dd);

	buf[1] = ioread32(base + _SMRDR0);
	buf[0] = ioread32(base + _SMRDR1);

	//pr_debug("%s: buf[1] = 0x%08x, buf[0] = 0x%08x\n", __func__, buf[1], buf[0]);

	return buf[0];
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
	/*
	 * ByteAddress(8bit) => WordAddress(16bit)
	 */
	regmap_write(dd->regmap, _SMADR, addr>>1);
	/*
	 * CA15-3(Reserved) = all 0
	 */
	regmap_write(dd->regmap, _SMOPR, 0x00000000);
	/*
	 *                           15 cycle dummy wait
	 */
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
		dev_err(dd->dev, "Byte count is wrong. byte_count = %u\n", byte_count);
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
		return -1;
	}


	/*
	 * Big Endian
	 * addr...addr+4
	 * SMRDR0 SMRDR1
	 * 63..32 31...0
	 */
	if (byte_count == 8) {
		/*  Read data[63:32] */
		regmap_read(dd->regmap, _SMRDR0, buf+1);
	}
	/* Read data[31:0] */
	regmap_read(dd->regmap, _SMRDR1, buf);
	return 0;
}

static uint32_t __reset_to_read_mode(void)
{
	/* Reset / ASO Exit */
	return issue_command(0x0, 0xF0);
}

static int __read_device_status(uint32_t *read_status)
{
	uint32_t read_data[2];
	uint32_t ret;

	/* 1st command write */
	ret = issue_command(0x555, 0x70);

	if (ret) {
		pr_debug("!!!HMM!!!\n");
		return -1;
	}

	ret = __read_register_data(0x0, read_data, 8);

	if (ret) {
		pr_debug("!!!HMM!!!\n");
		return -1;

	}
	*read_status =
	(((read_data[0] & 0xFF000000U)>>8))
	| ((read_data[0] & 0x00FF0000U)<<8)
	| ((read_data[0] & 0x0000FF00U)>>8)
	| ((read_data[0] & 0x000000FFU)<<8);

	*read_status = (*read_status & 0x0000FFFFU);
	pr_debug("STATUS: 0x%08x\n", *read_status);

	if ((*read_status & BIT(7)) != 0U) {
		//ret = FL_DEVICE_READY;
		ret = 1;
	} else {
		//ret = FL_DEVICE_BUSY;
		ret = 0;
	}

	return ret;
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
		pr_debug("BUF: 0x%08x, 0x%08x\n", tmpbuf[1], tmpbuf[0]);
#if 0
		if (addr == 0) {
			*buf =
			(((tmpbuf[0]&0xFF000000U)>>8) |
			((tmpbuf[0]&0x00FF0000U)<<8) |
			((tmpbuf[0]&0x0000FF00U)>>8) |
			((tmpbuf[0]&0x000000FFU)<<8));

		}
#else
		if (addr == 0) {
			*buf = (be16_to_cpu((tmpbuf[0] & 0xFFFF0000) >> 16) << 16) |
				be16_to_cpu(tmpbuf[0] & 0x0000FFFF);
		}
#endif
	}

	__reset_to_read_mode();
	return 0;
}

static void __erase_sector(uint32_t sector_addr)
{
	int i;
	u32 status;

	/* Clear status */
	//issue_command(0x555, 0x71);

	issue_command(0x555, 0xAA);
	issue_command(0x2AA, 0x55);
	issue_command(0x555, 0x80);
	issue_command(0x555, 0xAA);
	issue_command(0x2AA, 0x55);
	//issue_command((0x555 + (sector_addr>>1)), 0x30); /* Problem!!! Why? Address ??? */
	issue_command(sector_addr >> 1, 0x30); /* Problem!!! Why? Address ??? */

	i = 0;
	while (1) {
		if (__read_device_status(&status) == 1) {
			pr_debug("Done: status =  0x%x\n", status);
			break;
		}
		pr_debug("status =  0x%x\n", status);
		msleep(50);
		if (i++ == 10) {
			issue_command(0x0, HYPER_FL_RESET_COM);
			break;
		}
	}
	pr_debug("End of waiting for the status bit. count = %d\n", i);
}

static int reset_rpc(void)
{
	void __iomem *io_cpg;

	io_cpg = ioremap_nocache(0xe6150000, 0x1000);
	if (io_cpg == NULL) {
		pr_err("Failed to map CPG controller\n");
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
		value = 0x00000013; /* Default is 80 MHz, 8-67 */
	}
	iowrite32(~value, io_base + 0x900);
	iowrite32(value, io_base + 0x238);

	value = ioread32(io_base + 0x238);
	iounmap(io_base);

	pr_debug("RPCCKCR = 0x%08x\n", value);

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
		pr_info("RPC moudle is OFF.\n");
		return 1;
	} else {
		pr_info("RPC moudle is ON.\n");
		return 0;
	}
}

static u16 read_status(void)
{
	issue_command(0x555, 0x70);
	return read_2bytes_reg(0x0);
}

static void _send_word(uint32_t addr, uint32_t data)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	void __iomem *base = dd->io_base;

	iowrite32(0x80038263, base + _PHYCNT);
	iowrite32(0x81FFF301, base + _CMNCR);
	iowrite32(0x00005101, base + _SMDRENR);
	iowrite32(0xA222540C, base + _SMENR);

	iowrite32(0x00000000, base + _SMCMR); /* Write data to memory space */
	iowrite32(0x00000000, base + _SMOPR); /* CA15-3 = 0 */

	iowrite32(addr, base + _SMADR);
	iowrite32(data, base + _SMWDR0);

	iowrite32(0x00000003, base + _SMCR);

	wait_msg_xfer_end(dd);
}

static void __attribute__ ((unused)) program_word(uint32_t addr, uint32_t data)
{
	int i;

	disable_write_protection();

	issue_command(HYPER_FL_UNLOCK1_ADD, HYPER_FL_UNLOCK1_DATA);
	issue_command(HYPER_FL_UNLOCK2_ADD, HYPER_FL_UNLOCK2_DATA);
	issue_command(HYPER_FL_UNLOCK3_ADD, HYPER_FL_WORD_PROGRAM_COM);

	_send_word(addr, data);

	pr_debug("Waiting ....\n");
	for (i = 0; i < 10; i++) {
		msleep(100);
		if ((read_status() & BIT(7)) == BIT(7))
			break;
	}
	pr_debug("loop count i = %d\n", i);
	if (i == 10) {
		pr_warn("Reading status bits might be failed. status = 0x%08x\n",
			read_status());
	}
}


static void erase_sector(uint32_t sector_addr)
{
	int i;
	u32 status;

	/* Clear status */
	issue_command(0x555, 0x71);
	issue_command(0x555, 0xAA);
	issue_command(0x2AA, 0x55);
	issue_command(0x555, 0x80);
	issue_command(0x555, 0xAA);
	issue_command(0x2AA, 0x55);
	issue_command(sector_addr>>1, 0x30); /* Problem!!! Why? Address ??? */

	i = 0;
	while (1) {
		status = read_status();
		pr_debug("status =  0x%x\n", status);
		if (status & BIT(7))
			break;
		msleep(50);
		if (i++ == 10) {
			issue_command(0x0, HYPER_FL_RESET_COM);
			break;
		}
	}
	pr_debug("End of waiting for the status bit. count = %d\n", i);
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

	iowrite32(~0x33333333, io_base + 0x0000);
	iowrite32(0x33333333, io_base + 0x0300);
	iowrite32(~0x33333337, io_base + 0x0000);
	iowrite32(0x33333337, io_base + 0x0304);
	for (i = 0x300; i < 0x360; i += 4) {
		pr_debug("DRVCTRL%d: 0x%08x\n", i >> 2, ioread32(io_base + i));
	}

	//iowrite32(~0x00000FFF, io_base + 0x0000);
	//iowrite32(0x00000FFF, io_base + 0x0400);
	pr_debug("PUEN0: 0x%08x (Expected: 0x00000FFF)\n",
		ioread32(io_base + 0x400));
	pr_debug("PUD0: 0x%08x (Expected: 0x00005FBF)\n",
		ioread32(io_base + 0x440));

	iounmap(io_base);

	return 0;
}

/* External Bus Controller for EX-Bus (LBSC) */
/*
[   61.835033] CS0CTRL: 0x00008020
[   61.838180] CS1CTRL: 0x00000020
[   61.841325] CSWCR0: 0xff70ff70
[   61.844372] CSWCR1: 0xff70ff70
[   61.847431] CSPWCR0: 0x00000000
[   61.850577] CSPWCR1: 0x00000000
[   61.853724] EXWTSYNC: 0x00000000
[   61.856957] CS1GDST: 0x00000000
*/
static int __attribute__ ((unused)) lbsc_init(void)
{
	void __iomem *virt;
	
	virt = ioremap_nocache(0xee220000, 0x1000);
	if (virt == NULL) {
		pr_err("Faile to map the LBSC region.\n");
		return -ENOMEM;
	}

	iowrite32(0x00000020, virt + 0x0200);
	iowrite32(0x2A103320, virt + 0x0230);
	iowrite32(0x2A103320, virt + 0x0234);

	pr_debug("CS0CTRL: 0x%08x\n", ioread32(virt + 0x0200));
	pr_debug("CS1CTRL: 0x%08x\n", ioread32(virt + 0x0204));
	pr_debug("CSWCR0: 0x%08x\n", ioread32(virt + 0x0230));
	pr_debug("CSWCR1: 0x%08x\n", ioread32(virt + 0x0234));
	pr_debug("CSPWCR0: 0x%08x\n", ioread32(virt + 0x0280));
	pr_debug("CSPWCR1: 0x%08x\n", ioread32(virt + 0x0284));
	pr_debug("EXWTSYNC: 0x%08x\n", ioread32(virt + 0x02A0));
	pr_debug("CS1GDST: 0x%08x\n", ioread32(virt + 0x02C0));
}

static ssize_t status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	u32 status;
	__read_device_status(&status);
	return sprintf(buf, "0x%02x", status);
	//return sprintf(buf, "0x%02x", read_status());
}

static ssize_t bootparam_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	u32 value;
	if (read_4bytes(0x0, &value))
		return sprintf(buf, "INVALID");
	return sprintf(buf, "0x%08x", value);
}

static ssize_t ipl_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	u32 val;
	int ret;
	ret = read_4bytes(0x0, &val);
	if (ret)
		return sprintf(buf, "INVALID");
	if (val & 0x1)
		return sprintf(buf, "B");
	return sprintf(buf, "A");
}

static ssize_t ipl_store(struct device *dev,
				struct device_attribute *addr,
				const char *buf,
				size_t count)
{
	if (!strncmp(buf, "A", 1)) {
		program_word(0x0, 0x0);
		//erase_sector(0x0);
		/* write_word(0x0, 0x0000) */
	} else if (!strncmp(buf, "B", 1)) {
		__erase_sector(0x0);
		/* write_word(0x0, 0x0001) */
	} else {
		dev_dbg(dev, "Invalid arguments: A or B\n");
	}
	return count;
}

static DEVICE_ATTR_RO(bootparam);
static DEVICE_ATTR(ipl, 0660, ipl_show, ipl_store);
static DEVICE_ATTR_RO(status);

static struct attribute *hflash_attrs[] = {
	&dev_attr_bootparam.attr,
	&dev_attr_ipl.attr,
	&dev_attr_status.attr,
	NULL
};

static const struct attribute_group hflash_attr_group = {
	.attrs = hflash_attrs,
	.name = "hflash",
};

static void dump_first_16bytes(void)
{
	int i;
	u32 val;

	issue_command(0x0, HYPER_FL_RESET_COM);
	pr_debug("STATUS: 0x%08x\n", read_status());

	read_4bytes(0x0, &val);
	pr_debug("0x0000_0000: 0x%08x (Expected: 0x0000_00001)\n", val);
	read_4bytes(0xd54, &val);
	pr_debug("0x0000_0D54: 0x%08x (Expected: 0xE630_40000)\n", val);
	read_4bytes(0xe64, &val);
	pr_debug("0x0000_0E64: 0x%08x (Expected: 0x0000_aa00)\n", val);

	issue_command(0x555, 0x98); /* CFI Enter */
	for (i = 0; i < 40; i += 2 ) {
		pr_debug("%04x: 0x%04x\n", i, read_2bytes_mem(i));
	}
	issue_command(0x0, HYPER_FL_RESET_COM); /* ASO Exit */
	read_4bytes(0x0, &val);
	pr_debug("bootparam = 0x%08x\n", val);

}

static void testing(struct hf_drvdata *dd)
{
	dump_first_16bytes();

#if 0
	//disable_write_protection();
	erase_sector(0x00000);
	//program_word(0x0, 0x0);
	pr_debug("status = 0x%08x\n", read_status());

	pr_debug("\n=== after writing ===\n");
	dump_first_16bytes();
#endif
}

static int __init hflash_init(void)
{
	struct hf_drvdata *dd;
	struct resource *hf_res;
	struct device *dev;
	int ret;

	/* Hardware configuration */
#if 0
	drvctrl_init(); /* Hardware configuration might be OK. */
	//lbsc_init();
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

	hf_res = request_mem_region(RPCIF_BASE, RPCIF_SIZE, "rpc-if");
	if (hf_res == NULL) {
		pr_err("Failt to aquire the memory region.\n");
		return -ENODEV;
	}


	dd->io_base = ioremap_nocache(RPCIF_BASE, PAGE_SIZE);
	if (!dd->io_base) {
		return -ENOMEM;
	}
	pr_debug("RPF-IF mapped to 0x%p\n", dd->io_base);

	dev_set_drvdata(hflash_dev, dd);

	dd->regmap = devm_regmap_init_mmio(hflash_dev, dd->io_base,
						&rpcif_regmap_config);
	if (IS_ERR(dd->regmap)) {
		dev_err(dev, "Failed to init regmap\n");
		return PTR_ERR(dd->regmap);
	}

	power_on_rpc_module();
	setup_rpc_clock(dd, 80);
	//setup_rpc_clock(dd, 40); /* for debugging */
	reset_rpc();
	//check_ssl_delay(hflash_dev);

	/* Dump RPC-IF status */
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

	{
		u32 device_id;
		__read_device_id(&device_id);
		pr_info("DEVICE ID: 0x%08x\n", device_id);
	}

	testing(dd);

	return 0;
}

static void __exit hflash_exit(void)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);

	pr_debug("Removing the device ...\n");
	if (dd->io_base) {
		iounmap(dd->io_base);
	}
	sysfs_remove_group(&hflash_dev->kobj, &hflash_attr_group);
	release_mem_region(RPCIF_BASE, RPCIF_SIZE);
	root_device_unregister(hflash_dev);
	pr_info("Bye!\n");
}

module_init(hflash_init);
module_exit(hflash_exit);

MODULE_DESCRIPTION("Handling the boot parameters");
MODULE_AUTHOR("Djang Lyu");
MODULE_LICENSE("GPL v2");
