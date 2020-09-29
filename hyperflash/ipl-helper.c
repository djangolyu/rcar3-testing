// SPDX-License-Identifier: GPL-2.0
/*
 * To help select IPL
 *
 * Simple module, not using platform class.
 * It's very hacky code. Never do it.
 *
 * Base on:
 *	- Renesas' flash write; github:renesas/flash_writer
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


#if 0
static void wait_for_end_of_tx(void)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	int i;

	for (i = 0; i < 10000; i++) {
		if (ioread32(dd->io_base + _CMNSR) & BIT(0))
			break;
	}
	pr_debug("\t\tRPC-IF TX count = %d\n", i);
	if (i == 10000)
		pr_warn("Waiting for end of TX might be failed.\n");
}
#else
static int wait_msg_xfer_end(struct hf_drvdata *dd)
{
	u32 sts;

	return regmap_read_poll_timeout(dd->regmap, _CMNSR, sts,
					sts & BIT(0), 0,
					USEC_PER_SEC);
}
#endif

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

/*
 * cnt: count in byte
 */
static uint32_t read_hyperflash(uint32_t addr, uint32_t *buf, uint32_t cnt)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	void __iomem *base = dd->io_base;

	iowrite32(0x00038263, base + _PHYCNT);
	iowrite32(0x80038263, base + _PHYCNT);
	iowrite32(0x81FFF301, base + _CMNCR);
	iowrite32(0x00800000, base + _SMCMR);
	iowrite32( (addr>>1), base + _SMADR);
	iowrite32(0x00000000, base + _SMOPR);
	iowrite32(0x0000000E, base + _SMDMCR);
	iowrite32(0x00005101, base + _SMDRENR);

	switch (cnt) {
	case 2:
		iowrite32(0xA222D408, base + _SMENR);
		break;
	case 4:
		iowrite32(0xA222D40C, base + _SMENR);
		break;
	case 8:
		iowrite32(0xA222D40F, base + _SMENR);
		break;
	default:
		return -EINVAL;
		break;
	}

	iowrite32(0x00000005, base + _SMCR);

	//wait_for_end_of_tx();
	wait_msg_xfer_end(dd);

	if (cnt == 8)
		buf[1] = ioread32(base + _SMRDR0);
	buf[0] = ioread32(base + _SMRDR1);

	return buf[0];
}

/* Issue a flash command */
/* data 16 bits, hyperbus: write to register space, CA[47:45] = b010 */
static void issue_command(uint32_t addr, u16 cmd)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);

	regmap_write(dd->regmap, _PHYCNT, 0x80038263);
	regmap_write(dd->regmap, _CMNCR, 0x81FFF301);

	regmap_write(dd->regmap, _SMCMR, 0x00400000); /* set CA47-45 */
	regmap_write(dd->regmap, _SMDRENR, 0x00005101);
	regmap_write(dd->regmap, _SMENR, 0xA2225408);

	regmap_write(dd->regmap, _SMOPR, 0x00000000);
	regmap_write(dd->regmap, _SMWDR1, 0x00000000);
	regmap_write(dd->regmap, _SMWDR0, cpu_to_be16(cmd) << 16);
	if (addr == 0x555 || addr == 0x2AA)
		regmap_write(dd->regmap, _SMADR, addr);
	else
		regmap_write(dd->regmap, _SMADR, addr);

	regmap_write(dd->regmap, _SMCR, 0x00000003);
	pr_debug("\t\t[ADR:0x%08x, CMD:0x%08x] CMNSR = 0x%08x",
			//addr, cpu_to_be32(cmd),
			addr, cpu_to_be16(cmd) << 16,
			ioread32(dd->io_base + _CMNSR));

	wait_msg_xfer_end(dd);
}

/* addr is word */
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
	//*buf = be32_to_cpu(value);
	*buf = value;
	return 0;
}

/* hyperbus, read reg: CA[47:45] = b100 */
static uint16_t read_2bytes_mem(uint32_t addr)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	u32 value;

	regmap_write(dd->regmap, _PHYCNT, 0x00038263);
	regmap_write(dd->regmap, _PHYCNT, 0x80038263);
	regmap_write(dd->regmap, _CMNCR, 0x81FFF301);
	regmap_write(dd->regmap, _SMCMR, 0x00800000);
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
	//return be32_to_cpu(value) & 0xFFFF;
	return (value >> 16) & 0xFFFF;
}

/* hyperbus, read reg: CA[47:45] = b110 */
static u16 read_2bytes_reg(uint32_t addr)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	u32 value;

	regmap_write(dd->regmap, _PHYCNT, 0x00038263);
	regmap_write(dd->regmap, _PHYCNT, 0x80038263);
	regmap_write(dd->regmap, _CMNCR, 0x81FFF301);
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
	//return be32_to_cpu(value) & 0xFFFF;
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

static uint32_t __attribute__ ((unused)) read_flash_id(void)
{
	uint32_t buf[2];

	issue_command(HYPER_FL_UNLOCK1_ADD, HYPER_FL_UNLOCK1_DATA);
	issue_command(HYPER_FL_UNLOCK2_ADD, HYPER_FL_UNLOCK2_DATA);
	issue_command(HYPER_FL_UNLOCK3_ADD, HYPER_FL_ID_ENTRY_COM);

	read_hyperflash(0, buf, 8);
	pr_debug("0x%08x, 0x%08x\n", buf[1], buf[0]);

	issue_command(0x0, HYPER_FL_RESET_COM);

	return buf[0];
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

	pr_info("RPCCKCR = 0x%08x\n", value);

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

static u16 read_status_mine(void)
{
	issue_command(0x555, 0x70);
	return read_2bytes_reg(0x0);
}

static uint32_t read_status_renesas(void)
{
	uint32_t buf[2];
	uint32_t status;

	issue_command(0x555, 0x70);

	status = read_8bytes(0x0, buf);

	return (((status & 0xFF000000) >> 8 ) |
		((status & 0x00FF0000) << 8 ) |
		((status & 0x0000FF00) >> 8 ) |
		((status & 0x000000FF) << 8 )) & 0x0000FFFF;
}

/* Send data to a flash chip. data is stored in the hardware buf */
static void _send_data_from_hwbuf(void)
{
	struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);
	void __iomem *base = dd->io_base;
	void __iomem *buf;
	uint32_t addr = 0x0 >> 1;
	int i;

	buf = ioremap_nocache(RPCIF_WRBUF_BASE, RPCIF_WRBUF_SIZE);
	if (buf == NULL) {
		pr_err("Failed to map the hardware buffer.\n");
		return;
	}

	iowrite32(0x011F0301, base + _DRCR); /* Clear read cache */
	iowrite32(0x80038277, base + _PHYCNT);

	for (i = 0; i < RPCIF_WRBUF_SIZE; i += 4) {
		iowrite32(0x5a5a5a5a, buf + i);
	}

	iowrite32(0x81FFF301, base + _CMNCR);
	iowrite32(0x00000000, base + _SMCMR);
	iowrite32(addr, base + _SMADR);
	iowrite32(0x00000000, base + _SMOPR);
	iowrite32(0x00005101, base + _SMDRENR);
	iowrite32(0xA222540F, base + _SMENR);
	iowrite32(0x00000003, base + _SMCR);

	//wait_for_end_of_tx();
	wait_msg_xfer_end(dd);

	iowrite32(0x00038273, base + _PHYCNT);
	iowrite32(0x011F0301, base + _DRCR);

	iounmap(buf);
}

static void write_hwbuf(void)
{
	issue_command(0x00000555, 0xAA);
	issue_command(0x000002AA, 0x55);
	issue_command(0x00000555, 0xA0);

	issue_command(0x0, 0x51);
	//_send_data_from_hwbuf();
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
		if ((read_status_renesas() & BIT(7)) == BIT(7))
			break;
	}
	pr_debug("loop count i = %d\n", i);
	if (i == 10) {
		pr_warn("Reading status bits might be failed. status = 0x%08x\n",
			read_status_renesas());
	}
}

static void dump_first_16bytes(void)
{
	uint32_t buf[4];
	int i;
	//read_hyperflash(0x0, buf, 4);
	//pr_debug("0x0000: 0x%08x\n", buf[0]);

#if 0
	issue_command(0x00000000, &buf[0]);
	read_hyperflash(0x0, buf, 4);
	pr_debug("0x0000: 0x%08x\n", buf[0]);
#endif
	//issue_command(0x0, HYPER_FL_RESET_COM);
	//issue_command(0x0, 0xFF000000);
	//issue_command(0x555, 0x71000000);
	//issue_command(0x00000000, &buf[0]);

	read_8bytes(0x0, buf);
	pr_debug("8:0x0000: 0x%08x\n", buf[0]);
	read_8bytes(0xd54, buf);
	pr_debug("8:0x0D54: 0x%08x\n", buf[0]);

	issue_command(0x0, HYPER_FL_RESET_COM);
	pr_debug("STATUS: 0x%08x\n", read_status_mine());

	read_8bytes(0xe64, buf);
	pr_debug("0x0E64: 0x%08x\n", buf[0]);

	issue_command(0x555, 0x98);
	for (i = 0; i < 20; i++ ) {
		pr_debug("%03x: 0x%04x\n", i, read_2bytes_mem(i));
	}
	issue_command(0x0, HYPER_FL_RESET_COM);
	read_8bytes(0x0, buf);
	pr_debug("8:0x0000 = 0x%08x, 0x%08x\n", buf[1], buf[0]);

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
	//issue_command(0x0, HYPER_FL_RESET_COM); /* It works */
	issue_command(sector_addr>>1, 0x30); /* Why ?  Address ??? */
	//issue_command(0x555, 0x10); /           It's working, but ... */

	//msleep(1000);
	i = 0;
	while (1) {
		status = read_status_mine();
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


static void __attribute__ ((unused)) write_test(void)
{
	//struct hf_drvdata *dd = (struct hf_drvdata*)dev_get_drvdata(hflash_dev);

	//disable_write_protection();
	erase_sector(0x00000);
	//program_word(0x0, 0x0);
	pr_debug("status = 0x%08x\n", read_status_mine());
#if 0
	setup_rpc_clock(dd, 40);
	write_hwbuf();
	setup_rpc_clock(dd, 80);
#endif
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
	return sprintf(buf, "0x%02x", read_status_mine());
}

static ssize_t bootparam_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	u32 value;
	issue_command(0x555, 0x98);
	if (read_4bytes(0x0, &value))
		return sprintf(buf, "INVALID");
	//return sprintf(buf, "0x%08x", cpu_to_be32(value));
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
		erase_sector(0x0);
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

static void testing(struct hf_drvdata *dd)
{
	//ret = read_flash_id();

	dump_first_16bytes();
	dump_flash_memory();

#if 0
	write_test();

	pr_debug("\n=== after writing ===\n");
	//ret = read_flash_id();
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
#if 1
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

#if 0
	ret = check_mem_region(RPCIF_BASE, RPCIF_SIZE);
	if (ret) {
		pr_info("Someone already took the memory region");
		return ret;
	}
#endif

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


	//pr_debug("PHYINT = 0x%08x\n", ioread32(dd->io_base + _PHYINT));
	//iowrite32(0x07070002, dd->io_base + _PHYINT);
	power_on_rpc_module();
	//setup_rpc_clock(dd, 80);
	setup_rpc_clock(dd, 40); /* for debugging */
	reset_rpc();
	check_ssl_delay(hflash_dev);

	/* Dump RPC-IF status */
	dev_dbg(dev, "    PHYINT = 0x%08x\n", ioread32(dd->io_base + _PHYINT));
	dev_dbg(dev, "PHYOFFSET1 = 0x%08x\n", ioread32(dd->io_base + _OFFSET1));
	dev_dbg(dev, "PHYOFFSET2 = 0x%08x\n", ioread32(dd->io_base + _OFFSET2));

	//issue_command(0x0,HYPER_FL_RESET_COM);

	ret = sysfs_create_group(&dev->kobj, &hflash_attr_group);
	if (ret) {
		if (dd->io_base) {
			iounmap(dd->io_base);
		}
		release_mem_region(RPCIF_BASE, RPCIF_SIZE);
		root_device_unregister(hflash_dev);
		return ret;
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
