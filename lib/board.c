/*
 * Copyright (C) 2005 Texas Instruments.
 *
 * (C) Copyright 2004
 * Jian Zhang, Texas Instruments, jzhang@ti.com.
 *
 * (C) Copyright 2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <part.h>
#include <fat.h>
#include <mmc.h>
#include <version.h>
#include <asm/io.h>
#include <asm/arch/bits.h>
#include <asm/arch/clocks.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch-omap4/mux.h>


#ifdef CFG_PRINTF
int print_info(void)
{
	printf ("\n\n"X_LOADER_VERSION" ("
		__DATE__ " - " __TIME__ ")\n");
	return 0;
}
#endif
typedef int (init_fnc_t) (void);

init_fnc_t *init_sequence[] = {
	cpu_init,		/* basic cpu dependent setup */
	board_init,		/* basic board dependent setup */
#ifdef CFG_PRINTF
	serial_init,		/* serial communications setup */
	print_info,
#endif
	//nand_init,		/* board specific nand init */
	NULL,
};

#ifdef CFG_CMD_FAT
extern char * strcpy(char * dest,const char *src);
#else
char * strcpy(char * dest,const char *src)
{
	 char *tmp = dest;

	 while ((*dest++ = *src++) != '\0')
	         /* nothing */;
	 return tmp;
}
#endif

#ifdef CFG_CMD_MMC
extern block_dev_desc_t *mmc_get_dev(int dev);
int mmc_read_bootloader(int dev)
{
	unsigned char ret = 0;
	unsigned long offset = CFG_LOADADDR;

	ret = mmc_init(dev);
	if (ret != 0){
		printf("\n MMC init failed \n");
		return -1;
	}

#ifdef CFG_CMD_FAT
	long size;
	block_dev_desc_t *dev_desc = NULL;

	if (fat_boot()) {
		dev_desc = mmc_get_dev(dev);
		fat_register_device(dev_desc, 1);
		size = file_fat_read("u-boot.bin", (unsigned char *)offset, 0);
		if (size == -1)
			return -1;
	} else {
		/* FIXME: OMAP4 specific */
		 mmc_read(dev, 0x200, (unsigned char *)CFG_LOADADDR,
							0x00160000);
	}
#endif
	return 0;
}
#endif

/*
 * OMAP On-die temperature sensor check.
 * If the current temperature value is
 * greater than T_SHUT_HOT stop boot
 */

void omap_temp_sensor_check(void)
{
	u32 temp;

	/* Set the counter to 1 ms */
	sr32(CORE_BANDGAP_COUNTER, BGAP_COUNTER_START_BIT,
			BGAP_COUNTER_NUM_BITS, BGAP_COUNTER_VALUE);

	/* Enable continuous mode. */
	sr32(CORE_BANDGAP_CTRL, BGAP_SINGLE_MODE_START_BIT,
			BGAP_SINGLE_MODE_NUM_BITS, BGAP_CONTINUOUS_MODE);

	/* Wait till the first conversion is done wait for at least 1ms */
	spin_delay(20000);

	/* Read the temperature adc_value */
	temp = readl(CORE_TEMP_SENSOR);
	temp = temp & BGAP_TEMP_SENSOR_DTEMP_MASK;

	/* If the samples are untrimmed divide by 1.2 */
	if (readl(STD_FUSE_OPP_BGAP) == 0)
		temp = temp * 5 / 6;

	/*
	 * Compare with TSHUT high temperature. If high ask the
	 * user to shut down and restart after sometime else
	 * Disable continuous mode.
	 */
	if (temp < TSHUT_HIGH_ADC_CODE) {
		/* Disable contiuous mode */
		sr32(CORE_BANDGAP_CTRL, BGAP_SINGLE_MODE_START_BIT,
			BGAP_SINGLE_MODE_NUM_BITS, ~BGAP_CONTINUOUS_MODE);
	} else {
		printf("OMAP chip temperature is too high!!!\n");
		printf("Please power off and try booting after sometime\n");

		/* Bypass MPU, CORE, IVA, PER, ABE, USB DPLLs */
		sr32(CM_CLKMODE_DPLL_MPU, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_MPU, LDELAY);

		sr32(CM_CLKMODE_DPLL_CORE, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_CORE, LDELAY);

		sr32(CM_CLKMODE_DPLL_IVA, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_IVA, LDELAY);

		sr32(CM_CLKMODE_DPLL_PER, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_PER, LDELAY);

		sr32(CM_CLKMODE_DPLL_ABE, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_ABE, LDELAY);

		sr32(CM_CLKMODE_DPLL_USB, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_USB, LDELAY);

		while (1);
	}
}


extern int do_load_serial_bin(ulong offset, int baudrate);

#define __raw_readl(a)	(*(volatile unsigned int *)(a))
// memory test
//#define OMAP4_MEM_TESET
#if (defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP4430)) && defined(OMAP4_MEM_TESET)
#define TRUE (1)
#define FALSE (0)
#define LARGE_SIZE_THRESHOLD 0x40000
#define mem_rw_test(__offset, __size, __error_exit) omap4_mem_rw_test(__offset, __size, __error_exit, emif_num, interleaving_size)
#define SPLIT_LINE "===================="
#define DDR_START_ADDR 0x80000000
int omap4_mem_rw_test(unsigned int offset, unsigned int size, int error_exit, unsigned int emif_num, unsigned int interleaving_size)
{
	unsigned int pattern, index, addr, start_addr, end_addr, cent_size, reg_value;
	int result = TRUE;

	start_addr = DDR_START_ADDR; /*the start address is fixed at 0x80000000*/
	end_addr = start_addr+offset+size-1;

	cent_size = size/4/80+((size%(4*80) == 0)? 0 : 1);

	for (pattern = 0xffff0000, index = 0, addr = start_addr+offset; addr <= end_addr; pattern+=0xffff0001, index++, addr+=4) {
		if (index == 0)
			printf(" [0x%08X]~[0x%08X] (%d bytes)\n", addr, end_addr, size);

		// write the test pattern to the register then read it back
		__raw_writel(pattern, addr);
		reg_value = __raw_readl(addr);

		if (reg_value != pattern) {
			unsigned int tmp, data_err;
			unsigned long led_data = 0x01000000;
			data_err = 0;

			printf("\n Error @[0x%08X]! W:0x%08X, R:0x%08X\n", addr, pattern, reg_value);

			// double check the data of the register
			__raw_writel(pattern, addr);
			reg_value = __raw_readl(addr);
			__raw_writel(pattern, addr);
			tmp = __raw_readl(addr);
			if (tmp != reg_value)
				printf(" --[0x%08X] R1:0x%08X R2:0x%08X  (Warring: not consistent!)\n", addr, tmp, reg_value);

			// check the data bit by bit
			for (tmp = 0x00000001; tmp != 0x00000000; tmp*=2) {
				__raw_writel(tmp, addr);
				reg_value = __raw_readl(addr);
				data_err |= (tmp^reg_value);
			}
			for (tmp = 0xfffffffe; tmp != 0xffffffff; tmp=tmp*2+1) {
				__raw_writel(tmp, addr);
				reg_value = __raw_readl(addr);
				data_err |= (tmp^reg_value);
			}

			// show the result of anlysis
			if (data_err) {
				printf(" --Found error-bit");
				if (emif_num > 1 && interleaving_size > 0)
					printf(" @EMIF%d", (addr/interleaving_size)%emif_num+1);
				printf(": ");
				int bit;
				for (bit = 0, tmp = 0x00000001; tmp != 0x00000000; tmp*=2, bit++)
					if ((tmp&data_err) != 0) printf("%d,",bit);
				printf("\n");
			} else
				printf(" --No error-bit found! Maybe address problem?\n");

			__raw_writel(0x01030100, 0x4a100154);
			__raw_writel(0xfeffffff, 0x4805b134);
			while(1){
				__raw_writel(led_data, 0x4805b13c);
				for(tmp=0;tmp<2000;tmp++)
					spin_delay(50000);
				led_data ^= 0x01000000;
			}
			// set the result flag to false
			result = FALSE;

			// exit when error occurs?
			if (error_exit)
				break;
			else if (cent_size > LARGE_SIZE_THRESHOLD)  {
				tmp = index / cent_size + ((index % cent_size == 0) ? 0 : 1);
				while (tmp-- > 0)  printf(".");
			}
		}
		if (cent_size > LARGE_SIZE_THRESHOLD && index % cent_size == 0) printf(".");
	}
	printf("\n");

	return result;
}

void print_lable(char* str)
{
	printf("\n%s\n%s:\n%s\n", SPLIT_LINE, str, SPLIT_LINE);
}
#endif

void start_armboot (void)
{
	init_fnc_t **init_fnc_ptr;
	uchar *buf;
	char boot_dev_name[8];
	u32 si_type, omap4_rev;

	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		if ((*init_fnc_ptr)() != 0) {
			hang ();
		}
	}

	omap4_rev = omap_revision();
	si_type = omap4_silicon_type();

	if ((omap4_rev >= OMAP4460_ES1_0) &&
			(omap4_rev <= OMAP4460_MAX_REVISION)) {
		omap_temp_sensor_check();
		if (si_type == PROD_ID_1_SILICON_TYPE_HIGH_PERF)
			printf("OMAP4460: 1.5 GHz capable SOM\n");
		else if (si_type == PROD_ID_1_SILICON_TYPE_STD_PERF)
			printf("OMAP4460: 1.2 GHz capable SOM\n");
	} else if ((omap4_rev >= OMAP4470_ES1_0) &&
			(omap4_rev <= OMAP4470_MAX_REVISION)) {
		omap_temp_sensor_check();
		writel(((TSHUT_HIGH_ADC_CODE << 16) | TSHUT_COLD_ADC_CODE),
				CORE_TSHUT_THRESHOLD);
		MV1(WK(CONTROL_SPARE_RW) , (M1));

		if (si_type == PROD_ID_1_SILICON_TYPE_HIGH_PERF)
			printf("OMAP4470: 1.5 GHz capable SOM\n");
		else if (si_type == PROD_ID_1_SILICON_TYPE_STD_PERF)
			printf("OMAP4470: 1.3 GHz capable SOM\n");
	}

#ifdef START_LOADB_DOWNLOAD
	strcpy(boot_dev_name, "UART");
	do_load_serial_bin (CFG_LOADADDR, 115200);
#else
	buf = (uchar *) CFG_LOADADDR;

// memory test
#if (defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP4430)) && defined(OMAP4_MEM_TESET)
	do {
		unsigned int reg_value;

		/*read DMM_LISA_MAP_3*/
		reg_value = __raw_readl(0x4e00004c);
		unsigned int mem_size = 16 << ((reg_value>>20)&0x07) << 20;
		unsigned int emif_num = (((reg_value>>8)&0x03) == 3)? 2 : (((reg_value>>8)&0x03) == 0)? 0 : 1;
		unsigned int interleaving_size = (((reg_value>>18)&0x03) == 0)? 0 : 128 << (((reg_value>>18)&0x03)-1);
		/*read EMIF_SDRAM_CONFIG*/
		reg_value = __raw_readl(0x4c000008);
		unsigned int page_size = 256 << (reg_value&0x07);
		unsigned int cs_num = ((reg_value&0x08) == 0)? 1 : 2;
		unsigned int bank_num = 1 << ((reg_value>>4)&0x07);
		unsigned int row_num = ((reg_value>>7)&0x07) + 9;
		unsigned int bit_mode = (((reg_value>>14)&0x03) == 0)? 32 : 16;
		unsigned int row_size = 1 << row_num;

		/* show the basic information from OMAP4's registers */
		print_lable("MEMORY INFO");
		printf(" Total size: %d MiB", mem_size >> 20);
		if (emif_num != 0) {
			printf(", (%d x %d x %d x %d x %d x %d) bits\n",
				row_size, bank_num, page_size, bit_mode, cs_num, emif_num);
		} else
			printf("\n");
		printf(" Type: LPDDR2-S%d\n", ((((reg_value>>29)&0x03) == 4)? 4 : 2));
		printf(" Row : %d\n", row_num);
		printf(" Bank: %d\n", bank_num);
		printf(" Page: %d\n", page_size);
		printf(" Bit : %d\n", bit_mode);
		printf(" CS  : %d\n", cs_num);
		printf(" EMIF: %d\n", emif_num);
		if (emif_num > 1) printf(" Interleaving: %d bytes\n", interleaving_size);

		/* show register values from LPDDR */
		// Device info:  MR0 = 0x18, DAI[0]=0x00(completed), DI[1]=0x00(S2 or S4 SDRAM), DNVI[2]=0x00(unsupported), RZQI[4:3]=0x03(completed)
		// Refresh rate: MR4 = 0x01(4x) or 0x02(2x) or 0x03(1x) or 0x05(0.25x) or 0x06(0.25x with de-rate SDRAM AC timing)
		// Manufacturer: MR5 = 0x03(elpida) or 0x05(nanya) or 0x06(hynix) or 0xff(micro)
		// Type:         MR8 = 0x14, TYPE[1:0]=0x00(S4 SDRAM), Density[5:2]=0x05(2gb), I/O width[7:6]=0x00(x32)
		print_lable("MEMORY REG");
		int emif_id, cs_id, reg_id;
		int regs[4] = {0, 4, 5, 8};
		unsigned int tmp_reg;
		for (emif_id = 0; emif_id < emif_num; emif_id++) {
			for (cs_id = 0; cs_id < cs_num; cs_id++) {
				printf(" EMIF%d, CS%d\n", emif_id+1, cs_id);
				tmp_reg = 0x4c000050 + emif_id * 0x1000000;
				for (reg_id = 0; reg_id < sizeof(regs)/sizeof(int); reg_id++) {
					__raw_writel(regs[reg_id] + cs_id * 0x80000000, tmp_reg);
					reg_value = __raw_readl(tmp_reg - 0x10);
					printf("  MR%d: 0x%02X\n", regs[reg_id], reg_value & 0xff);
				}
				printf("\n");
			}
		}

		/* proceed a single R/W test to check out data lines */
		print_lable("SINGLE R/W");
		int pattern_id;
		unsigned int patterns[2] = {0xffffffff, 0x00000000};
		for (emif_id = 0; emif_id < emif_num && interleaving_size > 0; emif_id++) {
			tmp_reg = DDR_START_ADDR + interleaving_size * emif_id;
			for (pattern_id = 0; pattern_id < sizeof(patterns)/sizeof(unsigned int); pattern_id++) {
				__raw_writel(patterns[pattern_id], tmp_reg);
				reg_value = __raw_readl(tmp_reg);
				printf("  [0x%08X] W:0x%08X, R:0x%08X\n\n", tmp_reg, patterns[pattern_id], reg_value);
			}
		}
#if 1//SEQUENTIAL R/W
		/* proceed a sequential R/W test to check out addr&data lines */
		// We need to disable the sequnetial r/w test to reduce the image size
		print_lable("SEQUENTIAL R/W");
		int seq_test_passed = TRUE;
		unsigned int offset;
		unsigned int block_size = row_size*page_size*bit_mode/8;
		unsigned int test_size = block_size/bank_num/cs_num/emif_num;
		for (offset = block_size - test_size; offset < (mem_size - test_size); offset += (block_size - test_size))
			seq_test_passed &= mem_rw_test(offset, test_size, TRUE);
#endif
#if 0//FULL R/W
		if (seq_test_passed != FALSE) {
			/* proceed a full R/W test */
			print_lable("FULL R/W");
			mem_rw_test(0, mem_size, TRUE);
		}
#endif
	} while (0);
#endif

	switch (get_boot_device()) {
	case 0x03:
		strcpy(boot_dev_name, "ONENAND");
#if defined(CFG_ONENAND)
		for (i = ONENAND_START_BLOCK; i < ONENAND_END_BLOCK; i++) {
			if (!onenand_read_block(buf, i))
				buf += ONENAND_BLOCK_SIZE;
			else
				goto error;
		}
#endif
		break;
	case 0x02:
	default:
		strcpy(boot_dev_name, "NAND");
#if defined(CFG_NAND)
		for (i = NAND_UBOOT_START; i < NAND_UBOOT_END;
				i+= NAND_BLOCK_SIZE) {
			if (!nand_read_block(buf, i))
				buf += NAND_BLOCK_SIZE; /* advance buf ptr */
		}
#endif
		break;
	case 0x05:
		strcpy(boot_dev_name, "MMC/SD1");
#if defined(CONFIG_MMC)
		if (mmc_read_bootloader(0) != 0)
			goto error;
#endif
		break;
	case 0x06:
		strcpy(boot_dev_name, "EMMC");
#if defined(CONFIG_MMC)
		if (mmc_read_bootloader(1) != 0)
			goto error;
#endif
		break;
	};
#endif
	/* go run U-Boot and never return */
	printf("Starting OS Bootloader from %s ...\n", boot_dev_name);
	((init_fnc_t *)CFG_LOADADDR)();

	/* should never come here */
#if defined(CFG_ONENAND) || defined(CONFIG_MMC)
error:
#endif
	printf("Could not read bootloader!\n");
	hang();
}

void hang (void)
{
	/* call board specific hang function */
	board_hang();
	
	/* if board_hang() returns, hange here */
	printf("X-Loader hangs\n");
	for (;;);
}
