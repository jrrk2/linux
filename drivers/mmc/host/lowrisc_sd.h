/*
 *  LowRISC SD Host Controller Interface - register definitions
 *
 *  Ported for Sonata (RV32, TL-UL, 32-bit register access)
 *
 *  Copyright (C) 2018 LowRISC CIC
 *  Copyright (C) 2024 Jonathan Kimmitt
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef _LOWRISC_SD_H_
#define _LOWRISC_SD_H_

/* Write register offsets (index * 4) - decoded by addr[5:2] */
#define SD_ALIGN_REG      (0 * 4)
#define SD_CLK_DIV_REG    (1 * 4)
#define SD_ARG_REG        (2 * 4)
#define SD_CMD_REG        (3 * 4)
#define SD_SETTING_REG    (4 * 4)
#define SD_START_REG      (5 * 4)
#define SD_RESET_REG      (6 * 4)
#define SD_BLKCNT_REG     (7 * 4)
#define SD_BLKSIZE_REG    (8 * 4)
#define SD_TIMEOUT_REG    (9 * 4)
#define SD_IRQ_EN_REG     (11 * 4)
#define SD_IRQ_CLR_REG    (12 * 4)  /* Write-1-to-clear IRQ status */

/* Read register offsets (index * 4) - decoded by addr[6:2] */
#define SD_RESP0          (0 * 4)
#define SD_RESP1          (1 * 4)
#define SD_RESP2          (2 * 4)
#define SD_RESP3          (3 * 4)
#define SD_WAIT_RESP      (4 * 4)
#define SD_STATUS_RESP    (5 * 4)
#define SD_PACKET_RESP0   (6 * 4)
#define SD_PACKET_RESP1   (7 * 4)
#define SD_DATA_WAIT_RESP (8 * 4)
#define SD_TRANS_CNT_RESP (9 * 4)
#define SD_DETECT_RESP    (12 * 4)
#define SD_XFR_ADDR_RESP  (13 * 4)
#define SD_IRQ_STAT_RESP  (14 * 4)
/* Readback of write registers at offsets 16..27 */
#define SD_ALIGN_RESP     (16 * 4)
#define SD_CLK_DIV_RESP   (17 * 4)
#define SD_ARG_RESP       (18 * 4)
#define SD_CMD_RESP       (19 * 4)
#define SD_SETTING_RESP   (20 * 4)
#define SD_START_RESP     (21 * 4)
#define SD_RESET_RESP     (22 * 4)
#define SD_BLKCNT_RESP    (23 * 4)
#define SD_BLKSIZE_RESP   (24 * 4)
#define SD_TIMEOUT_RESP   (25 * 4)
#define SD_IRQ_EN_RESP    (27 * 4)
#define SD_VERSION_RESP   (15 * 4)  /* RTL version register */

/* Data buffer offset: 4KB at 0x1000 */
#define SD_DATA_BUF_OFFSET  0x1000

/* IRQ status/enable bits */
#define SD_CARD_RESP_END       0x01
#define SD_CARD_RW_END         0x02
#define SD_CARD_CARD_REMOVED   0x04
#define SD_CARD_CARD_INSERTED  0x08

struct lowrisc_sd_host {
	struct platform_device *pdev;
	struct mmc_host *mmc;
	spinlock_t lock;
	void __iomem *ioaddr;
	int int_en;
	int width_setting;
};

#endif /* _LOWRISC_SD_H_ */
