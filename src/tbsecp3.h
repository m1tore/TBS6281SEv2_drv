// SPDX-License-Identifier: GPL-2.0-or-later
// TBS ECP3 FPGA PCIeブリッジ共通ヘッダー

#ifndef _TBSECP3_H_
#define _TBSECP3_H_

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#include "cxd2878.h"

// GPIOレジスタベースアドレスとピン番号計算マクロ
#define TBSECP3_GPIO_BASE	0x0000
#define TBSECP3_GPIO_PIN(_bank, _pin)	(((_bank) << 5) + (_pin))

// I2Cレジスタオフセット定義
#define TBSECP3_I2C_BASE(_n)	((_n) < 4 ? (0x4000 + 0x1000 * (_n)) : (0x4800 + 0x1000 * ((_n) - 4)))
#define TBSECP3_I2C_STAT	0x0000
#define TBSECP3_I2C_CTRL	0x0000
#define TBSECP3_I2C_DATA	0x0004
#define TBSECP3_I2C_BAUD	0x0008

// DMAコントローラレジスタ定義
#define TBSECP3_DMA_BASE(_n)	((_n) < 8 ? (((_n) < 4) ? (0x8000 + 0x1000 * (_n)) : (0x8800 + 0x1000 * ((_n) - 4))) : \
				(((_n) < 12) ? (0x8400 + 0x1000 * ((_n) - 8)) : (0x8C00 + 0x1000 * ((_n) - 12))))
#define TBSECP3_DMA_STAT	0x0000
#define TBSECP3_DMA_EN		0x0000
#define TBSECP3_DMA_TSIZE	0x0004
#define TBSECP3_DMA_ADDRH	0x0008
#define TBSECP3_DMA_ADDRL	0x000c
#define TBSECP3_DMA_BSIZE	0x0010

// 割り込みコントローラレジスタ定義
#define TBSECP3_INT_BASE	0xc000
#define TBSECP3_INT_STAT	0x0000
#define TBSECP3_INT_EN		0x0004
#define TBSECP3_I2C_IE(_n)	(0x0008 + 4 * (_n))
#define TBSECP3_DMA_IE(_n)	(0x0018 + 4 * (_n))
#define TBSECP3_I2C_IF(_n)	(0x0001 << (_n))
#define TBSECP3_DMA_IF(_n)	(0x0010 << (_n))

// PCIベンダーIDおよびデバイスID定義
#define TBSECP3_VID		0x544d
#define TBSECP3_PID		0x6178

// サポート対象ボード識別番号定義
#define TBSECP3_BOARD_TBS6281SE	0

// チューナーアダプタ数およびI2Cバス数の上限
#define TBSECP3_MAX_ADAPTERS	2
#define TBSECP3_MAX_I2C_BUS	4

// GPIO出力アクティブ論理定義
#define TBSECP3_GPIODEF_NONE	0
#define TBSECP3_GPIODEF_HIGH	1
#define TBSECP3_GPIODEF_LOW	2

// DMA循環バッファ面数定義
#define TBSECP3_DMA_BUFFERS	16
#define TBSECP3_DMA_PRE_BUFFERS	2

// TSストリームリングバッファの割り当てバイトサイズ
#define TBS6281_RINGBUFFER_SIZE	(16 * 1024 * 1024)
// TSストリームリングバッファのビットマスク定義
#define TBS6281_RINGBUFFER_MASK	(TBS6281_RINGBUFFER_SIZE - 1)

struct tbsecp3_dev;

// 個別GPIOピンの属性情報構造体
struct tbsecp3_gpio_pin {
	u8 lvl;
	u8 nr;
};

// アダプタ関連GPIO構成構造体
struct tbsecp3_gpio_config {
	struct tbsecp3_gpio_pin demod_reset;
};

// アダプタ単位のハードウェア構成構造体
struct tbsecp3_adap_config {
	u32 ts_in;
	u8 i2c_bus_nr;
	struct tbsecp3_gpio_config gpio;
};

// ボード全体の仕様定義構造体
struct tbsecp3_board {
	u16 board_id;
	const char *name;
	int adapters;
	u32 i2c_speed;
	struct tbsecp3_adap_config adap_config[TBSECP3_MAX_ADAPTERS];
};

// I2Cバスマネージャ構造体
struct tbsecp3_i2c {
	struct tbsecp3_dev *dev;
	u32 base;
	struct i2c_adapter i2c_adap;
	struct i2c_client i2c_client;
	struct mutex lock;
	wait_queue_head_t wq;
	bool done;
};

// DMAチャンネル管理構造体
struct tbsecp3_dma_channel {
	u32 base;
	dma_addr_t dma_addr;
	u32 page_size;
	u32 buffer_size;
	u32 buffer_pkts;
	u8 *buf[TBSECP3_DMA_BUFFERS + 1];
	u8 offset;
	u8 cnt;
	u8 next_buffer;
};

// TSストリーム受信用リングバッファ構造体
struct tbs6281_ringbuffer {
	u8 *data;
	size_t size;
	size_t head;
	size_t tail;
	spinlock_t lock;
	wait_queue_head_t wq;
	bool running;
};

// チューナーアダプタ管理構造体
struct tbsecp3_adapter {
	int nr;
	const struct tbsecp3_adap_config *cfg;
	struct tbsecp3_dev *dev;
	struct tbsecp3_i2c *i2c;

	// 復調器およびチューナー制御ハンドル
	struct cxd2878_dev *fe;

	// DMA転送制御フィールド
	spinlock_t adap_lock;
	struct tasklet_struct tasklet;
	struct tbsecp3_dma_channel dma;

	// キャラクタデバイスおよびリングバッファ制御フィールド
	struct cdev cdev;
	struct device *chrdev_device;
	dev_t devno;
	int minor_num;
	bool cdev_registered;
	struct mutex lock;
	bool is_open;
	bool streaming;
	struct tbs6281_ringbuffer ringbuf;
};

// PCIeカードデバイス全体管理構造体
struct tbsecp3_dev {
	const struct tbsecp3_board *info;
	struct pci_dev *pci_dev;
	void __iomem *lmmio;
	bool msi;
	struct tbsecp3_adapter adapter[TBSECP3_MAX_ADAPTERS];
	struct tbsecp3_i2c i2c_bus[TBSECP3_MAX_I2C_BUS];
};

// FPGAレジスタ読み取りマクロ
#define tbs_read(_b, _o)	readl(dev->lmmio + ((_b) + (_o)))

// FPGAレジスタ書き込みマクロ
#define tbs_write(_b, _o, _v)	writel((_v), dev->lmmio + ((_b) + (_o)))

// tbsecp3-core.c 関数プロトタイプ
void tbsecp3_gpio_set_pin(struct tbsecp3_dev *dev,
			  const struct tbsecp3_gpio_pin *pin, int state);

// tbsecp3-i2c.c 関数プロトタイプ
extern int tbsecp3_i2c_init(struct tbsecp3_dev *dev);
extern void tbsecp3_i2c_exit(struct tbsecp3_dev *dev);
extern void tbsecp3_i2c_reg_init(struct tbsecp3_dev *dev);

// tbsecp3-cards.c 外部変数宣言
extern const struct tbsecp3_board tbsecp3_boards[];

// tbsecp3-dma.c 関数プロトタイプ
extern int tbsecp3_dma_init(struct tbsecp3_dev *dev);
extern void tbsecp3_dma_free(struct tbsecp3_dev *dev);
extern void tbsecp3_dma_reg_init(struct tbsecp3_dev *dev);
extern void tbsecp3_dma_enable(struct tbsecp3_adapter *adap);
extern void tbsecp3_dma_disable(struct tbsecp3_adapter *adap);

// tbs6281_chrdev.c 関数プロトタイプ
extern int __init tbs6281_chrdev_class_init(void);
extern void tbs6281_chrdev_class_exit(void);
extern int tbs6281_chrdev_init(struct tbsecp3_adapter *adapter);
extern void tbs6281_chrdev_exit(struct tbsecp3_adapter *adapter);
extern size_t tbs6281_ringbuffer_write(struct tbs6281_ringbuffer *rb,
				       const u8 *buf, size_t len);

#endif
