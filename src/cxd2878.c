// SPDX-License-Identifier: GPL-2.0-or-later
// Sony CXD2878 and CXD6802 demodulator and tuner driver

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/mutex.h>

#include "cxd2878.h"
#include "cxd2878_priv.h"

static LIST_HEAD(cxdlist);

struct cxd_base {
	struct list_head cxdlist;
	struct i2c_adapter *i2c;
	struct mutex i2c_lock;
	u8 adr;
	u32 count;
	const struct cxd2878_config *config;
};

#define CXD2878_TUNE_CACHE_SIZE 32
struct cxd2878_tune_cache_entry {
	u32 freq_khz;
	int rf_gain;
	u8 if_gc;
	int fif_offset;
	u8 bbagc_target;
	u16 snr_raw;
	bool valid;
};

struct cxd2878_dev {
	struct cxd_base *base;
	bool warm;
	enum sony_dtv_system_t system;
	enum sony_dtv_bandwidth_t bandwidth;
	enum sony_demod_state_t state;
	u8 slvt;
	u8 slvx;
	u8 tuner_addr;
	enum sony_demod_chip_id_t chipid;
	enum sony_ascot3_chip_id_t tunerid;
	struct sony_demod_iffreq_config_t iffreqConfig;
	u32 current_freq_hz;
	u16 isdbc_tsmf_slot;
	bool isdbc_256qam_pass;
	u8 bbagc_target;
	const u8 *itb_coef;
	int manual_rf_gain;
	u8 dynamic_if_gc;
	int dynamic_rf_gain;
	int dynamic_fif_offset;
	u8 dynamic_bbagc_target;
	struct cxd2878_tune_cache_entry tune_cache[CXD2878_TUNE_CACHE_SIZE];
	int tune_cache_count;
	// 実験用生ストリームモードフラグ
	bool is_j183;
};

// 復調調整用モジュールパラメータ群
static int isdbc_offset_khz = 0;
module_param(isdbc_offset_khz, int, 0644);
MODULE_PARM_DESC(isdbc_offset_khz, "ISDB-C frequency offset in kHz, default 0");
static int isdbc_fif_offset = 999;
module_param(isdbc_fif_offset, int, 0644);
MODULE_PARM_DESC(isdbc_fif_offset, "ISDB-C FIF offset, default 999 for dynamic auto");
static int isdbc_bw_offset = 999;
module_param(isdbc_bw_offset, int, 0644);
MODULE_PARM_DESC(isdbc_bw_offset, "ISDB-C BW offset, default 999 for dynamic auto");
static int isdbc_if_gc = -1;
module_param(isdbc_if_gc, int, 0644);
MODULE_PARM_DESC(isdbc_if_gc, "ISDB-C IF gain, default -1 for dynamic auto");
static int isdbc_bbagc_target = -1;
module_param(isdbc_bbagc_target, int, 0644);
MODULE_PARM_DESC(isdbc_bbagc_target, "ISDB-C BBAGC target, default -1 for dynamic auto");
static int isdbc_lower_local = -1;
module_param(isdbc_lower_local, int, 0644);
MODULE_PARM_DESC(isdbc_lower_local, "ISDB-C lower local, default -1 for dynamic auto");
static int isdbc_rf_gain = -1;
module_param(isdbc_rf_gain, int, 0644);
MODULE_PARM_DESC(isdbc_rf_gain, "ISDB-C RF gain, default -1 for dynamic auto");
static int isdbc_rfovld = -1;
module_param(isdbc_rfovld, int, 0644);
static int isdbc_ifovld = -1;
module_param(isdbc_ifovld, int, 0644);
static int isdbc_outlmt = -1;
module_param(isdbc_outlmt, int, 0644);
MODULE_PARM_DESC(isdbc_outlmt, "ISDB-C output limit, default -1 for dynamic auto");
static int isdbc_qam_mask = 0x1F;
module_param(isdbc_qam_mask, int, 0644);
static int isdbc_terr_tune = 0;
module_param(isdbc_terr_tune, int, 0644);
static int isdbt_lower_local = -1;
module_param(isdbt_lower_local, int, 0644);
static int isdbt_fif_offset = 999;
module_param(isdbt_fif_offset, int, 0644);
static int isdbt_bw_offset = 999;
module_param(isdbt_bw_offset, int, 0644);
static int isdbt_if_gc = -1;
module_param(isdbt_if_gc, int, 0644);
static int isdbt_rf_gain = -1;
module_param(isdbt_rf_gain, int, 0644);
static int isdbt_rfovld = -1;
module_param(isdbt_rfovld, int, 0644);
static int isdbt_ifovld = -1;
module_param(isdbt_ifovld, int, 0644);
static int isdbt_bbagc_target = -1;
module_param(isdbt_bbagc_target, int, 0644);
MODULE_PARM_DESC(isdbt_bbagc_target, "ISDB-T BBAGC target, default -1 for dynamic auto");

// ASCOT3チューナー用調整パラメータテーブル
static const struct sony_ascot3_adjust_param_t g_param_table_ascot3i[SONY_ASCOT3_TV_SYSTEM_NUM] = {
	{0x00, AUTO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, BW_6,   OFFSET(0),   OFFSET(0),   AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x05, 0x03, 0x06, 0x03, 0x01, 0x01, 0x01, 0x00, BW_6,   OFFSET(0),   OFFSET(1),   AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x05, 0x03, 0x06, 0x03, 0x01, 0x01, 0x01, 0x00, BW_6,   OFFSET(0),   OFFSET(1),   AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x05, 0x03, 0x06, 0x03, 0x01, 0x01, 0x01, 0x00, BW_6,   OFFSET(3),   OFFSET(1),   AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x05, 0x03, 0x06, 0x03, 0x01, 0x01, 0x01, 0x00, BW_7,   OFFSET(11),  OFFSET(5),   AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x05, 0x03, 0x06, 0x03, 0x01, 0x01, 0x01, 0x00, BW_8,   OFFSET(2),   OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x05, 0x03, 0x06, 0x03, 0x01, 0x01, 0x01, 0x00, BW_8,   OFFSET(2),   OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x03, 0x04, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x00, BW_8,   OFFSET(2),   OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x03, 0x04, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x00, BW_8,   OFFSET(-1),  OFFSET(4),   AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_6,   OFFSET(-6),  OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_6,   OFFSET(-6),  OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_6,   OFFSET(-9),  OFFSET(-5),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_7,   OFFSET(-7),  OFFSET(-6),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_8,   OFFSET(-5),  OFFSET(-7),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_6,   OFFSET(-8),  OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_6,   OFFSET(-8),  OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_7,   OFFSET(-6),  OFFSET(-5),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_8,   OFFSET(-4),  OFFSET(-6),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_1_7, OFFSET(-10), OFFSET(-10), AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_6,   OFFSET(-8),  OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_6,   OFFSET(-8),  OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_7,   OFFSET(-6),  OFFSET(-5),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_8,   OFFSET(-4),  OFFSET(-6),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x04, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, BW_6,   OFFSET(-6),  OFFSET(-4),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x04, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, BW_8,   OFFSET(-2),  OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x02, 0x09, 0x09, 0x09, 0x02, 0x02, 0x02, 0x00, BW_6,   OFFSET(-6),  OFFSET(-2),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x02, 0x09, 0x09, 0x09, 0x02, 0x02, 0x02, 0x00, BW_8,   OFFSET(-2),  OFFSET(0),   AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_6,   OFFSET(-8),  OFFSET(-3),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_7,   OFFSET(-6),  OFFSET(-5),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x08, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_8,   OFFSET(-4),  OFFSET(-6),  AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x04, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, BW_6,   OFFSET(-5),  OFFSET(2),   AUTO, AUTO, 0x00},
	{0x00, AUTO, 0x03, 0x0B, 0x0B, 0x0B, 0x02, 0x02, 0x02, 0x00, BW_8,   OFFSET(2),   OFFSET(1),   AUTO, AUTO, 0x00}
};

// FREIAチューナー用調整パラメータテーブル
static const struct sony_freia_terr_adjust_param_t g_terr_param_table_freia[SONY_FREIA_TERR_TV_SYSTEM_NUM] = {
	{AUTO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, BW_6,   OFFSET(0),   OFFSET(0),   AUTO, AUTO},
	{AUTO, 0x04, 0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x00, BW_6,   OFFSET(0),   OFFSET(1),   AUTO, AUTO},
	{AUTO, 0x04, 0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x00, BW_6,   OFFSET(0),   OFFSET(1),   AUTO, AUTO},
	{AUTO, 0x04, 0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x00, BW_6,   OFFSET(3),   OFFSET(1),   AUTO, AUTO},
	{AUTO, 0x04, 0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x00, BW_7,   OFFSET(11),  OFFSET(5),   AUTO, AUTO},
	{AUTO, 0x04, 0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x00, BW_8,   OFFSET(2),   OFFSET(-3),  AUTO, AUTO},
	{AUTO, 0x04, 0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x00, BW_8,   OFFSET(2),   OFFSET(-3),  AUTO, AUTO},
	{AUTO, 0x01, 0x08, 0x08, 0x08, 0x04, 0x04, 0x04, 0x00, BW_8,   OFFSET(2),   OFFSET(-3),  AUTO, AUTO},
	{AUTO, 0x01, 0x08, 0x08, 0x08, 0x04, 0x04, 0x04, 0x00, BW_8,   OFFSET(-1),  OFFSET(4),   AUTO, AUTO},
	{AUTO, 0x06, 0x0E, 0x0E, 0x0E, 0x03, 0x03, 0x03, 0x00, BW_6,   OFFSET(-6),  OFFSET(-3),  AUTO, AUTO},
	{AUTO, 0x06, 0x0E, 0x0E, 0x0E, 0x03, 0x03, 0x03, 0x00, BW_6,   OFFSET(-9),  OFFSET(-5),  AUTO, AUTO},
	{AUTO, 0x06, 0x0E, 0x0E, 0x0E, 0x03, 0x03, 0x03, 0x00, BW_7,   OFFSET(-7),  OFFSET(-6),  AUTO, AUTO},
	{AUTO, 0x06, 0x0E, 0x0E, 0x0E, 0x03, 0x03, 0x03, 0x00, BW_8,   OFFSET(-5),  OFFSET(-7),  AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_6,   OFFSET(-8),  OFFSET(-3),  AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_6,   OFFSET(-8),  OFFSET(-3),  AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_7,   OFFSET(-6),  OFFSET(-5),  AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_8,   OFFSET(-4),  OFFSET(-6),  AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_1_7, OFFSET(-10), OFFSET(-10), AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_6,   OFFSET(-8),  OFFSET(-3),  AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_6,   OFFSET(-8),  OFFSET(-3),  AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_7,   OFFSET(-6),  OFFSET(-5),  AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_8,   OFFSET(-4),  OFFSET(-6),  AUTO, AUTO},
	{AUTO, 0x03, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_6,   OFFSET(-6),  OFFSET(-4),  AUTO, AUTO},
	{AUTO, 0x03, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_8,   OFFSET(-2),  OFFSET(-3),  AUTO, AUTO},
	{AUTO, 0x04, 0x09, 0x09, 0x09, 0x02, 0x02, 0x02, 0x00, BW_6,   OFFSET(-6),  OFFSET(-2),  AUTO, AUTO},
	{AUTO, 0x04, 0x09, 0x09, 0x09, 0x02, 0x02, 0x02, 0x00, BW_8,   OFFSET(-2),  OFFSET(0),   AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_6,   OFFSET(-8),  OFFSET(-3),  AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_7,   OFFSET(-6),  OFFSET(-5),  AUTO, AUTO},
	{AUTO, 0x06, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_8,   OFFSET(-4),  OFFSET(-6),  AUTO, AUTO},
	{AUTO, 0x03, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x00, BW_6,   OFFSET(-5),  OFFSET(2),   AUTO, AUTO},
	{AUTO, 0x06, 0x0E, 0x0E, 0x0E, 0x03, 0x03, 0x03, 0x00, BW_8,   OFFSET(2),   OFFSET(1),   AUTO, AUTO}
};

static const u8 log2LookUp[] = {
	0, 4, 9, 13, 17, 21, 25, 29,
	32, 36, 39, 43, 46, 49, 52, 55,
	58, 61, 64, 67, 70, 73, 75, 78,
	81, 83, 86, 88, 91, 93, 95, 98
};

// PTX互換のCNR生データ変換テーブル
static const struct {
	u16 val;
	u32 cn;
} isdbt_cn_raw_table[] = {
	{ 0x51, 0xb19ff }, { 0x5a, 0x9eecd }, { 0x65, 0x8cd8b },
	{ 0x72, 0x7c302 }, { 0x7f, 0x6f132 }, { 0x8f, 0x6250d },
	{ 0xa0, 0x57a1c }, { 0xb4, 0x4db45 }, { 0xc9, 0x45725 },
	{ 0xe2, 0x3da59 }, { 0xfd, 0x36f9d }, { 0x11c, 0x30e58 },
	{ 0x13f, 0x2b76e }, { 0x166, 0x26abb }, { 0x191, 0x22794 },
	{ 0x1c2, 0x1eac7 }, { 0x1f9, 0x1b4a8 }, { 0x237, 0x1844d },
	{ 0x27c, 0x159a2 }, { 0x2ca, 0x13365 }, { 0x321, 0x11196 },
	{ 0x382, 0xf3ae }, { 0x3f0, 0xd8cb }, { 0x46b, 0xc0fd },
	{ 0x4f4, 0xabf9 }, { 0x58f, 0x9923 }, { 0x63d, 0x8868 },
	{ 0x700, 0x7995 }, { 0x7da, 0x6c78 }, { 0x8cf, 0x60cf },
	{ 0x9e2, 0x5675 }, { 0xb17, 0x4d43 }, { 0xc71, 0x4520 },
	{ 0xdf6, 0x3de4 }, { 0xfaa, 0x377b }, { 0x1193, 0x31cc },
	{ 0x13b8, 0x2cbf }, { 0x1620, 0x2843 }, { 0x18d3, 0x2447 },
	{ 0x1bdb, 0x20bb }, { 0x1f41, 0x1d95 }, { 0x2311, 0x1ac6 },
	{ 0x2758, 0x1846 }, { 0x2c25, 0x160a }, { 0x3188, 0x140c },
	{ 0x3793, 0x1243 }, { 0x3e5b, 0x10ab }, { 0x45f7, 0xf3c },
	{ 0x4e80, 0xdf3 }
};

// 整数演算による底2の対数近似計算関数
static u32 sony_math_log2(u32 x)
{
	u8 count = 0;
	u8 index = 0;
	u32 xval = x;

	if (x > 0)
		count = fls(x) - 1;

	x = count * 100;

	if (count > 0) {
		if (count <= MAX_BIT_PRECISION) {
			index = (u8)(xval << (MAX_BIT_PRECISION - count)) & FRAC_BITMASK;
			x += log2LookUp[index];
		} else {
			index = (u8)(xval >> (count - MAX_BIT_PRECISION)) & FRAC_BITMASK;
			x += log2LookUp[index];
		}
	}

	return x;
}

// 整数演算による自然対数近似計算関数
static u32 sony_math_log(u32 x)
{
	return (100 * sony_math_log2(x) + LOG2_E_100X / 2) / LOG2_E_100X;
}

// 複数レジスタのI2C一括書き込み
static int cxd2878_wrm(struct cxd2878_dev *dev, u8 addr, u8 reg, const u8 *buf, u8 len)
{
	int ret;
	u8 b0[50];
	struct i2c_msg msg;

	if (len + 1 > sizeof(b0))
		return -EINVAL;

	b0[0] = reg;
	memcpy(&b0[1], buf, len);
	msg.addr = addr;
	msg.flags = 0;
	msg.buf = b0;
	msg.len = len + 1;

	ret = i2c_transfer(dev->base->i2c, &msg, 1);
	if (ret < 0) {
		dev_warn(&dev->base->i2c->dev,
			 "%s: i2c wrm err %d at 0x%02x\n",
			 KBUILD_MODNAME, ret, reg);
		return ret;
	}
	return 0;
}

// 単一レジスタのI2C書き込み
static int cxd2878_wr(struct cxd2878_dev *dev, u8 addr, u8 reg, u8 data)
{
	return cxd2878_wrm(dev, addr, reg, &data, 1);
}

// レジスタのI2C読み込み
static int cxd2878_rdm(struct cxd2878_dev *dev, u8 addr, u8 reg, u8 *buf, u32 len)
{
	int ret;
	struct i2c_msg msg[2] = {
		{ .addr = addr, .flags = 0, .buf = &reg, .len = 1 },
		{ .addr = addr, .flags = I2C_M_RD, .buf = buf, .len = len }
	};

	ret = i2c_transfer(dev->base->i2c, msg, 2);
	if (ret < 0) {
		dev_warn(&dev->base->i2c->dev,
			 "%s: i2c rdm err %d at 0x%02x\n",
			 KBUILD_MODNAME, ret, addr);
		return ret;
	}
	return 0;
}

// 指定ビットマスクでのレジスタ更新
static int cxd2878_SetRegisterBits(struct cxd2878_dev *dev,
				   u8 slaveaddress, u8 registerAddr, u8 data, u8 mask)
{
	int ret;
	u8 rdata = 0x00;

	if (mask == 0)
		return 0;

	if (mask != 0xFF) {
		ret = cxd2878_rdm(dev, slaveaddress, registerAddr, &rdata, 1);
		if (ret)
			return ret;
		data = (data & mask) | (rdata & (u8)~mask);
		if (data == rdata)
			return 0;
	}

	ret = cxd2878_wr(dev, slaveaddress, registerAddr, data);
	if (ret)
		dev_err(&dev->base->i2c->dev, "set registerbits error\n");
	return ret;
}

// バンク切り替えとビットマスク更新
static int cxd2878_SetBankAndRegisterBits(struct cxd2878_dev *dev, u8 slaveAddress,
					  u8 bank, u8 registerAddress, u8 value, u8 bitMask)
{
	int ret;

	ret = cxd2878_wr(dev, slaveAddress, 0x00, bank);
	if (ret)
		return ret;

	return cxd2878_SetRegisterBits(dev, slaveAddress, registerAddress, value, bitMask);
}

// I2Cリピータ制御によるチューナーバス開閉
static int cxd2878_i2c_repeater(struct cxd2878_dev *dev, bool enable)
{
	int ret;

	ret = cxd2878_wr(dev, dev->slvx, 0x08, enable ? 1 : 0);
	if (ret) {
		dev_err(&dev->base->i2c->dev, "%s: repeater ctrl failed\n", KBUILD_MODNAME);
		return ret;
	}
	usleep_range(1000, 2000);
	return 0;
}

// 日本のCATV向けARIB 6MHzナイキスト補間フィルタ正規係数
static const u8 isdbc_itb_coef_6[14] = {
	0x31, 0xA8, 0x29, 0x9B, 0x27, 0x9C, 0x28, 0x9E, 0x29, 0xA4, 0x29, 0xA2, 0x29, 0xA8
};

// 日本のCATV 6MHz向け標準シンボルレート探索範囲設定値
static const u8 isdbc_symrate_6[4] = {
	0x07, 0x33, 0x00, 0xEE
};

// 動的チューニングパラメータ構造体
struct cxd2878_dynamic_tune_params {
	u8 lower_local;
	u8 outlmt;
	int rf_gain;
	u8 if_gc;
	int fif_offset;
	int bw_offset;
	u8 bbagc_target;
	const u8 *itb_coef;
};

// チューニング学習キャッシュを検索する関数
static int cxd2878_find_cache(struct cxd2878_dev *dev, u32 freq_khz)
{
	int i;

	if (!dev)
		return -1;
	for (i = 0; i < dev->tune_cache_count && i < CXD2878_TUNE_CACHE_SIZE; i++) {
		if (dev->tune_cache[i].valid && dev->tune_cache[i].freq_khz == freq_khz)
			return i;
	}
	return -1;
}

// チューニング学習キャッシュを更新する関数
static void cxd2878_update_cache(struct cxd2878_dev *dev, u32 freq_khz,
				 int rf_gain, u8 if_gc, int fif_offset,
				 u8 bbagc_target, u16 snr_raw)
{
	int idx;

	if (!dev)
		return;

	idx = cxd2878_find_cache(dev, freq_khz);
	if (idx < 0) {
		if (dev->tune_cache_count < CXD2878_TUNE_CACHE_SIZE)
			idx = dev->tune_cache_count++;
		else
			idx = (freq_khz / 1000) % CXD2878_TUNE_CACHE_SIZE;
	}
	dev->tune_cache[idx].freq_khz = freq_khz;
	dev->tune_cache[idx].rf_gain = rf_gain;
	dev->tune_cache[idx].if_gc = if_gc;
	dev->tune_cache[idx].fif_offset = fif_offset;
	dev->tune_cache[idx].bbagc_target = bbagc_target;
	dev->tune_cache[idx].snr_raw = snr_raw;
	dev->tune_cache[idx].valid = true;
}

// 周波数およびシステムに応じたチューナー最適パラメータの動的算出関数
static void cxd2878_calc_dynamic_params(struct cxd2878_dev *dev,
					enum sony_ascot3_tv_system_t aSystem,
					u32 freq_khz,
					struct cxd2878_dynamic_tune_params *p)
{
	if (aSystem == SONY_ASCOT3_DTV_DVBC_6) {
		int cache_idx = cxd2878_find_cache(dev, freq_khz);

		p->lower_local = (isdbc_lower_local >= 0) ? (u8)(isdbc_lower_local & 0x01) : 1;
		p->outlmt = (isdbc_outlmt >= 0) ? (u8)(isdbc_outlmt & 0x03) : 0;
		p->itb_coef = isdbc_itb_coef_6;

		if (cache_idx >= 0) {
			p->rf_gain = (dev->manual_rf_gain >= 0) ? dev->manual_rf_gain : dev->tune_cache[cache_idx].rf_gain;
			p->if_gc = (isdbc_if_gc >= 0) ? (u8)(isdbc_if_gc & 0x0F) : dev->tune_cache[cache_idx].if_gc;
			p->fif_offset = (isdbc_fif_offset != 999) ? isdbc_fif_offset : dev->tune_cache[cache_idx].fif_offset;
			p->bbagc_target = (isdbc_bbagc_target >= 0) ? (u8)(isdbc_bbagc_target & 0xFF) : dev->tune_cache[cache_idx].bbagc_target;
			p->bw_offset = (isdbc_bw_offset != 999) ? isdbc_bw_offset : -4;
		} else {
			if (freq_khz < 330000) {
				// 低域弱電界帯域向け初期設定
				p->if_gc = (isdbc_if_gc >= 0) ? (u8)(isdbc_if_gc & 0x0F) : 6;
				p->bbagc_target = (isdbc_bbagc_target >= 0) ? (u8)(isdbc_bbagc_target & 0xFF) : 0x50;
				p->rf_gain = (dev->manual_rf_gain >= 0) ? dev->manual_rf_gain : ((isdbc_rf_gain >= 0) ? isdbc_rf_gain : -1);
			} else if (freq_khz <= 395000) {
				// 中域安定帯域向け初期設定
				p->if_gc = (isdbc_if_gc >= 0) ? (u8)(isdbc_if_gc & 0x0F) : 4;
				p->bbagc_target = (isdbc_bbagc_target >= 0) ? (u8)(isdbc_bbagc_target & 0xFF) : 0x48;
				p->rf_gain = (dev->manual_rf_gain >= 0) ? dev->manual_rf_gain : ((isdbc_rf_gain >= 0) ? isdbc_rf_gain : -1);
			} else {
				// 高域歪み警戒帯域向け初期設定
				p->if_gc = (isdbc_if_gc >= 0) ? (u8)(isdbc_if_gc & 0x0F) : 2;
				p->bbagc_target = (isdbc_bbagc_target >= 0) ? (u8)(isdbc_bbagc_target & 0xFF) : 0x44;
				p->rf_gain = (dev->manual_rf_gain >= 0) ? dev->manual_rf_gain : ((isdbc_rf_gain >= 0) ? isdbc_rf_gain : -1);
			}
			p->fif_offset = (isdbc_fif_offset != 999) ? isdbc_fif_offset : -6;
			p->bw_offset = (isdbc_bw_offset != 999) ? isdbc_bw_offset : -4;
		}
		dev->dynamic_if_gc = p->if_gc;
		dev->dynamic_rf_gain = p->rf_gain;
		dev->dynamic_fif_offset = p->fif_offset;
		dev->dynamic_bbagc_target = p->bbagc_target;
	} else if (aSystem == SONY_ASCOT3_DTV_ISDBT_6) {
		// 日本の地上デジタル向け動的最適化パラメータ
		p->lower_local = (isdbt_lower_local >= 0) ? (u8)(isdbt_lower_local & 0x01) : 0;
		p->outlmt = 0;
		p->rf_gain = (dev->manual_rf_gain >= 0) ? dev->manual_rf_gain : ((isdbt_rf_gain >= 0) ? isdbt_rf_gain : -1);
		p->if_gc = (isdbt_if_gc >= 0) ? (u8)(isdbt_if_gc & 0x0F) : 8;
		p->fif_offset = (isdbt_fif_offset != 999) ? isdbt_fif_offset : -9;
		p->bw_offset = (isdbt_bw_offset != 999) ? isdbt_bw_offset : -5;
		p->bbagc_target = (isdbt_bbagc_target >= 0) ? (u8)(isdbt_bbagc_target & 0xFF) : 0x50;
		p->itb_coef = NULL;
	} else {
		// その他標準システム向けパラメータ
		p->lower_local = (u8)(g_param_table_ascot3i[aSystem].IS_LOWERLOCAL & 0x01);
		p->outlmt = (u8)(g_param_table_ascot3i[aSystem].OUTLMT & 0x03);
		p->rf_gain = (g_param_table_ascot3i[aSystem].RF_GAIN == AUTO) ? -1 : g_param_table_ascot3i[aSystem].RF_GAIN;
		p->if_gc = (u8)(g_param_table_ascot3i[aSystem].IF_BPF_GC & 0x0F);
		p->fif_offset = g_param_table_ascot3i[aSystem].FIF_OFFSET;
		p->bw_offset = g_param_table_ascot3i[aSystem].BW_OFFSET;
		p->bbagc_target = 128;
		p->itb_coef = NULL;
	}
}

// ASCOT3チューナー周波数選局処理
static int ascot3_tune(struct cxd2878_dev *dev, u32 frequency)
{
	enum sony_ascot3_tv_system_t aSystem;
	struct cxd2878_dynamic_tune_params dyn;
	u8 cdata[2] = { 0xC4, 0x40 };
	u8 data[2];
	u8 data1[2];
	u8 data2[9];
	u8 data3[17];

	switch (dev->system) {
	case SONY_DTV_SYSTEM_DVBC:
		switch (dev->bandwidth) {
		case SONY_DTV_BW_6_MHZ:
			aSystem = SONY_ASCOT3_DTV_DVBC_6;
			break;
		case SONY_DTV_BW_7_MHZ:
		case SONY_DTV_BW_8_MHZ:
		default:
			aSystem = SONY_ASCOT3_DTV_DVBC_8;
			break;
		}
		break;
	case SONY_DTV_SYSTEM_DVBT:
		switch (dev->bandwidth) {
		case SONY_DTV_BW_5_MHZ:
			aSystem = SONY_ASCOT3_DTV_DVBT_5;
			break;
		case SONY_DTV_BW_6_MHZ:
			aSystem = SONY_ASCOT3_DTV_DVBT_6;
			break;
		case SONY_DTV_BW_7_MHZ:
			aSystem = SONY_ASCOT3_DTV_DVBT_7;
			break;
		case SONY_DTV_BW_8_MHZ:
		default:
			aSystem = SONY_ASCOT3_DTV_DVBT_8;
			break;
		}
		break;
	case SONY_DTV_SYSTEM_DVBT2:
		switch (dev->bandwidth) {
		case SONY_DTV_BW_1_7_MHZ:
			aSystem = SONY_ASCOT3_DTV_DVBT2_1_7;
			break;
		case SONY_DTV_BW_5_MHZ:
			aSystem = SONY_ASCOT3_DTV_DVBT2_5;
			break;
		case SONY_DTV_BW_6_MHZ:
			aSystem = SONY_ASCOT3_DTV_DVBT2_6;
			break;
		case SONY_DTV_BW_7_MHZ:
			aSystem = SONY_ASCOT3_DTV_DVBT2_7;
			break;
		case SONY_DTV_BW_8_MHZ:
		default:
			aSystem = SONY_ASCOT3_DTV_DVBT2_8;
			break;
		}
		break;
	case SONY_DTV_SYSTEM_ISDBT:
		switch (dev->bandwidth) {
		case SONY_DTV_BW_6_MHZ:
			aSystem = SONY_ASCOT3_DTV_ISDBT_6;
			break;
		case SONY_DTV_BW_7_MHZ:
			aSystem = SONY_ASCOT3_DTV_ISDBT_7;
			break;
		case SONY_DTV_BW_8_MHZ:
		default:
			aSystem = SONY_ASCOT3_DTV_ISDBT_8;
			break;
		}
		break;
	case SONY_DTV_SYSTEM_ISDBC:
		if (isdbc_terr_tune)
			aSystem = SONY_ASCOT3_DTV_ISDBT_6;
		else
			aSystem = SONY_ASCOT3_DTV_DVBC_6;
		break;
	case SONY_DTV_SYSTEM_ATSC:
		aSystem = SONY_ASCOT3_DTV_8VSB;
		break;
	case SONY_DTV_SYSTEM_J83B:
		if (dev->bandwidth == SONY_DTV_BW_J83B_5_60_MSPS)
			aSystem = SONY_ASCOT3_DTV_J83B_5_6;
		else
			aSystem = SONY_ASCOT3_DTV_DVBC_6;
		break;
	default:
		return -EINVAL;
	}

	cxd2878_SetRegisterBits(dev, dev->tuner_addr, 0x74, 0x02, 0x03);
	cxd2878_wrm(dev, dev->tuner_addr, 0x87, cdata, 2);

	if ((aSystem == SONY_ASCOT3_DTV_DVBC_6) ||
	    (aSystem == SONY_ASCOT3_DTV_DVBC_8) ||
	    (aSystem == SONY_ASCOT3_DTV_J83B_5_6)) {
		data[0] = 0x16;
		data[1] = 0x26;
	} else {
		data[0] = 0x10;
		data[1] = 0x20;
	}
	cxd2878_wrm(dev, dev->tuner_addr, 0x91, data, 2);

	cxd2878_calc_dynamic_params(dev, aSystem, frequency, &dyn);
	dev->bbagc_target = dyn.bbagc_target;
	dev->itb_coef = dyn.itb_coef;
	dev_info(&dev->base->i2c->dev,
		 "%s: dyn tune sys=%d freq=%u kHz: lower_local=%u if_gc=%u fif=%d bw=%d bbagc=%u rf_gain=%d\n",
		 KBUILD_MODNAME, aSystem, frequency,
		 dyn.lower_local, dyn.if_gc, dyn.fif_offset, dyn.bw_offset, dyn.bbagc_target, dyn.rf_gain);

	data1[0] = 0x00;
	data1[1] = (u8)(dyn.lower_local & 0x01);
	cxd2878_wrm(dev, dev->tuner_addr, 0x9c, data1, 2);

	data2[0] = 0xEE;
	data2[1] = 0x02;
	data2[2] = 0x1E;
	data2[3] = 0x67;

	// ケーブル放送時はPLL基準分周比を1MHz比較用に設定する
	if ((aSystem == SONY_ASCOT3_DTV_DVBC_6) ||
	    (aSystem == SONY_ASCOT3_DTV_DVBC_8) ||
	    (aSystem == SONY_ASCOT3_DTV_J83B_5_6)) {
		switch (dev->base->config->tuner_xtal) {
		case SONY_ASCOT3_XTAL_16000KHz:
			data2[4] = 0x10;
			break;
		case SONY_ASCOT3_XTAL_20500KHz:
			data2[4] = 0x14;
			break;
		case SONY_ASCOT3_XTAL_24000KHz:
		default:
			data2[4] = 0x18;
			break;
		case SONY_ASCOT3_XTAL_41000KHz:
			data2[4] = 0x28;
			break;
		}
	} else {
		switch (dev->base->config->tuner_xtal) {
		case SONY_ASCOT3_XTAL_16000KHz:
		case SONY_ASCOT3_XTAL_20500KHz:
			data2[4] = 0x02;
			break;
		case SONY_ASCOT3_XTAL_24000KHz:
		default:
			data2[4] = 0x03;
			break;
		case SONY_ASCOT3_XTAL_41000KHz:
			data2[4] = 0x05;
			break;
		}
	}

	if ((aSystem == SONY_ASCOT3_DTV_DVBC_6) ||
	    (aSystem == SONY_ASCOT3_DTV_DVBC_8) ||
	    (aSystem == SONY_ASCOT3_DTV_J83B_5_6)) {
		data2[5] = 0x1C;
		data2[6] = 0x78;
		data2[7] = 0x08;
		data2[8] = 0x1C;
	} else {
		if (dev->base->config->tuner_xtal == SONY_ASCOT3_XTAL_20500KHz)
			data2[5] = 0x8C;
		else
			data2[5] = 0xB4;
		data2[6] = 0x78;
		data2[7] = 0x08;
		data2[8] = 0x30;
	}
	cxd2878_wrm(dev, dev->tuner_addr, 0x5E, data2, 9);
	cxd2878_SetRegisterBits(dev, dev->tuner_addr, 0x67, 0x00, 0x06);

	data3[0] = (u8)(dyn.outlmt & 0x03);
	if (dyn.rf_gain >= 0)
		data3[1] = (u8)((dyn.rf_gain << 4) & 0x70);
	else
		data3[1] = 0x80;

	data3[1] |= (u8)(dyn.if_gc & 0x0F);
	data3[2] = 0x00;

	if (frequency <= 172000) {
		data3[3] = (u8)(g_param_table_ascot3i[aSystem].RFOVLD_DET_LV1_VL & 0x0F);
		data3[4] = (u8)(g_param_table_ascot3i[aSystem].IFOVLD_DET_LV_VL & 0x07);
	} else if (frequency <= 464000) {
		data3[3] = (u8)(g_param_table_ascot3i[aSystem].RFOVLD_DET_LV1_VH & 0x0F);
		data3[4] = (u8)(g_param_table_ascot3i[aSystem].IFOVLD_DET_LV_VH & 0x07);
	} else {
		data3[3] = (u8)(g_param_table_ascot3i[aSystem].RFOVLD_DET_LV1_U & 0x0F);
		data3[4] = (u8)(g_param_table_ascot3i[aSystem].IFOVLD_DET_LV_U & 0x07);
	}
	if (aSystem == SONY_ASCOT3_DTV_ISDBT_6) {
		if (isdbt_rfovld >= 0)
			data3[3] = (u8)(isdbt_rfovld & 0x0F);
		if (isdbt_ifovld >= 0)
			data3[4] = (data3[4] & ~0x07) | (u8)(isdbt_ifovld & 0x07);
	} else if (aSystem == SONY_ASCOT3_DTV_DVBC_6) {
		if (isdbc_rfovld >= 0)
			data3[3] = (u8)(isdbc_rfovld & 0x0F);
		if (isdbc_ifovld >= 0)
			data3[4] = (data3[4] & ~0x07) | (u8)(isdbc_ifovld & 0x07);
	}
	data3[4] |= 0x20;

	data3[5] = (u8)((g_param_table_ascot3i[aSystem].IF_BPF_F0 << 4) & 0x30);
	data3[5] |= (u8)(g_param_table_ascot3i[aSystem].BW & 0x03);
	data3[6] = OFFSET(dyn.fif_offset);
	data3[7] = OFFSET(dyn.bw_offset);
	data3[8] = (u8)(frequency & 0xFF);
	data3[9] = (u8)((frequency >> 8) & 0xFF);
	data3[10] = (u8)((frequency >> 16) & 0x0F);
	data3[11] = 0xFF;
	data3[12] = 0x11;

	if ((aSystem == SONY_ASCOT3_DTV_DVBC_6) ||
	    (aSystem == SONY_ASCOT3_DTV_DVBC_8) ||
	    (aSystem == SONY_ASCOT3_DTV_J83B_5_6)) {
		data3[13] = 0xD9;
		data3[14] = 0x0F;
		data3[15] = 0x24;
		data3[16] = 0x87;
	} else {
		data3[13] = 0x99;
		data3[14] = 0x00;
		data3[15] = 0x24;
		data3[16] = 0x87;
	}
	cxd2878_wrm(dev, dev->tuner_addr, 0x68, data3, 17);

	msleep(50);
	cxd2878_wr(dev, dev->tuner_addr, 0x88, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xC0);
	msleep(10);
	return 0;
}

// ASCOT3チューナー初期化処理
static int ascot3_init(struct cxd2878_dev *dev)
{
	int ret;
	u8 data = 0;
	u8 rdata = 0;
	u8 cdata[2] = { 0x7A, 0x01 };
	u8 adata[20];
	u8 cdata1[3] = { 0x15, 0x00, 0x00 };

	ret = cxd2878_rdm(dev, dev->tuner_addr, 0x7F, &data, 1);
	if (ret)
		goto err;

	switch (data & 0xF0) {
	case 0xC0:
	case 0xD0:
		dev->tunerid = SONY_ASCOT3_CHIP_ID_UNKNOWN;
		break;
	case 0xE0:
		dev_info(&dev->base->i2c->dev, "Detected Tuner chip ASCOT3 ID 2871A\n");
		dev->tunerid = SONY_ASCOT3_CHIP_ID_2871A;
		break;
	default:
		dev->tunerid = SONY_ASCOT3_CHIP_ID_UNKNOWN;
		break;
	}

	cxd2878_wrm(dev, dev->tuner_addr, 0x99, cdata, 2);

	switch (dev->base->config->tuner_xtal) {
	case SONY_ASCOT3_XTAL_16000KHz:
		adata[0] = 0x10;
		break;
	case SONY_ASCOT3_XTAL_20500KHz:
		adata[0] = 0xD4;
		break;
	case SONY_ASCOT3_XTAL_24000KHz:
	default:
		adata[0] = 0x18;
		break;
	case SONY_ASCOT3_XTAL_41000KHz:
		adata[0] = 0x69;
		break;
	}

	adata[1] = 0x84;
	adata[2] = 0xA8;
	adata[3] = 0x82;
	adata[4] = 0x00;
	adata[5] = 0x00;
	adata[6] = 0xC4;
	adata[7] = 0x40;
	adata[8] = 0x10;
	adata[9] = 0x00;
	adata[10] = 0x45;
	adata[11] = 0x75;
	adata[12] = 0x07;
	adata[13] = 0x1C;
	adata[14] = 0x3F;
	adata[15] = 0x02;
	adata[16] = 0x10;
	adata[17] = 0x20;
	adata[18] = 0x0A;
	adata[19] = 0x00;

	cxd2878_wrm(dev, dev->tuner_addr, 0x81, adata, sizeof(adata));
	cxd2878_wr(dev, dev->tuner_addr, 0x9B, 0x00);
	msleep(10);

	cxd2878_rdm(dev, dev->tuner_addr, 0x1A, &rdata, 1);
	if (rdata != 0)
		return rdata;

	if (dev->tunerid == SONY_ASCOT3_CHIP_ID_UNKNOWN) {
		u8 id_cmd1[2] = { 0x8C, 0x06 };

		cxd2878_wrm(dev, dev->tuner_addr, 0x17, id_cmd1, sizeof(id_cmd1));
		usleep_range(1000, 2000);
		cxd2878_rdm(dev, dev->tuner_addr, 0x19, &data, 1);
		if (data & 0x08)
			dev->tunerid = SONY_ASCOT3_CHIP_ID_2875;

		if (dev->tunerid == SONY_ASCOT3_CHIP_ID_UNKNOWN) {
			u8 id_cmd2[2] = { 0x96, 0x06 };

			cxd2878_wrm(dev, dev->tuner_addr, 0x17, id_cmd2, sizeof(id_cmd2));
			usleep_range(1000, 2000);
			cxd2878_rdm(dev, dev->tuner_addr, 0x19, &data, 1);
			if (data & 0x40)
				dev->tunerid = SONY_ASCOT3_CHIP_ID_2871;
			else
				dev->tunerid = SONY_ASCOT3_CHIP_ID_2872;
		}
	}

	if (dev->tunerid == SONY_ASCOT3_CHIP_ID_2871A) {
		u8 vco_cmd[2] = { 0x2A, 0x0E };

		cxd2878_wrm(dev, dev->tuner_addr, 0x17, vco_cmd, sizeof(vco_cmd));
		usleep_range(1000, 2000);
		cxd2878_rdm(dev, dev->tuner_addr, 0x19, &data, 1);
		cxd2878_wr(dev, dev->tuner_addr, 0x95, data & 0x0F);
	} else {
		u8 vco_cmd[2] = { 0x8D, 0x06 };

		cxd2878_wrm(dev, dev->tuner_addr, 0x17, vco_cmd, sizeof(vco_cmd));
		usleep_range(1000, 2000);
		cxd2878_rdm(dev, dev->tuner_addr, 0x19, &data, 1);
		cxd2878_wr(dev, dev->tuner_addr, 0x95, (u8)(data >> 4));
	}

	if (dev->tunerid == SONY_ASCOT3_CHIP_ID_2871A) {
		cxd2878_wr(dev, dev->tuner_addr, 0xB0, 0x00);
		cxd2878_wr(dev, dev->tuner_addr, 0x30, 0xE0);
		cxd2878_wr(dev, dev->tuner_addr, 0xB1, 0x1E);
		cxd2878_wr(dev, dev->tuner_addr, 0xB3, 0x02);
		usleep_range(1000, 2000);
		cxd2878_rdm(dev, dev->tuner_addr, 0xB4, &data, 1);
		cxd2878_wr(dev, dev->tuner_addr, 0xB3, 0x00);
		usleep_range(1000, 2000);
		cxd2878_wr(dev, dev->tuner_addr, 0xB1, 0x00);
		cxd2878_wr(dev, dev->tuner_addr, 0x30, 0xE1);
		cxd2878_wr(dev, dev->tuner_addr, 0xB0, 0x01);
	}

	cxd2878_SetRegisterBits(dev, dev->tuner_addr, 0x67, 0x00, 0xFE);
	cxd2878_SetRegisterBits(dev, dev->tuner_addr, 0x74, 0x02, 0x03);
	cxd2878_wrm(dev, dev->tuner_addr, 0x5E, cdata1, sizeof(cdata1));
	cxd2878_wr(dev, dev->tuner_addr, 0x88, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xC0);
	cxd2878_wr(dev, dev->tuner_addr, 0x80, 0x01);

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: Tuner ASCOT3 i2c error\n", KBUILD_MODNAME);
	return ret;
}

// FREIAチューナー周波数選局処理
static int freia_tune(struct cxd2878_dev *dev, u32 frequencykHz)
{
	int ret = -EINVAL;
	enum sony_freia_tv_system_t tvSystem;
	u8 cdata[2] = { 0x84, 0x40 };
	u8 data[5];
	u8 data1[2];
	u8 data2[2] = { 0x8C, 0x01 };
	u8 data3[3] = { 0x6E, 0x02, 0x9E };
	u8 data4[6] = { 0x66, 0x01, 0x38, 0x1E, 0x02, 0x24 };
	u8 tmp[17];

	switch (dev->system) {
	case SONY_DTV_SYSTEM_DVBC:
		if (dev->bandwidth == SONY_DTV_BW_6_MHZ)
			tvSystem = SONY_FREIA_DTV_CABLE_6;
		else
			tvSystem = SONY_FREIA_DTV_CABLE_8;
		break;
	case SONY_DTV_SYSTEM_DVBT:
		switch (dev->bandwidth) {
		case SONY_DTV_BW_5_MHZ:
			tvSystem = SONY_FREIA_DTV_DVBT_5;
			break;
		case SONY_DTV_BW_6_MHZ:
			tvSystem = SONY_FREIA_DTV_DVBT_6;
			break;
		case SONY_DTV_BW_7_MHZ:
			tvSystem = SONY_FREIA_DTV_DVBT_7;
			break;
		default:
			tvSystem = SONY_FREIA_DTV_DVBT_8;
			break;
		}
		break;
	case SONY_DTV_SYSTEM_DVBT2:
		switch (dev->bandwidth) {
		case SONY_DTV_BW_1_7_MHZ:
			tvSystem = SONY_FREIA_DTV_DVBT2_1_7;
			break;
		case SONY_DTV_BW_5_MHZ:
			tvSystem = SONY_FREIA_DTV_DVBT2_5;
			break;
		case SONY_DTV_BW_6_MHZ:
			tvSystem = SONY_FREIA_DTV_DVBT2_6;
			break;
		case SONY_DTV_BW_7_MHZ:
			tvSystem = SONY_FREIA_DTV_DVBT2_7;
			break;
		default:
			tvSystem = SONY_FREIA_DTV_DVBT2_8;
			break;
		}
		break;
	case SONY_DTV_SYSTEM_ISDBT:
		switch (dev->bandwidth) {
		case SONY_DTV_BW_6_MHZ:
			tvSystem = SONY_FREIA_DTV_ISDBT_6;
			break;
		case SONY_DTV_BW_7_MHZ:
			tvSystem = SONY_FREIA_DTV_ISDBT_7;
			break;
		default:
			tvSystem = SONY_FREIA_DTV_ISDBT_8;
			break;
		}
		break;
	case SONY_DTV_SYSTEM_ISDBC:
		tvSystem = SONY_FREIA_DTV_CABLE_6;
		break;
	case SONY_DTV_SYSTEM_ATSC:
		tvSystem = SONY_FREIA_DTV_8VSB;
		break;
	case SONY_DTV_SYSTEM_J83B:
		if (dev->bandwidth == SONY_DTV_BW_J83B_5_60_MSPS)
			tvSystem = SONY_FREIA_DTV_SKP_OPT;
		else
			tvSystem = SONY_FREIA_DTV_CABLE_6;
		break;
	default:
		goto err;
	}

	ret = cxd2878_wr(dev, dev->tuner_addr, 0x01, 0x00);
	if (ret)
		goto err;

	cxd2878_wr(dev, dev->tuner_addr, 0x74, 0x02);
	cxd2878_wrm(dev, dev->tuner_addr, 0x87, cdata, 2);

	if (tvSystem == SONY_FREIA_DTV_CABLE_6 ||
	    tvSystem == SONY_FREIA_DTV_CABLE_8 ||
	    tvSystem == SONY_FREIA_DTV_SKP_OPT) {
		data[0] = 0xFC;
		data[1] = 0x87;
		data[2] = 0xCF;
		data[3] = 0x78;
		data[4] = 0x05;
	} else {
		data[0] = 0xFC;
		data[1] = 0x9C;
		data[2] = 0x8F;
		data[3] = 0x00;
		data[4] = 0x75;
	}
	cxd2878_wrm(dev, dev->tuner_addr, 0x3C, data, 3);
	cxd2878_wr(dev, dev->tuner_addr, 0x52, data[3]);
	cxd2878_wr(dev, dev->tuner_addr, 0x8B, data[4]);
	cxd2878_wr(dev, dev->tuner_addr, 0x8D, 0x01);

	if (SONY_FREIA_IS_DVB_T_T2(tvSystem)) {
		data1[0] = 0x00;
		data1[1] = 0x02;
	} else {
		data1[0] = 0x0A;
		data1[1] = 0x0C;
	}
	cxd2878_wrm(dev, dev->tuner_addr, 0x91, data1, 2);
	cxd2878_wrm(dev, dev->tuner_addr, 0x9C, data2, 2);

	cxd2878_wr(dev, dev->tuner_addr, 0x7C, 0x01);
	cxd2878_wrm(dev, dev->tuner_addr, 0x5E, data3, 3);
	cxd2878_wr(dev, dev->tuner_addr, 0x7C, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x1E, 0xA4);
	cxd2878_wr(dev, dev->tuner_addr, 0x5E, 0xEE);
	cxd2878_wrm(dev, dev->tuner_addr, 0x61, data4, 6);
	cxd2878_SetRegisterBits(dev, dev->tuner_addr, 0x67, 0x00, 0x02);

	tmp[0] = 0x00;
	if (g_terr_param_table_freia[tvSystem].RF_GAIN == AUTO)
		tmp[1] = 0x80;
	else
		tmp[1] = (u8)((g_terr_param_table_freia[tvSystem].RF_GAIN << 4) & 0x70);

	tmp[1] |= (u8)(g_terr_param_table_freia[tvSystem].IF_BPF_GC & 0x0F);
	tmp[2] = 0x00;

	if (frequencykHz <= 172000) {
		tmp[3] = (u8)(g_terr_param_table_freia[tvSystem].RFOVLD_DET_LV1_VL & 0x0F);
		tmp[4] = (u8)(g_terr_param_table_freia[tvSystem].IFOVLD_DET_LV_VL & 0x07);
	} else if (frequencykHz <= 464000) {
		tmp[3] = (u8)(g_terr_param_table_freia[tvSystem].RFOVLD_DET_LV1_VH & 0x0F);
		tmp[4] = (u8)(g_terr_param_table_freia[tvSystem].IFOVLD_DET_LV_VH & 0x07);
	} else {
		tmp[3] = (u8)(g_terr_param_table_freia[tvSystem].RFOVLD_DET_LV1_U & 0x0F);
		tmp[4] = (u8)(g_terr_param_table_freia[tvSystem].IFOVLD_DET_LV_U & 0x07);
	}
	if (!SONY_FREIA_IS_DVB_T_T2(tvSystem))
		tmp[4] |= 0x30;

	tmp[5] = (u8)((g_terr_param_table_freia[tvSystem].IF_BPF_F0 << 4) & 0x30);
	tmp[5] |= (u8)(g_terr_param_table_freia[tvSystem].BW & 0x03);
	tmp[6] = (u8)(g_terr_param_table_freia[tvSystem].FIF_OFFSET & 0x1F);
	tmp[7] = (u8)(g_terr_param_table_freia[tvSystem].BW_OFFSET & 0x1F);
	tmp[8] = (u8)(frequencykHz & 0xFF);
	tmp[9] = (u8)((frequencykHz >> 8) & 0xFF);
	tmp[10] = (u8)((frequencykHz >> 16) & 0x1F);
	if (tvSystem == SONY_FREIA_ATV_L_DASH)
		tmp[10] |= 0x40;

	tmp[11] = 0xFF;
	tmp[12] = 0x00;
	tmp[13] = 0xF1;
	tmp[14] = 0x0F;
	tmp[15] = 0x06;
	tmp[16] = 0x03;

	ret = cxd2878_wrm(dev, dev->tuner_addr, 0x68, tmp, 17);
	if (ret)
		goto err;

	msleep(50);
	cxd2878_wr(dev, dev->tuner_addr, 0x88, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x87, 0x80);
	return 0;

err:
	dev_err(&dev->base->i2c->dev, "%s: Tuner freia tune error\n", KBUILD_MODNAME);
	return ret;
}

// FREIAチューナー初期化処理
static int freia_init(struct cxd2878_dev *dev)
{
	int ret = 0;
	u8 tunerid = 0x00;
	u8 rdata = 0x00;
	u8 data1;
	u8 data[20];
	u8 dataT[4] = { 0x15, 0x00, 0x00, 0x00 };
	u8 tmp[3] = { 0x9e, 0x00, 0x00 };
	u8 cdata[3] = { 0xa9, 0x01, 0x00 };
	u8 tmp1[2] = { 0x00, 0x08 };
	u8 tmp2[2] = { 0x7F, 0x06 };
	u8 data2[2] = { 0x00, 0x00 };
	u8 tmp3[8] = { 0x0A, 0x00, 0x00, 0x11, 0x00, 0x03, 0x66, 0x08 };

	ret = cxd2878_rdm(dev, dev->tuner_addr, 0x7F, &tunerid, 1);
	if (ret)
		goto err;
	dev_info(&dev->base->i2c->dev, "Detected FREIA tuner id 0x%x\n", tunerid & 0xFC);

	cxd2878_wr(dev, dev->tuner_addr, 0x01, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x67, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x43, 0xc0);
	cxd2878_wrm(dev, dev->tuner_addr, 0x5E, &dataT[0], 3);
	cxd2878_wr(dev, dev->tuner_addr, 0x0c, 0x14);
	cxd2878_wrm(dev, dev->tuner_addr, 0x79, &tmp[0], 3);
	cxd2878_wrm(dev, dev->tuner_addr, 0x99, &cdata[0], 3);

	data[0] = 0x18;
	data[1] = 0x84;
	data[2] = 0xb5;
	data[3] = 0x82;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x84;
	data[7] = 0x40;
	data[8] = 0x10;
	data[9] = 0x00;
	data[10] = 0x45;
	data[11] = 0x75;
	data[12] = 0x01;
	data[13] = 0x00;
	data[14] = 0x00;
	data[15] = 0x00;
	data[16] = 0x0A;
	data[17] = 0x0C;
	data[18] = 0x3F;
	data[19] = 0x00;

	ret = cxd2878_wrm(dev, dev->tuner_addr, 0x81, data, 20);
	if (ret)
		goto err;

	cxd2878_wrm(dev, dev->tuner_addr, 0x22, tmp1, 2);
	cxd2878_wr(dev, dev->tuner_addr, 0x46, 0x00);
	msleep(10);

	cxd2878_rdm(dev, dev->tuner_addr, 0x1A, &rdata, 1);
	if (rdata != 0x00)
		goto err;

	cxd2878_wrm(dev, dev->tuner_addr, 0x17, tmp2, 2);
	usleep_range(1000, 2000);

	cxd2878_rdm(dev, dev->tuner_addr, 0x19, &rdata, 1);
	data1 = (rdata == 0x00) ? 0x00 : 0x9E;
	cxd2878_wr(dev, dev->tuner_addr, 0x79, data1);
	cxd2878_wr(dev, dev->tuner_addr, 0x74, 0x02);
	cxd2878_wr(dev, dev->tuner_addr, 0xA0, 0x0B);
	cxd2878_wr(dev, dev->tuner_addr, 0x88, 0x00);
	cxd2878_wr(dev, dev->tuner_addr, 0x87, 0x80);
	cxd2878_wr(dev, dev->tuner_addr, 0x80, 0x01);
	cxd2878_wrm(dev, dev->tuner_addr, 0x41, data2, 2);
	cxd2878_wrm(dev, dev->tuner_addr, 0x45, tmp3, 6);
	cxd2878_wrm(dev, dev->tuner_addr, 0xA6, &tmp3[6], 2);
	cxd2878_wr(dev, dev->tuner_addr, 0x1E, 0xA0);
	return 0;

err:
	dev_err(&dev->base->i2c->dev, "%s: Tuner freia init error\n", KBUILD_MODNAME);
	return ret;
}

// ストリーム出力の有効化および無効化
static int cxd2878_setstreamoutput(struct cxd2878_dev *dev, int enable)
{
	int ret = 0;
	u8 data = 0;

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	if (ret)
		goto err;
	ret = cxd2878_rdm(dev, dev->slvt, 0xA9, &data, 1);
	if (ret)
		goto err;

	// TLV出力有効化
	if ((data & 0x03) == 0x01) {
		ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x01);
		if (ret)
			goto err;
		ret = cxd2878_wr(dev, dev->slvt, 0xC0, enable ? 0x00 : 0x01);
		if (ret)
			goto err;
	}

	// ALP出力有効化
	if ((data & 0x03) == 0x02) {
		ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x02);
		if (ret)
			goto err;
		ret = cxd2878_wr(dev, dev->slvt, 0xC0, enable ? 0x00 : 0x01);
		if (ret)
			goto err;
	}

	// TS出力有効化
	if ((data & 0x03) == 0x00) {
		ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
		if (ret)
			goto err;
		ret = cxd2878_wr(dev, dev->slvt, 0xC3, enable ? 0x00 : 0x01);
		if (ret)
			goto err;
	}

	// レジスタバンク復帰
	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	if (ret)
		goto err;
	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: cxd2878_setstreamoutput error\n", KBUILD_MODNAME);
	return ret;
}

// TSクロックモードおよび周波数の設定
static int cxd2878_setTSClkModeAndFreq(struct cxd2878_dev *dev)
{
	int ret;
	u8 serialTS = 0;
	u8 tsRateCtrlOff = 0;
	struct sony_demod_ts_clk_configuration_t tsClkConfiguration;
	struct sony_demod_ts_clk_configuration_t serialTSClkSettings[2][6] = {
		{
			{ 3, 1, 8, 0 },
			{ 3, 1, 8, 1 },
			{ 3, 1, 8, 2 },
			{ 0, 2, 16, 0 },
			{ 0, 2, 16, 1 },
			{ 0, 2, 16, 2 }
		},
		{
			{ 1, 1, 8, 0 },
			{ 1, 1, 8, 1 },
			{ 1, 1, 8, 2 },
			{ 2, 2, 16, 0 },
			{ 2, 2, 16, 1 },
			{ 2, 2, 16, 2 }
		}
	};
	struct sony_demod_ts_clk_configuration_t parallelTSClkSetting = {
		0, 0, 8, 1
	};

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	if (ret)
		goto err;
	ret = cxd2878_rdm(dev, dev->slvt, 0xC4, &serialTS, 1);
	if (ret)
		goto err;

	if (dev->system == SONY_DTV_SYSTEM_ISDBT ||
	    dev->system == SONY_DTV_SYSTEM_ISDBC ||
	    dev->system == SONY_DTV_SYSTEM_ATSC)
		tsRateCtrlOff = 1;

	// 高度放送生ストリーム受信時のレート制御と入力設定
	cxd2878_SetRegisterBits(dev, dev->slvt, 0xD3, tsRateCtrlOff, 0x01);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0xDE, 0x00, 0x01);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0xDA, 0x00, 0x01);

	if (serialTS & 0x80) {
		tsClkConfiguration = serialTSClkSettings[1][1];
	} else {
		tsClkConfiguration = parallelTSClkSetting;
		tsClkConfiguration.tsClkPeriod = 0x08;
	}

	if (serialTS & 0x80) {
		cxd2878_SetRegisterBits(dev, dev->slvt, 0xC4, tsClkConfiguration.serialClkMode, 0x03);
		cxd2878_SetRegisterBits(dev, dev->slvt, 0xD1, tsClkConfiguration.serialDutyMode, 0x03);
	}

	ret = cxd2878_wr(dev, dev->slvt, 0xD9, tsClkConfiguration.tsClkPeriod);
	if (ret)
		goto err;

	cxd2878_SetRegisterBits(dev, dev->slvt, 0x32, 0x00, 0x01);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x33, tsClkConfiguration.clkSelTSIf, 0x03);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x32, 0x01, 0x01);

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
	if (ret)
		goto err;
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x66, 0x01, 0x01);

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
	if (ret)
		goto err;
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x66, 0x01, 0x01);

	// チャネルボンディングおよびパラレル出力設定
	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x01);
	if (ret)
		goto err;
	cxd2878_SetRegisterBits(dev, dev->slvt, 0xE7, 0x01, 0x01);
	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	if (ret)
		goto err;

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: set TSClkModeAndFreq error\n", KBUILD_MODNAME);
	return ret;
}

// TSデータピンのHi-Z状態制御
static int cxd2878_setTSDataPinHiZ(struct cxd2878_dev *dev, u8 enable)
{
	u8 data = 0;
	u8 tsDataMask = 0;
	int ret = 0;

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	if (ret)
		goto err;
	ret = cxd2878_rdm(dev, dev->slvt, 0xA9, &data, 1);
	if (ret)
		goto err;

	if (data & 0x01) {
		ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x01);
		if (ret)
			goto err;
		ret = cxd2878_rdm(dev, dev->slvt, 0xC1, &data, 1);
		if (ret)
			goto err;

		switch (data & 0x88) {
		case 0x80:
			tsDataMask = 0x01;
			break;
		case 0x88:
			tsDataMask = 0x80;
			break;
		default:
			ret = cxd2878_rdm(dev, dev->slvt, 0xCF, &data, 1);
			if (ret)
				goto err;
			if (data & 0x01) {
				ret = cxd2878_rdm(dev, dev->slvt, 0xEA, &data, 1);
				if (ret)
					goto err;
				tsDataMask = (0x01 << (data & 0x07)) |
					     (0x01 << ((data >> 4) & 0x07));
			} else {
				tsDataMask = 0xFF;
			}
			break;
		}
	} else {
		ret = cxd2878_rdm(dev, dev->slvt, 0xC4, &data, 1);
		if (ret)
			goto err;

		switch (data & 0x88) {
		case 0x80:
			tsDataMask = 0x01;
			break;
		case 0x88:
			tsDataMask = 0x80;
			break;
		default:
			tsDataMask = 0xFF;
			break;
		}
	}

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	if (ret)
		goto err;
	ret = cxd2878_SetRegisterBits(dev, dev->slvt, 0x81, enable ? 0xFF : 0x00, tsDataMask);
	if (ret)
		goto err;

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: cxd2878_setTSDataPinHiZ error\n", KBUILD_MODNAME);
	return ret;
}

// 復調器のスリープ状態遷移処理
static int cxd2878_sleep_unlocked(struct cxd2878_dev *dev)
{
	if (dev->state == SONY_DEMOD_STATE_ACTIVE) {
		cxd2878_setstreamoutput(dev, 0);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
		cxd2878_SetRegisterBits(dev, dev->slvt, 0x80, 0x1F, 0x1F);
		cxd2878_setTSDataPinHiZ(dev, 1);

		switch (dev->system) {
		case SONY_DTV_SYSTEM_ISDBT:
		{
			u8 data0[2] = { 0x01, 0x02 };
			u8 nominalRate_def[5] = { 0x15, 0x00, 0x00, 0x00, 0x00 };
			u8 dataxd9_def[2] = { 0x15, 0x28 };

			cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
			cxd2878_wr(dev, dev->slvt, 0x69, 0x05);
			cxd2878_wr(dev, dev->slvt, 0x6B, 0x07);
			cxd2878_wr(dev, dev->slvt, 0x9D, 0x14);
			cxd2878_wrm(dev, dev->slvt, 0x9F, nominalRate_def, 5);
			cxd2878_wr(dev, dev->slvt, 0xD3, 0x00);
			cxd2878_wr(dev, dev->slvt, 0xD7, 0x00);
			cxd2878_wrm(dev, dev->slvt, 0xD9, dataxd9_def, 2);
			cxd2878_wr(dev, dev->slvt, 0xED, 0x01);
			cxd2878_wr(dev, dev->slvt, 0xE2, 0x4E);
			cxd2878_wr(dev, dev->slvt, 0xF2, 0x03);
			cxd2878_wr(dev, dev->slvt, 0xDE, 0x32);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x12);
			cxd2878_wr(dev, dev->slvt, 0x71, 0x03);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x15);
			cxd2878_wr(dev, dev->slvt, 0xBE, 0x03);
			cxd2878_wr(dev, dev->slvt, 0xDE, 0x03);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x17);
			cxd2878_wrm(dev, dev->slvt, 0x38, data0, 2);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x1E);
			cxd2878_wr(dev, dev->slvt, 0x73, 0x00);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x60);
			cxd2878_wr(dev, dev->slvt, 0xA8, 0x00);
			cxd2878_wr(dev, dev->slvt, 0xA9, 0x00);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x63);
			cxd2878_wr(dev, dev->slvt, 0x81, 0x01);
			break;
		}
		case SONY_DTV_SYSTEM_ISDBC:
			cxd2878_wr(dev, dev->slvt, 0x00, 0x11);
			cxd2878_wr(dev, dev->slvt, 0xA3, 0x00);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x48);
			cxd2878_wr(dev, dev->slvt, 0x2C, 0x01);
			// スリープ遷移時にRSエラー時のTS出力ゲート初期値を復元
			cxd2878_wr(dev, dev->slvt, 0xAC, 0x30);
			cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
			cxd2878_wr(dev, dev->slvt, 0x14, 0x1F);
			cxd2878_wr(dev, dev->slvt, 0x1E, 0x00);
			break;
		default:
			break;
		}
	}

	cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvx, 0x18, 0x01);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x49, 0x33);
	cxd2878_wr(dev, dev->slvt, 0x4B, 0x21);
	cxd2878_wr(dev, dev->slvt, 0xFE, 0x01);
	cxd2878_wr(dev, dev->slvt, 0x2C, 0x00);
	cxd2878_wr(dev, dev->slvt, 0xA9, 0x00);
	cxd2878_wr(dev, dev->slvx, 0x17, 0x01);

	dev->state = SONY_DEMOD_STATE_SLEEP;
	dev->system = SONY_DTV_SYSTEM_UNKNOWN;
	return 0;
}

// 復調器をスリープ状態へ遷移させる公開関数
int cxd2878_sleep(struct cxd2878_dev *dev)
{
	int ret;

	if (!dev)
		return -EINVAL;

	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_sleep_unlocked(dev);
	mutex_unlock(&dev->base->i2c_lock);
	return ret;
}

// 選局シーケンスの完了とTS出力再開処理
static int cxd2878_tuneEnd(struct cxd2878_dev *dev)
{
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvt, 0xFE, 0x01);
	cxd2878_setstreamoutput(dev, 1);
	return 0;
}

// ISDB-T用の帯域設定処理
static int SLtoAIT_BandSetting(struct cxd2878_dev *dev)
{
	int ret = 0;
	u8 data[3];
	u8 dataxd9[2];
	u8 nominalRate_8[5] = { 0x11, 0xB8, 0x00, 0x00, 0x00 };
	u8 itbCoef_8[14] = {
		0x2F, 0xBA, 0x28, 0x9B, 0x28, 0x9D, 0x28, 0xA1, 0x29, 0xA5, 0x2A, 0xAC, 0x29, 0xB5
	};
	u8 nominalRate_7[5] = { 0x14, 0x40, 0x00, 0x00, 0x00 };
	u8 itbCoef_7[14] = {
		0x30, 0xB1, 0x29, 0x9A, 0x28, 0x9C, 0x28, 0xA0, 0x29, 0xA2, 0x2B, 0xA6, 0x2B, 0xAD
	};
	u8 nominalRate_6[5] = { 0x17, 0xA0, 0x00, 0x00, 0x00 };
	u8 itbCoef_6[14] = {
		0x31, 0xA8, 0x29, 0x9B, 0x27, 0x9C, 0x28, 0x9E, 0x29, 0xA4, 0x29, 0xA2, 0x29, 0xA8
	};

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
	if (ret)
		goto err;

	switch (dev->bandwidth) {
	case SONY_DTV_BW_8_MHZ:
		cxd2878_wrm(dev, dev->slvt, 0x9F, nominalRate_8, 5);
		cxd2878_wrm(dev, dev->slvt, 0xA6, itbCoef_8, 14);
		data[0] = (u8)((dev->iffreqConfig.configISDBT_8 >> 16) & 0xFF);
		data[1] = (u8)((dev->iffreqConfig.configISDBT_8 >> 8) & 0xFF);
		data[2] = (u8)(dev->iffreqConfig.configISDBT_8 & 0xFF);
		cxd2878_wrm(dev, dev->slvt, 0xB6, data, 3);
		cxd2878_wr(dev, dev->slvt, 0xD7, 0x00);
		dataxd9[0] = 0x13;
		dataxd9[1] = 0xFC;
		cxd2878_wrm(dev, dev->slvt, 0xD9, dataxd9, 2);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x12);
		cxd2878_wr(dev, dev->slvt, 0x71, 3);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x15);
		cxd2878_wr(dev, dev->slvt, 0xBE, 3);
		break;
	case SONY_DTV_BW_7_MHZ:
		cxd2878_wrm(dev, dev->slvt, 0x9F, nominalRate_7, 5);
		cxd2878_wrm(dev, dev->slvt, 0xA6, itbCoef_7, 14);
		data[0] = (u8)((dev->iffreqConfig.configISDBT_7 >> 16) & 0xFF);
		data[1] = (u8)((dev->iffreqConfig.configISDBT_7 >> 8) & 0xFF);
		data[2] = (u8)(dev->iffreqConfig.configISDBT_7 & 0xFF);
		cxd2878_wrm(dev, dev->slvt, 0xB6, data, 3);
		cxd2878_wr(dev, dev->slvt, 0xD7, 0x02);
		dataxd9[0] = 0x1A;
		dataxd9[1] = 0xFA;
		cxd2878_wrm(dev, dev->slvt, 0xD9, dataxd9, 2);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x12);
		cxd2878_wr(dev, dev->slvt, 0x71, 3);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x15);
		cxd2878_wr(dev, dev->slvt, 0xBE, 2);
		break;
	case SONY_DTV_BW_6_MHZ:
		cxd2878_wrm(dev, dev->slvt, 0x9F, nominalRate_6, 5);
		cxd2878_wrm(dev, dev->slvt, 0xA6, itbCoef_6, 14);
		data[0] = (u8)((dev->iffreqConfig.configISDBT_6 >> 16) & 0xFF);
		data[1] = (u8)((dev->iffreqConfig.configISDBT_6 >> 8) & 0xFF);
		data[2] = (u8)(dev->iffreqConfig.configISDBT_6 & 0xFF);
		cxd2878_wrm(dev, dev->slvt, 0xB6, data, 3);
		// 復調器IFAGC設定を適用
		cxd2878_SetRegisterBits(dev, dev->slvt, 0xD2, 0x0C, 0x1F);
		cxd2878_wr(dev, dev->slvt, 0xD7, 0x04);
		dataxd9[0] = 0x1F;
		dataxd9[1] = 0x79;
		cxd2878_wrm(dev, dev->slvt, 0xD9, dataxd9, 2);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x12);
		cxd2878_wr(dev, dev->slvt, 0x71, 7);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x15);
		cxd2878_wr(dev, dev->slvt, 0xBE, 2);
		break;
	default:
		goto err;
	}

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: SLtoAIT_BandSetting error\n", KBUILD_MODNAME);
	return ret;
}

// スリープ状態からISDB-Tアクティブ状態への遷移処理
static int SLtoAIT(struct cxd2878_dev *dev)
{
	int ret = 0;
	u8 dataxce[2] = { 0x00, 0x00 };
	u8 datax38[2] = { 0x00, 0x03 };
	u8 datax33[3] = { 0x00, 0x03, 0x3B };
	u8 dataxa8[2] = { 0xB7, 0x1B };

	ret = cxd2878_setTSClkModeAndFreq(dev);
	if (ret)
		goto err;
	ret = cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
	if (ret)
		goto err;

	cxd2878_wr(dev, dev->slvx, 0x17, 0x06);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvt, 0xA9, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x2C, 0x01);
	cxd2878_wr(dev, dev->slvt, 0x4B, 0x74);
	cxd2878_wr(dev, dev->slvt, 0x49, 0x00);
	cxd2878_wr(dev, dev->slvx, 0x18, 0x00);

	cxd2878_wr(dev, dev->slvt, 0x00, 0x11);
	cxd2878_wr(dev, dev->slvt, 0x6A, dev->bbagc_target ? dev->bbagc_target : 0x50);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
	cxd2878_wr(dev, dev->slvt, 0xA5, 0x01);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);

	cxd2878_wrm(dev, dev->slvt, 0xCE, dataxce, 2);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
	cxd2878_wr(dev, dev->slvt, 0x69, 0x04);
	cxd2878_wr(dev, dev->slvt, 0x6B, 0x03);
	cxd2878_wr(dev, dev->slvt, 0x9D, 0x50);
	cxd2878_wr(dev, dev->slvt, 0xD3, 0x06);
	cxd2878_wr(dev, dev->slvt, 0xED, 0x00);
	cxd2878_wr(dev, dev->slvt, 0xE2, 0xCE);
	cxd2878_wr(dev, dev->slvt, 0xF2, 0x13);
	cxd2878_wr(dev, dev->slvt, 0xDE, 0x2E);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x15);
	cxd2878_wr(dev, dev->slvt, 0xDE, 0x02);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x17);

	cxd2878_wrm(dev, dev->slvt, 0x38, datax38, 2);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x1E);
	cxd2878_wr(dev, dev->slvt, 0x73, 0x68);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x63);
	cxd2878_wr(dev, dev->slvt, 0x81, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x11);

	cxd2878_wrm(dev, dev->slvt, 0x33, datax33, 3);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x60);
	cxd2878_wrm(dev, dev->slvt, 0xA8, dataxa8, 2);

	SLtoAIT_BandSetting(dev);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x80, 0x08, 0x1F);
	cxd2878_setTSDataPinHiZ(dev, 0);

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: SLtoAIT error\n", KBUILD_MODNAME);
	return ret;
}

// ISDB-CおよびDVB-C用の6MHz帯域およびTSMF設定処理
static int SLtoAIC_BandAndTSMFSetting(struct cxd2878_dev *dev)
{
	int ret = 0;
	u8 data[3];

	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
	if (ret)
		goto err;

	// 動的ナイキスト補間フィルタ係数の適用
	if (dev->itb_coef)
		cxd2878_wrm(dev, dev->slvt, 0xA6, dev->itb_coef, 14);

	data[0] = (u8)((dev->iffreqConfig.configDVBC_6 >> 16) & 0xFF);
	data[1] = (u8)((dev->iffreqConfig.configDVBC_6 >> 8) & 0xFF);
	data[2] = (u8)(dev->iffreqConfig.configDVBC_6 & 0xFF);
	cxd2878_wrm(dev, dev->slvt, 0xB6, data, 3);
	// 復調器IFAGC設定を適用
	cxd2878_SetRegisterBits(dev, dev->slvt, 0xD2, 0x09, 0x1F);

	// サンプリング分周を6MHz動作に設定
	cxd2878_wr(dev, dev->slvt, 0x00, 0x11);
	cxd2878_wr(dev, dev->slvt, 0xA3, 0x14);

	cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
	// 6MHz標準シンボルレート探索範囲を設定
	cxd2878_wrm(dev, dev->slvt, 0x26, isdbc_symrate_6, 4);

	if (dev->isdbc_256qam_pass || dev->isdbc_tsmf_slot == 0) {
		// 全QAM自動追従とTSMFスルー出力を設定
		cxd2878_wr(dev, dev->slvt, 0x14, (u8)(isdbc_qam_mask & 0x1F));
		cxd2878_wr(dev, dev->slvt, 0x1E, 0x00);
	} else {
		// ISDB-CのTSMFハードウェア分離を有効化
		cxd2878_wr(dev, dev->slvt, 0x14, (u8)(isdbc_qam_mask & 0x1F));
		cxd2878_wr(dev, dev->slvt, 0x1E, 0x80);
		cxd2878_wr(dev, dev->slvt, 0x00, 0x49);
		if (dev->isdbc_tsmf_slot < 16) {
			// 相対TS番号によるTSMFスロット指定
			cxd2878_SetRegisterBits(dev, dev->slvt, 0x36,
						(u8)(((dev->isdbc_tsmf_slot & 0x0F) << 1) | 0x01), 0x3F);
		} else {
			// TSID指定によるTSMFストリーム指定とONID判定無効化
			u8 tsid_buf[2];

			tsid_buf[0] = (u8)((dev->isdbc_tsmf_slot >> 8) & 0xFF);
			tsid_buf[1] = (u8)(dev->isdbc_tsmf_slot & 0xFF);
			cxd2878_wrm(dev, dev->slvt, 0x37, tsid_buf, 2);
			cxd2878_SetRegisterBits(dev, dev->slvt, 0x36, 0x20, 0x3F);
		}
	}

	// TSクロックマスクと標準TS出力設定
	cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0xC6,
				       dev->is_j183 ? 0x00 : dev->base->config->ts_clk_mask, 0x1F);
	cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0xA9, 0x00, 0x03);
	// 生ストリーム時の同期入力バイパスとクロック常時出力設定
	cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0xDE, 0x00, 0x01);
	cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0xDA,
				       dev->is_j183 ? 0x01 : 0x00, 0x01);

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: SLtoAIC_BandAndTSMFSetting error\n", KBUILD_MODNAME);
	return ret;
}

// スリープ状態からISDB-Cアクティブ状態への遷移処理
static int SLtoAIC(struct cxd2878_dev *dev)
{
	int ret = 0;
	u8 datax33[3] = { 0x00, 0x03, 0x3B };
	u8 dataxce[2] = { 0x01, 0x01 };

	ret = cxd2878_setTSClkModeAndFreq(dev);
	if (ret)
		goto err;
	ret = cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
	if (ret)
		goto err;

	cxd2878_wr(dev, dev->slvx, 0x17, 0x04);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvt, 0xA9, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x2C, 0x01);
	cxd2878_wr(dev, dev->slvt, 0x4B, 0x74);
	cxd2878_wr(dev, dev->slvt, 0x49, 0x00);
	cxd2878_wr(dev, dev->slvx, 0x18, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x11);
	{
		u8 bbagc = (isdbc_bbagc_target >= 0) ? (u8)(isdbc_bbagc_target & 0xFF) :
			   (dev->bbagc_target ? dev->bbagc_target : 0x48);
		cxd2878_wr(dev, dev->slvt, 0x6A, bbagc);
	}
	cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
	cxd2878_wr(dev, dev->slvt, 0xA5, 0x01);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x11);

	cxd2878_wrm(dev, dev->slvt, 0x33, datax33, 3);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x48);
	cxd2878_wr(dev, dev->slvt, 0x2C, 0x03);
	// RSエラー発生時もTS出力を継続
	cxd2878_wr(dev, dev->slvt, 0xAC, 0x00);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_wrm(dev, dev->slvt, 0xCE, dataxce, 2);

	ret = SLtoAIC_BandAndTSMFSetting(dev);
	if (ret)
		goto err;

	cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
	cxd2878_SetRegisterBits(dev, dev->slvt, 0x80, 0x08, 0x1F);

	ret = cxd2878_setTSDataPinHiZ(dev, 0);
	if (ret)
		goto err;

	ret = cxd2878_setstreamoutput(dev, 1);
	if (ret)
		goto err;

	return 0;
err:
	dev_err(&dev->base->i2c->dev, "%s: SLtoAIC error\n", KBUILD_MODNAME);
	return ret;
}

// 復調同期インジケータフラグ設定関数
static void cxd2878_lock_flag(struct cxd2878_dev *dev, bool enable)
{
	cxd2878_SetBankAndRegisterBits(dev, dev->slvx, 0x00, 0xA5, 0x00, 0x0F);
	cxd2878_SetBankAndRegisterBits(dev, dev->slvx, 0x00, 0x82, 1, 0x04);
	cxd2878_SetBankAndRegisterBits(dev, dev->slvx, 0x00, 0xA2, enable ? 0x07 : 0x00, 0x04);
}

// 復調器全体の初期化内部処理関数
static int cxd2878_init_unlocked(struct cxd2878_dev *dev)
{
	int ret;

	if (dev->warm)
		return 0;

	ret = cxd2878_wr(dev, dev->slvx, 0x02, 0x00);
	if (ret)
		goto err;

	usleep_range(4000, 5000);
	cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
	cxd2878_wr(dev, dev->slvx, 0x1D, 0x00);
	cxd2878_wr(dev, dev->slvx, 0x14, dev->base->config->xtal);
	usleep_range(2000, 3000);
	cxd2878_wr(dev, dev->slvx, 0x50, 0x00);

	if (dev->base->config->atscCoreDisable)
		cxd2878_wr(dev, dev->slvx, 0x90, 0x00);

	usleep_range(2000, 3000);
	cxd2878_wr(dev, dev->slvx, 0x10, 0x00);

	if (dev->base->config->atscCoreDisable)
		usleep_range(1000, 2000);
	else
		msleep(21);

	if (dev->chipid == SONY_DEMOD_CHIP_ID_CXD6802) {
		u8 data[4] = { 0x00, 0x00, 0x00, 0x00 };

		cxd2878_wr(dev, dev->slvt, 0x00, 0x9C);
		cxd2878_wrm(dev, dev->slvt, 0x10, data, 4);
	} else {
		cxd2878_wr(dev, dev->slvt, 0x00, 0x95);
		cxd2878_wr(dev, dev->slvt, 0x23, 0x03);
	}

	dev->state = SONY_DEMOD_STATE_SLEEP;

	cxd2878_SetBankAndRegisterBits(dev, dev->slvx, 0x00, 0x1A, 0x01, 0xFF);
	usleep_range(2000, 3000);

	cxd2878_i2c_repeater(dev, true);
	if (dev->chipid == SONY_DEMOD_CHIP_ID_CXD6822 ||
	    dev->chipid == SONY_DEMOD_CHIP_ID_CXD2878A)
		freia_init(dev);
	else
		ascot3_init(dev);
	cxd2878_i2c_repeater(dev, false);

	cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0xC4,
				       dev->base->config->ts_mode ? 0x00 : 0x80, 0x80);
	cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x02, 0xE4,
				       (dev->base->config->ts_mode == 2) ? 0x01 : 0x00, 0x01);
	if (dev->base->config->ts_mode == 0) {
		cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0xC4,
					       dev->base->config->ts_ser_data ? 0x08 : 0x00, 0x08);
		cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0xC4, 0x00, 0x10);
	}
	if (dev->base->config->ts_clk_mask) {
		cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0xC6,
					       dev->base->config->ts_clk_mask, 0x1F);
		cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x60, 0x52,
					       dev->base->config->ts_clk_mask, 0x1F);
	}

	cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0x95, 1, 0x03);
	cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0x95, 0x14, 0x3C);
	cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0x96, 0x54, 0xFF);
	cxd2878_SetBankAndRegisterBits(dev, dev->slvt, 0x00, 0x97, 0x54, 0xFF);

	if (dev->base->config->lock_flag)
		cxd2878_lock_flag(dev, false);

	dev->warm = true;
	return 0;

err:
	dev_err(&dev->base->i2c->dev, "%s: Init failed\n", KBUILD_MODNAME);
	return ret;
}

// 復調器全体の初期化公開関数
int cxd2878_init(struct cxd2878_dev *dev)
{
	int ret;

	if (!dev)
		return -EINVAL;

	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_init_unlocked(dev);
	mutex_unlock(&dev->base->i2c_lock);
	return ret;
}

// ISDB-T選局実行関数
int cxd2878_tune_isdbt(struct cxd2878_dev *dev, u32 freq_hz)
{
	int ret = 0;

	if (!dev)
		return -EINVAL;

	mutex_lock(&dev->base->i2c_lock);

	if (!dev->warm) {
		ret = cxd2878_init_unlocked(dev);
		if (ret)
			goto err;
	}

	if (dev->base->config->RF_switch)
		dev->base->config->RF_switch(dev->base->i2c, dev->base->config->rf_port, 1);
	if (dev->base->config->TS_switch)
		dev->base->config->TS_switch(dev->base->i2c, 1);
	if (dev->base->config->LED_switch)
		dev->base->config->LED_switch(dev->base->i2c, 5);

	dev->bandwidth = SONY_DTV_BW_6_MHZ;
	dev->current_freq_hz = freq_hz;

	// 動的チューニングパラメータの算出
	{
		struct cxd2878_dynamic_tune_params dyn;
		cxd2878_calc_dynamic_params(dev, SONY_ASCOT3_DTV_ISDBT_6, freq_hz / 1000, &dyn);
		dev->bbagc_target = dyn.bbagc_target;
		dev->iffreqConfig.configISDBT_6 = SONY_DEMOD_MAKE_IFFREQ_CONFIG_KHZ(4000 + dyn.fif_offset * 50);
	}

	if (dev->state == SONY_DEMOD_STATE_ACTIVE && dev->system == SONY_DTV_SYSTEM_ISDBT) {
		cxd2878_wr(dev, dev->slvt, 0x00, 0x00);
		cxd2878_wr(dev, dev->slvt, 0xC3, 0x01);
		ret = SLtoAIT_BandSetting(dev);
	} else if (dev->state == SONY_DEMOD_STATE_ACTIVE && dev->system != SONY_DTV_SYSTEM_ISDBT) {
		cxd2878_sleep_unlocked(dev);
		dev->system = SONY_DTV_SYSTEM_ISDBT;
		ret = SLtoAIT(dev);
	} else {
		dev->system = SONY_DTV_SYSTEM_ISDBT;
		ret = SLtoAIT(dev);
	}
	if (ret)
		goto err;

	dev->state = SONY_DEMOD_STATE_ACTIVE;

	ret = cxd2878_i2c_repeater(dev, true);
	if (dev->chipid == SONY_DEMOD_CHIP_ID_CXD6822 ||
	    dev->chipid == SONY_DEMOD_CHIP_ID_CXD2878A)
		ret |= freia_tune(dev, freq_hz / 1000);
	else
		ret |= ascot3_tune(dev, freq_hz / 1000);
	ret |= cxd2878_i2c_repeater(dev, false);
	ret |= cxd2878_tuneEnd(dev);
	if (ret)
		goto err;

	mutex_unlock(&dev->base->i2c_lock);
	return 0;

err:
	mutex_unlock(&dev->base->i2c_lock);
	dev_err(&dev->base->i2c->dev, "%s: tune isdbt failed\n", KBUILD_MODNAME);
	return ret;
}


// ISDB-C選局実行関数
int cxd2878_tune_isdbc(struct cxd2878_dev *dev, u32 freq_hz,
		       u16 tsmf_slot_or_tsid, bool is_256qam_pass)
{
	int ret = 0;

	if (!dev)
		return -EINVAL;

	freq_hz += isdbc_offset_khz * 1000;

	mutex_lock(&dev->base->i2c_lock);

	if (!dev->warm) {
		ret = cxd2878_init_unlocked(dev);
		if (ret)
			goto err;
	}

	if (dev->base->config->RF_switch)
		dev->base->config->RF_switch(dev->base->i2c, dev->base->config->rf_port, 1);
	if (dev->base->config->TS_switch)
		dev->base->config->TS_switch(dev->base->i2c, 1);
	if (dev->base->config->LED_switch)
		dev->base->config->LED_switch(dev->base->i2c, 4);

	dev->bandwidth = SONY_DTV_BW_6_MHZ;
	dev->current_freq_hz = freq_hz;
	dev->isdbc_tsmf_slot = tsmf_slot_or_tsid;
	dev->isdbc_256qam_pass = is_256qam_pass;

	// 選局周波数に応じた動的チューニングパラメータの先行算出
	{
		struct cxd2878_dynamic_tune_params dyn;
		cxd2878_calc_dynamic_params(dev, SONY_ASCOT3_DTV_DVBC_6, freq_hz / 1000, &dyn);
		dev->bbagc_target = dyn.bbagc_target;
		dev->itb_coef = dyn.itb_coef;
		dev->iffreqConfig.configDVBC_6 = SONY_DEMOD_MAKE_IFFREQ_CONFIG_KHZ(4000 + dyn.fif_offset * 50);
	}

	// ISDB-C標準復調選局
	if (dev->state == SONY_DEMOD_STATE_ACTIVE && dev->system != SONY_DTV_SYSTEM_ISDBC)
		cxd2878_sleep_unlocked(dev);
	dev->system = SONY_DTV_SYSTEM_ISDBC;
	dev->bandwidth = SONY_DTV_BW_6_MHZ;
	if (dev->state == SONY_DEMOD_STATE_ACTIVE) {
		ret |= cxd2878_setstreamoutput(dev, 0);
		// 再選局時もRSエラーミュート解除を維持
		cxd2878_wr(dev, dev->slvt, 0x00, 0x48);
		cxd2878_wr(dev, dev->slvt, 0xAC, 0x00);
		ret |= SLtoAIC_BandAndTSMFSetting(dev);
	} else {
		ret = SLtoAIC(dev);
	}
	if (ret)
		goto err;

	dev->state = SONY_DEMOD_STATE_ACTIVE;

	ret = cxd2878_i2c_repeater(dev, true);
	if (dev->chipid == SONY_DEMOD_CHIP_ID_CXD6822 ||
	    dev->chipid == SONY_DEMOD_CHIP_ID_CXD2878A)
		ret |= freia_tune(dev, freq_hz / 1000);
	else
		ret |= ascot3_tune(dev, freq_hz / 1000);
	ret |= cxd2878_i2c_repeater(dev, false);
	ret |= cxd2878_tuneEnd(dev);
	if (ret)
		goto err;

	mutex_unlock(&dev->base->i2c_lock);
	return 0;

err:
	mutex_unlock(&dev->base->i2c_lock);
	dev_err(&dev->base->i2c->dev, "%s: tune isdbc failed\n", KBUILD_MODNAME);
	return ret;
}

// 復調ロック状態の読み取り関数
int cxd2878_read_status(struct cxd2878_dev *dev, bool *is_locked)
{
	int ret = 0;
	u8 data = 0;

	if (!dev || !is_locked)
		return -EINVAL;

	*is_locked = false;
	mutex_lock(&dev->base->i2c_lock);

	switch (dev->system) {
	case SONY_DTV_SYSTEM_ISDBT:
		ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x60);
		if (ret)
			goto out;
		ret = cxd2878_rdm(dev, dev->slvt, 0x10, &data, 1);
		if (ret)
			goto out;
		if ((data & 0x13) == 0x03)
			*is_locked = true;
		break;

	case SONY_DTV_SYSTEM_ISDBC:
	case SONY_DTV_SYSTEM_DVBC:
		ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
		if (ret)
			goto out;
		ret = cxd2878_rdm(dev, dev->slvt, 0x88, &data, 1);
		if (ret || !(data & 0x01))
			goto out;

		ret = cxd2878_rdm(dev, dev->slvt, 0x10, &data, 1);
		if (ret || !(data & 0x20))
			goto out;

		if (dev->system == SONY_DTV_SYSTEM_ISDBC &&
		    !dev->isdbc_256qam_pass && dev->isdbc_tsmf_slot > 0) {
			// ハードウェアTSMF分離時はTSMFフレーム同期ロックも確認
			ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x49);
			if (ret)
				goto out;
			ret = cxd2878_rdm(dev, dev->slvt, 0x20, &data, 1);
			if (ret)
				goto out;
			if (data & 0x01)
				*is_locked = true;
		} else {
			*is_locked = true;
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	if (dev->base->config->lock_flag)
		cxd2878_lock_flag(dev, *is_locked);

out:
	mutex_unlock(&dev->base->i2c_lock);
	return ret;
}

// ISDB-C復調レジスタの生値読み取り関数
int cxd2878_read_isdbc_regs(struct cxd2878_dev *dev, int *val)
{
	u8 r88 = 0, r10 = 0, r19 = 0, r49_20 = 0;

	mutex_lock(&dev->base->i2c_lock);
	if (dev->system == SONY_DTV_SYSTEM_ISDBT) {
		// ISDB-T復調レジスタ状態を読み取り
		u8 r10_11[2];
		u8 r28_29[2];

		cxd2878_wr(dev, dev->slvt, 0x00, 0x60);
		cxd2878_rdm(dev, dev->slvt, 0x10, r10_11, 2);
		cxd2878_rdm(dev, dev->slvt, 0x28, r28_29, 2);
		mutex_unlock(&dev->base->i2c_lock);
		*val = (r28_29[0] << 24) | (r28_29[1] << 16) | (r10_11[1] << 8) | r10_11[0];
		return 0;
	}
	cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
	cxd2878_rdm(dev, dev->slvt, 0x88, &r88, 1);
	cxd2878_rdm(dev, dev->slvt, 0x10, &r10, 1);
	cxd2878_rdm(dev, dev->slvt, 0x19, &r19, 1);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x49);
	cxd2878_rdm(dev, dev->slvt, 0x20, &r49_20, 1);
	mutex_unlock(&dev->base->i2c_lock);
	*val = (r49_20 << 24) | (r88 << 16) | (r10 << 8) | r19;
	return 0;
}

// ISDB-C復調バンクのダンプ取得関数
int cxd2878_dump_isdbc_banks(struct cxd2878_dev *dev, u8 *buf)
{
	mutex_lock(&dev->base->i2c_lock);
	if (dev->system == SONY_DTV_SYSTEM_ISDBT) {
		// ISDB-T復調バンク60hのレジスタを全ダンプ
		cxd2878_wr(dev, dev->slvt, 0x00, 0x60);
		cxd2878_rdm(dev, dev->slvt, 0x00, buf, 128);
		cxd2878_rdm(dev, dev->slvt, 0x80, buf + 128, 128);
		mutex_unlock(&dev->base->i2c_lock);
		return 0;
	}
	cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
	cxd2878_rdm(dev, dev->slvt, 0x10, buf, 128);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x48);
	cxd2878_rdm(dev, dev->slvt, 0x10, buf + 128, 64);
	cxd2878_wr(dev, dev->slvt, 0x00, 0x49);
	cxd2878_rdm(dev, dev->slvt, 0x10, buf + 192, 64);
	mutex_unlock(&dev->base->i2c_lock);
	return 0;
}

// 任意バンクのレジスタ読み書き関数
int cxd2878_reg_rw(struct cxd2878_dev *dev, u8 bank, u8 reg, u8 *val, bool is_write)
{
	int ret = 0;

	if (!dev || !val)
		return -EINVAL;

	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_wr(dev, dev->slvt, 0x00, bank);
	if (ret)
		goto out;
	if (is_write)
		ret = cxd2878_wr(dev, dev->slvt, reg, *val);
	else
		ret = cxd2878_rdm(dev, dev->slvt, reg, val, 1);
out:
	mutex_unlock(&dev->base->i2c_lock);
	return ret;
}

// チューナーのIFとRFのAGC生値読み取り関数
int cxd2878_read_agc_raw(struct cxd2878_dev *dev, int *val)
{
	int ret;
	u8 ifagc = 0, rfagc = 0;
	u8 data[2];

	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_i2c_repeater(dev, true);
	data[0] = 0xC4;
	data[1] = 0x41;
	ret |= cxd2878_wrm(dev, dev->tuner_addr, 0x87, data, 2);
	data[0] = 0x05;
	data[1] = 0x01;
	ret |= cxd2878_wrm(dev, dev->tuner_addr, 0x59, data, 2);
	ret |= cxd2878_rdm(dev, dev->tuner_addr, 0x5B, &ifagc, 1);
	data[0] = 0x03;
	data[1] = 0x01;
	ret |= cxd2878_wrm(dev, dev->tuner_addr, 0x59, data, 2);
	ret |= cxd2878_rdm(dev, dev->tuner_addr, 0x5B, &rfagc, 1);
	cxd2878_wr(dev, dev->tuner_addr, 0x59, 0x04);
	cxd2878_wr(dev, dev->tuner_addr, 0x88, 0x04);
	cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xC0);
	cxd2878_i2c_repeater(dev, false);
	mutex_unlock(&dev->base->i2c_lock);
	*val = (rfagc << 8) | ifagc;
	return ret;
}

// PTX_GET_CNR用のC/N値読み取り関数
int cxd2878_read_cnr_raw(struct cxd2878_dev *dev, int *cnr_val)
{
	int ret = 0;
	u8 tmp[2];
	u8 data = 0;
	u16 val = 0;

	if (!dev || !cnr_val)
		return -EINVAL;

	*cnr_val = 0;
	mutex_lock(&dev->base->i2c_lock);

	if (dev->system == SONY_DTV_SYSTEM_ISDBT) {
		int i, i_min, i_max;

		ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x60);
		if (ret)
			goto out;
		ret = cxd2878_rdm(dev, dev->slvt, 0x10, &data, 1);
		if (ret || (data & 0x13) != 0x03)
			goto out;

		ret = cxd2878_wr(dev, dev->slvt, 0x01, 0x01);
		if (ret)
			goto out;
		ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x60);
		if (ret)
			goto out;
		ret = cxd2878_rdm(dev, dev->slvt, 0x28, tmp, 2);
		cxd2878_wr(dev, dev->slvt, 0x01, 0x00);
		if (ret)
			goto out;

		val = (tmp[0] << 8) | tmp[1];
		i_min = 0;
		i_max = ARRAY_SIZE(isdbt_cn_raw_table) - 1;

		if (isdbt_cn_raw_table[i_min].val >= val) {
			*cnr_val = isdbt_cn_raw_table[i_min].cn;
			goto out;
		}
		if (isdbt_cn_raw_table[i_max].val <= val) {
			*cnr_val = isdbt_cn_raw_table[i_max].cn;
			goto out;
		}

		while (i_min <= i_max) {
			i = i_min + (i_max - i_min) / 2;
			if (isdbt_cn_raw_table[i].val == val) {
				*cnr_val = isdbt_cn_raw_table[i].cn;
				goto out;
			}
			if (isdbt_cn_raw_table[i].val < val)
				i_min = i + 1;
			else
				i_max = i - 1;
		}
		*cnr_val = isdbt_cn_raw_table[i_max].cn;
	} else if (dev->system == SONY_DTV_SYSTEM_ISDBC ||
		   dev->system == SONY_DTV_SYSTEM_DVBC) {
		s32 snr_mdB;
		int idx;
		u8 lock_stat = 0;

		ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
		if (ret)
			goto out;
		ret = cxd2878_rdm(dev, dev->slvt, 0x88, &lock_stat, 1);
		if (ret || !(lock_stat & 0x01))
			goto out;
		ret = cxd2878_rdm(dev, dev->slvt, 0x10, &lock_stat, 1);
		if (ret || !(lock_stat & 0x20))
			goto out;

		// マルチバイトレジスタ読み取り前にレジスタラッチを固定
		ret = cxd2878_wr(dev, dev->slvt, 0x01, 0x01);
		if (ret)
			goto out;
		ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
		if (ret) {
			cxd2878_wr(dev, dev->slvt, 0x01, 0x00);
			goto out;
		}
		ret = cxd2878_rdm(dev, dev->slvt, 0x19, &data, 1);
		if (ret) {
			cxd2878_wr(dev, dev->slvt, 0x01, 0x00);
			goto out;
		}
		ret = cxd2878_rdm(dev, dev->slvt, 0x4C, tmp, 2);
		cxd2878_wr(dev, dev->slvt, 0x01, 0x00);
		if (ret)
			goto out;

		val = ((tmp[0] & 0x1F) << 8) | tmp[1];
		if (val == 0)
			goto out;

		if (((data & 0x07) == 1) || ((data & 0x07) == 3)) {
			if (val < 69)
				val = 69;
			snr_mdB = -88 * (s32)sony_math_log(val) + 86999;
		} else {
			if (val < 126)
				val = 126;
			snr_mdB = -95 * (s32)sony_math_log(val) + 95941;
		}

		// ISDB-CのSNRを0.5dB刻みインデックスに変換してPTX互換CN値を返す
		idx = (snr_mdB - 10000) / 500;
		if (idx < 0)
			idx = 0;
		if (idx >= (int)ARRAY_SIZE(isdbt_cn_raw_table))
			idx = ARRAY_SIZE(isdbt_cn_raw_table) - 1;
		*cnr_val = isdbt_cn_raw_table[idx].cn;
	}

out:
	mutex_unlock(&dev->base->i2c_lock);
	return ret;
}

// 同一I2Cバス上の既存共有ベース構造体を検索する関数
static struct cxd_base *match_base(struct i2c_adapter *i2c, u8 adr)
{
	struct cxd_base *p;

	list_for_each_entry(p, &cxdlist, cxdlist) {
		if (p->i2c == i2c && p->adr != adr)
			return p;
	}
	return NULL;
}

// 復調器インスタンスおよび共有ベース構造体を解放する関数
void cxd2878_release(struct cxd2878_dev *dev)
{
	if (!dev)
		return;

	if (dev->base) {
		dev->base->count--;
		if (dev->base->count == 0) {
			list_del(&dev->base->cxdlist);
			kfree(dev->base);
		}
	}
	kfree(dev);
}

// 復調器デバイスインスタンスを割り当ててハードウェアを接続する関数
struct cxd2878_dev *cxd2878_attach(const struct cxd2878_config *config,
				   struct i2c_adapter *i2c)
{
	struct cxd2878_dev *dev;
	struct cxd_base *base;
	u16 id;
	u8 data[2] = { 0, 0 };

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	dev->slvt = config->addr_slvt;
	dev->slvx = config->addr_slvt + 2;
	dev->tuner_addr = config->tuner_addr;
	dev->state = SONY_DEMOD_STATE_UNKNOWN;
	dev->system = SONY_DTV_SYSTEM_UNKNOWN;

	dev->iffreqConfig.configDVBC_6 = SONY_DEMOD_MAKE_IFFREQ_CONFIG_KHZ(3700);
	dev->iffreqConfig.configISDBT_6 = SONY_DEMOD_MAKE_IFFREQ_CONFIG_KHZ(3550);
	dev->iffreqConfig.configISDBT_7 = SONY_DEMOD_MAKE_IFFREQ_CONFIG_KHZ(4150);
	dev->iffreqConfig.configISDBT_8 = SONY_DEMOD_MAKE_IFFREQ_CONFIG_KHZ(4750);

	dev->warm = false;

	base = match_base(i2c, config->addr_slvt);
	if (base) {
		base->count++;
		dev->base = base;
	} else {
		base = kzalloc(sizeof(*base), GFP_KERNEL);
		if (!base)
			goto err1;
		base->i2c = i2c;
		base->config = config;
		base->adr = config->addr_slvt;
		base->count = 1;
		mutex_init(&base->i2c_lock);
		dev->base = base;
		list_add(&base->cxdlist, &cxdlist);
	}

	cxd2878_wr(dev, dev->slvx, 0x00, 0x00);
	cxd2878_rdm(dev, dev->slvx, 0xFB, &data[0], 1);
	cxd2878_rdm(dev, dev->slvx, 0xFD, &data[1], 1);

	id = ((data[0] & 0x03) << 8) | data[1];
	switch (id) {
	case SONY_DEMOD_CHIP_ID_CXD2857:
		dev_info(&i2c->dev, "Detected CXD2857 chip\n");
		break;
	case SONY_DEMOD_CHIP_ID_CXD2878:
		dev_info(&i2c->dev, "Detected CXD2878/CXD6801 SiP chip\n");
		break;
	case SONY_DEMOD_CHIP_ID_CXD6802:
		dev_info(&i2c->dev, "Detected CXD2878/CXD6802 SiP chip\n");
		break;
	case SONY_DEMOD_CHIP_ID_CXD6822:
		dev_info(&i2c->dev, "Detected CXD2878/CXD6822 SiP2 chip\n");
		break;
	case SONY_DEMOD_CHIP_ID_CXD2878A:
		dev_info(&i2c->dev, "Detected CXD2878A/CXD6821 SiP2 chip\n");
		break;
	default:
		dev_err(&i2c->dev, "%s: Cannot detect chip id 0x%x\n", KBUILD_MODNAME, id);
		goto err2;
	}
	dev->chipid = id;
	dev->manual_rf_gain = -1;
	dev->dynamic_rf_gain = -1;
	dev->dynamic_if_gc = 4;
	dev->dynamic_fif_offset = -6;
	dev->dynamic_bbagc_target = 0x48;
	dev->tune_cache_count = 0;
	return dev;

err2:
	cxd2878_release(dev);
	return NULL;
err1:
	kfree(dev);
	return NULL;
}

// J.183または生ストリームモード判定関数
bool cxd2878_is_j183(struct cxd2878_dev *dev)
{
	if (!dev)
		return false;
	return dev->is_j183;
}

// J.183または生ストリームモード設定関数
void cxd2878_set_j183(struct cxd2878_dev *dev, bool enable)
{
	if (dev)
		dev->is_j183 = enable;
}

// チューナーRFゲインを動的設定する関数
int cxd2878_set_rf_gain(struct cxd2878_dev *dev, int rf_gain)
{
	u8 val = 0;
	int ret;

	if (!dev)
		return -EINVAL;

	// 保持された手動RFゲインを更新
	dev->manual_rf_gain = rf_gain;

	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_i2c_repeater(dev, true);
	if (ret)
		goto out;

	ret = cxd2878_rdm(dev, dev->tuner_addr, 0x69, &val, 1);
	if (ret)
		goto out;

	if (rf_gain < 0) {
		val |= 0x80;
	} else {
		val &= ~0x80;
		val = (val & ~0x70) | (u8)((rf_gain << 4) & 0x70);
	}
	ret = cxd2878_wrm(dev, dev->tuner_addr, 0x69, &val, 1);
	// チューナーレジスタのラッチを実行
	cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xC0);

out:
	cxd2878_i2c_repeater(dev, false);
	mutex_unlock(&dev->base->i2c_lock);
	return ret;
}

// 設定された手動RFゲインを取得する関数
int cxd2878_get_manual_rf_gain(struct cxd2878_dev *dev)
{
	if (!dev)
		return -EINVAL;
	return dev->manual_rf_gain;
}

// チューナーIFゲインを即時適用する関数
int cxd2878_apply_if_gc(struct cxd2878_dev *dev, u8 if_gc)
{
	u8 val = 0;
	int ret;

	if (!dev)
		return -EINVAL;

	dev->dynamic_if_gc = if_gc;
	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_i2c_repeater(dev, true);
	if (ret)
		goto out;

	ret = cxd2878_rdm(dev, dev->tuner_addr, 0x69, &val, 1);
	if (!ret) {
		val = (val & ~0x0F) | (if_gc & 0x0F);
		ret = cxd2878_wrm(dev, dev->tuner_addr, 0x69, &val, 1);
		ret |= cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xC0);
	}

out:
	cxd2878_i2c_repeater(dev, false);
	mutex_unlock(&dev->base->i2c_lock);
	return ret;
}

// ベースバンドAGC目標値を即時適用する関数
int cxd2878_apply_bbagc_target(struct cxd2878_dev *dev, u8 target)
{
	int ret;

	if (!dev)
		return -EINVAL;

	dev->bbagc_target = target;
	dev->dynamic_bbagc_target = target;
	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x11);
	if (!ret)
		ret = cxd2878_wr(dev, dev->slvt, 0x6A, target);
	mutex_unlock(&dev->base->i2c_lock);
	return ret;
}

// 中間周波数オフセットを即時適用する関数
int cxd2878_apply_fif_offset(struct cxd2878_dev *dev, int fif_offset)
{
	u8 data[3];
	u32 if_freq;
	int ret;

	if (!dev)
		return -EINVAL;

	dev->dynamic_fif_offset = fif_offset;
	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_i2c_repeater(dev, true);
	if (ret)
		goto out;

	ret = cxd2878_wr(dev, dev->tuner_addr, 0x6E, (u8)OFFSET(fif_offset));
	ret |= cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xC0);
	cxd2878_i2c_repeater(dev, false);

	if (dev->system == SONY_DTV_SYSTEM_ISDBC || dev->system == SONY_DTV_SYSTEM_DVBC) {
		if_freq = SONY_DEMOD_MAKE_IFFREQ_CONFIG_KHZ(4000 + fif_offset * 50);
		dev->iffreqConfig.configDVBC_6 = if_freq;
		data[0] = (u8)((if_freq >> 16) & 0xFF);
		data[1] = (u8)((if_freq >> 8) & 0xFF);
		data[2] = (u8)(if_freq & 0xFF);
		ret |= cxd2878_wr(dev, dev->slvt, 0x00, 0x10);
		ret |= cxd2878_wrm(dev, dev->slvt, 0xB6, data, 3);
	}

out:
	mutex_unlock(&dev->base->i2c_lock);
	return ret;
}

// チューナーRFゲインを即時適用する関数
int cxd2878_apply_rf_gain(struct cxd2878_dev *dev, int rf_gain)
{
	u8 val = 0;
	int ret;

	if (!dev)
		return -EINVAL;

	dev->dynamic_rf_gain = rf_gain;
	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_i2c_repeater(dev, true);
	if (ret)
		goto out;

	ret = cxd2878_rdm(dev, dev->tuner_addr, 0x69, &val, 1);
	if (!ret) {
		if (rf_gain < 0) {
			val |= 0x80;
		} else {
			val &= ~0x80;
			val = (val & ~0x70) | (u8)((rf_gain << 4) & 0x70);
		}
		ret = cxd2878_wrm(dev, dev->tuner_addr, 0x69, &val, 1);
		ret |= cxd2878_wr(dev, dev->tuner_addr, 0x87, 0xC0);
	}

out:
	cxd2878_i2c_repeater(dev, false);
	mutex_unlock(&dev->base->i2c_lock);
	return ret;
}

// 復調器内部AGC生値を取得する関数
int cxd2878_get_demod_agc(struct cxd2878_dev *dev, u16 *demod_agc)
{
	u8 data[2] = { 0, 0 };
	int ret;

	if (!dev || !demod_agc)
		return -EINVAL;

	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
	if (!ret)
		ret = cxd2878_rdm(dev, dev->slvt, 0x49, data, 2);
	mutex_unlock(&dev->base->i2c_lock);

	if (!ret)
		*demod_agc = (((u16)data[0] & 0x0F) << 8) | (u16)data[1];
	return ret;
}

// 復調器内部SNR生値を取得する関数
int cxd2878_get_snr_raw(struct cxd2878_dev *dev, u16 *snr_raw)
{
	u8 data[2] = { 0, 0 };
	int ret;

	if (!dev || !snr_raw)
		return -EINVAL;

	mutex_lock(&dev->base->i2c_lock);
	ret = cxd2878_wr(dev, dev->slvt, 0x01, 0x01);
	ret |= cxd2878_wr(dev, dev->slvt, 0x00, 0x40);
	if (!ret)
		ret = cxd2878_rdm(dev, dev->slvt, 0x4C, data, 2);
	cxd2878_wr(dev, dev->slvt, 0x01, 0x00);
	mutex_unlock(&dev->base->i2c_lock);

	if (!ret)
		*snr_raw = (((u16)data[0] & 0x1F) << 8) | (u16)data[1];
	return ret;
}

// 信号状態に応じた適応的フィードバック制御とロック待機を行う関数
int cxd2878_adaptive_monitor_and_lock(struct cxd2878_dev *dev, int timeout_ms, bool *is_locked)
{
	int elapsed = 0;
	int regs = 0;
	u16 demod_agc = 0;
	u16 best_snr_raw = 0xFFFF;
	u16 snr_raw = 0;
	bool locked = false;
	int offset_step = 0;
	static const int offset_trials[] = { -6, -5, -7, -4, -8 };

	if (!dev || !is_locked)
		return -EINVAL;

	*is_locked = false;

	if (dev->system != SONY_DTV_SYSTEM_ISDBC && dev->system != SONY_DTV_SYSTEM_DVBC) {
		// 地デジなど非ケーブル時は標準ロック待機を実行
		while (elapsed < timeout_ms) {
			if (cxd2878_read_status(dev, &locked) == 0 && locked) {
				*is_locked = true;
				msleep(80);
				return 0;
			}
			usleep_range(8000, 10000);
			elapsed += 10;
		}
		return 0;
	}

	while (elapsed < timeout_ms) {
		if (cxd2878_read_status(dev, &locked) == 0 && locked) {
			// ロック完了時の変調判定
			if (cxd2878_read_isdbc_regs(dev, &regs) == 0) {
				if ((regs & 0x0F) == 0x04)
					cxd2878_set_j183(dev, true);
			}

			// ロック成立後のSNR微細動的最適化
			if (cxd2878_get_snr_raw(dev, &snr_raw) == 0 && snr_raw > 0) {
				best_snr_raw = snr_raw;
				if (cxd2878_get_demod_agc(dev, &demod_agc) == 0) {
					// AGCがやや高い場合IFゲイン加算をテスト
					if (demod_agc > 550 && dev->dynamic_if_gc < 8) {
						u16 test_snr = 0;
						u8 orig_if = dev->dynamic_if_gc;
						cxd2878_apply_if_gc(dev, orig_if + 1);
						usleep_range(15000, 20000);
						if (cxd2878_get_snr_raw(dev, &test_snr) == 0 && test_snr < best_snr_raw) {
							best_snr_raw = test_snr;
						} else {
							cxd2878_apply_if_gc(dev, orig_if);
						}
					}
					// 高域の歪み帯域でSNR不足時はIF減衰をテスト
					if (dev->current_freq_hz > 395000000U && best_snr_raw > 1800 && dev->dynamic_if_gc > 1) {
						u16 test_snr = 0;
						u8 orig_if = dev->dynamic_if_gc;
						cxd2878_apply_if_gc(dev, orig_if - 1);
						usleep_range(15000, 20000);
						if (cxd2878_get_snr_raw(dev, &test_snr) == 0 && test_snr < best_snr_raw) {
							best_snr_raw = test_snr;
						} else {
							cxd2878_apply_if_gc(dev, orig_if);
						}
					}
				}
			}

			// 学習キャッシュへ最適パラメータを記録
			cxd2878_update_cache(dev, dev->current_freq_hz / 1000,
					     dev->dynamic_rf_gain, dev->dynamic_if_gc,
					     dev->dynamic_fif_offset, dev->dynamic_bbagc_target,
					     best_snr_raw);

			*is_locked = true;
			msleep(40);
			return 0;
		}

		// 未ロック時の動的フィードバック制御
		if (elapsed == 150) {
			if (cxd2878_get_demod_agc(dev, &demod_agc) == 0) {
				if (demod_agc > 700) {
					// 入力信号過小時のゲイン引き上げ
					cxd2878_apply_if_gc(dev, min((int)dev->dynamic_if_gc + 2, 10));
					cxd2878_apply_bbagc_target(dev, 0x50);
				} else if (demod_agc < 200) {
					// 入力信号過大時のゲイン引き下げ
					cxd2878_apply_if_gc(dev, max((int)dev->dynamic_if_gc - 2, 1));
					cxd2878_apply_bbagc_target(dev, 0x44);
				}
			}
		}

		if (elapsed >= 300) {
			if (cxd2878_read_isdbc_regs(dev, &regs) == 0) {
				// QAM同期検出中は待機時間を延長
				if ((regs & 0x80) || ((regs >> 16) & 0x01))
					timeout_ms = max(timeout_ms, 2000);

				if ((regs & 0x0F) == 0x04)
					cxd2878_set_j183(dev, true);
			}

			// キャリア引き込み未完了時の周波数偏差探索
			if ((elapsed % 200) == 0 && offset_step < ARRAY_SIZE(offset_trials)) {
				int trial_fif = offset_trials[offset_step++];
				if (trial_fif != dev->dynamic_fif_offset) {
					cxd2878_apply_fif_offset(dev, trial_fif);
					// 弱電界帯域で自律AGCが過減衰している場合は最大ゲインを試行
					if (dev->current_freq_hz < 330000000U && dev->dynamic_rf_gain < 0)
						cxd2878_apply_rf_gain(dev, 0);
				}
			}
		}

		usleep_range(10000, 15000);
		elapsed += 15;
	}

	return 0;
}



