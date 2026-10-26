// SPDX-License-Identifier: GPL-2.0-or-later
// Sony CXD2878およびCXD6802復調器ヘッダー

#ifndef _CXD2878_H_
#define _CXD2878_H_

#include <linux/types.h>
#include <linux/i2c.h>

// 復調回路用クロック用水晶振動子周波数種別
enum sony_demod_xtal_t {
	SONY_DEMOD_XTAL_16000KHz = 0,
	SONY_DEMOD_XTAL_24000KHz = 1,
	SONY_DEMOD_XTAL_32000KHz = 2
};

// チューナー回路用クロック用水晶振動子周波数種別
enum sony_ascot3_xtal_t {
	SONY_ASCOT3_XTAL_16000KHz,
	SONY_ASCOT3_XTAL_20500KHz,
	SONY_ASCOT3_XTAL_24000KHz,
	SONY_ASCOT3_XTAL_41000KHz
};

// 整数演算による中間周波数設定値計算マクロ
#define SONY_DEMOD_MAKE_IFFREQ_CONFIG_KHZ(iffreq_khz) \
	((u32)((((u64)(iffreq_khz) * 16777216ULL) + 24000ULL) / 48000ULL))

// チューナーおよび復調器のハードウェア構成構造体
struct cxd2878_config {
	u8 addr_slvt;
	enum sony_demod_xtal_t xtal;
	u8 tuner_addr;
	enum sony_ascot3_xtal_t tuner_xtal;
	u8 ts_mode;
	u8 ts_ser_data;
	u8 ts_clk_mask;
	u8 atscCoreDisable;
	bool lock_flag;
	void (*RF_switch)(struct i2c_adapter *i2c, u8 rf_in, u8 flag);
	u8 rf_port;
	void (*TS_switch)(struct i2c_adapter *i2c, u8 flag);
	void (*LED_switch)(struct i2c_adapter *i2c, u8 flag);
};

struct cxd2878_dev;

// 復調器ドライバインスタンスを割り当てて初期接続する関数
struct cxd2878_dev *cxd2878_attach(const struct cxd2878_config *config,
				   struct i2c_adapter *i2c);

// 復調器ドライバインスタンスを解放する関数
void cxd2878_release(struct cxd2878_dev *dev);

// 復調器を通常動作状態へ初期化する関数
int cxd2878_init(struct cxd2878_dev *dev);

// 復調器を省電力スリープ状態へ移行する関数
int cxd2878_sleep(struct cxd2878_dev *dev);

// 指定周波数のISDB-T信号を選局する関数
int cxd2878_tune_isdbt(struct cxd2878_dev *dev, u32 freq_hz);

// 指定周波数およびTSMFスロットのISDB-C信号を選局する関数
int cxd2878_tune_isdbc(struct cxd2878_dev *dev, u32 freq_hz,
		       u16 tsmf_slot_or_tsid, bool is_256qam_pass);

// キャリア同期およびTSロック状態を取得する関数
int cxd2878_read_status(struct cxd2878_dev *dev, bool *is_locked);

// CNR未加工レジスタ値を取得する関数
int cxd2878_read_cnr_raw(struct cxd2878_dev *dev, int *cnr_val);

// AGC未加工レベル値を取得する関数
int cxd2878_read_agc_raw(struct cxd2878_dev *dev, int *val);

// ISDB-Cステータスレジスタ値を取得する関数
int cxd2878_read_isdbc_regs(struct cxd2878_dev *dev, int *val);

// ISDB-C関連レジスタバンクを一括ダンプする関数
int cxd2878_dump_isdbc_banks(struct cxd2878_dev *dev, u8 *buf);

// 復調器レジスタの直接読み書き関数
int cxd2878_reg_rw(struct cxd2878_dev *dev, u8 bank, u8 reg, u8 *val, bool is_write);

#endif
