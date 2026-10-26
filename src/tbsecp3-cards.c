// SPDX-License-Identifier: GPL-2.0-or-later
// TBS6281SE v2 ボード構成定義

#include "tbsecp3.h"

// 対応ボードのハードウェア構成テーブル
const struct tbsecp3_board tbsecp3_boards[] = {
	[TBSECP3_BOARD_TBS6281SE] = {
		.board_id = TBSECP3_BOARD_TBS6281SE,
		.name = "TurboSight TBS 6281 SE v2",
		.adapters = 2,
		.i2c_speed = 39,
		.adap_config = {
			{
				// アダプタ0のTS入力およびGPIOリセット設定
				.ts_in = 0,
				.i2c_bus_nr = 0,
				.gpio.demod_reset.lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset.nr = TBSECP3_GPIO_PIN(0, 0),
			},
			{
				// アダプタ1のTS入力およびGPIOリセット設定
				.ts_in = 1,
				.i2c_bus_nr = 1,
				.gpio.demod_reset.lvl = TBSECP3_GPIODEF_LOW,
				.gpio.demod_reset.nr = TBSECP3_GPIO_PIN(1, 0),
			},
		},
	},
};
