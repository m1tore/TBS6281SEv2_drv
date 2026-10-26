// SPDX-License-Identifier: GPL-2.0-or-later
// TBS ECP3 FPGA I2Cバスコントローラドライバ

#include "tbsecp3.h"

// I2C送受信制御レジスタ共用体
union tbsecp3_i2c_ctrl {
	struct {
		u32 ctrl;
		u32 data;
	} raw;
	struct {
		u8 size:4;
		u8 saddr:2;
		u8 stop:1;
		u8 start:1;
		u8 read:1;
		u8 addr:7;
		u8 buf[6];
	} bits;
};

// I2Cバスマスター転送実行関数
static int i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msg, int num)
{
	struct tbsecp3_i2c *bus = i2c_get_adapdata(adapter);
	struct tbsecp3_dev *dev = bus->dev;
	union tbsecp3_i2c_ctrl i2c_ctrl;
	int i, j, retval;
	u16 len, remaining, xfer_max;
	u8 *b;

	mutex_lock(&bus->lock);
	// I2Cステータスレジスタを空読みしてクリア
	tbs_read(bus->base, 0x00);

	for (i = 0; i < num; i++) {
		b = msg[i].buf;
		remaining = msg[i].len;

		i2c_ctrl.raw.ctrl = 0;
		i2c_ctrl.bits.start = 1;
		i2c_ctrl.bits.addr = msg[i].addr;

		if (msg[i].flags & I2C_M_RD) {
			i2c_ctrl.bits.read = 1;
			xfer_max = 4;
		} else {
			xfer_max = 6;
		}

		do {
			if (remaining <= xfer_max)
				i2c_ctrl.bits.stop = 1;

			len = remaining > xfer_max ? xfer_max : remaining;
			i2c_ctrl.bits.size = len;

			if (!(msg[i].flags & I2C_M_RD)) {
				// 送信データをバッファへコピーしてレジスタへ書き込み
				memcpy(i2c_ctrl.bits.buf, b, len);
				b += len;
				tbs_write(bus->base, TBSECP3_I2C_DATA, i2c_ctrl.raw.data);
			}
			bus->done = 0;
			tbs_write(bus->base, TBSECP3_I2C_CTRL, i2c_ctrl.raw.ctrl);
			// 転送完了割り込みを待機
			retval = wait_event_timeout(bus->wq, bus->done == 1, HZ);
			if (retval == 0) {
				tbs_read(bus->base, TBSECP3_I2C_STAT);
				dev_err(&dev->pci_dev->dev, "i2c xfer timeout\n");
				retval = -EIO;
				goto i2c_xfer_exit;
			}

			// NACKエラー発生の有無を検証
			j = tbs_read(bus->base, TBSECP3_I2C_CTRL);
			if (j & 0x04) {
				dev_err(&dev->pci_dev->dev, "i2c nack %x\n", j);
				retval = -EIO;
				goto i2c_xfer_exit;
			}

			if (msg[i].flags & I2C_M_RD) {
				// 受信データレジスタからバッファへ読み出し
				i2c_ctrl.raw.data = tbs_read(bus->base, TBSECP3_I2C_DATA);
				memcpy(b, &i2c_ctrl.raw.data, len);
				b += len;
			}

			i2c_ctrl.bits.start = 0;
			remaining -= len;
		} while (remaining);
	}
	retval = num;

i2c_xfer_exit:
	mutex_unlock(&bus->lock);
	return retval;
}

// サポートするI2C機能フラグを返却する関数
static u32 i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL;
}

// I2Cバスアルゴリズム構造体
static const struct i2c_algorithm tbsecp3_i2c_algo_template = {
	.master_xfer   = i2c_xfer,
	.functionality = i2c_functionality,
};

// I2Cアダプタをカーネルへ登録する関数
static int tbsecp3_i2c_register(struct tbsecp3_i2c *bus)
{
	struct tbsecp3_dev *dev = bus->dev;
	struct i2c_adapter *adap;

	init_waitqueue_head(&bus->wq);
	mutex_init(&bus->lock);

	adap = &bus->i2c_adap;
	strscpy(adap->name, "tbsecp3", sizeof(adap->name));
	adap->algo = &tbsecp3_i2c_algo_template;
	adap->algo_data = (void *)bus;
	adap->dev.parent = &dev->pci_dev->dev;
	adap->owner = THIS_MODULE;

	strscpy(bus->i2c_client.name, "tbsecp3cli", sizeof(bus->i2c_client.name));
	bus->i2c_client.adapter = adap;

	i2c_set_adapdata(&bus->i2c_adap, bus);
	return i2c_add_adapter(&bus->i2c_adap);
}

// I2Cアダプタの登録を解除する関数
static void tbsecp3_i2c_unregister(struct tbsecp3_i2c *bus)
{
	i2c_del_adapter(&bus->i2c_adap);
}

// I2Cバス通信速度と割り込みを初期化する関数
void tbsecp3_i2c_reg_init(struct tbsecp3_dev *dev)
{
	int i;
	u32 baud = dev->info->i2c_speed;

	if (!baud)
		baud = 9;

	for (i = 0; i < TBSECP3_MAX_I2C_BUS; i++) {
		tbs_write(dev->i2c_bus[i].base, TBSECP3_I2C_BAUD, baud);
		tbs_read(dev->i2c_bus[i].base, TBSECP3_I2C_STAT);
		tbs_write(TBSECP3_INT_BASE, TBSECP3_I2C_IE(i), 1);
	}
}

// 全I2Cバスの生成と初期設定を行う関数
int tbsecp3_i2c_init(struct tbsecp3_dev *dev)
{
	int i, ret = 0;

	for (i = 0; i < TBSECP3_MAX_I2C_BUS; i++) {
		dev->i2c_bus[i].base = TBSECP3_I2C_BASE(i);
		dev->i2c_bus[i].dev = dev;
		ret = tbsecp3_i2c_register(&dev->i2c_bus[i]);
		if (ret)
			break;
	}
	if (ret) {
		while (--i >= 0)
			tbsecp3_i2c_unregister(&dev->i2c_bus[i]);
	} else {
		tbsecp3_i2c_reg_init(dev);
	}
	return ret;
}

// 全I2Cバスを解放して終了する関数
void tbsecp3_i2c_exit(struct tbsecp3_dev *dev)
{
	int i;

	for (i = 0; i < TBSECP3_MAX_I2C_BUS; i++)
		tbsecp3_i2c_unregister(&dev->i2c_bus[i]);
}
