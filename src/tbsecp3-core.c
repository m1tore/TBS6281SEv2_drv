// SPDX-License-Identifier: GPL-2.0-or-later
// TBS6281SE v2 PCIeコア制御ドライバ

#include <linux/delay.h>
#include "tbsecp3.h"

// MSI割り込み使用可否のモジュールパラメータ
static bool enable_msi = true;
module_param(enable_msi, bool, 0444);
MODULE_PARM_DESC(enable_msi, "use an msi interrupt if available");

// CXD6802パラレルTS出力構成パラメータ
static const struct cxd2878_config cxd6802_parallel_cfg = {
	.addr_slvt = 0x64,
	.xtal = SONY_DEMOD_XTAL_24000KHz,
	.tuner_addr = 0x60,
	.tuner_xtal = SONY_ASCOT3_XTAL_24000KHz,
	.ts_mode = 1,
	.ts_ser_data = 0,
	.ts_clk_mask = 1,
	.atscCoreDisable = 0,
	.lock_flag = false,
	.RF_switch = NULL,
	.rf_port = 0,
	.TS_switch = NULL,
	.LED_switch = NULL,
};

// 指定されたGPIOピンの論理出力レベルを設定する関数
void tbsecp3_gpio_set_pin(struct tbsecp3_dev *dev,
			  const struct tbsecp3_gpio_pin *pin, int state)
{
	u32 tmp, bank, bit;

	if (pin->lvl == TBSECP3_GPIODEF_NONE)
		return;

	if (pin->lvl == TBSECP3_GPIODEF_LOW)
		state = !state;

	bank = (pin->nr >> 3) & ~3;
	bit = pin->nr % 32;

	tmp = tbs_read(TBSECP3_GPIO_BASE, bank);
	if (state)
		tmp |= 1U << bit;
	else
		tmp &= ~(1U << bit);
	tbs_write(TBSECP3_GPIO_BASE, bank, tmp);
}

// GPIOピン操作による復調器のハードウェアリセット
static void reset_demod(struct tbsecp3_adapter *adapter)
{
	struct tbsecp3_dev *dev = adapter->dev;
	const struct tbsecp3_gpio_pin *reset = &adapter->cfg->gpio.demod_reset;

	tbsecp3_gpio_set_pin(dev, reset, 1);
	usleep_range(10000, 20000);

	tbsecp3_gpio_set_pin(dev, reset, 0);
	usleep_range(50000, 100000);
}

// PCIe割り込みを処理するトップハーフハンドラ
static irqreturn_t tbsecp3_irq_handler(int irq, void *dev_id)
{
	struct tbsecp3_dev *dev = (struct tbsecp3_dev *)dev_id;
	struct tbsecp3_i2c *i2c;
	int i, in;
	u32 stat = tbs_read(TBSECP3_INT_BASE, TBSECP3_INT_STAT);

	if (!stat)
		return IRQ_NONE;

	tbs_write(TBSECP3_INT_BASE, TBSECP3_INT_STAT, stat);

	// DMA受信完了割り込みの検出とタスクレットの起動
	if (stat & 0x00000ff0) {
		for (i = 0; i < dev->info->adapters; i++) {
			in = dev->adapter[i].cfg->ts_in;
			if (stat & TBSECP3_DMA_IF(in))
				tasklet_schedule(&dev->adapter[i].tasklet);
		}
	}

	// I2C転送完了割り込みの検出と待機キューの起床
	if (stat & 0x0000000f) {
		for (i = 0; i < TBSECP3_MAX_I2C_BUS; i++) {
			i2c = &dev->i2c_bus[i];
			if (stat & TBSECP3_I2C_IF(i)) {
				i2c->done = 1;
				wake_up(&i2c->wq);
			}
		}
	}

	tbs_write(TBSECP3_INT_BASE, TBSECP3_INT_EN, 1);
	return IRQ_HANDLED;
}

// 各アダプタの復調器初期化とキャラクタデバイス登録
static int tbsecp3_adapters_attach(struct tbsecp3_dev *dev)
{
	int i, ret = 0;
	struct tbsecp3_adapter *adapter;

	for (i = 0; i < dev->info->adapters; i++) {
		adapter = &dev->adapter[i];
		reset_demod(adapter);

		adapter->fe = cxd2878_attach(&cxd6802_parallel_cfg,
					     &adapter->i2c->i2c_adap);
		if (!adapter->fe) {
			dev_err(&dev->pci_dev->dev,
				"Failed to attach CXD6802/CXD2878 frontend on adapter %d\n",
				adapter->nr);
			adapter->nr = -1;
			ret = -ENODEV;
			goto err;
		}

		ret = cxd2878_init(adapter->fe);
		if (ret) {
			dev_err(&dev->pci_dev->dev,
				"Failed to initialize frontend on adapter %d\n",
				adapter->nr);
			cxd2878_release(adapter->fe);
			adapter->fe = NULL;
			adapter->nr = -1;
			goto err;
		}

		ret = tbs6281_chrdev_init(adapter);
		if (ret) {
			dev_err(&dev->pci_dev->dev,
				"Failed to initialize character device on adapter %d\n",
				adapter->nr);
			cxd2878_release(adapter->fe);
			adapter->fe = NULL;
			adapter->nr = -1;
			goto err;
		}
	}
	return 0;

err:
	while (--i >= 0) {
		adapter = &dev->adapter[i];
		if (adapter->nr == -1)
			continue;
		tbs6281_chrdev_exit(adapter);
		if (adapter->fe) {
			cxd2878_release(adapter->fe);
			adapter->fe = NULL;
		}
		adapter->nr = -1;
	}
	return ret;
}

// 各アダプタのキャラクタデバイスおよび復調器を解放する関数
static void tbsecp3_adapters_detach(struct tbsecp3_dev *dev)
{
	struct tbsecp3_adapter *adapter;
	int i;

	for (i = 0; i < dev->info->adapters; i++) {
		adapter = &dev->adapter[i];
		if (adapter->nr == -1)
			continue;

		tbs6281_chrdev_exit(adapter);
		if (adapter->fe) {
			cxd2878_release(adapter->fe);
			adapter->fe = NULL;
		}
		adapter->nr = -1;
	}
}

// 各アダプタの基本ポインタおよび構成構造体を初期設定する関数
static void tbsecp3_adapters_init(struct tbsecp3_dev *dev)
{
	struct tbsecp3_adapter *adapter;
	int i;

	for (i = 0; i < dev->info->adapters; i++) {
		adapter = &dev->adapter[i];
		adapter->nr = i;
		adapter->cfg = &dev->info->adap_config[i];
		adapter->dev = dev;
		adapter->i2c = &dev->i2c_bus[adapter->cfg->i2c_bus_nr];
	}
}

// 各アダプタのタスクレットを停止して安全に終了する関数
static void tbsecp3_adapters_release(struct tbsecp3_dev *dev)
{
	struct tbsecp3_adapter *adapter;
	int i;

	for (i = 0; i < dev->info->adapters; i++) {
		adapter = &dev->adapter[i];
		tasklet_kill(&adapter->tasklet);
	}
}

// MSIメッセージ割り込みの有効化を試行する関数
static bool tbsecp3_enable_msi(struct pci_dev *pci_dev, struct tbsecp3_dev *dev)
{
	int err;

	if (!enable_msi) {
		dev_warn(&dev->pci_dev->dev,
			 "MSI disabled by module parameter enable_msi\n");
		return false;
	}

	err = pci_enable_msi(pci_dev);
	if (err) {
		dev_err(&dev->pci_dev->dev,
			"Failed to enable MSI interrupt, falling back to shared IRQ\n");
		return false;
	}

	err = request_irq(pci_dev->irq, tbsecp3_irq_handler, 0,
			  "tbs6281_drv", dev);
	if (err) {
		dev_err(&dev->pci_dev->dev,
			"Failed to get MSI interrupt, falling back to shared IRQ\n");
		pci_disable_msi(pci_dev);
		return false;
	}
	return true;
}

// PCIeデバイス検出時の初期化プローブ関数
static int tbsecp3_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct tbsecp3_dev *dev;
	int ret = -ENODEV;

	if (pci_enable_device(pdev) < 0)
		return -ENODEV;

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))) {
		dev_err(&pdev->dev, "32-bit PCI DMA not supported\n");
		goto err0;
	}

	pci_set_master(pdev);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err0;
	}

	dev->pci_dev = pdev;
	pci_set_drvdata(pdev, dev);

	dev->info = (const struct tbsecp3_board *)id->driver_data;
	dev_info(&pdev->dev, "%s\n", dev->info->name);

	dev->lmmio = ioremap(pci_resource_start(pdev, 0),
			     pci_resource_len(pdev, 0));
	if (!dev->lmmio) {
		ret = -ENOMEM;
		goto err1;
	}

	tbs_write(TBSECP3_INT_BASE, TBSECP3_INT_EN, 0);
	tbs_write(TBSECP3_INT_BASE, TBSECP3_INT_STAT, 0xff);

	tbsecp3_adapters_init(dev);

	ret = tbsecp3_dma_init(dev);
	if (ret < 0)
		goto err2;

	ret = tbsecp3_i2c_init(dev);
	if (ret < 0)
		goto err3;

	if (tbsecp3_enable_msi(pdev, dev)) {
		dev->msi = true;
	} else {
		ret = request_irq(pdev->irq, tbsecp3_irq_handler,
				  IRQF_SHARED, "tbs6281_drv", dev);
		if (ret < 0) {
			dev_err(&pdev->dev, "%s: cannot get IRQ %d\n",
				dev->info->name, pdev->irq);
			goto err4;
		}
		dev->msi = false;
	}

	tbs_write(TBSECP3_INT_BASE, TBSECP3_INT_EN, 1);

	ret = tbsecp3_adapters_attach(dev);
	if (ret < 0)
		goto err5;

	dev_info(&pdev->dev, "%s: PCI %s, IRQ %d, MMIO 0x%lx\n",
		 dev->info->name, pci_name(pdev), pdev->irq,
		 (unsigned long)pci_resource_start(pdev, 0));

	return 0;

err5:
	tbsecp3_adapters_detach(dev);
	tbs_write(TBSECP3_INT_BASE, TBSECP3_INT_EN, 0);
	free_irq(dev->pci_dev->irq, dev);
	if (dev->msi) {
		pci_disable_msi(pdev);
		dev->msi = false;
	}
err4:
	tbsecp3_i2c_exit(dev);
err3:
	tbsecp3_dma_free(dev);
err2:
	tbsecp3_adapters_release(dev);
	iounmap(dev->lmmio);
err1:
	pci_set_drvdata(pdev, NULL);
	kfree(dev);
err0:
	pci_disable_device(pdev);
	dev_err(&pdev->dev, "probe error\n");
	return ret;
}

// PCIeデバイス取り外し時の解放処理関数
static void tbsecp3_remove(struct pci_dev *pdev)
{
	struct tbsecp3_dev *dev = pci_get_drvdata(pdev);

	tbsecp3_adapters_detach(dev);
	tbs_write(TBSECP3_INT_BASE, TBSECP3_INT_EN, 0);
	free_irq(pdev->irq, dev);
	if (dev->msi) {
		pci_disable_msi(pdev);
		dev->msi = false;
	}
	tbsecp3_adapters_release(dev);
	tbsecp3_dma_free(dev);
	tbsecp3_i2c_exit(dev);
	iounmap(dev->lmmio);
	pci_set_drvdata(pdev, NULL);
	pci_disable_device(pdev);
	kfree(dev);
}

// サスペンド状態からの復帰処理関数
static int tbsecp3_resume(struct pci_dev *pdev)
{
	struct tbsecp3_dev *dev = pci_get_drvdata(pdev);

	tbsecp3_i2c_reg_init(dev);
	tbsecp3_dma_reg_init(dev);
	tbs_write(TBSECP3_INT_BASE, TBSECP3_INT_EN, 1);
	return 0;
}

#define TBSECP3_ID(_board_id, _subvend, _subdev) { \
	.vendor = TBSECP3_VID, .device = TBSECP3_PID, \
	.subvendor = _subvend, .subdevice = _subdev, \
	.driver_data = (unsigned long)&tbsecp3_boards[_board_id] }

// TBS6281SE v2専用のPCIデバイスIDテーブル
static const struct pci_device_id tbsecp3_id_table[] = {
	TBSECP3_ID(TBSECP3_BOARD_TBS6281SE, 0x6281, 0x0003),
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, tbsecp3_id_table);

// PCIドライバ操作構造体
static struct pci_driver tbsecp3_driver = {
	.name = "tbs6281_drv",
	.id_table = tbsecp3_id_table,
	.probe    = tbsecp3_probe,
	.remove   = tbsecp3_remove,
	.resume   = tbsecp3_resume,
	.suspend  = NULL,
};

// モジュール読み込み時のエントリポイント
static int __init module_init_tbsecp3(void)
{
	int ret;

	ret = tbs6281_chrdev_class_init();
	if (ret)
		return ret;

	ret = pci_register_driver(&tbsecp3_driver);
	if (ret) {
		tbs6281_chrdev_class_exit();
		return ret;
	}
	return 0;
}

// モジュール解放時のクリーンアップ関数
static void __exit module_exit_tbsecp3(void)
{
	pci_unregister_driver(&tbsecp3_driver);
	tbs6281_chrdev_class_exit();
}

module_init(module_init_tbsecp3);
module_exit(module_exit_tbsecp3);

MODULE_AUTHOR("Luis Alves <ljalvs@gmail.com>");
MODULE_DESCRIPTION("TurboSight TBS6281SE v2 PTX compatible driver");
MODULE_LICENSE("GPL");
