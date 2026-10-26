// SPDX-License-Identifier: GPL-2.0-or-later
// TBS6281SE v2 PTX compatible character device and ringbuffer implementation

#include <linux/version.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include "tbsecp3.h"
#include "../include/ptx_ioctl.h"

#define TBS6281_DEVICE_NAME	"tbs6281video"
#define TBS6281_MAX_MINORS	16

static struct class *tbs6281_class;
static dev_t tbs6281_dev_base;
static DEFINE_MUTEX(tbs6281_minor_lock);
static bool tbs6281_minor_used[TBS6281_MAX_MINORS];

// リングバッファ内の読み取り可能バイト数を取得
static size_t tbs6281_ringbuffer_avail(struct tbs6281_ringbuffer *rb)
{
	return (rb->head - rb->tail) & TBS6281_RINGBUFFER_MASK;
}

// リングバッファの空きバイト数を取得
static size_t tbs6281_ringbuffer_free_space(struct tbs6281_ringbuffer *rb)
{
	return TBS6281_RINGBUFFER_MASK - tbs6281_ringbuffer_avail(rb);
}

// リングバッファの状態を初期位置にリセット
static void tbs6281_ringbuffer_reset(struct tbs6281_ringbuffer *rb)
{
	unsigned long flags;

	spin_lock_irqsave(&rb->lock, flags);
	rb->head = 0;
	rb->tail = 0;
	spin_unlock_irqrestore(&rb->lock, flags);
}

// DMAタスクレットからリングバッファへTSデータを書き込む
size_t tbs6281_ringbuffer_write(struct tbs6281_ringbuffer *rb,
				const u8 *buf, size_t len)
{
	unsigned long flags;
	size_t free_space;
	size_t first_chunk;

	if (unlikely(!rb || !rb->data || !len))
		return 0;

	spin_lock_irqsave(&rb->lock, flags);
	if (unlikely(!rb->running)) {
		spin_unlock_irqrestore(&rb->lock, flags);
		return 0;
	}

	if (len >= TBS6281_RINGBUFFER_SIZE)
		len = TBS6281_RINGBUFFER_MASK;

	free_space = tbs6281_ringbuffer_free_space(rb);
	if (unlikely(len > free_space)) {
		// バッファ溢れ時は古いデータを破棄して最新ストリームを優先
		size_t overflow = len - free_space;

		rb->tail = (rb->tail + overflow) & TBS6281_RINGBUFFER_MASK;
	}

	first_chunk = min(len, TBS6281_RINGBUFFER_SIZE - rb->head);
	memcpy(rb->data + rb->head, buf, first_chunk);
	if (len > first_chunk)
		memcpy(rb->data, buf + first_chunk, len - first_chunk);

	rb->head = (rb->head + len) & TBS6281_RINGBUFFER_MASK;
	spin_unlock_irqrestore(&rb->lock, flags);

	wake_up_interruptible(&rb->wq);
	return len;
}

// 復調器のロック完了を待機し安定化するヘルパー関数
static bool tbs6281_wait_lock(struct cxd2878_dev *fe, int timeout_ms)
{
	int elapsed = 0;
	int regs = 0;
	bool is_locked = false;

	while (elapsed < timeout_ms) {
		if (cxd2878_read_status(fe, &is_locked) == 0 && is_locked) {
			msleep(80);
			return true;
		}
		if (elapsed == 300 && timeout_ms == 700) {
			// QAM同期検出時はTS同期完了まで待機時間を延長する
			if (cxd2878_read_isdbc_regs(fe, &regs) == 0 && ((regs >> 16) & 0x01))
				timeout_ms = 1500;
		}
		usleep_range(8000, 10000);
		elapsed += 10;
	}
	return false;
}

// キャラクタデバイスのオープン処理
static int tbs6281_chrdev_open(struct inode *inode, struct file *file)
{
	struct tbsecp3_adapter *adapter =
		container_of(inode->i_cdev, struct tbsecp3_adapter, cdev);
	int ret = 0;

	mutex_lock(&adapter->lock);
	if (adapter->is_open) {
		mutex_unlock(&adapter->lock);
		return -EBUSY;
	}

	if (adapter->fe) {
		ret = cxd2878_init(adapter->fe);
		if (ret) {
			mutex_unlock(&adapter->lock);
			return ret;
		}
	}

	adapter->is_open = true;
	adapter->streaming = false;
	file->private_data = adapter;
	mutex_unlock(&adapter->lock);
	return 0;
}

// キャラクタデバイスのクローズ処理
static int tbs6281_chrdev_release(struct inode *inode, struct file *file)
{
	struct tbsecp3_adapter *adapter = file->private_data;
	unsigned long flags;

	if (!adapter)
		return 0;

	mutex_lock(&adapter->lock);
	if (adapter->streaming) {
		tbsecp3_dma_disable(adapter);
		spin_lock_irqsave(&adapter->ringbuf.lock, flags);
		adapter->ringbuf.running = false;
		spin_unlock_irqrestore(&adapter->ringbuf.lock, flags);
		wake_up_interruptible(&adapter->ringbuf.wq);
		adapter->streaming = false;
	}

	if (adapter->fe)
		cxd2878_sleep(adapter->fe);

	adapter->is_open = false;
	mutex_unlock(&adapter->lock);
	return 0;
}

// リングバッファからユーザ空間へTSデータを読み出す関数
static ssize_t tbs6281_chrdev_read(struct file *file, char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct tbsecp3_adapter *adapter = file->private_data;
	struct tbs6281_ringbuffer *rb;
	size_t copied = 0;
	int ret = 0;

	if (!adapter || !buf)
		return -EINVAL;

	rb = &adapter->ringbuf;

	while (copied < count) {
		unsigned long flags;
		size_t avail;
		size_t to_copy;
		size_t first_chunk;
		size_t tail;
		bool running;

		spin_lock_irqsave(&rb->lock, flags);
		avail = tbs6281_ringbuffer_avail(rb);
		tail = rb->tail;
		running = rb->running;
		spin_unlock_irqrestore(&rb->lock, flags);

		if (avail == 0) {
			if (!running)
				break;
			if (file->f_flags & O_NONBLOCK) {
				if (copied == 0)
					ret = -EAGAIN;
				break;
			}
			if (copied > 0)
				break;

			ret = wait_event_interruptible(rb->wq,
				tbs6281_ringbuffer_avail(rb) > 0 || !rb->running);
			if (ret)
				break;
			continue;
		}

		to_copy = min(count - copied, avail);
		first_chunk = min(to_copy, TBS6281_RINGBUFFER_SIZE - tail);

		if (unlikely(copy_to_user(buf + copied, rb->data + tail, first_chunk))) {
			ret = -EFAULT;
			break;
		}

		if (to_copy > first_chunk) {
			if (unlikely(copy_to_user(buf + copied + first_chunk,
						  rb->data, to_copy - first_chunk))) {
				ret = -EFAULT;
				break;
			}
		}

		spin_lock_irqsave(&rb->lock, flags);
		rb->tail = (rb->tail + to_copy) & TBS6281_RINGBUFFER_MASK;
		spin_unlock_irqrestore(&rb->lock, flags);

		copied += to_copy;
	}

	return copied > 0 ? (ssize_t)copied : ret;
}

// 読み取り可能データの有無をポーリング監視する関数
static __poll_t tbs6281_chrdev_poll(struct file *file, poll_table *wait)
{
	struct tbsecp3_adapter *adapter = file->private_data;
	struct tbs6281_ringbuffer *rb = &adapter->ringbuf;
	__poll_t mask = 0;

	poll_wait(file, &rb->wq, wait);

	if (tbs6281_ringbuffer_avail(rb) > 0)
		mask |= EPOLLIN | EPOLLRDNORM;
	else if (!READ_ONCE(rb->running))
		mask |= EPOLLHUP;

	return mask;
}

// PTX_SET_CHANNELの周波数変換およびISDB-TとISDB-Cの振り分け処理
static int tbs6281_set_channel(struct tbsecp3_adapter *adapter,
			       const struct ptx_freq *freq)
{
	int ret = 0;
	u32 base_freq_hz = 0;

	if (freq->freq_no >= 63 && freq->freq_no <= 112) {
		// UHF地上波およびスーパーハイパーバンド13chから62chの選局処理
		base_freq_hz = (473 + (freq->freq_no - 63) * 6) * 1000000U;
		if (freq->slot == 0) {
			// 標準周波数オフセットでISDB-T地上波を選局する
			ret = cxd2878_tune_isdbt(adapter->fe, base_freq_hz + 142857U);
			if (ret)
				return ret;
			tbs6281_wait_lock(adapter->fe, 1500);
		} else if (freq->slot > 10000000) {
			// 直接指定された周波数でISDB-T選局
			ret = cxd2878_tune_isdbt(adapter->fe, (u32)freq->slot);
			if (ret)
				return ret;
			tbs6281_wait_lock(adapter->fe, 1500);
		} else if (freq->slot > 65535) {
			// slot値の周波数オフセットを加算してISDB-T選局
			ret = cxd2878_tune_isdbt(adapter->fe, base_freq_hz + (u32)freq->slot);
			if (ret)
				return ret;
			tbs6281_wait_lock(adapter->fe, 1500);
		} else if (freq->slot > 0) {
			// UHF帯におけるISDB-CのTSMF分離モード選局
			ret = cxd2878_tune_isdbc(adapter->fe, base_freq_hz,
						 (u16)freq->slot, false);
			if (ret)
				return ret;
			tbs6281_wait_lock(adapter->fe, 700);
		} else {
			// CATV ISDB-C再送信スルー選局
			ret = cxd2878_tune_isdbc(adapter->fe, base_freq_hz, 0, true);
			if (ret)
				return ret;
			tbs6281_wait_lock(adapter->fe, 700);
		}
		return 0;
	}

	if (freq->freq_no >= 0 && freq->freq_no <= 62) {
		// JCTEA規格に基づくVHFおよびCATV帯域の中心周波数を算出
		if (freq->freq_no <= 2) {
			// VHF 1chから3ch
			base_freq_hz = (93 + freq->freq_no * 6) * 1000000U;
		} else if (freq->freq_no >= 3 && freq->freq_no <= 11) {
			// C13chからC21ch
			base_freq_hz = (111 + (freq->freq_no - 3) * 6) * 1000000U;
		} else if (freq->freq_no == 12) {
			// C22chは規格上164から170MHz帯の中心167MHz
			base_freq_hz = 167000000U;
		} else if (freq->freq_no >= 13 && freq->freq_no <= 16) {
			// VHF 4chから7ch
			base_freq_hz = (173 + (freq->freq_no - 13) * 6) * 1000000U;
		} else if (freq->freq_no >= 17 && freq->freq_no <= 21) {
			// VHF 8chから12ch
			base_freq_hz = (195 + (freq->freq_no - 17) * 6) * 1000000U;
		} else {
			// C23chからC63ch
			base_freq_hz = (225 + (freq->freq_no - 22) * 6) * 1000000U;
		}

		if (freq->slot > 10000000) {
			// 直接指定された周波数でISDB-C選局
			ret = cxd2878_tune_isdbc(adapter->fe, (u32)freq->slot, 0, true);
			if (ret)
				return ret;
			tbs6281_wait_lock(adapter->fe, 700);
		} else if (freq->slot <= 0) {
			// CATV帯域の全QAM自動追従ISDB-Cスルー選局
			ret = cxd2878_tune_isdbc(adapter->fe, base_freq_hz, 0, true);
			if (ret)
				return ret;
			tbs6281_wait_lock(adapter->fe, 700);
		} else {
			// slotが1以上の時はISDB-CのTSMF分離モード選局
			ret = cxd2878_tune_isdbc(adapter->fe, base_freq_hz,
						 (u16)freq->slot, false);
			if (ret)
				return ret;
			tbs6281_wait_lock(adapter->fe, 700);
		}
		return 0;
	}

	return -EINVAL;
}

// PTX互換IOCTLリクエストを処理するディスパッチャ関数
static long tbs6281_chrdev_ioctl(struct file *file,
				 unsigned int cmd, unsigned long arg)
{
	struct tbsecp3_adapter *adapter = file->private_data;
	unsigned long flags;
	int ret = 0;

	if (!adapter)
		return -EINVAL;

	// LNB電源やシステムモード設定等のダミーリクエストは即時復帰
	switch (cmd) {
	case PTX_ENABLE_LNB_POWER:
	case PTX_DISABLE_LNB_POWER:
	case PTX_SET_SYSTEM_MODE:
		return 0;
	}

	mutex_lock(&adapter->lock);

	switch (cmd) {
	// チャンネル設定および選局実行
	case PTX_SET_CHANNEL:
	{
		struct ptx_freq freq;

		if (copy_from_user(&freq, (void __user *)arg, sizeof(freq))) {
			ret = -EFAULT;
			break;
		}
		ret = tbs6281_set_channel(adapter, &freq);
		break;
	}

	// DMA受信およびストリーミング開始
	case PTX_START_STREAMING:
		if (adapter->streaming) {
			ret = -EALREADY;
			break;
		}
		tbs6281_ringbuffer_reset(&adapter->ringbuf);
		spin_lock_irqsave(&adapter->ringbuf.lock, flags);
		adapter->ringbuf.running = true;
		spin_unlock_irqrestore(&adapter->ringbuf.lock, flags);

		tbsecp3_dma_enable(adapter);
		adapter->streaming = true;
		break;

	// DMA受信およびストリーミング停止
	case PTX_STOP_STREAMING:
		if (!adapter->streaming) {
			ret = -EALREADY;
			break;
		}
		tbsecp3_dma_disable(adapter);
		spin_lock_irqsave(&adapter->ringbuf.lock, flags);
		adapter->ringbuf.running = false;
		spin_unlock_irqrestore(&adapter->ringbuf.lock, flags);

		wake_up_interruptible(&adapter->ringbuf.wq);
		adapter->streaming = false;
		break;

	// CNR未加工値の取得
	case PTX_GET_CNR:
	{
		int cnr = 0;

		ret = cxd2878_read_cnr_raw(adapter->fe, &cnr);
		if (ret)
			break;
		if (put_user(cnr, (int __user *)arg))
			ret = -EFAULT;
		break;
	}

	// AGC未加工値の取得
	case _IOR(0x8d, 0x20, int):
	{
		int agc = 0;

		ret = cxd2878_read_agc_raw(adapter->fe, &agc);
		if (ret)
			break;
		if (put_user(agc, (int __user *)arg))
			ret = -EFAULT;
		break;
	}

	// ISDB-Cステータスレジスタ値の取得
	case _IOR(0x8d, 0x21, int):
	{
		int regs = 0;

		cxd2878_read_isdbc_regs(adapter->fe, &regs);
		if (put_user(regs, (int __user *)arg))
			ret = -EFAULT;
		break;
	}

	// ISDB-C全レジスタバンクの一括ダンプ
	case _IOR(0x8d, 0x22, int):
	{
		u8 dump[256];

		cxd2878_dump_isdbc_banks(adapter->fe, dump);
		if (copy_to_user((void __user *)arg, dump, sizeof(dump)))
			ret = -EFAULT;
		break;
	}

	// 復調器レジスタの直接読み書きアクセス
	case _IOWR(0x8d, 0x23, int):
	{
		u8 rargs[4];

		if (copy_from_user(rargs, (void __user *)arg, 4)) {
			ret = -EFAULT;
			break;
		}
		ret = cxd2878_reg_rw(adapter->fe, rargs[0], rargs[1], &rargs[2], rargs[3] ? true : false);
		if (ret)
			break;
		if (copy_to_user((void __user *)arg, rargs, 4))
			ret = -EFAULT;
		break;
	}

	default:
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&adapter->lock);
	return ret;
}

// キャラクタデバイスのファイル操作構造体
static const struct file_operations tbs6281_fops = {
	.owner          = THIS_MODULE,
	.open           = tbs6281_chrdev_open,
	.release        = tbs6281_chrdev_release,
	.read           = tbs6281_chrdev_read,
	.poll           = tbs6281_chrdev_poll,
	.unlocked_ioctl = tbs6281_chrdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = tbs6281_chrdev_ioctl,
#endif
};

// 作成されるデバイスノードのパーミッションを設定するコールバック
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
static char *tbs6281_devnode(const struct device *dev, umode_t *mode)
#else
static char *tbs6281_devnode(struct device *dev, umode_t *mode)
#endif
{
	if (mode)
		*mode = 0666;
	return NULL;
}

// デバイスメジャー番号割り当てとデバイスクラスを生成する関数
int __init tbs6281_chrdev_class_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&tbs6281_dev_base, 0,
				  TBS6281_MAX_MINORS, TBS6281_DEVICE_NAME);
	if (ret < 0)
		return ret;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
	tbs6281_class = class_create(TBS6281_DEVICE_NAME);
#else
	tbs6281_class = class_create(THIS_MODULE, TBS6281_DEVICE_NAME);
#endif
	if (IS_ERR(tbs6281_class)) {
		ret = PTR_ERR(tbs6281_class);
		unregister_chrdev_region(tbs6281_dev_base, TBS6281_MAX_MINORS);
		return ret;
	}

	tbs6281_class->devnode = tbs6281_devnode;
	return 0;
}

// デバイスクラスを破棄してメジャー番号を返却する関数
void tbs6281_chrdev_class_exit(void)
{
	if (tbs6281_class && !IS_ERR(tbs6281_class))
		class_destroy(tbs6281_class);
	unregister_chrdev_region(tbs6281_dev_base, TBS6281_MAX_MINORS);
}

// アダプタ単位のキャラクタデバイスおよびリングバッファを生成する関数
int tbs6281_chrdev_init(struct tbsecp3_adapter *adapter)
{
	int i, minor = -1, ret;

	mutex_lock(&tbs6281_minor_lock);
	for (i = 0; i < TBS6281_MAX_MINORS; i++) {
		if (!tbs6281_minor_used[i]) {
			tbs6281_minor_used[i] = true;
			minor = i;
			break;
		}
	}
	mutex_unlock(&tbs6281_minor_lock);

	if (minor < 0)
		return -ENOSPC;

	adapter->minor_num = minor;
	adapter->devno = MKDEV(MAJOR(tbs6281_dev_base),
			       MINOR(tbs6281_dev_base) + minor);
	mutex_init(&adapter->lock);
	adapter->is_open = false;
	adapter->streaming = false;

	// 16MBリングバッファをvmallocで確保
	spin_lock_init(&adapter->ringbuf.lock);
	init_waitqueue_head(&adapter->ringbuf.wq);
	adapter->ringbuf.size = TBS6281_RINGBUFFER_SIZE;
	adapter->ringbuf.head = 0;
	adapter->ringbuf.tail = 0;
	adapter->ringbuf.running = false;
	adapter->ringbuf.data = vmalloc(TBS6281_RINGBUFFER_SIZE);
	if (!adapter->ringbuf.data) {
		ret = -ENOMEM;
		goto err_minor;
	}

	cdev_init(&adapter->cdev, &tbs6281_fops);
	adapter->cdev.owner = THIS_MODULE;

	ret = cdev_add(&adapter->cdev, adapter->devno, 1);
	if (ret < 0)
		goto err_ringbuf;

	adapter->cdev_registered = true;
	adapter->chrdev_device = device_create(tbs6281_class,
					       &adapter->dev->pci_dev->dev,
					       adapter->devno, NULL,
					       "%s%d", TBS6281_DEVICE_NAME, minor);
	if (IS_ERR(adapter->chrdev_device)) {
		ret = PTR_ERR(adapter->chrdev_device);
		adapter->chrdev_device = NULL;
		goto err_cdev;
	}

	dev_info(&adapter->dev->pci_dev->dev,
		 "Registered character device /dev/%s%d\n",
		 TBS6281_DEVICE_NAME, minor);
	return 0;

err_cdev:
	cdev_del(&adapter->cdev);
	adapter->cdev_registered = false;
err_ringbuf:
	vfree(adapter->ringbuf.data);
	adapter->ringbuf.data = NULL;
err_minor:
	mutex_lock(&tbs6281_minor_lock);
	tbs6281_minor_used[minor] = false;
	mutex_unlock(&tbs6281_minor_lock);
	return ret;
}

// アダプタ単位のキャラクタデバイスおよびリングバッファを解放する関数
void tbs6281_chrdev_exit(struct tbsecp3_adapter *adapter)
{
	if (!adapter)
		return;

	if (adapter->chrdev_device) {
		device_destroy(tbs6281_class, adapter->devno);
		adapter->chrdev_device = NULL;
	}

	if (adapter->cdev_registered) {
		cdev_del(&adapter->cdev);
		adapter->cdev_registered = false;
	}

	if (adapter->ringbuf.data) {
		vfree(adapter->ringbuf.data);
		adapter->ringbuf.data = NULL;
	}

	if (adapter->minor_num >= 0 && adapter->minor_num < TBS6281_MAX_MINORS) {
		mutex_lock(&tbs6281_minor_lock);
		tbs6281_minor_used[adapter->minor_num] = false;
		mutex_unlock(&tbs6281_minor_lock);
	}
}
