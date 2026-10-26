// SPDX-License-Identifier: GPL-2.0-or-later
// TBS ECP3 FPGA DMAコントローラドライバ

#include "tbsecp3.h"

// アダプタごとのDMA転送パケット数設定モジュールパラメータ
static unsigned int dma_pkts[TBSECP3_MAX_ADAPTERS] = { 128, 128 };
module_param_array(dma_pkts, uint, NULL, 0444);
MODULE_PARM_DESC(dma_pkts, "DMA buffer size in TS packets from 16 to 256, default 128");

#define TS_PACKET_SIZE		188

// 受信TSパケットをDMAバッファから取り出してリングバッファへ渡すタスクレット
static void tbsecp3_dma_tasklet(unsigned long adap)
{
	struct tbsecp3_adapter *adapter = (struct tbsecp3_adapter *)adap;
	struct tbsecp3_dev *dev = adapter->dev;
	u32 read_buffer, next_buffer;
	u8 *data;
	int i;

	spin_lock(&adapter->adap_lock);

	next_buffer = (tbs_read(adapter->dma.base, TBSECP3_DMA_STAT) -
		       TBSECP3_DMA_PRE_BUFFERS + 1) & (TBSECP3_DMA_BUFFERS - 1);

	// 初期転送時はバッファ充填完了を待機
	if (adapter->dma.cnt < TBSECP3_DMA_PRE_BUFFERS) {
		adapter->dma.cnt++;
	} else {
		read_buffer = (u32)adapter->dma.next_buffer;

		// 未処理の全DMAバッファを走査してデータ取り出し
		while (read_buffer != next_buffer) {
			data = adapter->dma.buf[read_buffer];

			// TSヘッダー同期バイトの位置合わせ処理
			if (unlikely(data[adapter->dma.offset] != 0x47)) {
				for (i = 0; i < TS_PACKET_SIZE; i++) {
					if ((data[i] == 0x47) &&
					    (data[i + TS_PACKET_SIZE] == 0x47) &&
					    (data[i + 2 * TS_PACKET_SIZE] == 0x47) &&
					    (data[i + 3 * TS_PACKET_SIZE] == 0x47)) {
						adapter->dma.offset = i;
						break;
					}
				}
			}

			// 同期オフセットが存在する場合は先頭ポインタを補正
			if (unlikely(adapter->dma.offset != 0)) {
				data += adapter->dma.offset;
				if (read_buffer == (TBSECP3_DMA_BUFFERS - 1)) {
					memcpy(adapter->dma.buf[TBSECP3_DMA_BUFFERS],
					       adapter->dma.buf[0], adapter->dma.offset);
				}
			}

			// DVBデマルチプレクサを介さず直接リングバッファへTSパケットを書き込む
			tbs6281_ringbuffer_write(&adapter->ringbuf, data,
						 adapter->dma.buffer_size);

			read_buffer = (read_buffer + 1) & (TBSECP3_DMA_BUFFERS - 1);
		}
	}

	adapter->dma.next_buffer = (u8)next_buffer;
	spin_unlock(&adapter->adap_lock);
}

// DMA受信および関連ハードウェア割り込みを有効化する関数
void tbsecp3_dma_enable(struct tbsecp3_adapter *adap)
{
	struct tbsecp3_dev *dev = adap->dev;
	unsigned long flags;

	spin_lock_irqsave(&adap->adap_lock, flags);
	adap->dma.offset = 0;
	adap->dma.cnt = 0;
	adap->dma.next_buffer = 0;
	tbs_read(adap->dma.base, TBSECP3_DMA_STAT);
	tbs_write(TBSECP3_INT_BASE, TBSECP3_DMA_IE(adap->cfg->ts_in), 1);
	tbs_write(adap->dma.base, TBSECP3_DMA_EN, 1);
	spin_unlock_irqrestore(&adap->adap_lock, flags);
}

// DMA受信およびハードウェア割り込みを無効化する関数
void tbsecp3_dma_disable(struct tbsecp3_adapter *adap)
{
	struct tbsecp3_dev *dev = adap->dev;
	unsigned long flags;

	spin_lock_irqsave(&adap->adap_lock, flags);
	tbs_read(adap->dma.base, TBSECP3_DMA_STAT);
	tbs_write(TBSECP3_INT_BASE, TBSECP3_DMA_IE(adap->cfg->ts_in), 0);
	tbs_write(adap->dma.base, TBSECP3_DMA_EN, 0);
	spin_unlock_irqrestore(&adap->adap_lock, flags);
}

// FPGA内のDMAアドレスおよびバッファサイズレジスタを設定する関数
void tbsecp3_dma_reg_init(struct tbsecp3_dev *dev)
{
	int i;
	struct tbsecp3_adapter *adapter = dev->adapter;

	for (i = 0; i < dev->info->adapters; i++) {
		tbs_write(adapter->dma.base, TBSECP3_DMA_EN, 0);
		tbs_write(adapter->dma.base, TBSECP3_DMA_ADDRH,
			  upper_32_bits(adapter->dma.dma_addr));
		tbs_write(adapter->dma.base, TBSECP3_DMA_ADDRL,
			  lower_32_bits(adapter->dma.dma_addr));
		tbs_write(adapter->dma.base, TBSECP3_DMA_TSIZE, adapter->dma.page_size);
		tbs_write(adapter->dma.base, TBSECP3_DMA_BSIZE, adapter->dma.buffer_size);
		adapter++;
	}
}

// 確保済みのDMAコヒーレントバッファを解放する関数
void tbsecp3_dma_free(struct tbsecp3_dev *dev)
{
	struct tbsecp3_adapter *adapter = dev->adapter;
	int i;

	for (i = 0; i < dev->info->adapters; i++) {
		if (adapter->dma.buf[0] == NULL) {
			adapter++;
			continue;
		}

		dma_free_coherent(&dev->pci_dev->dev,
				  adapter->dma.page_size + 0x100,
				  adapter->dma.buf[0], adapter->dma.dma_addr);
		adapter->dma.buf[0] = NULL;
		adapter++;
	}
}

// 各アダプタのDMAバッファ確保とタスクレット初期化を行う関数
int tbsecp3_dma_init(struct tbsecp3_dev *dev)
{
	struct tbsecp3_adapter *adapter = dev->adapter;
	int i, j;

	for (i = 0; i < dev->info->adapters; i++) {
		if (dma_pkts[i] < 16)
			dma_pkts[i] = 16;
		if (dma_pkts[i] > 256)
			dma_pkts[i] = 256;

		adapter->dma.buffer_pkts = dma_pkts[i];
		adapter->dma.buffer_size = dma_pkts[i] * TS_PACKET_SIZE;
		adapter->dma.page_size = adapter->dma.buffer_size * TBSECP3_DMA_BUFFERS;

		// 連続したDMAコヒーレントメモリ領域を割り当て
		adapter->dma.buf[0] = dma_alloc_coherent(&dev->pci_dev->dev,
							 adapter->dma.page_size + 0x100,
							 &adapter->dma.dma_addr, GFP_KERNEL);
		if (!adapter->dma.buf[0])
			goto err;

		adapter->dma.base = TBSECP3_DMA_BASE(adapter->cfg->ts_in);
		adapter->dma.cnt = 0;
		adapter->dma.next_buffer = 0;
		for (j = 1; j < TBSECP3_DMA_BUFFERS + 1; j++)
			adapter->dma.buf[j] = adapter->dma.buf[j - 1] + adapter->dma.buffer_size;

		tasklet_init(&adapter->tasklet, tbsecp3_dma_tasklet, (unsigned long)adapter);
		spin_lock_init(&adapter->adap_lock);
		adapter++;
	}
	tbsecp3_dma_reg_init(dev);
	return 0;

err:
	dev_err(&dev->pci_dev->dev, "dma: memory alloc failed\n");
	tbsecp3_dma_free(dev);
	return -ENOMEM;
}
