# TBS6281SE v2 デバッグ・実験ガイド

本ドキュメントは、TBS6281SE v2ドライバにおけるデバッグ機能および実験用インターフェースについてまとめた技術資料です。

---

## 復調器レジスタ直接アクセスIOCTL

キャラクタデバイスに対して以下のIOCTLを発行することで、CXD2878復調器の任意バンク・レジスタを直接読み書きできます。

- **IOCTL番号**: `_IOWR(0x8d, 0x23, int)`
- **バッファ構成（4バイト）**:
  - Byte 0: バンク番号
  - Byte 1: レジスタアドレス
  - Byte 2: 書き込み値（読み出し時は取得値が格納されます）
  - Byte 3: 操作種別（0 = 読み出し, 1 = 書き込み）

### Pythonスクリプト例
```python
import fcntl
import struct

fd = open("/dev/tbs6281video0", "rb+", buffering=0)

# 例: Bank 0x48 Reg 0xAC（RSエラーミュート設定）を読み出す
data = struct.pack("BBBB", 0x48, 0xAC, 0x00, 0x00)
res = fcntl.ioctl(fd, 0xC0048D23, data)
bank, reg, val, op = struct.unpack("BBBB", res)
print(f"Bank 0x{bank:02X} Reg 0x{reg:02X} = 0x{val:02X}")

# 例: Bank 0x48 Reg 0xAC を 0x00（ミュート解除）に設定する
data = struct.pack("BBBB", 0x48, 0xAC, 0x00, 0x01)
fcntl.ioctl(fd, 0xC0048D23, data)

fd.close()
```

---

## チューナー微調整用モジュールパラメータ

復調回路のゲインやフィルタ帯域を動的に調整するためのパラメータです。`/etc/modprobe.d/tbs6281_drv.conf` や `/sys/module/tbs6281_drv/parameters/` から設定可能です。

- `isdbc_offset_khz`: ISDB-C周波数オフセット（kHz、デフォルト0）
- `isdbc_fif_offset`: IF中心周波数オフセット（デフォルト-6）
- `isdbc_bw_offset`: フィルタ帯域オフセット（デフォルト-4）
- `isdbc_bbagc_target`: ベースバンドAGC目標値（デフォルト0x28）
- `isdbc_outlmt`: 出力リミッタ設定（デフォルト1）
- `dma_tlv_unk`: FPGA DMA生データ受信制御レジスタ設定（デフォルト0）
