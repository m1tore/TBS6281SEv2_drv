# TBS6281SE v2 Linux PTX互換ドライバ

TBS(TurboSight)社製チューナーカード **TBS6281SE v2** を、日本国内の地上デジタル放送（ISDB-T）およびケーブルテレビ放送（ISDB-C / CATV）向けに動作させるための Linux カーネルドライバです。

PTX互換キャラクタデバイス（`/dev/tbs6281video*`）を提供し、`recisdb-rs` や `recptx` 等から直接利用できます。

本家(tbsdtv/linux_media)と違い、DKMSパッケージのためサーバー本体のDVBドライバを変更せず使用することができます。

---

## 主な特徴

- **日本国内放送規格準拠**: ISDB-T（地デジ）および ISDB-C（CATVスルー・全QAM自動追従・TSMFハードウェア分離）に対応。
- **PTX互換インターフェース**: 既存の録画ツールからシームレスに利用可能。

---

## 動作環境

- **対応ハードウェア**: TBS6281SE v2
- **動作確認済みOS**: Linux Kernel 7.x
- **必要パッケージ**: `build-essential`, `linux-headers-$(uname -r)`

recisdb-rs等でも使用可能ですが、[recisdb-catv](https://github.com/m1tore/recisdb-catv)との併用をおすすめします。

---

## インストール手順

### 1. 依存ツールのインストール（Ubuntu / Debian 例）

```bash
sudo apt-get update
sudo apt-get install -y build-essential linux-headers-$(uname -r) dkms
```

### 2. ビルドとインストール

#### DKMS によるインストール（推奨）

```bash
sudo cp -r TBS6281SEv2_drv /usr/src/tbs6281_drv-1.0.0
sudo dkms add -m tbs6281_drv -v 1.0.0
sudo dkms build -m tbs6281_drv -v 1.0.0
sudo dkms install -m tbs6281_drv -v 1.0.0
sudo modprobe tbs6281_drv
```

アンインストールする場合:
```bash
sudo dkms remove -m tbs6281_drv/1.0.0 --all
```

#### 手動ビルド・インストール

```bash
cd TBS6281SEv2_drv
make
sudo make install
sudo modprobe tbs6281_drv
```

アンインストールする場合:
```bash
sudo make uninstall
```

---

## デバイスノード

ロード後、以下のデバイスノードが生成されます（パーミッション: `0666`）。

- `/dev/tbs6281video0`（アダプタ 0）
- `/dev/tbs6281video1`（アダプタ 1）

---

## 録画ツールでの利用例（[recisdb-catv](https://github.com/m1tore/recisdb-catv)）

```bash
# 地上波 27ch を録画
recisdb-catv tune -i /dev/tbs6281video0 -c 27 -t GR --time 10 test_gr27.ts

# CATV C50ch（NHK BS トランスモジュレーション波）を録画
recisdb-catv tune -i /dev/tbs6281video0 -c C50 -t BS --time 10 test_c50.ts

# CATV C30ch（TSMF方式・スロット1）を録画
recisdb-catv tune -i /dev/tbs6281video0 -c C30_1 -t CATV --time 10 test_c30_s1.ts
```

---

## モジュールパラメータ

`/etc/modprobe.d/tbs6281_drv.conf` 等で設定可能です。

| パラメータ | 型 | デフォルト | 説明 |
| :--- | :--- | :--- | :--- |
| `enable_msi` | bool | `true` | MSI 割り込みを有効化 |
| `dma_pkts` | uint[] | `128, 128` | アダプタごとの DMA バッファサイズ（TSパケット数 16〜256） |
| `isdbc_offset_khz` | int | `0` | ISDB-C 周波数微調整オフセット（kHz） |

---

## ライセンス

本ドライバは GNU General Public License version 2 or later（GPL-2.0-or-later）の下で公開されています。詳細については同封の [LICENSE](LICENSE) ファイルをご参照ください。
