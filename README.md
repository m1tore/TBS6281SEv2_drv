# TBS6281SE v2 Linux PTX互換ドライバ

TBS（TurboSight）社製チューナーカード **TBS6281SE v2** を、日本国内の地上デジタル放送（ISDB-T）およびケーブルテレビ放送（ISDB-C / CATV）向けに動作させるための Linux カーネルドライバです。

PTX互換キャラクタデバイス（`/dev/tbs6281video*`）を提供し、`recisdb-catv` や `recptx` 等から直接利用できます。

公式ドライバ（tbsdtv/linux_media）と異なり、独立したDKMSモジュールとしてビルド可能なため、ホストシステムの既存DVB環境を変更することなく導入できます。

**ISDB-T（地上波）および ISDB-C 64QAM（CATV・BS再送信）は安定して動作しますが、ISDB-C 256QAM（高度CATV等）の動作は不安定です。レジスタ周りの設定を手探りでやっているので、より良い設定があればPRしていただけると幸いです。**

---

## 主な特徴

- **国内放送規格準拠**: ISDB-T（地デジ）および ISDB-C（64QAM / 256QAM、TSMFハードウェア分離）に対応。
- **PTX互換インターフェース**: 既存の録画ツールからシームレスに利用可能。
- **DKMS対応**: カーネル更新時も自動で再ビルド。

---

## 動作環境

- **対応ハードウェア**: TBS6281SE v2
- **動作確認済みOS**: Linux Kernel 7.x
- **必要パッケージ**: `build-essential`, `linux-headers-$(uname -r)`, `dkms`
- **推奨録画ツール**: [recisdb-catv](https://github.com/m1tore/recisdb-catv)

---

## インストール手順

### 1. 依存パッケージのインストール

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

- `/dev/tbs6281video0`（チューナー 0）
- `/dev/tbs6281video1`（チューナー 1）

---

## 録画ツールでの利用例（[recisdb-catv](https://github.com/m1tore/recisdb-catv)）

```bash
# 地上波 27ch を録画
recisdb-catv tune -i /dev/tbs6281video0 -c 27 -t GR --time 10 test_gr27.ts

# CATV C50ch（NHK BS トランスモジュレーション波）を録画
recisdb-catv tune -i /dev/tbs6281video0 -c C50 -t BS --time 10 test_c50.ts

# CATV C30ch（TSMF方式・スロット1）を録画
recisdb-catv tune -i /dev/tbs6281video0 -c C30_1 -t CATV --time 10 test_c30_s1.ts

# 物理周波数を直接指定して録画
recisdb-catv tune -i /dev/tbs6281video0 -c 105.18MHz -t CATV --time 10 test_freq.ts
```

---

## モジュールパラメータ

通常は変更不要（デフォルト値で動作）ですが、必要に応じて `/etc/modprobe.d/tbs6281_drv.conf` 等で調整可能です。

| パラメータ | 型 | デフォルト | 説明 |
| :--- | :--- | :--- | :--- |
| `enable_msi` | bool | `true` | MSI 割り込みの有効化 |
| `dma_pkts` | uint[] | `128, 128` | アダプタごとの DMA バッファサイズ（TSパケット数 16〜256） |
| `isdbc_offset_khz` | int | `0` | ISDB-C 周波数微調整オフセット（kHz）。送出設備の都合で中心周波数にズレがある局のみ指定します。recisdb-catv 等で直接周波数を指定して選局する場合は変更不要です。 |

---

## デバッグ・実験インターフェース

復調器（CXD2878）のレジスタ直接アクセス IOCTL や各種微調整用パラメータについては、[RESEARCH_NOTES.md](RESEARCH_NOTES.md) をご参照ください。

---

## ライセンス

本ドライバは GNU General Public License version 2 or later（GPL-2.0-or-later）の下で公開されています。詳細については同封の [LICENSE](LICENSE) ファイルをご参照ください。
