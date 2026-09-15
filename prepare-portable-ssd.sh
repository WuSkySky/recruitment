#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# prepare-portable-ssd.sh
#
# 为「便携 Linux 外接固态硬盘」准备分区结构。
#
# 默认行为：只做只读检查并打印将要执行的命令，绝不修改磁盘。
# 真正写入必须显式传入 --confirm <设备>，且设备必须通过全部安全校验。
#
# 用法:
#   ./prepare-portable-ssd.sh /dev/sda              # 只读预检（安全）
#   ./prepare-portable-ssd.sh /dev/sda --confirm /dev/sda   # 真正分区（会清空！）
#
# 分区方案 (GPT):
#   p1   2 GiB   FAT32   ESP     → UEFI 引导
#   p2  96 GiB   ext4    PORTROOT→ 便携 Linux 根分区
#   p3   其余    exFAT   DATA    → 跨平台数据区
# ---------------------------------------------------------------------------
set -euo pipefail

ESP_SIZE="2GiB"
ROOT_SIZE="96GiB"
ESP_LABEL="ESP"
ROOT_LABEL="PORTROOT"
DATA_LABEL="DATA"

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
DEV=""
CONFIRM=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --confirm)
            CONFIRM="${2:-}"
            if [[ -z "$CONFIRM" ]]; then
                echo "错误: --confirm 需要跟一个设备路径，例如 --confirm /dev/sda" >&2
                exit 2
            fi
            shift 2
            ;;
        -h|--help)
            sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            if [[ -z "$DEV" ]]; then DEV="$1"; else
                echo "错误: 多余的参数 '$1'" >&2; exit 2
            fi
            shift
            ;;
    esac
done

if [[ -z "$DEV" ]]; then
    echo "用法: $0 <设备> [--confirm <设备>]" >&2
    echo "例如: $0 /dev/sda" >&2
    exit 2
fi

red()  { printf '\033[31m%s\033[0m\n' "$*"; }
grn()  { printf '\033[32m%s\033[0m\n' "$*"; }
ylw()  { printf '\033[33m%s\033[0m\n' "$*"; }
hdr()  { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }

# ===========================================================================
# 第 1 道闸：设备名不得包含 nvme（内置盘铁定是 nvme0n1 / nvme1n1）
# ===========================================================================
hdr "第 1 道闸：拒绝内置 NVMe 设备"
if [[ "$DEV" == *nvme* ]]; then
    red "拒绝执行：'$DEV' 看起来是内置 NVMe 设备。"
    red "本机内置盘为 nvme0n1 (Windows) 和 nvme1n1 (Ubuntu)，绝不能动。"
    exit 1
fi
grn "通过：'$DEV' 不含 nvme。"

# ===========================================================================
# 第 2 道闸：设备必须存在，且是整盘（不是分区）
# ===========================================================================
hdr "第 2 道闸：设备类型校验"
if [[ ! -e "$DEV" ]]; then
    red "拒绝执行：'$DEV' 不存在。"
    red "请先用 lsblk 确认外接盘的实际设备名（插拔后名称可能变化）。"
    exit 1
fi
if [[ ! -b "$DEV" ]]; then
    red "拒绝执行：'$DEV' 存在但不是块设备。"
    exit 1
fi

DEV_TYPE=$(lsblk -ndo TYPE "$DEV" 2>/dev/null || true)
if [[ "$DEV_TYPE" != "disk" ]]; then
    red "拒绝执行：'$DEV' 的 TYPE 是 '$DEV_TYPE'，不是 'disk'（整盘）。"
    red "请传整盘设备（如 /dev/sda），不要传分区（如 /dev/sda1）。"
    exit 1
fi
grn "通过：'$DEV' 是整盘设备。"

# ===========================================================================
# 第 3 道闸：必须是可移动 / USB 设备
# ===========================================================================
hdr "第 3 道闸：USB 可移动性校验"
TRAN=$(lsblk -ndo TRAN "$DEV" 2>/dev/null | tr -d ' ' || true)
RM=$(lsblk -ndo RM "$DEV" 2>/dev/null | tr -d ' ' || true)
echo "  TRAN = ${TRAN:-<空>}    RM = ${RM:-<空>}"

if [[ "$TRAN" != "usb" ]]; then
    red "拒绝执行：'$DEV' 的传输类型是 '${TRAN:-未知}'，不是 'usb'。"
    red "本脚本只允许对外接 USB 盘操作。"
    exit 1
fi
if [[ "$RM" != "1" ]]; then
    ylw "注意：RM=${RM:-空}（该硬盘盒未把自己标记为可移动设备）。"
    ylw "这本身不阻塞操作，但请再次确认 $DEV 确实是你插的外接盘。"
fi
grn "通过：'$DEV' 走 USB 通道。"

# ===========================================================================
# 第 4 道闸：容量合理性（防止把 U 盘之类的小盘当目标）
# ===========================================================================
hdr "第 4 道闸：容量校验"
SIZE_BYTES=$(lsblk -ndbo SIZE "$DEV" 2>/dev/null || echo 0)
SIZE_GB=$(( SIZE_BYTES / 1000000000 ))
echo "  容量: ${SIZE_GB} GB"
if (( SIZE_GB < 400 )); then
    red "拒绝执行：容量 ${SIZE_GB} GB 小于 400 GB，不像是目标固态硬盘。"
    exit 1
fi
grn "通过：容量 ${SIZE_GB} GB。"

# ===========================================================================
# 第 5 道闸：设备不得处于挂载状态
# ===========================================================================
hdr "第 5 道闸：挂载状态校验"
MOUNTS=$(lsblk -nlo MOUNTPOINT "$DEV" 2>/dev/null | grep -v '^$' || true)
if [[ -n "$MOUNTS" ]]; then
    red "拒绝执行：'$DEV' 上有分区处于挂载状态："
    echo "$MOUNTS" | sed 's/^/    /'
    red "请先卸载，例如："
    red "    sudo umount ${DEV}?*"
    red "（若提示 target is busy，先关闭文件管理器里打开的该盘窗口）"
    exit 1
fi
grn "通过：'$DEV' 未挂载。"

# ===========================================================================
# 预检全部通过 —— 打印当前状态与将要执行的命令
# ===========================================================================
hdr "预检全部通过，当前磁盘状态"
lsblk -o NAME,SIZE,TYPE,FSTYPE,LABEL,MOUNTPOINT "$DEV"

cat <<EOF

将要执行的操作（GPT 分区表，会清空整盘）：
  p1  ${ESP_SIZE}   FAT32  label=${ESP_LABEL}      (UEFI 系统分区)
  p2  ${ROOT_SIZE}  ext4   label=${ROOT_LABEL}     (便携 Linux 根分区, 挂载到 /)
  p3  剩余空间      exFAT  label=${DATA_LABEL}     (跨平台数据区)

等价命令：
  sudo wipefs -a ${DEV}
  sudo parted -s ${DEV} mklabel gpt
  sudo parted -s -a optimal ${DEV} mkpart ESP  fat32 1MiB ${ESP_SIZE}
  sudo parted -s ${DEV} set 1 esp on
  sudo parted -s -a optimal ${DEV} mkpart PORTROOT ext4 ${ESP_SIZE} $(( 2 + 96 ))GiB
  sudo parted -s -a optimal ${DEV} mkpart DATA exfat $(( 2 + 96 ))GiB 100%
  sudo partprobe ${DEV}
  sudo mkfs.vfat -F32 -n ${ESP_LABEL} ${DEV}1
  sudo mkfs.ext4 -L ${ROOT_LABEL} -m 1 ${DEV}2
  sudo mkfs.exfat -L ${DATA_LABEL} ${DEV}3

EOF

# ===========================================================================
# 写入阶段：必须显式 --confirm，且必须与目标设备一致
# ===========================================================================
if [[ -z "$CONFIRM" ]]; then
    ylw "当前为【只读预检】模式。未对磁盘做任何修改。"
    echo
    echo "确认无误后，重新运行并加上 --confirm："
    echo "    $0 $DEV --confirm $DEV"
    echo
    echo "注意：该操作不可逆，${DEV} 上的数据会被全部清空。"
    exit 0
fi

if [[ "$CONFIRM" != "$DEV" ]]; then
    red "拒绝执行：--confirm 的值 '$CONFIRM' 与目标设备 '$DEV' 不一致。"
    red "这是防止误敲的保护，请让两者完全相同。"
    exit 1
fi

# ===========================================================================
# 第 6 道闸：人工二次确认（必须逐字输入设备名）
# ===========================================================================
hdr "最后确认"
red "警告：即将清空 ${DEV}（${SIZE_GB} GB）上的全部数据，不可恢复！"
echo
read -r -p "如确认无误，请逐字输入设备名 [${DEV}] : " TYPED
if [[ "$TYPED" != "$DEV" ]]; then
    red "输入不匹配，已取消。磁盘未做任何修改。"
    exit 1
fi

hdr "开始分区"

echo "--> 清除旧文件系统签名"
wipefs -a "$DEV"

echo "--> 写入 GPT 分区表"
parted -s "$DEV" mklabel gpt

echo "--> 创建 ESP (${ESP_SIZE}, FAT32)"
parted -s -a optimal "$DEV" mkpart ESP fat32 1MiB "$ESP_SIZE"
parted -s "$DEV" set 1 esp on

echo "--> 创建根分区 (${ROOT_SIZE}, ext4)"
parted -s -a optimal "$DEV" mkpart PORTROOT ext4 "$ESP_SIZE" 98GiB

echo "--> 创建数据分区 (剩余空间, exFAT)"
parted -s -a optimal "$DEV" mkpart DATA exfat 98GiB 100%

echo "--> 重新读取分区表"
partprobe "$DEV"
udevadm settle 2>/dev/null || sleep 2

echo "--> 格式化 ESP 为 FAT32"
mkfs.vfat -F32 -n "$ESP_LABEL" "${DEV}1"

echo "--> 格式化根分区为 ext4"
mkfs.ext4 -L "$ROOT_LABEL" -m 1 "${DEV}2"

echo "--> 格式化数据分区为 exFAT"
mkfs.exfat -L "$DATA_LABEL" "${DEV}3"

hdr "完成，最终结果"
lsblk -o NAME,SIZE,TYPE,FSTYPE,LABEL,MOUNTPOINT "$DEV"
echo
grn "分区准备完毕。下一步：用 Ubuntu 安装盘启动，在安装器中选择手动分区，"
grn "把 ${DEV}2 挂载为 /，${DEV}1 作为 EFI 分区，并把启动引导器安装到 ${DEV}。"
