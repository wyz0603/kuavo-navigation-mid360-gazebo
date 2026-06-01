#!/bin/bash

# 将 /home 目录直接挂载到 1T 固态硬盘
# 功能：分区格式化 -> 数据迁移 -> 直接挂载 /home

set -e

# 配置参数
DISK="/dev/nvme0n1"
PARTITION="${DISK}p1"
HOME_MOUNT="/home"
FSTAB_BACKUP="/etc/fstab.backup"
BACKUP_SUFFIX=".backup_$(date +%Y%m%d_%H%M%S)"

# 检查 root 权限
if [[ $EUID -ne 0 ]]; then
  echo "Error: 请使用 sudo 运行此脚本"
  exit 1
fi

# 步骤1：检查并卸载 /media/data
if mountpoint -q /media/data 2>/dev/null; then
  if ! umount /media/data 2>/dev/null; then
    echo "Error: 无法卸载 /media/data，请关闭占用进程后重试"
    exit 1
  fi
fi

# 步骤2：检查并创建分区
if ! lsblk -no NAME "${PARTITION}" &>/dev/null; then
  wipefs -a "$DISK"
  sleep 1
  parted -s "$DISK" mklabel gpt
  parted -s "$DISK" mkpart primary ext4 0% 100%
  sleep 2
  mkfs.ext4 "$PARTITION"
fi

# 步骤3：获取分区 UUID
UUID=$(blkid -s UUID -o value "$PARTITION")
if [ -z "$UUID" ]; then
  echo "Error: 无法获取分区 UUID"
  exit 1
fi

# 步骤4：临时挂载分区
mkdir -p /media/data
if ! mountpoint -q /media/data; then
  mount "$PARTITION" /media/data
fi

# 创建子目录
DATA_SUBDIR="/media/data/home"
mkdir -p "$DATA_SUBDIR"

# 检查 /home 挂载状态
CURRENT_UUID=$(findmnt -no UUID "$HOME_MOUNT" 2>/dev/null || echo "")
if [ "$CURRENT_UUID" = "$UUID" ]; then
  SKIP_MIGRATION=1
elif mountpoint -q "$HOME_MOUNT"; then
  SKIP_MIGRATION=1
else
  SKIP_MIGRATION=0
fi

# 步骤5：迁移数据
if [ "$SKIP_MIGRATION" -eq 0 ]; then
  HOME_SIZE=$(du -sb --exclude='.cache' --exclude='.local/share/gvfs' --exclude='.gvfs' "$HOME_MOUNT" 2>/dev/null | cut -f1 || echo "0")
  if [ "$HOME_SIZE" -gt 1024 ]; then
    # 先删除目标目录，确保干净复制
    rm -rf "$DATA_SUBDIR"
    mkdir -p "$DATA_SUBDIR"

    echo "复制 /home 数据到分区..."
    rsync -av --progress --exclude='.cache' --exclude='.local/share/gvfs' --exclude='.gvfs' "$HOME_MOUNT/" "$DATA_SUBDIR/" || true

    # 验证数据完整性：使用 rsync -n 检查哪些文件未同步
    rsync -avn --exclude='.cache' --exclude='.local/share/gvfs' --exclude='.gvfs' "$HOME_MOUNT/" "$DATA_SUBDIR/" > /tmp/rsync_check.log 2>&1
    MISSING=$(grep -c '^[<>cdltp]' /tmp/rsync_check.log || echo "0")
    TOTAL=$(grep -c '^\.' /tmp/rsync_check.log || echo "0")
    [ "$TOTAL" -gt 0 ] && PERCENT=$((MISSING * 100 / TOTAL)) || PERCENT=0

    if [ $PERCENT -gt 5 ]; then
      echo "Error: 数据完整性验证失败！未同步文件: $MISSING ($PERCENT%)"
      if mountpoint -q /media/data 2>/dev/null; then
        umount /media/data 2>/dev/null || true
      fi
      exit 1
    fi

    # 移动数据到分区根目录
    mv "$DATA_SUBDIR"/* "$DATA_SUBDIR"/.[!.]* /media/data/ 2>/dev/null || true
    rmdir "$DATA_SUBDIR" 2>/dev/null || true

    # 备份原 /home
    mv "$HOME_MOUNT" "$HOME_MOUNT$BACKUP_SUFFIX"
    mkdir -p "$HOME_MOUNT"
  fi
fi

# 步骤6：卸载并挂载到 /home
if mountpoint -q /media/data; then
  if ! umount /media/data 2>/dev/null; then
    echo "Error: 无法卸载 /media/data，请关闭占用进程后重试"
    exit 1
  fi
fi
mount "$PARTITION" "$HOME_MOUNT"

# 步骤7：配置 fstab
sed -i '/\/media\/data/d' /etc/fstab 2>/dev/null || true
if ! grep -q "$UUID" /etc/fstab; then
  echo "UUID=$UUID  $HOME_MOUNT  ext4  defaults,nofail  0  2" >> /etc/fstab
fi

# 完成
echo "完成！请执行: sudo reboot"