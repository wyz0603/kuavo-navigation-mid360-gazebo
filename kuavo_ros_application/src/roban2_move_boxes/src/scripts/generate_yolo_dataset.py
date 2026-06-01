import os
import random
import shutil
import yaml
import argparse

def split_dataset(img_dir, label_dir, output_dir, split_ratio=0.8, shuffle=True):
    """
    按照设定比例将数据集划分为训练集和验证集，并生成 YOLO 所需的 YAML 配置文件。

    :param img_dir: 图片所在目录
    :param label_dir: 标注文件所在目录
    :param output_dir: 输出目录
    :param split_ratio: 训练集所占比例，默认为 0.8
    :param shuffle: 是否打乱数据集文件列表，默认为 True
    """
    # 创建输出目录结构
    img_dir = os.path.expanduser(img_dir)
    label_dir = os.path.expanduser(label_dir)
    output_dir = os.path.expanduser(output_dir)

    train_img_dir = os.path.join(output_dir, 'images', 'train')
    train_label_dir = os.path.join(output_dir, 'labels', 'train')
    val_img_dir = os.path.join(output_dir, 'images', 'val')
    val_label_dir = os.path.join(output_dir, 'labels', 'val')
    classes_file = os.path.join(label_dir, 'classes.txt')

    # 读取类别列表
    classes = None
    with open(classes_file, 'r') as f:
        classes = [line.strip() for line in f.readlines()]

    # 创建目标目录
    for dir_path in [train_img_dir, train_label_dir, val_img_dir, val_label_dir]:
        os.makedirs(dir_path, exist_ok=True)

    # 获取所有图片文件名
    img_files = [f for f in os.listdir(img_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

    # 根据 shuffle 参数决定是否打乱文件列表
    if shuffle:
        random.shuffle(img_files)

    # 计算划分点
    split_index = int(len(img_files) * split_ratio)

    # 划分训练集和验证集
    train_files = img_files[:split_index]
    val_files = img_files[split_index:]

    def copy_files(file_list, img_dest, label_dest):
        for img_file in file_list:
            img_src = os.path.join(img_dir, img_file)
            label_file = os.path.splitext(img_file)[0] + '.txt'
            label_src = os.path.join(label_dir, label_file)

            # 复制图片和对应的标注文件
            if os.path.exists(img_src) and os.path.exists(label_src):
                shutil.copy2(img_src, img_dest)
                shutil.copy2(label_src, label_dest)

    # 复制训练集文件
    copy_files(train_files, train_img_dir, train_label_dir)
    # 复制验证集文件
    copy_files(val_files, val_img_dir, val_label_dir)

    print(f"训练集: {len(train_files)} 张图片")
    print(f"验证集: {len(val_files)} 张图片")

    # 生成 YAML 文件
    yaml_data = {
        "train": os.path.relpath(train_img_dir, output_dir),
        "val": os.path.relpath(val_img_dir, output_dir)
    }
    if classes:
        yaml_data["nc"] = len(classes)
        yaml_data["names"] = {i: cls for i, cls in enumerate(classes)}

    yaml_path = os.path.join(output_dir, 'dataset.yaml')
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
    print(f"YAML 文件已生成: {yaml_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="划分YOLO数据集并生成YAML配置文件")
    parser.add_argument('--img_dir', type=str, required=True,
                        help='图片所在目录路径 (例如: ~/Desktop/yolo/images)')
    parser.add_argument('--label_dir', type=str, required=True,
                        help='标注文件所在目录路径 (例如: ~/Desktop/yolo/labels)')
    parser.add_argument('--output_dir', type=str, required=True,
                        help='输出目录路径 (例如: ~/Desktop/yolo/dataset)')
    parser.add_argument('--split_ratio', type=float, default=0.8,
                        help='训练集所占比例 (默认: 0.8)')
    parser.add_argument('--shuffle', action='store_true',
                        help='是否打乱数据集文件列表 (默认: 不打乱)')

    args = parser.parse_args()

    split_dataset(
        img_dir=args.img_dir,
        label_dir=args.label_dir,
        output_dir=args.output_dir,
        split_ratio=args.split_ratio,
        shuffle=args.shuffle
    )