from ultralytics import YOLO
import argparse

if __name__ == "__main__":
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="Train YOLO model with custom dataset and epochs.")
    parser.add_argument('--model_path', type=str, default="/home/robot/Projects/ultralytics/yolov8n.pt", help='Path to the YOLO model file')
    parser.add_argument('--dataset_path', type=str, default="/home/robot/Desktop/dataset/dataset.yaml", help='Path to the dataset YAML file')
    parser.add_argument('--epochs', type=int, default=100, help='Number of training epochs')

    args = parser.parse_args()

    # 加载模型
    model = YOLO(args.model_path)

    # 训练模型
    results = model.train(data=args.dataset_path, epochs=args.epochs)
