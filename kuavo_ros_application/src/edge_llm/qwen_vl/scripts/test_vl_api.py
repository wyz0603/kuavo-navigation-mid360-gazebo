#!/usr/bin/env python3
"""
Qwen2.5-VL API 测试脚本
测试文本对话和图像理解功能
"""

import requests
import json
import base64
from io import BytesIO
from PIL import Image, ImageDraw, ImageFont
import sys

API_URL = "http://localhost:9000"

def test_health():
    """测试健康检查"""
    print("="*60)
    print("测试 1: 健康检查")
    print("="*60)
    
    response = requests.get(f"{API_URL}/health")
    result = response.json()
    
    print(f"状态: {result['status']}")
    print(f"模型: {result['model']}")
    print(f"CUDA: {result['cuda_available']}")
    print()

def test_text_chat():
    """测试纯文本对话"""
    print("="*60)
    print("测试 2: 文本对话")
    print("="*60)
    
    questions = [
        "什么是人工智能？用一句话回答。",
        "Python和Java有什么区别？",
        "解释一下什么是深度学习。"
    ]
    
    for i, question in enumerate(questions, 1):
        print(f"\n问题 {i}: {question}")
        
        response = requests.post(
            f"{API_URL}/v1/chat/completions",
            json={
                "messages": [
                    {"role": "user", "content": question}
                ],
                "max_tokens": 200
            }
        )
        
        result = response.json()
        
        if "error" in result:
            print(f"错误: {result['error']}")
        else:
            answer = result["choices"][0]["message"]["content"]
            tokens = result["usage"]
            print(f"回答: {answer}")
            print(f"Token使用: {tokens['prompt_tokens']} + {tokens['completion_tokens']} = {tokens['total_tokens']}")
    
    print()

def create_test_image(text="Hello World", size=(400, 300)):
    """创建一个测试图像"""
    # 创建白色背景图像
    img = Image.new('RGB', size, color='white')
    draw = ImageDraw.Draw(img)
    
    # 绘制一些图形
    # 红色矩形
    draw.rectangle([50, 50, 150, 150], fill='red', outline='black', width=3)
    # 蓝色圆形
    draw.ellipse([200, 50, 300, 150], fill='blue', outline='black', width=3)
    # 绿色三角形
    draw.polygon([(125, 180), (75, 250), (175, 250)], fill='green', outline='black')
    
    # 添加文字
    try:
        # 尝试使用系统字体
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
    except:
        font = ImageFont.load_default()
    
    draw.text((50, 260), text, fill='black', font=font)
    
    return img

def image_to_base64(image):
    """将PIL图像转换为base64字符串"""
    buffered = BytesIO()
    image.save(buffered, format="PNG")
    img_str = base64.b64encode(buffered.getvalue()).decode()
    return f"data:image/png;base64,{img_str}"

def test_image_understanding():
    """测试图像理解"""
    print("="*60)
    print("测试 3: 图像理解")
    print("="*60)
    
    # 创建测试图像
    print("\n创建测试图像...")
    test_img = create_test_image("Test Image")
    test_img.save("/tmp/test_image.png")
    print("测试图像已保存到: /tmp/test_image.png")
    
    # 转换为base64
    img_base64 = image_to_base64(test_img)
    
    # 测试问题
    test_cases = [
        {
            "question": "这张图片里有什么？描述你看到的内容。",
            "image": img_base64
        },
        {
            "question": "图片中有哪些颜色的形状？",
            "image": img_base64
        },
        {
            "question": "图片中的文字是什么？",
            "image": img_base64
        }
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n问题 {i}: {test_case['question']}")
        
        response = requests.post(
            f"{API_URL}/v1/chat/completions",
            json={
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "image_url", "image_url": {"url": test_case['image']}},
                            {"type": "text", "text": test_case['question']}
                        ]
                    }
                ],
                "max_tokens": 300
            }
        )
        
        result = response.json()
        
        if "error" in result:
            print(f"错误: {result.get('error', 'Unknown error')}")
            if "traceback" in result:
                print(f"详细信息:\n{result['traceback']}")
        else:
            answer = result["choices"][0]["message"]["content"]
            tokens = result["usage"]
            print(f"回答: {answer}")
            print(f"Token使用: {tokens['total_tokens']}")
    
    print()

def test_models_endpoint():
    """测试模型列表接口"""
    print("="*60)
    print("测试 4: 模型列表")
    print("="*60)
    
    response = requests.get(f"{API_URL}/v1/models")
    result = response.json()
    
    print(f"可用模型数量: {len(result['data'])}")
    for model in result['data']:
        print(f"  - {model['id']}")
    print()

def main():
    print("\n" + "="*60)
    print("Qwen2.5-VL-7B-Instruct API 测试套件")
    print("="*60)
    print()
    
    try:
        # 1. 健康检查
        test_health()
        
        # 2. 文本对话
        test_text_chat()
        
        # 3. 图像理解
        test_image_understanding()
        
        # 4. 模型列表
        test_models_endpoint()
        
        print("="*60)
        print("所有测试完成！")
        print("="*60)
        
    except requests.exceptions.ConnectionError:
        print("\n❌ 错误: 无法连接到 API 服务器")
        print("请确保服务正在运行: sudo docker logs qwen_vl_api")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
