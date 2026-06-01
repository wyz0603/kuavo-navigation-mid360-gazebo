#!/usr/bin/env python3
"""
Create a test image for Qwen VL ROS node testing.
This script generates an image with various shapes and text for vision-language model testing.
"""

from PIL import Image, ImageDraw, ImageFont


def create_test_image(output_path):
    """
    Create a test image with shapes and text.

    Args:
        output_path: Path where the test image should be saved
    """
    # Create test image
    img = Image.new('RGB', (400, 300), color='white')
    draw = ImageDraw.Draw(img)

    # Draw shapes
    draw.rectangle([50, 50, 150, 150], fill='red', outline='black', width=3)
    draw.ellipse([200, 50, 300, 150], fill='blue', outline='black', width=3)
    draw.polygon([(125, 180), (75, 250), (175, 250)], fill='green', outline='black')

    # Add text
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
    except:
        font = ImageFont.load_default()

    draw.text((50, 260), "Test VL Image", fill='black', font=font)

    # Save
    img.save(output_path)
    print(f"Test image created at: {output_path}")


if __name__ == "__main__":
    import sys

    # Default path if not specified
    output_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/qwen_vl_test_image.png'
    create_test_image(output_path)
