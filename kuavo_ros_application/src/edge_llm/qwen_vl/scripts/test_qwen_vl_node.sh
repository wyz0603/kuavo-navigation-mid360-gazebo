#!/bin/bash
# Test script for Qwen VL ROS node - tests both base64 and filesystem path services

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Qwen VL ROS Node Test Script${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS environment not sourced${NC}"
    echo "Please run: source /opt/ros/YOUR_DISTRO/setup.bash"
    exit 1
fi

# Check if roscore is running
if ! rostopic list &> /dev/null; then
    echo -e "${RED}Error: roscore is not running${NC}"
    echo "Please start roscore in another terminal"
    exit 1
fi

# Check if node is running
if ! rosservice list | grep -q "/qwen_vl_query"; then
    echo -e "${RED}Error: Qwen VL node is not running${NC}"
    echo "Please launch the node first:"
    echo "  roslaunch qwen_vl qwen_vl.launch"
    exit 1
fi

echo -e "${GREEN}✓ ROS environment OK${NC}"
echo -e "${GREEN}✓ Qwen VL node is running${NC}"
echo ""

# Create a test image using Python
echo -e "${YELLOW}Creating test image...${NC}"
TEST_IMAGE_PATH="/tmp/qwen_vl_test_image.png"
python3 << 'EOF'
from PIL import Image, ImageDraw, ImageFont

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
img.save('/tmp/qwen_vl_test_image.png')
print("Test image created at: /tmp/qwen_vl_test_image.png")
EOF

if [ ! -f "$TEST_IMAGE_PATH" ]; then
    echo -e "${RED}Error: Failed to create test image${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Test image created${NC}"
echo ""

# Test 1: Filesystem Path Service
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Test 1: Filesystem Path Service${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

QUERY1="这张图片里有什么？描述你看到的形状和颜色。"
echo -e "${YELLOW}Query:${NC} $QUERY1"
echo -e "${YELLOW}Image:${NC} $TEST_IMAGE_PATH"
echo ""

# Subscribe to response topic in background
echo -e "${YELLOW}Subscribing to /qwen_vl_response topic...${NC}"
rostopic echo /qwen_vl_response -n 1 > /tmp/qwen_vl_response_path.txt &
TOPIC_PID=$!

# Give it a moment to start subscribing
sleep 1

# Call the service
echo -e "${YELLOW}Calling /qwen_vl_query_path service...${NC}"
rosservice call /qwen_vl_query_path "query: '$QUERY1'
image_path: '$TEST_IMAGE_PATH'" > /tmp/qwen_vl_service_response_path.txt

# Show service response
echo -e "${YELLOW}Service Response:${NC}"
cat /tmp/qwen_vl_service_response_path.txt
echo ""

# Wait for topic response (max 30 seconds)
echo -e "${YELLOW}Waiting for VL model response on topic...${NC}"
for i in {1..30}; do
    if [ -s /tmp/qwen_vl_response_path.txt ]; then
        break
    fi
    sleep 1
    echo -n "."
done
echo ""

# Check if we got a response
if [ -s /tmp/qwen_vl_response_path.txt ]; then
    echo -e "${GREEN}✓ Received response on topic:${NC}"
    cat /tmp/qwen_vl_response_path.txt
    echo ""
else
    echo -e "${RED}✗ No response received within 30 seconds${NC}"
fi

# Cleanup background process
kill $TOPIC_PID 2>/dev/null || true

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Test 2: Base64 Image Service${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Convert image to base64
echo -e "${YELLOW}Converting image to base64...${NC}"
IMAGE_BASE64=$(python3 << EOF
import base64
with open('$TEST_IMAGE_PATH', 'rb') as f:
    img_data = f.read()
    b64 = base64.b64encode(img_data).decode()
    print(b64)
EOF
)

QUERY2="图片中有多少个形状？它们分别是什么颜色？"
echo -e "${YELLOW}Query:${NC} $QUERY2"
echo -e "${YELLOW}Image:${NC} base64-encoded ($(echo $IMAGE_BASE64 | wc -c) bytes)"
echo ""

# Subscribe to response topic in background
echo -e "${YELLOW}Subscribing to /qwen_vl_response topic...${NC}"
rostopic echo /qwen_vl_response -n 1 > /tmp/qwen_vl_response_base64.txt &
TOPIC_PID=$!

# Give it a moment to start subscribing
sleep 1

# Call the service with base64 image
echo -e "${YELLOW}Calling /qwen_vl_query_base64 service...${NC}"

# Create a temporary file with the service call
cat > /tmp/qwen_vl_base64_call.yaml << EOF
query: '$QUERY2'
image_base64: '$IMAGE_BASE64'
EOF

rosservice call /qwen_vl_query_base64 "$(cat /tmp/qwen_vl_base64_call.yaml)" > /tmp/qwen_vl_service_response_base64.txt

# Show service response
echo -e "${YELLOW}Service Response:${NC}"
cat /tmp/qwen_vl_service_response_base64.txt
echo ""

# Wait for topic response (max 30 seconds)
echo -e "${YELLOW}Waiting for VL model response on topic...${NC}"
for i in {1..30}; do
    if [ -s /tmp/qwen_vl_response_base64.txt ]; then
        break
    fi
    sleep 1
    echo -n "."
done
echo ""

# Check if we got a response
if [ -s /tmp/qwen_vl_response_base64.txt ]; then
    echo -e "${GREEN}✓ Received response on topic:${NC}"
    cat /tmp/qwen_vl_response_base64.txt
    echo ""
else
    echo -e "${RED}✗ No response received within 30 seconds${NC}"
fi

# Cleanup
kill $TOPIC_PID 2>/dev/null || true
rm -f /tmp/qwen_vl_*.txt /tmp/qwen_vl_base64_call.yaml

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}All tests completed!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Test image location: $TEST_IMAGE_PATH"
echo "You can delete it with: rm $TEST_IMAGE_PATH"
