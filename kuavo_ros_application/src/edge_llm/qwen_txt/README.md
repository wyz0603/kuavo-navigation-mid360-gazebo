# Qwen Text LLM ROS Package

ROS package that provides a service-topic bridge for the Qwen text-based large language model (LLM). This package allows ROS nodes to query the Qwen LLM API running on port 9000 and receive responses asynchronously via ROS topics.

## Features

- **ROS Service**: `/qwen_query` - Send queries and receive UUID for tracking
- **ROS Topic**: `/qwen_response` - Receive LLM responses asynchronously
- **UUID Tracking**: Each query gets a unique UUID based on content and timestamp
- **Non-blocking**: Service returns immediately with UUID while processing happens in background
- **Error Handling**: Comprehensive error handling with timeout and retry logic

## Architecture

```
ROS Client → /qwen_query service → Qwen Node → HTTP API (port 9000)
                                      ↓
ROS Client ← /qwen_response topic ← Qwen Node
```

## Installation

### Prerequisites

- ROS Noetic (or compatible version)
- Python 3.6+
- `requests` Python library

Install Python dependencies:
```bash
pip3 install requests
```

### Building the Package

1. Clone this package into your catkin workspace:
```bash
cd ~/catkin_ws/src
# Assuming the package is already here
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
# or
catkin build qwen_txt
```

3. Source the workspace:
```bash
source ~/catkin_ws/devel/setup.bash
```

## Usage

### Starting the Qwen LLM API Server

Before running the ROS node, ensure the Qwen LLM API server is running on port 9000:

```bash
# Start your Qwen LLM server (example - adjust based on your setup)
# The server should expose an OpenAI-compatible API at http://localhost:9000
```

You can test the API with the provided script:
```bash
rosrun qwen_txt test_llm.sh "你好，请介绍一下你自己"
```

### Starting the ROS Node

#### Using roslaunch (Recommended)

The easiest way to start the node is with the provided launch file:

```bash
# Start with default parameters
roslaunch qwen_txt qwen_text.launch

# Start with custom parameters
roslaunch qwen_txt qwen_text.launch api_url:=http://192.168.1.100:9000/v1/chat/completions temperature:=0.8

# All available launch arguments:
roslaunch qwen_txt qwen_text.launch \
  api_url:=http://localhost:9000/v1/chat/completions \
  model:=Qwen2.5-7B-Instruct-q4f16_ft-MLC \
  temperature:=0.7 \
  max_tokens:=500 \
  node_name:=qwen_text_node
```

#### Using rosrun (Manual)

Alternatively, you can start the node directly:

```bash
# Start roscore if not already running
roscore &

# Start the Qwen ROS node
rosrun qwen_txt main.py
```

### Configuration Parameters

The node accepts the following ROS parameters:

- `~api_url`: URL of the LLM API (default: `http://localhost:9000/v1/chat/completions`)
- `~model`: Model name to use (default: `Qwen2.5-7B-Instruct-q4f16_ft-MLC`)
- `~temperature`: Sampling temperature (default: `0.7`)
- `~max_tokens`: Maximum tokens in response (default: `500`)

Example with custom parameters:
```bash
rosrun qwen_txt main.py _api_url:=http://192.168.1.100:9000/v1/chat/completions _temperature:=0.8
```

## API Reference

### Service: `/qwen_query`

**Type**: `qwen_txt/QwenQuery`

**Request**:
- `string query` - The text query to send to the LLM

**Response**:
- `string uuid` - Unique identifier for tracking this query

**Example**:
```bash
rosservice call /qwen_query "query: '你好，请介绍一下你自己'"
```

### Topic: `/qwen_response`

**Type**: `qwen_txt/QwenResponse`

**Fields**:
- `string uuid` - UUID of the original query
- `string query` - The original query text
- `string response` - The LLM's response text
- `time timestamp` - ROS timestamp when response was received

**Example** (Python):
```python
import rospy
from qwen_txt.msg import QwenResponse

def callback(msg):
    print(f"UUID: {msg.uuid}")
    print(f"Query: {msg.query}")
    print(f"Response: {msg.response}")

rospy.init_node('listener')
rospy.Subscriber('/qwen_response', QwenResponse, callback)
rospy.spin()
```

## Testing

A comprehensive test script is provided to test both the service and topic:

```bash
# Test with default query
rosrun qwen_txt test_qwen_node.py

# Test with custom query
rosrun qwen_txt test_qwen_node.py "请解释一下ROS是什么"
```

The test script will:
1. Check if roscore is running
2. Verify the service is available
3. Call the service with your query
4. Listen for the response on the topic
5. Display the complete response with UUID

## Example Client Code

### Python Client

```python
#!/usr/bin/env python3
import rospy
from qwen_txt.srv import QwenQuery
from qwen_txt.msg import QwenResponse

class QwenClient:
    def __init__(self):
        rospy.init_node('qwen_client')

        # Wait for service
        rospy.wait_for_service('/qwen_query')
        self.query_service = rospy.ServiceProxy('/qwen_query', QwenQuery)

        # Subscribe to responses
        self.response_sub = rospy.Subscriber('/qwen_response', QwenResponse, self.response_callback)
        self.pending_queries = {}

    def response_callback(self, msg):
        if msg.uuid in self.pending_queries:
            query = self.pending_queries[msg.uuid]
            print(f"Received response for: {query}")
            print(f"Response: {msg.response}")
            del self.pending_queries[msg.uuid]

    def query(self, text):
        try:
            response = self.query_service(text)
            self.pending_queries[response.uuid] = text
            print(f"Query submitted with UUID: {response.uuid}")
            return response.uuid
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return None

if __name__ == '__main__':
    client = QwenClient()
    client.query("你好，请介绍一下你自己")
    rospy.spin()
```

## Troubleshooting

### Service not found

If you get "service not found" error:
1. Ensure roscore is running: `roscore`
2. Start the Qwen node: `rosrun qwen_txt main.py`
3. Verify service is available: `rosservice list | grep qwen`

### No response on topic

If you're not receiving responses:
1. Check if the LLM API server is running on port 9000
2. Test the API directly: `rosrun qwen_txt test_llm.sh`
3. Check node logs: `rosnode info /qwen_text_node`

### Connection timeout

If you get timeout errors:
1. Verify the API URL is correct
2. Check network connectivity to the API server
3. Increase timeout in main.py if needed

## Files

- `scripts/main.py` - Main ROS node implementation
- `scripts/test_llm.sh` - Direct API test script
- `scripts/test_qwen_node.py` - ROS service/topic test script
- `launch/qwen_text.launch` - ROS launch file with configurable parameters
- `srv/QwenQuery.srv` - Service definition
- `msg/QwenResponse.msg` - Message definition
- `CMakeLists.txt` - Build configuration
- `package.xml` - Package metadata

## License

MIT License

## Author

Developed as part of the edge_llm ROS package suite.
