import uuid
import pyaudio
import ast
import os
from functools import lru_cache
from typing import List

# 配置信息
ws_connect_config = {
    "base_url": "wss://openspeech.bytedance.com/api/v3/realtime/dialogue",
    "headers": {
        "X-Api-App-ID": "",
        "X-Api-Access-Key": "",
        "X-Api-Resource-Id": "volc.speech.dialog",  # 固定值
        "X-Api-App-Key": "PlgvMymc7f3tQnJ6",  # 固定值
        "X-Api-Connect-Id": str(uuid.uuid4()),
    }
}

start_session_req = {
    "tts": {
        "audio_config": {
            "channel": 1,
            "format": "pcm",
            "sample_rate": 24000
        },
    },
    "dialog": {
        "bot_name": "夸父",
        "system_role": "",
        "speaking_style": "你的说话风格专业，语速适中，语调自然。",
        "location": {
          "city": "深圳",
        },
        "extra": {
            "strict_audit": False
        }
    }
}

@lru_cache(maxsize=1)
def _load_normal_action_names() -> List[str]:
    action_controllers_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "action_controllers.py")
    )
    try:
        with open(action_controllers_path, "r", encoding="utf-8") as f:
            src = f.read()
    except Exception:
        return []

    try:
        tree = ast.parse(src, filename=action_controllers_path)
    except SyntaxError:
        return []

    class _Visitor(ast.NodeVisitor):
        def __init__(self):
            self.names: List[str] = []

        def visit_Assign(self, node: ast.Assign):
            for target in node.targets:
                if (
                    isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "self"
                    and target.attr == "normal_actions"
                    and isinstance(node.value, ast.Dict)
                ):
                    keys: List[str] = []
                    for k in node.value.keys:
                        if isinstance(k, ast.Constant) and isinstance(k.value, str):
                            keys.append(k.value)
                    self.names = keys
            self.generic_visit(node)

    v = _Visitor()
    v.visit(tree)
    return v.names


def build_system_role() -> str:
    normal_actions = _load_normal_action_names()
    normal_actions_text = "、".join(normal_actions) if normal_actions else "（未检测到常规动作）"
    return f"""你叫夸父，是由乐聚机器人研发的功能丰富的人形机器人，回答要专业。

你具备以下能力：
1. 移动能力：你具备单向行走能力，可向前、向后、向左、向右行走，场地无限大，但你每一次走的步数不能超过十步，超出了就说目前步数限制是十步。
2. 转向能力：只能向左转1~180度和向右转1~180度，超出了就说转不了，180度也可以转。
3. 打太极
4. 常规动作：{normal_actions_text}。当用户明确要求其中某个常规动作时，你的回复内容只需要输出对应动作名（例如“握手”），不要输出多余解释。
5. 用户单次提到多个你的以上能力时，你只回复第一个能力，其余请求忽略。例如，用户说"向前走一步，向右走五步，向左转90度"，你应该只回应："好的，向前走一步"。

例如，用户说"向前走五步"，你应该这样回应：
"向前走五步"

注意：
- 语调要平稳，不要急促
- 整个过程只要关键词识别到：“打太极”就回答：“起势，野马分鬃”。
- 让你向左转的时候你要说向左转，让你向右转的时候你要说向右转
"""


start_session_req["dialog"]["system_role"] = build_system_role()

input_audio_config = {
    "chunk": 3200,
    "format": "pcm",
    "channels": 1,
    "sample_rate": 16000,
    "bit_size": pyaudio.paInt16
}

output_audio_config = {
    "chunk": 3200,
    "format": "pcm",
    "channels": 1,
    "sample_rate": 24000,
    "bit_size": pyaudio.paFloat32
}
