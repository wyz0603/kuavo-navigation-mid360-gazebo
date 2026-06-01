__version__ = "0.0.1"
import sys
from pathlib import Path

# isort:skip
# ruff: noqa
from omni.isaac.kit import SimulationApp  # pylint: disable = E0401

sys.argv.extend(
    [
        "--/log/level=error",
        "--/log/fileLogLevel=error",
        "--/log/outputStreamLevel=error",
    ]
)
app = SimulationApp(
    {"headless": False, "renderer": "RayTracedLighting", "multi_gpu": False}
)


from omni.isaac.core.utils.extensions import enable_extension  # noqa # pylint: disable = C0413

enable_extension("omni.isaac.version")
enable_extension("omni.kit.tool.measure")

ROOT_PATH = Path(__file__).parent
