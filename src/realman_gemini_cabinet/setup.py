from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "realman_gemini_cabinet"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config/panel_layout", glob("config/panel_layout/*")),
        (f"share/{package_name}/config/panel_layout_right_knob", [str(p) for p in Path("config/panel_layout_right_knob").iterdir() if p.is_file()]),
        (f"share/{package_name}/config/panel_layout_right_knob/precontact", glob("config/panel_layout_right_knob/precontact/*")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/docs", ["README.md"] if Path("README.md").exists() else []),
        (f"share/{package_name}/docs", ["PANEL_LAYOUT_MODE.md"]),
        (f"share/{package_name}/docs", ["ALIGNMENT_PREFLIGHT.md"]),
        (f"share/{package_name}/docs", ["ALIGNMENT_REROUTE.md"]),
        (f"share/{package_name}/docs", ["COMMANDS.md"]),
    ],
    install_requires=["setuptools", "PyYAML>=6.0"],
    zip_safe=True,
    maintainer="jxs",
    maintainer_email="user@example.com",
    description="ROS2 task layer for RealMan + Gemini2 electrical cabinet manipulation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cabinet_task_node = rg_cabinet_ros2.task_node:main",
            "gripper_joint_state_node = rg_cabinet_ros2.gripper_joint_state_node:main",
            "rg_cabinet_demo = rg_cabinet.task_runner:main",
        ],
    },
)
