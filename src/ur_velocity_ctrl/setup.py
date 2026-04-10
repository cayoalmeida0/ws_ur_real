from setuptools import setup

package_name = "ur_velocity_ctrl"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cayo-sousa",
    maintainer_email="cayo-sousa@todo.todo",
    description="Velocity control node for real UR3e using ROS 2 Jazzy.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_velocity_ctrl = ur_velocity_ctrl.joint_velocity_ctrl:main",
        ],
    },
)