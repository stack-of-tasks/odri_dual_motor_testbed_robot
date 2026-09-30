from setuptools import find_packages, setup

package_name = "odri_five_bar_force_velocity_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/force_velocity.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Olivier Stasse",
    maintainer_email="olivier.stasse@laas.fr",
    description=(
        "Python endpoint force-to-velocity controller (qdot = J^T f_c) for "
        "the odri_dual_motor_testbed five-bar mechanism, driven through the "
        "odri_forward_command_controller topic interface."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "force_velocity_node = "
            "odri_five_bar_force_velocity_py.force_velocity_node:main",
        ],
    },
)
