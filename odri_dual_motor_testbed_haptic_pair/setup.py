from setuptools import find_packages, setup

package_name = "odri_dual_motor_testbed_haptic_pair"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/haptic_pair.launch.py"]),
        (
            "share/" + package_name + "/config",
            ["config/haptic_pair_controllers.yaml", "config/haptic_pair.config"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Olivier Stasse",
    maintainer_email="olivier.stasse@laas.fr",
    description=(
        "Two odri_dual_motor_testbed five-bar robots simulated as a "
        "haptic pair, connected through the existing "
        "odri_five_bar_force_velocity_controller on each side."
    ),
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "contact_force_relay = "
            "odri_dual_motor_testbed_haptic_pair.contact_force_relay:main",
        ],
    },
)
