from setuptools import find_packages, setup

package_name = "driving_unity"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fix_jer",
    maintainer_email="jeremy.fix@centralesupelec.fr",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_teleop = driving_unity.keyboard_teleop:main",
            "recorder = driving_unity.recorder:main",
        ],
    },
)
