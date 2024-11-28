import os
from glob import glob
from setuptools import find_packages, setup


package_name = "tangram_bot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    py_modules=[f"{package_name}.robot_commander"],
    zip_safe=True,
    maintainer="jingkun",
    maintainer_email="jingkunliu2025@u.northwestern.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"action_generator = {package_name}.action_generator:main",
            f"action_executor = {package_name}.action_executor:main",
        ],
    },
)
