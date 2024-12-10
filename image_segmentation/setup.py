from setuptools import find_packages, setup
from glob import glob
import os

package_name = "image_segmentation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "images"), glob("images/*")),
        (os.path.join("share", package_name, "model"), glob("model/*")),
    ],
    py_modules=[f"{package_name}.network"],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jingkun",
    maintainer_email="jingkunliu2025@u.northwestern.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"webcam = {package_name}.webcam:main",
            f"puzzle_segment = {package_name}.puzzle_segment:main",
            f"piece_segment = {package_name}.piece_segment:main",
            f"shape_classify = {package_name}.shape_classify:main",
            f"shape_detect = {package_name}.shape_detect:main",
        ],
    },
)
