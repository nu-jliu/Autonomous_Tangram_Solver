from setuptools import find_packages
from setuptools import setup

setup(
    name='tangram_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('tangram_msgs', 'tangram_msgs.*')),
)
