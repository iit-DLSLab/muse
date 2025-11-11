from setuptools import find_packages
from setuptools import setup

setup(
    name='state_estimator_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('state_estimator_msgs', 'state_estimator_msgs.*')),
)
