from setuptools import find_packages, setup
setup(
    name='lds006',
    packages=find_packages(),
    version='0.1.1',
    description='Lidar Sensor library',
    author='Me',
    license='MIT',
    install_requires=['pyserial']
)