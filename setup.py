from setuptools import find_packages, setup
setup(
    name='lds006',
    packages=find_packages(),
    version='0.1.2',
    description='Lidar Sensor library',
    author='aur20',
    license='MIT',
    install_requires=['pyserial']
)