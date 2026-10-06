from setuptools import find_packages, setup

package_name = 'nala_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='philipfei',
    maintainer_email='feijinghao2002@gmail.com',
    description='Pi to MCU driver of the NALA mecanum base',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'base_node = nala_base.base_node:main',
        ],
    },
)
