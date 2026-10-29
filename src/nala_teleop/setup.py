from setuptools import find_packages, setup

package_name = 'nala_teleop'

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
    description='Keyboard (PC) and Xbox controller (Pi) teleop of the NALA robot with fixed speeds',
    license='CC-BY-NC-4.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'keyboard_teleop = nala_teleop.keyboard_node:main',
            'xbox_teleop = nala_teleop.xbox_node:main',
        ],
    },
)
