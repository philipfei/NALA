from setuptools import find_packages, setup

package_name = 'nala_panel'

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
    description='Touchscreen panel on the robot for app 3: maps, base station, path planning and driving',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'panel = nala_panel.ui:main',
        ],
    },
)
