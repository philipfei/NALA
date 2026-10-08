from setuptools import find_packages, setup

package_name = 'nala_coverage'
setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='philip',
    maintainer_email='philip@example.com',
    description='Stage 3: drives coverage_tool plans with Nav2 from a base station and back; velocity gate and coverage measurement.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'coverage_supervisor = nala_coverage.supervisor:main',
        'coverage_meter = nala_coverage.meter:main',
        'velocity_gate = nala_coverage.gate_node:main',
        'coverage_teleop = nala_coverage.keyboard_teleop:main',
        'base_station_picker = nala_coverage.base_station_picker_node:main',
    ]},
)
