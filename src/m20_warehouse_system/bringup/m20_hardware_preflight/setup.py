from setuptools import setup

package_name = 'm20_hardware_preflight'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/m20_hardware_preflight.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'm20_hardware_preflight = m20_hardware_preflight.preflight_node:main',
        ],
    },
)
