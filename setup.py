from setuptools import find_packages, setup

package_name = 'rohand'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='He Bin',
    maintainer_email='hebin7611@hotmail.com',
    description='Package for ROHand',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rohand_modbus_a001 = scripts.roh_a001_node.rohand_modbus:main',
            'rohand_serial_a001 = scripts.roh_a001_node.rohand_serial:main',
            'rohand_teleop_a001 = scripts.roh_a001_node.rohand_teleop:main',
            'rohand_modbus_ap001 = scripts.roh_ap001_node.rohand_modbus:main',
            'rohand_serial_ap001 = scripts.roh_ap001_node.rohand_serial:main',
            'rohand_teleop_ap001 = scripts.roh_ap001_node.rohand_teleop:main',
            'rohand_modbus_lites001 = scripts.roh_lites001_node.rohand_modbus:main',
            'rohand_serial_lites001 = scripts.roh_lites001_node.rohand_serial:main',
            'rohand_teleop_lites001 = scripts.roh_lites001_node.rohand_teleop:main',
        ],
    },
)
