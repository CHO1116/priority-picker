from setuptools import setup

package_name = 'picker_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='류창표',
    maintainer_email='todo@example.com',
    description='E0509 + RH-P12-RN-A 픽앤플레이스 제어',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = picker_control.robot_controller:main',
        ],
    },
)
