from setuptools import setup

package_name = 'picker_vision'

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
    maintainer='이준희',
    maintainer_email='todo@example.com',
    description='RealSense 기반 색상 큐브 3D 인식',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = picker_vision.vision_node:main',
            'fake_world = picker_vision.fake_world:main',
            'calibrate = picker_vision.calibrate:main',
        ],
    },
)
