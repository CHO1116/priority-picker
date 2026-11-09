from setuptools import setup

package_name = 'picker_planner'

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
    maintainer='조영준',
    maintainer_email='chogae97@gmail.com',
    description='주문 관리 및 피킹 우선순위 계획',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_planner = picker_planner.task_planner:main',
            'order_manager = picker_planner.order_manager:main',
        ],
    },
)
