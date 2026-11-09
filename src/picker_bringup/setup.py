from glob import glob

from setuptools import setup

package_name = 'picker_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='조영준',
    maintainer_email='chogae97@gmail.com',
    description='Priority Picker 런치 파일과 설정',
    license='MIT',
)
