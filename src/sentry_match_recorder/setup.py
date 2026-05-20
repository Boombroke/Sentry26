from glob import glob

from setuptools import setup

package_name = 'sentry_match_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name,
            [package_name + '/topics.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boombroke',
    maintainer_email='boombroke@icloud.com',
    description='Auto rosbag recorder triggered by referee game_progress (RoboMaster sentry)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'match_recorder_node = sentry_match_recorder.match_recorder_node:main',
        ],
    },
)
