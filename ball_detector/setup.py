from setuptools import find_packages, setup

package_name = 'ball_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ball_detector/launch', ['launch/ball_detector.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibuki',
    maintainer_email='8326dmc@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ball_node = ball_detector.ball_detector:main'
        ],
    },
)
