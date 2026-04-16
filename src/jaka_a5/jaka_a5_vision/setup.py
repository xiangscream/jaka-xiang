from setuptools import setup

setup(
    name='jaka_a5_vision',
    version='0.1.0',
    packages=['jaka_a5_vision'],
    data_files=[
        ('share/jaka_a5_vision', ['package.xml']),
        ('share/jaka_a5_vision/config', ['config/apriltag.yaml']),
        ('share/jaka_a5_vision/launch', ['launch/vs.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiangscream',
    description='JAKA A5 Visual Servo Control with AprilTag',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_subscriber = jaka_a5_vision.apriltag_subscriber:main',
            'vs_controller = jaka_a5_vision.vs_controller:main',
        ],
    },
)
