from setuptools import setup
import os

package_name = 'jaka_a5'

setup(
    name='jaka_a5',
    version='0.1.0',
    packages=[],
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/integration.launch.py']),
    ],
)