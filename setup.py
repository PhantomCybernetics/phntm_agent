from setuptools import setup
import os
from glob import glob

package_name = 'phntm_agent'

setup(
    name=phntm_agent,
    version='0.0.1',
    packages=[phntm_agent],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'python-engineio'],
    zip_safe=True,
    maintainer='Mirek Burkon',
    maintainer_email='mirek@phntm.io',
    description='Light agent for phntm_bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = phntm_agent.agent:main',
        ],
    },
)