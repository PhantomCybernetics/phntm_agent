from setuptools import setup
import os
from glob import glob

package_name = 'phntm_agent'

setup(
    name=package_name,
    version='0.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['iwlib' ],
    zip_safe=True,
    maintainer='Mirek Burkon',
    maintainer_email='mirek@phntm.io',
    description='Monitoring Agent for the Phantom Bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'agent = phntm_agent.agent:main',
        ],
    }
)