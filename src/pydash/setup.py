from setuptools import setup
import os
from glob import glob

package_name = 'pydash'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' , package_name, "images"), glob("images/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jw9vq',
    maintainer_email='jwl9vq@virginia.edu',
    description='Graphical ',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = pydash.node:main'
        ],
    },
)
