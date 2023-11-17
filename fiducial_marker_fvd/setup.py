from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'fiducial_marker_fvd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        # ...
    ],
    install_requires=['setuptools', 'fiducial_msgs'],
    zip_safe=True,
    maintainer='sid',
    maintainer_email='siddhant.wadhwa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fiducial_marker_fvd = fiducial_marker_fvd.fiducial_marker_fvd:main',
        ],
    },
)
