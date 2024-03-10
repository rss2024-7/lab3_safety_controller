from setuptools import setup
from setuptools import find_packages
import glob
import os

package_name = 'safety_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/'+package_name, ['package.xml', "safety_controller/params.yaml"]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/safety_controller/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('share/safety_controller/launch', glob.glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='michaelszeng@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_controller = safety_controller.safety_controller:main',
	        'viz_example = safety_controller.viz_example:main',
        	'test_safety_controller = safety_controller.test_safety_controller:main',
        ],
    },
)
