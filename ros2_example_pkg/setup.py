import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_example_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Walter',
    maintainer_email='wsacul@gmail.com',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "generate_image = ros2_example_pkg.generate_image:main",
            "image_to_contour = ros2_example_pkg.image_to_contour:main",
            "msg_age = ros2_example_pkg.msg_age:main",
            "process_image = ros2_example_pkg.process_image:main",
        ],
    },
)
