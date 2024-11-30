from setuptools import find_packages, setup

package_name = 'isaac_ros_semantics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bsb',
    maintainer_email='briansbingham@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convert_32SC1 = isaac_ros_semantics.convert_32SC1:main'
        ],
    },
)
