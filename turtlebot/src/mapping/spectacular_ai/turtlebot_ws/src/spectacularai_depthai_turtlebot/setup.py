from setuptools import find_packages, setup

package_name = 'spectacularai_depthai_turtlebot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deblaci',
    maintainer_email='deblaci10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ros2_node = spectacularai_depthai_turtlebot.spectacularai_node:main',
        ],
    },
)
