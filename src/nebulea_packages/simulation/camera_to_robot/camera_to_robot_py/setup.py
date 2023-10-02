from setuptools import find_packages, setup

package_name = 'camera_to_robot_py'

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
    maintainer='Loi Dinh',
    maintainer_email='loidinh.git@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_to_robot_py.camera_publisher:main',
            'robotic_arm_publishing_subscriber = camera_to_robot_py.robotic_arm_publishing_subscriber:main',
        ],
    },
)
