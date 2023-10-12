
from setuptools import find_packages, setup

package_name = 'charging_dock_py'

setup(
    name=package_name,
    version='0.17.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jacob Perron',
    author_email='jacob@openrobotics.org',
    maintainer='Jacob Perron',
    maintainer_email='jacob@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Python action tutorials code.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'connect_to_charging_dock_server = charging_dock_py.connect_to_charging_dock_action_server:main',
            'connect_to_charging_dock_client = charging_dock_py.connect_to_charging_dock_action_client:main',
        ],
    },
)   