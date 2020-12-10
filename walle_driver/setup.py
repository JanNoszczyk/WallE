from setuptools import setup

package_name = 'walle_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jannoszczyk',
    maintainer_email='panainz@gmail.com',
    description='Robot Drivers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = walle_driver.position_publisher:main',
            'listener = walle_driver.position_subscriber:main',
        ],
    },
)
