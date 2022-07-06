from setuptools import setup

package_name = 'low_level_servo_control'

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
    maintainer='me',
    maintainer_email='fabian.domberg@student.uni-luebeck.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'low_level_servo_control = low_level_servo_control.llvlsrvctrl:main',
        ],
    },
)
