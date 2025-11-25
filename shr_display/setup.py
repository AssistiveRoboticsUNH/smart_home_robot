from setuptools import find_packages, setup

package_name = 'shr_display'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/shr_display/config', ['config/protocol_routines.json',
                                      'config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hello-robot',
    maintainer_email='mostafa.hussein@unh.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'display_node = shr_display.display_node:main'
        ],
    },
)
