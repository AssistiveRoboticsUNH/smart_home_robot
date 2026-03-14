from glob import glob
from setuptools import find_packages, setup

package_name = 'shr_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static', glob('shr_dashboard/static/*')),
    ],
    install_requires=['setuptools', 'Flask', 'PyYAML'],
    zip_safe=False,
    maintainer='Moniruzzaman Akash',
    maintainer_email='moniruzzaman.akash@unh.edu',
    description='Web dashboard server for protocol design and robot operations.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_server = shr_dashboard.web_server:main',
        ],
    },
)
