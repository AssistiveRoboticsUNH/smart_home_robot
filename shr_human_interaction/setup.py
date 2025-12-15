from setuptools import find_packages, setup

package_name = 'shr_human_interaction'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moniruzzaman Akash',
    maintainer_email='moniruzzaman.akash@unh.edu',
    description='SHR human-robot interaction: conversation + intent + action requests',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_interaction_node = shr_human_interaction.human_interaction_node:main',
        ],
    },
)
