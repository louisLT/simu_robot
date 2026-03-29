from setuptools import find_packages, setup

package_name = 'soarm_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', []),
        ('share/' + package_name + '/launch', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Application nodes for SO-ARM101 color tracking simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'square_mover = soarm_tracking.square_mover:main',
            'color_detector = soarm_tracking.color_detector:main',
            'arm_planner = soarm_tracking.arm_planner:main',
            'recorder = soarm_tracking.recorder:main',
        ],
    },
)
