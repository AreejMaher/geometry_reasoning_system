from setuptools import find_packages, setup

package_name = 'geometry_reasoning_system'

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
    maintainer='rejo',
    maintainer_email='areejmaher57@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_stream = geometry_reasoning_system.camera_stream:main',
            'keypoint_detection = geometry_reasoning_system.keypoint_detection:main',
            'descriptor_extraction = geometry_reasoning_system.descriptor_extraction:main',
            'feature_matching = geometry_reasoning_system.feature_matching:main',
            'match_filtering = geometry_reasoning_system.match_filtering:main',
            'geometric_consistency = geometry_reasoning_system.geometric_consistency:main',
            'motion_node = geometry_reasoning_system.motion_estimation:main',
            'reliability_decision = geometry_reasoning_system.reliability_decision:main',
        ],
    },
)
