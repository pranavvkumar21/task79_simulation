from setuptools import setup
import os.path
from glob import glob

package_name = 'task79_data_converter'

setup(
    name=package_name,
    version='2.0.0',
    packages=[
        'task79_data_converter',
        'tools',
        'tools.data_analysis',
        'tools.data_processing',
        'tools.file_management',
        'tools.format_management',
        'tools.math_support',
        'tools.ros2_management',
        'tools.unit_conversion',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
        (
            os.path.join(
                'share', 
                package_name, 
                'data', 
                'mission_data', 
                'test_9',
            ),
            glob(
                os.path.join(
                    'data',
                    'mission_data',
                    'test_9',
                    '*',
                ),
            ),
        ),
        (
            os.path.join(
                'share', 
                package_name, 
                'data', 
                'mission_data', 
                'test_12',
            ),
            glob(
                os.path.join(
                    'data',
                    'mission_data',
                    'test_12',
                    '*',
                ),
            ),
        ),
    ],
    install_requires=[
        'numpy>=1.21.5',
        'matplotlib>=3.5.1',
        'pandas>=2.2.0',
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Achille MARTIN',
    maintainer_email='achille.martin@noc.ac.uk',
    description='Mission Data Conversion Module for Progeny Task 79',
    license='NOC',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'formatted_data_converter = task79_data_converter.formatted_data_conversion_node:main',
            'configuration_manager = task79_data_converter.configuration_manager_node:main',
            'mission_data_analyser = tools.data_analysis.plot_analysis:main',
            'mission_data_processor = tools.data_processing.process_syrinx_raw_data:main',
        ],
    },
)
