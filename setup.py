import os

from setuptools import find_packages, setup

package_name = 'mhseals_testing'


def package_files(directory):
    """Install every file under ``directory`` to share/<package>/<directory>."""
    paths = []
    for path, _, files in os.walk(directory):
        for file in files:
            paths.append((os.path.join('share', package_name, path), [os.path.join(path, file)]))
    return paths


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
data_files += package_files('config')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'rich', 'PyYAML', 'numpy', 'Pillow', 'pyserial>=3.5', 'pyubx2>=1.3.9', 'pyrtcm', 'pymavlink>=2.4.41'],
    zip_safe=True,
    author='MHS Seals',
    description='Boat test suite for sensors, actuation, navigation and perception',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boat_test = mhseals_testing.ui.app:main',
            'boat_trial = mhseals_testing.harness.cli:main',
            'rtk_base = mhseals_testing.rtk:main',
            'host_diagnostic = mhseals_testing.ros.host_diagnostic:main',
        ],
    },
)
