from pathlib import Path
from setuptools import setup

package_name = 'omx_hand_input'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
for directory in ('launch', 'config'):
    for path in Path(directory).rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts:
            data_files.append(('share/' + package_name + '/' + str(path.parent), [str(path)]))

setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=data_files, install_requires=['setuptools'], zip_safe=True,
    maintainer='OMX project maintainers', maintainer_email='maintainer@example.com',
    description='OpenManipulator-X hand control integration', license='Apache-2.0',
    entry_points={'console_scripts': ["hand_control_node = omx_hand_input.hand_control_node:main"]},
)
