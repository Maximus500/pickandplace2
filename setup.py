from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tiago_pick_and_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools', 'cv_bridge'],  # Assurez-vous que cv_bridge est installé
    zip_safe=True,
    maintainer='rosdev',
    maintainer_email='rosdev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick = tiago_pick_and_place.pick:main',
            'depth_image_subscriber = tiago_pick_and_place.Test:main',  # Veuillez vérifier que "test.py" existe
        ],
    },
)
