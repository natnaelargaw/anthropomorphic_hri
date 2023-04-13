from setuptools import setup
import os
from glob import glob
package_name = 'mods'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natnael',
    maintainer_email='natnaelargaw@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              'moving_object = mods.moving_object:main',
              # 'saliency_prediction = mods.saliency_prediction:main',
              'center_calculator = mods.feed_behaviour:main',
              'behaviour = mods.behaviour_manager:main'


        ],
    },
)
