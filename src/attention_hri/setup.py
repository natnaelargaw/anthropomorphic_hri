from setuptools import setup
import os
from glob import glob

package_name = 'attention_hri'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natnael',
    maintainer_email='natnaelargaw@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              'moving_object = attention_hri.moving_object:main',
              # 'saliency_prediction = attention_hri.saliency_prediction:main',
              'center_calculator = attention_hri.feed_behaviour:main',
              'behaviour = attention_hri.behaviour_manager:main'
        ],
    },
)
