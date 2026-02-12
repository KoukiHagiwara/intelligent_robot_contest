import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'intelligent_robot_contest'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # modelsディレクトリにある全ての.ptファイルをインストール対象にする
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wanglab',
    maintainer_email='kouki-abc-1108@outlook.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = intelligent_robot_contest.detector_node:main',
            'ball_color_node = intelligent_robot_contest.ball_color_node:main',
            'ball_node = intelligent_robot_contest.ball_node:main',
            'experiment_node = intelligent_contest.experiment_node:main',
        ],
    },
)
