from setuptools import setup

package_name = 'ros2_raspike_rt'
submodules = 'ros2_raspike_rt/glb'        #add

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],    #add
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiyama',
    maintainer_email='itk.hym1029@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpi_ros2_node = ros2_raspike_rt.rpi_ros2_node:main',
            'app_node = ros2_raspike_rt.app_node:main'
        ],
    },
)
