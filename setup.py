from setuptools import find_packages, setup

package_name = 'slm_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', ['launch/pendulum_sim_launch.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/pendulum.urdf']),
        ('share/' + package_name + '/rviz', ['rviz/manipulator.rviz']),
        ('share/' + package_name + '/models', ['models/Arm_2.stl']),
        ('share/' + package_name + '/models', ['models/Base_2.stl']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jesus',
    maintainer_email='jesus@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pendulum = slm_sim.slm_sim:main'
        ],
    },
)
