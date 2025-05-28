from setuptools import setup

package_name = 'my_robot_cell_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='UR5e robot description and planning tools',
    license='MIT',
    entry_points={
        'console_scripts': [
            'move_to_joint_goal = my_robot_cell_description.move_to_joint_goal:main',
        ],
    },
)
