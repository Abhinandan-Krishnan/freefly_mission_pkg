from setuptools import find_packages, setup

package_name = 'freefly_mission_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'templates/*',
            'static/*'
        ]
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drone_control_system.launch.py', 'launch/drone_web_app.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinandan',
    maintainer_email='abhi@evevehicles.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_nav_node = freefly_mission_pkg.drone_nav_node:main',
            'drone_web_app = freefly_mission_pkg.web_app:main'
        ],
    },
)
