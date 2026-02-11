from setuptools import setup, find_packages

package_name = 'newton_raphson_px4'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='egmc',
    maintainer_email='egmc@todo.todo',
    description='Standard Newton-Raphson controller for quadrotor trajectory tracking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_node = newton_raphson_px4.run_node:main',
        ],
    },
)
