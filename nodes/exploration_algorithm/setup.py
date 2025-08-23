from setuptools import find_packages, setup

package_name = 'exploration_algorithm'

random_direction_node_name = 'random_direction_node'
planned_node_name = 'planned_node'
boundary_finder_node_name = 'boundary_finder_node'
boundary_finder_with_map_dump = 'boundary_finder_with_map_dump'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        f'{package_name}.{random_direction_node_name}',
        f'{package_name}.{planned_node_name}',
    ],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            f'{random_direction_node_name} = {package_name}.{random_direction_node_name}:main',
            f'{planned_node_name} = {package_name}.{planned_node_name}:main',
            f'{boundary_finder_node_name} = {package_name}.{boundary_finder_node_name}:main',
            f'{boundary_finder_with_map_dump} = {package_name}.{boundary_finder_with_map_dump}:main'
        ],
    },
    zip_safe=True,
    maintainer='Szymon Rys',
    maintainer_email='rysszymon00@gmail.com',
    description='Exploration Algorithms',
    license='TODO: License declaration',
    tests_require=['pytest']
)
