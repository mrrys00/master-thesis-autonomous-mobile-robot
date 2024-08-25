from setuptools import find_packages, setup

package_name = 'exploration_algorithm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        f'{package_name}.{package_name}_node'
    ],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            f'{package_name}_node = {package_name}.{package_name}_node:main'
        ],
    },
    zip_safe=True,
    maintainer='szymon',
    maintainer_email='szymon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest']
)
