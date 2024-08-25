from setuptools import setup

setup(
    name='python_data_processor',
    version='0.0.0',
    packages=[],
    py_modules=[
        'python_data_processor.data_processor_node'
    ],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/python_data_processor']),
        ('share/python_data_processor', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'data_processor = python_data_processor.data_processor_node:main'
        ],
    },
)


