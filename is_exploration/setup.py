from setuptools import find_packages, setup

package_name = 'is_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='issaiass',
    maintainer_email='issaiass@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'custom_publisher = scripts.custom_publisher:main',
          'gotopoint = scripts.gotopoint:main',
        ],
    },
)
