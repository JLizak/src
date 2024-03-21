from setuptools import find_packages, setup

package_name = 'robocik_zadanie'

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
    maintainer='jeremi',
    maintainer_email='jeremi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "board_node = robocik_zadanie.board_node:main",
            "submarine_controller = robocik_zadanie.submarine_controller:main"
        ],
    },
)
