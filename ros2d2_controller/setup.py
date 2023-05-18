from setuptools import setup

package_name = 'ros2d2_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jammy',
    maintainer_email='jammy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = ros2d2_controller.my_first_node:main",
            "draw_circle = ros2d2_controller.draw_circle:main"
        ],
    },
)
