from setuptools import find_packages, setup

package_name = 'my_package_tut1'

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
    maintainer='youssef-shemela',
    maintainer_email='youssefshemela12@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "py_node = my_package_tut1.my_first_node:main", #ros2 (run my_package_tut1 py_node) run my_first_nodeby this command
            "robot_news_station = my_package_tut1.robot_news_station:main",
            "smart_phone = my_package_tut1.smartphone:main",
            "Int_robot_news_station = my_package_tut1.robot_news_int:main",
            "Int_smart_phone = my_package_tut1.robot_int:main",
            "py_server = my_package_tut1.py_server:main",
            "py_client = my_package_tut1.py_server_client:main",
            "py_client_opp = my_package_tut1.py_servclie_oop:main",
            "hw_status_publisher = my_package_tut1.hardware_status:main",
            "turtlesim_kinematics = my_package_tut1.turtlesim_kinematics:main",
            "simple_QOS_pub = my_package_tut1.simple_QOS:main",
            "simple_QOS_sub = my_package_tut1.simple_QOS_sub:main"
            "simple_lifecycle = my_package_tut1.simple_lifecycle:main"
        ],
    },
)
