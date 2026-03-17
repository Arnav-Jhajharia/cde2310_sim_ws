from setuptools import find_packages, setup

setup(
    name='tb3_cv',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'aruco_pose = tb3_cv.aruco_pose_node:main',
        ],
    },
)
