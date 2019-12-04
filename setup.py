from setuptools import setup

package_name = 'lickport'

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
    author='Peter Polidoro',
    author_email='peterpolidoro@gmail.com',
    maintainer='Peter Polidoro',
    maintainer_email='peterpolidoro@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal subscribers using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber_old_school ='
            ' lickport.subscriber_old_school:main',
            'subscriber_lambda = lickport.subscriber_lambda:main',
            'subscriber_member_function ='
            ' lickport.subscriber_member_function:main',
        ],
    },
)
