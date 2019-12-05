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
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Lickport ROS interface.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lickport ='
            ' lickport.lickport:main',
        ],
    },
)
