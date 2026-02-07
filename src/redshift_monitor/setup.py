from setuptools import find_packages, setup

package_name = 'redshift_monitor'

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
    maintainer='redshift',
    maintainer_email='bzeliger@gmail.com',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'redshift_monitor = redshift_monitor.redshift_monitor:main',
            'redshift_lifesigns = redshift_monitor.redshift_lifesigns:main',            
        ],
    },
)
