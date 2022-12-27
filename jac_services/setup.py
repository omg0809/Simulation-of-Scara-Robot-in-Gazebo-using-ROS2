from setuptools import setup

package_name = 'jac_services'

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
    maintainer='om',
    maintainer_email='melangille@wpi.edu',
    description='Group Assignment Part 3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'service = jac_services.part3:main',
        
    ],
},
)
