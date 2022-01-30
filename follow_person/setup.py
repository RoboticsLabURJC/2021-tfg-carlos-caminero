from setuptools import setup

package_name = 'follow_person'

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
    maintainer='Carlos Caminero (Carlosalpha1)',
    maintainer_email='carlos2caminero@gmail.com',
    description='It is a implementation of Follow Person in Python to use as example in the Unibotics exercise',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_person_main = follow_person.follow_person_main:main'
        ],
    },
)
