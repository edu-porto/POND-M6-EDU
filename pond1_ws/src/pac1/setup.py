from setuptools import find_packages, setup

package_name = 'pac1'

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
    maintainer='tiringa',
    maintainer_email='eduardo.porto@sou.inteli.edu.br',
    description='Ponderada 1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = pac1.my_node:main'
        ],
    },
)
