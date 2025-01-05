from setuptools import setup

package_name = 'kruskal'

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
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Your package description',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_to_coords = kruskal.navigate_to_coords:main',  # <-- Your script here
        ],
    },
)
