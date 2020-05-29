from setuptools import setup

package_name = 'rc_reason_clients'

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
    maintainer='Felix Ruess',
    maintainer_email='felix.ruess@roboception.de',
    description='Clients for interfacing with Roboception reason modules on rc_visard and rc_cube.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tagdetect_client = rc_reason_clients.tagdetect:main'
        ],
    },
)
