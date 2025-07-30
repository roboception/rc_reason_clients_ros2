from setuptools import setup

package_name = 'rc_reason_clients'

setup(
    name=package_name,
    version='0.1.0',
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
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'rc_april_tag_detect_client = rc_reason_clients.tagdetect:rc_april_tag_detect_client',
            'rc_qr_code_detect_client = rc_reason_clients.tagdetect:rc_qr_code_detect_client',
            'rc_silhouettematch_client = rc_reason_clients.silhouettematch:main',
            'rc_itempick_client = rc_reason_clients.pick:rc_itempick_client',
            'rc_boxpick_client = rc_reason_clients.pick:rc_boxpick_client',
            'rc_hand_eye_calibration_client = rc_reason_clients.hand_eye_calib:main',
            'rc_cadmatch_client = rc_reason_clients.cadmatch:main',
            'rc_load_carrier_client = rc_reason_clients.load_carrier:main'
        ],
    },
)
