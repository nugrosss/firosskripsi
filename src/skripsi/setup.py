from setuptools import find_packages, setup

package_name = 'skripsi'

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
    maintainer='aroc2',
    maintainer_email='aprilifiros04@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kalibrasi_jarak = skripsi.mainpolinom:main',
            'servo_tilt = skripsi.servo_tilt:main',
            'head_tilt_controller = skripsi.tilit_control:main',
            'head_tilt_reader = skripsi.readenco:main',
        ],
    },
)
