from setuptools import find_packages, setup

package_name = 'cosserat_nordbo'

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
    maintainer='lar95',
    maintainer_email='mathijs.svrc@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wrench_publisher = cosserat_nordbo.ftsensoreth:main",
            "wrench_writer = cosserat_nordbo.wrench_writer:main",
            "cosserat_shape = cosserat_nordbo.cosserat_shape:main",
            "cosserat_shooting = cosserat_nordbo.cosserat_shooting:main",
            "cosserat_jac_computing = cosserat_nordbo.cosserat_jac_computing:main",
            "pyel_control = cosserat_nordbo.pyel_control:main",
            "pyel_jacobian = cosserat_nordbo.pyel_jacobian:main",
            "test_tcp_pub = cosserat_nordbo.test_tcp_pub:main",
            "test_target_pub = cosserat_nordbo.test_target_pub:main"
        ],
    },
)
