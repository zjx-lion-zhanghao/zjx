from setuptools import find_packages, setup

package_name = 'control'

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
    maintainer='kpc',
    maintainer_email='kpc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control=control.control:main',
            'test=control.control_test:main',
            'pub=control.pub:main',
            'bucket_filter=control.bucket_filter_node:main',  # 添加这一行
            'test_camera=control.test_camera:main',  # 添加一个测试相机节点
        ],
    },
)
