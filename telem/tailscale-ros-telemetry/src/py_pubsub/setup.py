from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='erics118',
    maintainer_email='ericshen118@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_spi:main',
            'listener = py_pubsub.subscriber_spi:main',
            'talkerdemo = py_pubsub.publisher_member_function:main',
            'listenerdemo = py_pubsub.subscriber_member_function:main',
            'publisher_test = py_pubsub.test_spi_publisher:main',
            'latency_reporter = py_pubsub.latency_reporter:main',
        ],
    },
)
