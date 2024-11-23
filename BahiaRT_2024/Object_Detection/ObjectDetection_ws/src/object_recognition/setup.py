from setuptools import find_packages, setup

package_name = 'object_recognition'

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
    maintainer='acso',
    maintainer_email='acso@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "object_recognition_node = object_recognition.object_recognition_node:main",
            "voice_node = object_recognition.voice_node:main"
        ],
    },
)
