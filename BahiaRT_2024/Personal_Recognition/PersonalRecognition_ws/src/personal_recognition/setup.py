from setuptools import find_packages, setup

package_name = 'personal_recognition'

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
            "voice_node = personal_recognition.voice_node:main",
            "image_capture_node = personal_recognition.image_capture_node:main",
            "people_recognition_node = personal_recognition.people_recognition_node:main",
            "turn_around_node = personal_recognition.turn_around_node:main",
        ],
    },
)
