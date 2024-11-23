from setuptools import find_packages, setup

package_name = 'qualification_submission'

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
            "voice_node = qualification_submission.voice_node:main",
            "image_capture_node = qualification_submission.image_capture_node:main",
            "people_recognition_node = qualification_submission.people_recognition_node:main",
            "navigation = qualification_submission.navigation:main",
            "turn_around_node = qualification_submission.turn_around_node:main",
            "object_recognition_node = qualification_submission.object_recognition_node:main",
        ],
    },
)
