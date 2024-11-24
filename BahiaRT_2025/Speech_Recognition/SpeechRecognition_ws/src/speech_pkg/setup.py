from setuptools import find_packages, setup

package_name = 'speech_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/audios', ['speech_pkg/audios/microphoneOn.mp3',
                                               'speech_pkg/audios/keywordDetected.mp3',
                                               'speech_pkg/audios/erro.mp3',
                                               'speech_pkg/audios/dontUnderstand.mp3',
                                               'speech_pkg/audios/askQuestion.mp3']),
        ('share/' + package_name, ['speech_pkg/context.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='filipe',
    maintainer_email='filipesn.inc@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_context_node = speech_pkg.speech_context_node:main',
        ],
    },
)
