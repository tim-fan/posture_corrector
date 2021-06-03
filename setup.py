from setuptools import setup

package_name = 'posture_corrector'

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
    maintainer='tfanselo',
    maintainer_email='tim.fanselow@crown.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'poor_posture_prompter ='
            ' posture_corrector.poor_posture_prompter:main',
        ],
    },
)
