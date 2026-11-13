from setuptools import setup

package_name = 'control_library'

setup(
    name=package_name,
    version='0.1.0',
    description='Inverse-kinematics and gait-planning helpers for OpenQuadruped',
    url='https://github.com/adham-elarabawy/open-quadruped',
    author='Adham Elarabawy',
    author_email='aelarabawy@berkeley.edu',
    license='MIT',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)
