from glob import glob
from setuptools import setup

setup(
    name='damvi_teleop_ui', version='0.1.0', packages=['damvi_teleop_ui'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/damvi_teleop_ui']),
        ('share/damvi_teleop_ui', ['package.xml', 'README.md']),
        ('share/damvi_teleop_ui/config', glob('config/*.yaml')),
        ('share/damvi_teleop_ui/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='damvi simulator developers', maintainer_email='maintainer@example.com',
    description='Keyboard mode selector and command monitor for damvi simulation',
    license='Apache-2.0',
    entry_points={'console_scripts': ['teleop_ui = damvi_teleop_ui.app:main']},
)
