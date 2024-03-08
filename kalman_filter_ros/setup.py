from setuptools import setup

setup(
    name='kalman_filter_ros',
    version='0.0.0',
    packages=['kalman_filter_ros'],
    install_requires=['setuptools'],
    package_dir={'': 'src'},
    scripts=['src/kalman_filter_node.py'],
)

