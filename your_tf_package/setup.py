from setuptools import setup

setup(
    name='your_tf_package',
    version='0.0.1',
    packages=['your_tf_package'],
    install_requires=['setuptools'],
    package_dir={'': 'src'},
    scripts=['src/tf_publisher.py'],
)

