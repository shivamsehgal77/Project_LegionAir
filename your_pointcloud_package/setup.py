from setuptools import setup

setup(
    name='your_pointcloud_package',
    version='0.0.0',
    packages=['your_pointcloud_package'],
    install_requires=['setuptools'],
    package_dir={'': 'src'},
    scripts=['src/pointcloud_transformer.py', 'src/pointcloud_transformer_rgb.py'],
)
