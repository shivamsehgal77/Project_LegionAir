from setuptools import setup

setup(
    name='tflite_prop_detection',
    version='0.0.0',
    packages=['tflite_prop_detection'],
    install_requires=['setuptools'],
    package_dir={'': 'src'},
    scripts=['src/tflite_prop_detection_node.py'],
)
