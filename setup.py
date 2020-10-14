from setuptools import setup, find_packages

setup(
    name='minmax',
    version='0.0.0',
    url='https://github.com/Silviatulli/minmax',
    author='Silvia Tulli',
    author_email='TODO',
    description='Code for the AAAI2021 project.',
    packages=find_packages(),
    install_requires=[
        'numpy >= 1.18.2',
        'matplotlib >= 3.2.1',
        'minihex',
        'gym',
        'pygame',
        'openpyxl',
        'pandas',
        'cachetools'
    ]
)
