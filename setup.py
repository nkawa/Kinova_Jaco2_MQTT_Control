from setuptools import setup, Extension

import pybind11

ext_modules = [
    Extension(
        'jacomodule',
        ['jacomodule.cpp'],
        include_dirs=[pybind11.get_include()],
        language='c++'
    ),
]

setup(
    name='jacomodule',
    version='0.0.1',
    author='Nobuo Kawaguchi',
    author_email = 'kawaguti@nagoya-u.jp',
    description='Kinova Jaco2 SDK python module',
    ext_modules=ext_modules,
    setup_requires=['pybind11>=2.9.1'],
    zip_safe=False,
)