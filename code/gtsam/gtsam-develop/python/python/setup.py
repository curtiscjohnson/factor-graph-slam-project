"""Setup file to install the GTSAM package."""

try:
    from setuptools import setup, find_packages
except ImportError:
    from distutils.core import setup, find_packages

packages = find_packages(where=".")
print("PACKAGES: ", packages)

package_data = {
    '': [
        "./*.so",
        "./*.dll",
        "Data/*"  # Add the data files to the package
        "Data/**/*"  # Add the data files in subdirectories
    ]
}

# Cleaner to read in the contents rather than copy them over.
readme_contents = open("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/README.md").read()

setup(
    name='gtsam',
    description='Georgia Tech Smoothing And Mapping library',
    url='https://gtsam.org/',
    version='4.2a7',  # https://www.python.org/dev/peps/pep-0440/
    author='Frank Dellaert et. al.',
    author_email='frank.dellaert@gtsam.org',
    license='Simplified BSD license',
    keywords='slam sam robotics localization mapping optimization',
    long_description_content_type='text/markdown',
    long_description=readme_contents,
    # https://pypi.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Education',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'Operating System :: MacOS',
        'Operating System :: Microsoft :: Windows',
        'Operating System :: POSIX',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 3',
    ],
    packages=packages,
    include_package_data=True,
    package_data=package_data,
    test_suite="gtsam.tests",
    install_requires=open("/home/kody/SLAM/factor-graph-slam-project/code/gtsam/gtsam-develop/python/requirements.txt").readlines(),
    zip_safe=False,
)
