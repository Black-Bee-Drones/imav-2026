import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'indoor'


def _recursive_data_files(src_root: str, dest_root: str):
    """Install every file under src_root into share/<pkg>/<dest_root>/..."""
    entries = []
    for dirpath, _dirnames, filenames in os.walk(src_root):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, src_root)
        install_dir = os.path.join('share', package_name, dest_root, rel)
        if rel == '.':
            install_dir = os.path.join('share', package_name, dest_root)
        entries.append((install_dir, [os.path.join(dirpath, f) for f in filenames]))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob("share/models/*")),
        *(_recursive_data_files('simulation', 'simulation')),
        (os.path.join('share', package_name, 'models'), glob('share/models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Ronchi',
    maintainer_email='lucascronchi2005@gmail.com',
    description='IMAV 2026 indoor',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            f'mangalarga = {package_name}.mangalarga:main'
        ],
    },
)
