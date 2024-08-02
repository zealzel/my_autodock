from setuptools import find_packages, setup
from glob import glob

package_name = 'my_autodock'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ['package.xml']),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/material/tag10/script", glob("material/tag10/script/*")),
        (f"share/{package_name}/material/tag10/textures", glob("material/tag10/textures/*")),
        (f"share/{package_name}/material/tag20/script", glob("material/tag20/script/*")),
        (f"share/{package_name}/material/tag20/textures", glob("material/tag20/textures/*")),
        (f"share/{package_name}/material/tag30/script", glob("material/tag30/script/*")),
        (f"share/{package_name}/material/tag30/textures", glob("material/tag30/textures/*")),
        (f"share/{package_name}/worlds", glob("worlds/*")),
        (f"share/{package_name}/maps", glob("maps/*")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        (f"share/{package_name}/params", glob("params/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='zealzel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_pose_node = my_autodock.detect_pose:main',
        ],
    },
)
