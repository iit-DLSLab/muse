import sys
from setuptools import setup


# Some colcon/setuptools combinations pass '--editable' to setup.py when
# using --symlink-install. Strip it so setup.py remains compatible.
if "--editable" in sys.argv:
    sys.argv.remove("--editable")

if "--build-directory" in sys.argv:
    idx = sys.argv.index("--build-directory")
    del sys.argv[idx]
    if idx < len(sys.argv):
        del sys.argv[idx]

if "--uninstall" in sys.argv:
    sys.argv.remove("--uninstall")

package_name = "muse_point_lio"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/point_lio_bridge.launch.py",
            "launch/muse_with_point_lio.launch.py",
        ]),
        (f"share/{package_name}/config", ["config/point_lio_bridge.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MUSE Team",
    maintainer_email="ylenia.nistico@iit.it",
    description="Point-LIO integration wrapper for MUSE (launch + odometry bridge)",
    license="TODO",
    entry_points={
        "console_scripts": [
            "odom_bridge = muse_point_lio.odom_bridge:main",
        ],
    },
)
