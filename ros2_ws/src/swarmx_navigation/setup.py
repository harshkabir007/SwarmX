from glob import glob

from setuptools import setup

package_name = "swarmx_navigation"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SwarmX team",
    maintainer_email="sorting.tech@shoonyarecycling.in",
    description="Nav2 configuration for SwarmX robots",
    license="Apache-2.0",
)
