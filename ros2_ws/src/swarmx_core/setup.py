import os
from glob import glob

from setuptools import find_packages, setup

package_name = "swarmx_core"
static = glob(os.path.join(package_name, "dashboard", "static", "*"))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["dashboard/static/*"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/dashboard/static", static),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="SwarmX team",
    maintainer_email="sorting.tech@shoonyarecycling.in",
    description="Decentralized AMR fleet coordination core (CBBA, ORCA, zone locks, P2P, simulator, dashboard).",
    license="Apache-2.0",
    tests_require=["pytest"],
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "swarmx_sim = swarmx_core.sim.run:main",
            "swarmx_benchmark = swarmx_core.sim.benchmark:main",
            "swarmx_edge = swarmx_core.edge:main",
        ],
    },
)
