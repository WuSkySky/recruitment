import os

from setuptools import find_packages, setup


package_name = "recruitment_sim_player_web"


def web_data_files():
    entries = []
    root = os.path.join(package_name, "web")
    if not os.path.isdir(root):
        return entries
    for directory, _, files in os.walk(root):
        if files:
            destination = os.path.join("share", package_name, directory)
            entries.append(
                (
                    destination,
                    [os.path.join(directory, item) for item in files],
                )
            )
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ] + web_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Recruitment Simulation Maintainers",
    maintainer_email="maintainers@example.com",
    description=(
        "Unified browser player and referee UI with WebRTC and ROS 2 "
        "integration."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "player_web = recruitment_sim_player_web.server:main",
        ],
    },
)
