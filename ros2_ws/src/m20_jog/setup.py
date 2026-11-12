from setuptools import find_packages, setup

package_name = "m20_jog"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/m20_jog.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="slam",
    maintainer_email="noreply@example.com",
    description="Qt jog panel publishing /cmd_vel for the M20.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["m20_jog = m20_jog.jog_node:main"]},
)
