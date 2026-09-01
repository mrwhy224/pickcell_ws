from setuptools import find_packages, setup


package_name = "pickcell_motion_server"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mahdi",
    maintainer_email="mahdi@example.com",
    description="Motion-planning solver abstractions.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_solver = pickcell_motion_server.runtime_node:main",
            "motion_cycle = pickcell_motion_server.cycle_node:main",
            "trajectory_executor = "
            "pickcell_motion_server.trajectory_executor:main",
        ],
    },
)
