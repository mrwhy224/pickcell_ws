from pathlib import Path

from setuptools import find_packages, setup


package_name = "pickcell_bringup"


def launch_files() -> list[str]:
    """Return launch files installed with this package."""
    return [str(path) for path in Path("launch").glob("*.launch.py")]


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
        (f"share/{package_name}", ["README.md"]),
        (f"share/{package_name}/launch", launch_files()),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mahdi",
    maintainer_email="mahdi@example.com",
    description="Top-level launch and configuration resolution for PickCell.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
