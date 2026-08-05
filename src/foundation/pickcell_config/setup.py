from pathlib import Path

from setuptools import find_packages, setup


package_name = "pickcell_config"


def collect_share_files(*roots: str) -> list[tuple[str, list[str]]]:
    """Collect non-Python package resources recursively."""

    result: list[tuple[str, list[str]]] = []

    for root_name in roots:
        root = Path(root_name)

        if not root.exists():
            continue

        for path in root.rglob("*"):
            if not path.is_file():
                continue

            destination = Path("share") / package_name / path.parent

            result.append((
                str(destination),
                [str(path)],
            ))

    return result


data_files = [
    (
        "share/ament_index/resource_index/packages",
        [f"resource/{package_name}"],
    ),
    (
        f"share/{package_name}",
        ["package.xml"],
    ),
]

data_files.extend(
    collect_share_files(
        "config",
        "calibration",
        "schema",
    )
)


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=[
        "setuptools",
        "PyYAML",
    ],
    zip_safe=True,
    maintainer="mahdi",
    maintainer_email="mahdi@example.com",
    description=(
        "Configuration resolution, profiles, manifests and calibration "
        "files for the PickCell ROS 2 system."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
