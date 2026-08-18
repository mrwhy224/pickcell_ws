from setuptools import find_packages, setup


package_name = "pickcell_dataset_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mahdi",
    maintainer_email="mahdi@example.com",
    description="Synchronized RGB-D dataset capture tools for PickCell.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dataset_capture = pickcell_dataset_tools.dataset_capture:main",
        ],
    },
)
