from setuptools import find_packages, setup


package_name = "pickcell_mock_nodes"


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
    description="Deterministic mock point cloud and target publishers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_sim = pickcell_mock_nodes.joint_state_sim:main",
            "point_cloud_mock = pickcell_mock_nodes.point_cloud_mock:main",
        ],
    },
)
