from setuptools import find_packages, setup


package_name = "pickcell_test_support"


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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mahdi",
    maintainer_email="mahdi@example.com",
    description=(
        "Reusable synthetic fixtures and contract assertions for PickCell."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
)
