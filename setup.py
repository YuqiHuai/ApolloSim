import re
from setuptools import setup, find_packages

def get_version():
    with open("apollo_sim/__init__.py", "r", encoding="utf-8") as f:
        match = re.search(r'__version__ = ["\'](.*?)["\']', f.read())
        if not match:
            raise RuntimeError("Unable to find version string in apollo_sim/__init__.py.")
        return match.group(1)

def parse_requirements(filename):
    """Load requirements from a pip requirements file."""
    with open(filename, "r", encoding="utf-8") as f:
        return [line.strip() for line in f if line.strip() and not line.startswith("#")]

setup(
    name="apollo_sim",
    version=get_version(),
    author="Mingfei Cheng",
    author_email="snowbirds.mf@gmail.com",
    description="ApolloSim: A lightweight traffic simulator for Baidu Apollo.",
    long_description=open("README.md", "r", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/MingfeiCheng/ApolloSim",
    packages=find_packages(include=["apollo_sim", "apollo_sim.*"]),  # Include `apollo_sim` and sub-packages
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
    install_requires=parse_requirements("requirements.txt"),
)
