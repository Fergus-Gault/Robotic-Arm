from setuptools import setup, find_packages

setup(
    name="robot-arm",
    version="0.1.0",
    description="Robot arm control package",
    packages=find_packages(),
    install_requires=[
        "dynamixel-sdk",
    ],
    python_requires=">=3.6",
)
