# This file is kept for backward compatibility with older pip versions
# For modern pip (25+), pyproject.toml is used instead

from setuptools import setup, find_packages

# Always include full metadata to ensure compatibility with all pip versions
# This resolves the issue where package might be installed as "UNKNOWN-0.0.0"
# on certain Python versions (like Python 3.10 on Ubuntu 22.04)

if __name__ == '__main__':
    setup(
        name="lcm_msgs",
        version="0.1.2",
        description="LCM generated Python bindings for ROS based types",
        author="Dimensional",
        packages=find_packages(),  # This will find lcm_msgs and all subpackages
        install_requires=[
            "lcm",
        ],
        python_requires=">=3.8",
    )