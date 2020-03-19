import setuptools

with open("README.md"), "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="ftl-python",
    version="0.0.1", # TODO: CMAKE
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3"
    ],
    python_requires='>=3.6',
)
