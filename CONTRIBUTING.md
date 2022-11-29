# Contributing

---

When making changes to this repository, please first discuss the change you wish to make via issue, email, or any
other method with the owners of this repository before make a change. It is essential to understand our guidelines as well.

### Setup
Please read README.md to understand the project and its use, especially the "Requirements" section. It is important to have everything installed before 
contributing a change. 

### Pull Requests
First fork the repository and clone it locally. Connect your local to the original upstream repository by adding it as a remote. Pull in 
changes from upstream often so that you stay up to date. This is so when you submit a pull request, merge conflicts are less likely to occur.

When creating a pull request, it is essential to make sure the following tests pass
* Integration Tests
* Unit Tests
* PyLint
* Flake8
* MyPy
* Black Format


### Formatting

Pteranodon uses the [PEP-8 Coding Standard](https://peps.python.org/pep-0008/) with required typing through
[MyPy](https://mypy.readthedocs.io/en/stable/).

In order to merge into the `main` branch, a contributor must satisfy these standards as well as the
[Black Code Style](https://black.readthedocs.io/en/stable/).

This can be done automatically by setting your IDE to format on save, or you can run `black --safe ./pteranodon` locally.

### Our standards
* Using welcoming and inclusive language
* Being respectful of differing viewpoints and experiences
* Gracefully accepting constructive criticism
* Focusing on what is best for the community
* Showing empathy towards other community members


We appreciate you taking the time to contribute to our open source project!