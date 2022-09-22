# Pteranodon


<img src="https://static.wikia.nocookie.net/animals/images/a/a1/Pterathumb.png/revision/latest?cb=20200311123111" alt="drawing" width="200"/>

A framework to built on top of MAVSDK which provides physical/virtual drone abstraction, abstraction for all async calls,
gives the end users an arduino-eqsue interface, and provides movement and utility methods based on relative cordinate systems.

## Contributing

### Formatting

Pteranodon uses the [PEP-8 Coding Standard](https://peps.python.org/pep-0008/) with required typing through
[MyPy](https://mypy.readthedocs.io/en/stable/).

In order to merge into the `main` branch, a contributor must satisfy these standards as well as the
[Black Code Style](https://black.readthedocs.io/en/stable/).

This can be done automatically by setting your IDE to format on save, or you can run `black --safe ./pteranodon` locally.