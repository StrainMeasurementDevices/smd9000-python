# SMD9000 Python Package
Developed and maintained by Strain Measurement Devices Inc.

## Introduction
This repository is for a Python package that communicates and abstracts the SMD9000 flow sensor over UART. The package uses a serial port to communicate with the sensor, which can be easily setup with a USB to UART bridge such as the FT232.

## Docs
The documentation can be built with Sphinx by running `make html` in the docs folder.

## Reporting Issues
If you find any issues or bug with this package, please report an Issue on GitHub. If the issue is with the sensor and not this package specifically, feel free to contact us.

## Git Versioning
This git repository's default branch titled `release` is a tested and released version of this library under our ISO9001 requirements, as long as the library's revision is >= `1.x.x.`. Each "commit" on this branch is tagged with a release revision.

Any other branch, including the main development branch `develop`, contains in-progress development code.

## License
This package is under the MIT license.
