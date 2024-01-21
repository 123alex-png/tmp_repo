# Utrace

Utrace is a project that aims to provide a platform for data generation for UOS.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Contributing](#contributing)
- [License](#license)
- [Authors](#authors)
- [Acknowledgments](#acknowledgments)
- [Troubleshooting](#troubleshooting)

## Installation

To install utrace, follow these steps:

1. Run `cmake .` in the project directory.
2. Run `make` to generate the executable file.

## Usage

utrace provides the following options:

- `--config, -c [filename]`: Specifies the config file (Now simplified as a number for target config).

    No config file means only catpure command line.
- `--pid, -p [pid]`: Attaches to an existing process.
- `--exec, -e [commands]`: Runs a process from here.
- `--port, -p [port]`: Listens to a port, waiting for a process calling it to hook.
- `--output`: Set output file. [TODO]

The process uses pid first, followed by exec, and then port. When nothing is input, the default will be used, where access NETLINK is needed. Note that except for --exec, super user's privilege is needed. 

Example usage:

```
./utrace --config 1 --pid 1234
./utrace --config 2 --exec ./myProgram 
./utrace --config 3 --port 1145
./utrace --config 3
```



## Configuration

TODO

## Contributing

TODO

## License

TODO

## Authors

- Yanyan Jiang
- Yunfan Cao


## Acknowledgments

TODO

## Troubleshooting

TODO
