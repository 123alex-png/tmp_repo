# SimpleAttach

SimpleAttach is a project that uses gdb to hook processes and generate logs.

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

To install SimpleAttach, follow these steps:

1. Run `cmake .` in the project directory.
2. Run `make` to generate the executable file.

## Usage

SimpleAttach provides the following options:

- `--config [filename]`: Specifies the config file (Now simplified as a number for target config).
- `--pid [pid]`: Attaches to an existing process.
- `--exec [commands]`: Runs a process from here.
- `--port [port]`: Listens to a port, waiting for a process calling it to hook.

The process uses pid first, followed by exec, and then port. When nothing is input, the default will be used, where access NETLINK is needed. Note that except for --exec, super user's privilege is needed. 

Example usage:

```
./simpleAttach --config 1 --pid 1234
./simpleAttach --config 2 --exec ./myProgram 
./simpleAttach --config 3 --port 1145
./simpleAttach --config 3
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
