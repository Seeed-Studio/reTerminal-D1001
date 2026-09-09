# reTerminal D1001

## Introduction

The project provides basic SDK for the reTerminal D1001, as well as the examples for getting started. It is based on the [ESP-IDF](https://github.com/espressif/esp-idf).

## Getting Started

### Install ESP IDF

Follow instructions in this guide [ESP-IDF - Get Started](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) to setup the built toolchain. 
Currently we're using the latest version `v6.1`.

### Clone and Setup the Repository

1. Clone our repository.

    ```sh
    git clone https://github.com/Seeed-Studio/reTerminal-D1001
    ```

2. Go to `reTerminal-D1001` folder.

    ```sh
    cd reTerminal-D1001
    ```

### Build and Run Examples

1. Go to examples folder and list all available examples.

    ```sh
    cd examples && \
    ls
    ```

2. Choose a `<demo>` and enter its folder.

    ```sh
    cd '<demo>'
    ```

3. Generate build config using ESP-IDF.

    ```sh
    # set build target
    idf.py set-target esp32p4
    ```

4. Build the demo firmware.

    ```sh
    idf.py build
    ```

5. Flash the demo firmware to device and Run.

    To flash (the target serial port may vary depend on your operating system, please replace `/dev/ttyACM0` with your device serial port).

    ```
    idf.py --port /dev/ttyACM0 flash
    ```

    Monitor the serial output.

    ```
    idf.py --port /dev/ttyACM0 monitor
    ```

#### Tip

- Use `Ctrl+]` to exit monitor.

- The previous two commands can be combined.

    ```sh
    idf.py --port /dev/ttyACM0 flash monitor
    ```

## Contributing

- If you find any issue in using these examples, or wish to submit an enhancement request, please use the raise a [Issue](https://github.com/Seeed-Studio/reTerminal-D1001/issues) or submit a [Pull Request](https://github.com/Seeed-Studio/reTerminal-D1001/pulls).


## License

```
This project is released under the Apache 2.0 license.
```

