# Arribada STMF07-based tracker

## Building

Prerequisites:

* Linux (while the build works on other systems, these instructions assume a
  recent Linux distribution such as Ubuntu 19.04).

* System packages:
  * `gcc-arm-none-eabi`
  * `openocd`
  * `dfu-util`

  On Ubuntu 19.04 you can install it with:
  ```
  sudo apt install gcc-arm-none-eabi openocd dfu-util
  ```

To build the firmware, move into `ports/stm32` and call `make`. The build
firmware will be placed into `ports/stm32/build` under the name `ArribadaSTM32.bin`.

### Building in debug mode

The default makefile builds in release mode (no debugging symbols, debug output
on USART1 disabled). If you want to build in debug mode, you must run `make` as
follows:
```
make DEBUG=1
```

If you have built the firmware before make sure to run `make clean` first.

## Flashing

You can use the `dfu-util` utility to flash the firmware onto the tracker once
it has been switched into DFU mode. To flash the firmware, run the following
command from the `ports/stm32` directory after the firmware has been built:
```
sudo dfu-util --download build/ArribadaSTM32.bin -a 0 -s 0x8000000
```

