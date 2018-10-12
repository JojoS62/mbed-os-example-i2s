# Sample program for using the I2S interface with CS43L22 DAC on a STM Discovery 407VG board

Please install [mbed CLI](https://github.com/ARMmbed/mbed-cli#installing-mbed-cli).

## Import the example application

From the command-line, import the example:

```
mbed import https://github.com/JojoS62/mbed-os-example-i2s
cd mbed-os-example-i2s
```

### Now compile

Invoke `mbed compile`, and specify the name of your platform and your favorite toolchain (`GCC_ARM`, `ARM`, `IAR`). For example, for the ARM Compiler 5:

```
mbed compile -m DISCO_F407VG -t GCC_ARM
```

```

### Program your board

The Discovery F407VG board is not an official mbed enabled board. However, it contains a STLINK-V2 interface for programming.
Use the STLink Utility to program the flash. The binary file is created in the BUILD directory.

## Troubleshooting

If you have problems, you can review the [documentation](https://os.mbed.com/docs/latest/tutorials/debugging.html) for suggestions on what could be wrong and how to fix it.
