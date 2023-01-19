## About SVD

ARM defines an SVD (System View Description) file format in its CMSIS standard as a means for Cortex-M-based chip manufacturers to provide a common description of peripherals, registers, and register fields.

## About SVD-Tools

https://github.com/1udo6arre/svd-tools

It's a python module for gdb. This adds some gdb commands to:

- get information on peripherals | registers | fields
- get registers values | register fields
- set registers values | register fields
- dump registers values to file

## Install

1. install cmsis-svd
   pip install -U cmsis-svd

2. install terminaltables
   pip install -U terminaltables

3. copy `gdb-svd.py` to your project folder.
   get `gdb-svd.py` from <https://github.com/1udo6arre/svd-tools>

4. get the SVD file for your MCU, download from:

   - https://github.com/posborne/cmsis-svd
   - st.com product document page, e.g. [section System View Description](https://www.st.com/en/microcontrollers-microprocessors/stm32f030c8.html).
   - https://developer.arm.com/tools-and-software/embedded/cmsis

## Load

1. launch GDB (client).
   `$ arm-none-eabi-gdb firmware.elf`
   or
   `$ arm-none-eabi-gdb firmware.elf -ex "your_gdb_init_command" -ex "another_init_command"`
   or
   `$ arm-none-eabi-gdb firmware.elf -x your_gdb_init_commands_file.gdb`

2. load SVD file.
   type commands in GDB:
   `(gdb) source path/to/gdb-svd.py`
   `(gdb) svd path/to/*.svd`

## SVD command usage

1. Help
   `(gdb) help svd`

2. List available peripherals
   `(gdb) svd info ` + `<TAB>` key

3. Info on peripheral register field
   `(gdb) svd info peripheral_name`
   `(gdb) svd info peripheral_name register_name`
   `(gdb) svd info peripheral_name register_name field_name`

4. Get value
   `(gdb) svd get peripheral_name`
   `(gdb) svd get peripheral_name register_name`

5. Set value
   `(gdb) svd set peripheral_name value`
   `(gdb) svd set peripheral_name register_name value`
