# Zephyr z_tire_temp Description
Z tire temp is a repository containing firmware for the tire temp board. Its job is to measure the tire temperature using its thermal camera. 

### Building and editor setup

Open this application directory in VS Code. New integrated PowerShell terminals
automatically activate the west workspace's shared `../.venv`. Outside VS Code,
activate it with `. ../.venv/Scripts/Activate.ps1`.

From this application directory:

```powershell
west build -b vcu_stm32 .
```

CMake exports `build/compile_commands.json`, and `.clangd` locates it relative to
the application directory. Include paths and C/C++ language settings come from
that database; do not duplicate them in editor flags.

The tracked `.vscode/settings.json` supplies this `clangd.arguments` entry so
clangd can discover standard-library headers from both compilers in the
workspace's `sdk/` directory:

```text
--query-driver=**/sdk/zephyr-sdk-*/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc.exe,**/sdk/zephyr-sdk-*/arm-zephyr-eabi/bin/arm-zephyr-eabi-g++.exe
```

Install the clangd VS Code extension and restart clangd after changing its
arguments. Open a new terminal to pick up terminal profile changes. Other
`.vscode/` files remain local and ignored by Git. After moving the west workspace,
refresh its local SDK configuration using the workspace setup script, then run
`west build -p always -b vcu_stm32 .` to regenerate paths and generated headers.
For Cortex-Debug, point `armToolchainPath` at the installed workspace SDK's
`arm-zephyr-eabi/bin` directory and use `toolchainPrefix: arm-zephyr-eabi`.

### Tire Measurement 
Tire measurement is done by getting the averages of strips of the tire from the thermal Camera.


# Firmware Architecture  

The firmware is divided into two main classes 

1. **Thermal Camera**
    * Acts as a driver for the thermal camera
    * Is basically an Object to interact with the camera 
2. **Thermal Pipeline**
    * Is the processing thread responsible for processing and transmitting the information provided by the camera 

## Custom data Handoff 
These two classes would traditionally be using a queue this was not implemented largely due to the size of the thermal frames. Arguably a queue would be simpler but theres more metadata involved per queue object (also arguably trivial). Regardless the current implementation uses two Thermal Frame pointers one internal one where new thermal camera data is passed and one external one where an "consumer" such as thermal pipeline can get the current most thermal frame and then poll for a newer frame (some IRQ implementation here would make this more efficient). 


# TODO
- [ ] Create a live viewer to debug the camera frames.
- [ ] Implement ISR's for frame handoff
- [ ] Crank up the refresh rate and see what can be reliably processed
- [ ] Create proper DBC files and CAN outputs 

### Testing
- [ ] Test CAN 
- [ ] TEST STUFF WHEN IMPLEMENTED



