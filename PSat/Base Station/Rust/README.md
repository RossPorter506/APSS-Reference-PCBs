# How to use

Install Rust: https://www.rust-lang.org/learn/get-started

Install MSP430-GCC (installer or toolchain-only): https://www.ti.com/tool/MSP430-GCC-OPENSOURCE. Make sure it's on your PATH (or on Linux you can use `rustc-wrapper.sh`).

Then run the following commands:
`rustup install toolchain nightly`
`rustup component add rust-src --toolchain nightly`

You can now build the executable with `cargo build`. You can build in release mode with `cargo build --release`

(On Linux you will have to change the 'runner' line in `.cargo/config.toml` from `run.bat` to `run.sh`.)

# Flashing the board

Either use Code Composer Studio, under the 'flash' option click the dropdown and select the option that says 'select file to flash'. Point CCStudio to the binary at ./target/msp430-none-elf/release/apss_mcu_pcb_firmware

Alternatively, download uniflash from https://www.ti.com/tool/UNIFLASH#downloads. After installation open the program and either use auto-detect or input the board name (MSP430FR2355) manually. Click on 'standalone command-line' to generate a .zip file with all you need to flash the board.
Extract this folder so that dslite.bat is at `./uniflash/dslite.bat` within the project. 
After setting up uniflash you can flash the board by using `cargo run` or `cargo run --release`. (This also builds the project.)
