# PSat Reference PCBs
This is a collection of general-purpose PCBs for use in Psat payloads. This is designed to minimise the boilerplate that every PSat requires and allows you to specialise on whatever unique payload you choose.
Feel free to use this in your own payload. you could (in order of increasing complexity):

-   Use a previously assembled board, no work necessary
-   Assemble a board from an existing, unpopulated PCB and components (recommended)
-   Modify the PCB layout to suit your needs, print it, and assemble it
-   Use the schematic as a reference for your own custom design and layout
-   etc.

The original v1 and v2 designs were designed by Ross Porter (me), but I don’t expect this to be a static project: This is designed to be a community project, so once I’m gone I expect those with some electrical experience to take ownership, making adjustments or adding new features as required.

## Payload Slices
There are several boards that can be mixed and matched according to your needs. A brief description of each is given below. A full description can be found in the relevant subfolder's readme.

### Power Supply Unit (PSU)
The PSU slice is responsible for both charging the Lithium battery that powers your PSat, but also providing usable power rails for all your other devices. The board provides two step-down converters (default 1.8V and 3.3V) and one step-up converter (default 5V), all user-configurable during assembly. The battery can be charged using a USB-C or micro USB cable.
See the documentation in the PSU folder for more information.

### Microcontroller (MCU)
The brains of the operation. A simple breakout board with programming header attached - never get stuck trying to program your microcontroller again! UART, SPI and I2C buses (among other pins) are broken out onto additional, optional stack headers for communication with other devices or slices.
See the readme in the MCU folder for more information.

### Beacon
Lost your PSat? Not any more. The beacon slice provides three locator devices:
 - A fully automatic audio beeper. This beeper produces a sharp chirp (100dB from 10cm away) every couple of seconds, which should be audible from about 50-100m away.
 - A 915MHz LoRa radio module. No radio license necessary, this radio can be used by a microcontroller to transmit a signal that you and a base station can use to triangulate the location of your payload. With a theoretical range of up to 2km (or 20km with a directional antenna on the base station), this radio is always an option. 
 - A self-contained GPS module. Use in combination with the LoRa radio and a microcontroller to broadcast your exact payload's GPS coordinates.
See the readme in the Beacon folder for more information.

## Versioning
This project uses a form of semantic versioning to manage board interoperability:
ver X.Y.Z means version X, revision Y, patch Z.

The version X relates to both electrical and physical compatibility. Boards that share a major version X shall be physically and electrically compatible with eachother. If a change prevents a board from either physically mating (e.g. changing stack header location) or electrically interoperating (e.g. changing the location of the stack's ground pin) with other boards then this must be associated with an increase in version number.

The revision number Y relates to software-level changes. If a change does not affect physical or electrical interoperability but does require changing software (e.g. the location of two control pins have been swapped) then this must be associated with an increase in the revision number.

The patch number Z relates to internal bug fixes or additions to functionality. An increase in a board's patch number must be completely transparent to the wider stack, only adding additional functionality or solving issues/bugs internal to the board. Any already working stack shall continue to function (physically, electrically, and programmatically) even if one board receives a patch.

Examples:

PSU v1.0.0 <=/=> MCU v2.0.0

MCU v2.0.0 <=> Beacon v2.1.0, with software changes

MCU v2.2.0 <=> Beacon v2.2.37, with no software changes
