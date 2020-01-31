[![BluePrint logo](.media/logo.png)](.media/logo.png?raw=true "BluePrint logo")

# BluePrint

<p>
<a href="https://github.com/mupfelofen-de/BluePrint">
  <img src="https://img.shields.io/badge/project-GitHub-blue?style=flat?svg=true" alt="GitHub project" />
</a>
<a href="https://github.com/mupfelofen-de/BluePrint/blob/master/LICENCE.md">
  <img src="https://img.shields.io/badge/licence-BEER--WARE-blue?style=flat?svg=true" alt="Licence" />
</a>
<a href="https://travis-ci.org/mupfelofen-de/BluePrint">
  <img src="https://travis-ci.org/mupfelofen-de/BluePrint.svg?branch=master" alt="Build status" />
</a>
</p>

## About

A base project for the STM32F103C8T6 aka [blue pill
board](http://reblag.dk/stm32/).

This template is running FreeRTOS 10.2.1.  Preconfigured peripherals:
I²C, SPI, ADC and CRC.

## Installation

1. Install [PlatformIO Core](http://docs.platformio.org/page/core.html)
2. Run these commands:

```bash
    # Build project
    > platformio run

    # Upload firmware
    > platformio run --target upload

    # Clean build files (optional)
    > platformio run --target clean
```

## Licence

The source code of this software is licenced, unless stated otherwise,
under the terms of [The Beerware Licence](LICENCE.md).

Logo icon made by [Freepik](https://www.flaticon.com/authors/freepik)
from [Flaticon](https://www.flaticon.com/).
