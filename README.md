# README #

Repository for xMCU project.

## Overview

The xMCU is a Hardware Abstraction Layer (HAL) utilizing the power and flexibility of modern C++.

### Our goals:

 - __Modular SoC Support:__ Implement each System on Chip (SoC) as a separate Git submodule, enabling users to selectively download only necessary components tailored to their hardware.
 - __Compile-Time Checks:__ Conduct thorough compile-time validation to ensure compatibility with available hardware resources such as GPIO pins, ADC channels, and other peripherals, minimizing runtime errors and optimizing resource utilization.
 - __Embedded Documentation:__ Implement self-documenting practices within the codebase. This approach enables developers to leverage IDE features like code completion and error checking to catch potential issues early in the development process.
 - __Minimal Dependencies:__ Utilize only CMSIS (Cortex Microcontroller Software Interface Standard) and the C++ Standard Library to maintain portability, reduce overhead, and promote compatibility across various microcontroller platforms.
 - __Low Overhead Design:__ Design the HAL with a focus on minimizing memory footprint and processing overhead, optimizing performance for resource-constrained embedded environments.

## Supported SOC:

- ST
  - stm32l0 - rm0451
  - stm32wb - rm0434

## Usage

Examples of usage you can find here in another repository.

## Contribution and Support

We welcome contributions from the community to help improve and extend the xMCU Project. Please refer to the Contributing Guide for more details on how to contribute. For support and questions, feel free to open an issue on GitHub or contact us at main@xembedded.io

## License

The xMCU Project is licensed under the Apache 2.0 License. See the LICENSE file for more details.
