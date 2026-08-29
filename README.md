# cdh-tsat6
Contains software for the TSAT6 (2020-2023) Command and Data Handling (CDH) board, which uses an STM32 microcontroller.

## Project Directories
**cdh-tsat6-stm32project-nucleo** : STM32 Project used to test code on the Nucleo development board

**cdh-tsat6-stm32project-boardV1** : STM32 Project for the first version of the CDH custom PCB

## Setup
To set up the project you must also have the [tsat-utilities-kit](https://github.com/UMSATS/tsat-utilities-kit) repository cloned and positioned in the correct directory locations so that the project linker can find the library.

Use the git command `git clone <repository_url>` to clone the CDH and [TUK](https://github.com/UMSATS/tsat-utilities-kit) repos in the desired workspace for UMSATS projects or use the github desktop app UI to achieve the same result.

The following image gives you the correct project layout needed for the code to function and compile properly.

![Directory Layout](git-repo-layout.PNG)  

With this directory setup, the main branch of the project should compile and you can begin coding.


## Software Development Process
-1. Write your code within the Nucleo development board project (**cdh-tsat6-stm32project-nucleo**).

-2. Test your code using the Nucleo development board. If you can not test the code using only the Nucleo development board and require the CDH custom PCB, just ensure the code builds with no errors. For example, this may be the case if you need to test a driver file for a peripheral not on the Nucleo development board.

-3. Once your code works as expected, copy and paste the files you have written, or any changes you have made into the CDH custom PCB project (**cdh-tsat6-stm32project-boardV1**). You generally won't need to modify the main code since both projects use an L4-series STM32 microcontroller. However, you may need to modify some configuration values. For example, you may need to change the CAN baud rate configuration value if the baud rate used on the Nucleo development board is different than the baud rate used on the CDH custom PCB. Furthermore, you will likely need to change the pin configurations, since the pins are configured differently for each project.

-4. Submit a pull request.

-5. We will test your new code at an in-person worksession using the CDH custom PCB.

## About UMSATS
UMSATS is a student driven group that works to build 3U nanosatellites to compete in the Canadian Satellite Design Challenge (CSDC).

Our website can be found here: http://www.umsats.ca/
