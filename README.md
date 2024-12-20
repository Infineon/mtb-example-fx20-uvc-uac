# EZ-USB&trade; FX20: USB Video Class (UVC) application

This code example demonstrates the implementation of a UVC 1.1 compliant camera application using the EZ-USB&trade; FX20 device. An integrated USB Audio Class interface is provided to stream the audio data received from a PDM microphone. This example illustrates the configuration and usage of Sensor Interface Port (SIP) on the FX20 device to implement the Synchronous Slave FIFO IN protocol.

> **Note:** This code example is an alpha release for the EZ-USB&trade; FX20 device, and is also applicable to EZ-USB&trade; FX10/EZ-USB&trade; FX5 devices

[View this README on GitHub.](https://github.com/Infineon/mtb-example-fx20-uvc-uac)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyNDA4NjUiLCJTcGVjIE51bWJlciI6IjAwMi00MDg2NSIsIkRvYyBUaXRsZSI6IkVaLVVTQiZ0cmFkZTsgRlgyMDogVVNCIFZpZGVvIENsYXNzIChVVkMpIGFwcGxpY2F0aW9uIiwicmlkIjoic3VrdSIsIkRvYyB2ZXJzaW9uIjoiMS4wLjEiLCJEb2MgTGFuZ3VhZ2UiOiJFbmdsaXNoIiwiRG9jIERpdmlzaW9uIjoiTUNEIiwiRG9jIEJVIjoiV0lSRUQiLCJEb2MgRmFtaWx5IjoiU1NfVVNCIn0=)


## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.2 or later (tested with v3.2)
- Board support package (BSP) minimum required version: 4.3.3
- Programming language: C


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v11.3.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`


## Supported kits (make variable 'TARGET')

- [EZ-USB&trade; FX20 DVK](https://www.infineon.com/fx20) (`KIT_FX20_FMC_001`) – Default value of `TARGET`

## Hardware setup

- This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.
- This example demonstrates the implementation of a UVC compliant camera application with [Titanium Ti180 J484 Development Kit] (https://www.efinixinc.com/products-devkits-titaniumti180j484.html) as the video source.
> **Note:** Titanium Ti180 J484 Development Kit is used for demonstration purpose only. The Ti180 FPGA bit files listed below supports upto 4K (3840x2160) colorbar video streaming.

## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.

Install a terminal emulator if you don't have one. Instructions in this document use [Tera Term](https://teratermproject.github.io/index-en.html).

This example requires no additional software or tools.


## Using the code example

### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool.

   There are several ways to do this, including launching it from the dashboard or from inside the Eclipse IDE. For more details, see the [Project Creator user guide](https://www.infineon.com/ModusToolboxProjectCreator) (locally available at *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/docs/project-creator.pdf*).

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-kits-make-variable-target).

   > **Note:** To use this code example for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. On the **Select Application** page:

   a. Select the **Applications(s) Root Path** and the **Target IDE**.

   > **Note:** Depending on how you open the Project Creator tool, these fields may be pre-selected for you.

   b.	Select this code example from the list by enabling its check box.

   > **Note:** You can narrow the list of displayed examples by typing in the filter box.

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**.

   d. Click **Create** to complete the application creation process.

</details>

<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing "modus-shell" in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The following example clones the "[EZ-USB&trade; FX20: USB_video_class_application](https://github.com/Infineon/mtb-example-fx20-uvc-uac)" example with the desired name "UVC_UAC" configured for the *KIT_FX20_FMC_001* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id KIT_FX20_FMC_001 --app-id mtb-example-fx20-uvc-uac --user-app-name UVC_UAC --target-dir "C:/mtb_projects"
   ```
The 'project-creator-cli' tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the <id> field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the <id> field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

> **Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at {ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf).

</details>


### Open the project

After the project has been created, you can open it in your preferred development environment.


<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>


<details><summary><b>Visual Studio (VS) Code</b></summary>

Launch VS Code manually, and then open the generated *{project-name}.code-workspace* file located in the project directory.

For more details, see the [Visual Studio Code for ModusToolbox&trade; user guide](https://www.infineon.com/MTBVSCodeUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_vscode_user_guide.pdf*).

</details>


<details><summary><b>Command line</b></summary>

If you prefer to use the CLI, open the appropriate terminal, and navigate to the project directory. On Windows, use the command-line 'modus-shell' program; on Linux and macOS, you can use any terminal application. From there, you can run various `make` commands.

For more details, see the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation

1. Connect the board (J2) to your PC using the provided USB cable. Connect the USBFS port (J3) on the board to PC for debug logs.

2. Open a terminal program and select the Serial COM port. Set the serial port parameters to 8N1 and 921600 baud.

3. Connect the FPGA (Titanium Ti180 J484 Development Kit) DVK to the FMC (J8) connector of *KIT_FX20_FMC_001* board.

4. Browse the **\<CE Title>/BitFiles/** folder for Ti180 FPGA binary and program the FPGA. For more details, see the kit [user guide] (https://www.efinixinc.com/products-devkits-titaniumti180j484.html).

5. Perform the following steps to program the board using the **EZ-USB&trade; FX Control Center** (Alpha) application.

      1. Open the **EZ-USB&trade; FX Control Center** application. <br>
      The EZ-USB&trade; FX20 device enumerates as **EZ-USB&trade; FX Bootloader**.

      2. Select the **EZ-USB&trade; FX Bootloader** device in **EZ-USB&trade; FX Control Center**.

      3. Click **Program** > **Internal Flash**.

      4. Navigate to the **\<CE Title>/build/APP_KIT_FX20_FMC_001/Release** folder within the CE directory and locate the *.hex* file and program.

      5. Confirm if the programming is successful in the log window of the **EZ-USB&trade; FX Control Center** application.

6. After programming, the application starts automatically. Confirm that the following title is displayed on the UART terminal.

   **Figure 1. Terminal output on program startup**

   ![](images/terminal-fx20-uvc-uac.png)

7. Open any third party camera application, select the FX20 device and video resolution to stream video. 

8. By default, the device will stream 4K (3840 x 2160 ~60 fps) video data.


## Debugging

By default, the USBFS port is enabled for debug logs. 

To enable debug logs on UART, set the **USBFS_LOGS_ENABLE** compiler flag to '0u' in the *makefile*. SCB1 of the FX20 device is used as UART with a baud rate of 921,600 to send out log messages through the P8.1 pin.

Debug the code example by setting debug levels for the UART logs. Set the **DEBUG_LEVEL** macro in the *main.c* file with the following values for debugging.

**Table 1. Debug values**

Macro value     |    Description
:-------------  | :------------
1u              | Enable error messages
2u		          | Enable warning messages
3u              | Enable info messages
4u           	 | Enable all messages

<br>


## Design and implementation

This code example demonstrates the implementation of the USB Video Class specification and the USB Audio Class specification, allowing the FX20 device to function as a UVC- and UAC-compliant composite device. This allows the video and audio data to be streamed over USB 2.0. This application uses various low-performance peripherals to interface with the system such as:

1. I2C master to configure the video source. 
2. PDM receiver interface to connect audio source. 
3. SMIF (in x4 or quad mode) interface for downloading of FPGA configuration binary on every bootup.
4. Enable debug prints over CDC using USBFS block on FX20. 

### Features

1. **USB specification:** USB 2.0 (HS only) and USB3.2 Gen2/Gen1

> **Note:** UVC and UAC functions are not supported in USB Full-Speed connections.

2. **Video format:** Uncompressed YUY2 (YUYV)

3. **Video resolutions:**
   - **USB3.2 (Gen1/Gen2):** 3840*2160 (4K), 1280*720 (720p), and 640*480 (VGA)
   - **USB2.0 (HS):** 640*480 (VGA)

4. FPGA configuration using SMIF block. The application supports FPGA configuration in Passive serial (x4) mode only.

> **Note:** This example does not currently implement any UVC control functions such as brightness, contrast, exposure, etc.

This code example demonstrates the following:
1. The usage of the FX20 APIs to implement a standard USB video class device. Handling of class-specific USB control requests at the application level.
2. Streaming of video data at USB 3.2 Gen2 speeds (4K video stream at ~60 fps) from the SIP to USB endpoint. A Bulk endpoint is used for video streaming to enable the maximum throughput, which can exceed the maximum that the USB specification allows over Isochronous endpoints.
3. FPGA configuration using SMIF Block of the FX20 device.
4. I2C register writes to configure FPGA register.

### Video/audio streaming data path

#### Audio streaming

1. The application enables EP 3-IN as Isochronous endpoints with maximum packet size of 192 bytes.
2. The device receives the audio data through the PDM receiver and sends it on EP 3-IN. 
3. Four DMA buffers of size 192 bytes are used to hold the data while it is being forwarded to the USB.

#### Video streaming

1. The application enables EP 1-IN as Bulk endpoints with maximum packet size of 1024 bytes. In case of USB3.2 Gen1/Gen2, the endpoint is configured to support a maximum burst of 16 packets.
2. The device receives the video data through LVDS/LVCMOS Adapter 0 Socket 0 and sends it on EP 1-IN. 
3. Four DMA buffers of 64512 bytes size are used to hold the data while it is being forwarded to USB.

#### Modes of UVC Header addition

A 32-byte UVC Header is added to each buffer containing 64480 bytes of video data, and the resulting 64512 bytes are sent to the USB host. By default, the Header addition is performed by a firmware task. 
If the FPGA, which drives the SIP interface of FX20 supports the Header addition, the application can be configured to enable FPGA to add the UVC Header; thereby speeding up the streaming operation.

1. tHE Firmware adds 32 bytes of UVC Header in the data received from the LVCMOS/LVDS side before the data is sent to the USB side on UVC Bulk endpoint 1-IN.
2. UVC Header is inserted by FPGA itself. To enable this feature, enable the FPGA_ADDS_HEADER compiler flag in the *makefile*.
3. This example does not support UVC hHader insertion using the LVCMOS/LVDS IP. This option can only be used if the FPGA which provides the video data supports `enhanced mode` commands to trigger the UVC Header (metadata) addition. This feature will be available in the next release.

### Application workflow

The application flow involves three main steps:

#### Initialization

During initialization, the following steps are performed:

1. All the required data structures are initialized.
2. USBD and USB driver (CAL) layers are initialized.
3. Application registers all descriptors supported by function/application with the USBD layer.
4. Application registers callback functions for different events like `RESET`, `SUSPEND`, `RESUME`, `SET_CONFIGURATION`, `SET_INTERFACE`, `SET_FEATURE`, and `CLEAR_FEATURE`. USBD will call the respective callback function when the corresponding events are detected.
5. Initialize the data transfer state machines.
6. Application registers handlers for all relevant interrupts.
7. Application makes the USB device visible to the host by calling the Connect API.
8. FPGA is configured using the SMIF (in x4 or quad mode) block to read the bit file stored in the external flash. FPGA sees the data on bus and gets configured.
9. FPGA is initialized using I2C writes to FPGA registers.
10. Application initializes the SIP block on FX20 as required by the selected LVCMOS operating mode.

#### USB device enumeration

1. During USB device enumeration, the host requests for descriptors, which are already registered with the USBD layer during the initialization phase.
2. Host will send `SET_CONFIGURATION` command and `SET_INTERFACE` commands to activate the required function in the device.
3. After the `SET_CONFIGURATION` and `SET_INTERFACE` commands, the application task takes control and enables the endpoints for data transfer.

#### UVC data transfer

**Loopback mode (LVDS interface):**

   - Loopback program (Video data and control bytes) is stored in HBWSS SRAM.
   - Data is moved by Thread 3 to PORT1 through DMA and GPIF.
   - Link level loopback is enabled so that the data generated by PORT1 is received by PORT0.
   - Data moves from PORT0 to USB 3.x Bulk endpoint-IN through GPIF and DMA.
   - Data moves from device Bulk endpoint-IN to Host UVC application.

**Normal mode:**

   - Depending on the compile-time options, LVDS/LVCMOS interface, PORT1/PORT0, and Narrow_Link/Wide_Link are selected.
   - Once the streaming DMA channel is enabled, DMA ready flag on the SIP interface asserts and the FPGA data source starts streaming the video data to the FX20 device.
   - Video data moves from the LVDS/LVCMOS subsystem to SRAM through high-bandwidth DMA.
   - The data is forwarded on the USBSS or USBHS EP 1-IN, based on the active USB connection speed. DataWire DMA channels are used in the case of USBHS transfers and high-bandwidth DMA channels are used in case of USBSS transfers.
   - Video data moves from USB device Bulk endpoint-IN to Host UVC application.

#### Integrated UAC data transfer

In addition to the USB video camera interface, this application also implements a USB Audio Class interface, which carries data received from a PDM microphone.

- The P4.1 and P4.2 pins of the FX20 device are used to interface to the PDM microphone.
- P4.1 is the clock output from the FX20 device, which will be generated at 3.072 MHz.
- P4.2 is the data input from the PDM microphones to the FX20 device.

A pair of microphones in stereo configuration can be connected. Selection between Mono and Stereo microphone configuration is done using the STEREO_ENABLE pre-processor setting, which can be updated through the *makefile*. By default, the interface is configured for Mono operation.  

## Compile-time configurations

This application's functionality can be customized through the compile-time parameters that can be turned ON or OFF through the *makefile*.
The application uses the GNU Arm&reg; 11.3 toolchain, which is part of the ModusToolbox&trade; installation for compilation.
- Run the `make` command or build the project in your IDE to compile the application and generate a USB bootloader compatible binary. This binary can be programmed to the FX20 device using the **EZ-USB&trade; Control Center** application.
- Run the `make BLENABLE=no` command or set the variable in the **makefile** to compile the application and generate the standalone binary. This binary can be programmed onto the FX20 device through the SWD interface using the OpenOCD tool. For more details, see the [EZ-USB&trade; FX SDK user guide](https://www.infineon.com/fx20).
- Add `GEN1=yes` to the build command line or set the variable in the **makefile** to enable a USB 3.2 Gen1 data connection instead of a USB 3.2 Gen2 data connection. E.g., make `GEN1=yes`.
- Add `SINGLELANE=yes` to the build command line or set the variable in the **makefile** to enable a USB 3.2 Gen2x1 data connection instead of a USB 3.2 Gen2x2 data connection. E.g., make `SINGLELANE=yes`.
- Add `LPBK_EN=yes` to the build command line or set the variable in the **makefile** to make the application use an internally-generated colorbar pattern for streaming instead of an external video source. This configuration can be used on the FX20 DVK without requiring any additional boards or connections. E.g., make `LPBK_EN=yes`.

> **Note:** The internal data path used for this function has throughput limitations and can only support about 48 frames per second of a 4K video stream. Using an external video source is required to achieve the data rates enabled by the USB 3.2 Gen2 data connection.

By default, the application is configured to receive data from a 32-bit wide LVCMOS interface in DDR mode and make a USB 3.2 Gen2 data connection. These settings can be modified by passing additional options on the build command line as well as by modifying settings in the *Makefile*.


**Table 2: Macro description** 

Macro name         |    Description                           | Allowed values
:-------------     | :------------                            | :--------------
LVCMOS_EN          | Select the LVCMOS/LVDS                   | 1u for LVCMOS. <br> 0u for LVDS.
WL_EN              | Select the Wide Link or Narrow Link     | 1u for 32-bit/WideLink. <br> 0u for NarrowLink.
PORT1_EN           | Select LVCMOS/LVDS Port                  | 1u to selecting PORT1. <br> 0u to select PORT0.
LVCMOS_DDR_EN      | Select LVCMOS Clock Configuration        | 1u for LVCMOS DDR clock. <br> 0u to select LVCMOS SDR.
INTERLEAVE_EN      | Enable thread interleave PORT0           | 1u to enable thread interleaving on PORT0 PORT1. <br> 0u to select single thread.
FPGA_ENABLE        | Select FPGA as data source               | 1u if FPGA is interfaced to FX20. <br> 0u for to disable FPGA interface.
FPGA_ADDS_HEADER   | UVC Header Addition                      | 1u for FPGA added header. <br> 0u for FX20 added header.
AUDIO_IF_EN        | Enable Integrated UAC function           | 1u to enabled. <br> 0u to disable.
STEREO_ENABLE      | Enabled Audio in Stereo Mode             | 1u for stereo audio. <br> 0u for mono audio.
USBFS_LOGS_ENABLE  | Enable debug logs through USBFS port     | 1u for debug logs over USBFS. <br> 0u for debug logs over UART (SCB1).

<br>


### FPGA BitFile information

The FPGA binary in the BitFile folder of the project can be programmed to the external flash on the FX20 DVK using the **EZ-USB&trade;FX20 USB Bootloader**. Perform the following steps to program the binary file to the external flash.

> **Note:** The following steps are assuming that the device is pre-programmed with **EZ-USB&trade; FX20 USB Bootloader**. 

   1. Open the **EZ USB&trade; FX Control Center** application.
   2. Navigate to **Program** > **External Flash** and browse the FPGA binary file.
   3. Check the programming status.

**Table 3: BitFile description**

BitFile                                                | Description                   | Supported features
:-------------                                         | :------------                 | :------------  
fxn_ti180_dvk_multi_cam_ref_des_lvcmos_ddr_nl_p1.hex   | LVCMOS Port1 Narrow Link DDR  | PORT 1 Single Thread, FPGA added header      
fxn_ti180_dvk_multi_cam_ref_des_lvcmos_sdr_nl_p1.hex   | LVCMOS Port1 Narrow Link SDR  | PORT 1 Single Thread
fxn_ti180_dvk_multi_cam_ref_des_lvcmos_ddr_nl_p0.hex   | LVCMOS Port0 Narrow Link DDR  | PORT 0 Single Thread, PORT 0 Thread Interleave, FPGA added header
fxn_ti180_dvk_multi_cam_ref_des_lvcmos_ddr_wl.hex      | LVCMOS Wide Link DDR          | PORT 0 Single Thread, FPGA added header
fxn_ti180_dvk_multi_cam_ref_des_lvcmos_sdr_wl.hex      | LVCMOS Wide Link SDR          | PORT 0 Single Thread, PORT 0 Thread Interleave, FPGA added header
fxn_ti180_dvk_multi_cam_ref_des_lvds_nl_p0_148m.hex    | LVDS Port0 Narrow Link        | PORT 0 Single Thread, FPGA added header
fxn_ti180_dvk_multi_cam_ref_des_lvds_wl_148m.hex       | LVDS Wide Link                | PORT 0 Single Thread, FPGA added header

<br>


## Application files

**Table 4: Application file description**

File                                            |    Description   
:-------------                                  | :------------                
gpif_header_lvcmos.h                            | Generated Header file for GPIF State configuration for LVCMOS interface. 
gpif_header_lvds.h                              | Generated Header file for GPIF State configuration for LVDS interface.            
cy_usb_app.c                                    | C source file implementing UVC 1.1 application logic.
cy_usb_app.h                                    | Header file for application data structures and functions declaration.
cy_usb_uvc_device.h                             | Header file with UVC application constants and the video frame configurations.
cy_usb_descriptors.c                            | C source file containing the USB descriptors.
main.c                                          | Source file for device initialization, ISRs, LVCMOS/LVDS interface initialization, etc.
cy_uac_app.c                                    | C source file with UAC interface handlers.
cy_usb_i2c.c                                    | C source file with I2C handlers.
cy_usb_i2c.h                                    | Header file with I2C application constants and the function  definitions.
cy_usb_qspi.c                                   | C source file with SMIF handlers and FPGA configuration functions.
cy_usb_qspi.h                                   | Header file with SMIF application constants and the function definitions.
cm0_code.c                                      | CM0 initialization code.
Makefile                                        | GNU make compliant build script for compiling this example.

<br>


## Related resources


Resources  | Links
-----------|----------------------------------
Code examples  | [Using ModusToolbox&trade;](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [EZ-USB&trade; FX20 datasheets](https://www.infineon.com/fx20)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board)
Libraries on GitHub |  [mtb-pdl-cat1](https://github.com/Infineon/mtb-pdl-cat1) – Peripheral Driver Library (PDL) and docs
Middleware on GitHub | [usbfxstack](https://github.com/Infineon/usbfxstack) – USBFXStack middleware library and docs
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSOC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development.

<br>



## Other resources

Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.



## Document history


Document title: *CE240865* – *EZ-USB&trade; FX20: USB Video Class (UVC) application*

 Version | Description of change
 ------- | ---------------------
 1.0.1   | Updated to use latest USBFXStack from GitHub
 1.0.0   | New code example
<br>



All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.


---------------------------------------------------------

© Cypress Semiconductor Corporation, 2024. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress's patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress's published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, ModusToolbox, PSoC, CAPSENSE, EZ-USB, F-RAM, and TRAVEO are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.
