# CC2530 Zigbee
Build platform base on the zigbee protocol with CC2530

## Prerequisite 
1. Download and install [EW8051-9301 IAR Tool](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/tools/iar_setup_guide)
2. Choose "File -> Open -> Workspace"
3. Install [SmartRF](http://www.ti.com/tool/FLASH-PROGRAMMER) to check CC2530 Node with CC-Debugger
4. Install [CCDebugDriver_x64 ](http://www.ti.com/tool/CC-DEBUGGER) for debugging and flash programming

## Hardware

![Architectural Hardware](Docs/hardware.PNG?raw=true)

- Frequency: 2.4 GHz
- chip: CC2530
- Communication: UART, SPI
- Distance: 150m
- Anten: PCB Antena and IPEX

## Software 

### Zigbee Stack 

![Zigbee Architecture](Docs/zigbee.PNG?raw=true)

### Z-Stack Mesh

Download and install Z-Stack Mesh package

![Architectural Z-Stack](Docs/software.PNG?raw=true)

1. Physical Layer: This layer does modulation and demodulation operations up on transmitting and receiving 
signals respectively.

2. MAC Layer: This layer is responsible for reliable transmission of data by accessing different networks
with the carrier sense multiple access collision avoidance (CSMA). This also transmits the beacon frames
for synchronizing communication.

3. Zigbee Stack: This layer contain the sub-layer as Network layer(NWK), Application support sublayer(APS)
Security

4. OSAL Layer: Operating System Abstract Layer

5. HAL Layer: Hardware Abstract Layer

6. Application Framework include 240 Zigbee Device Object(ZDO)






