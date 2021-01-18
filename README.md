# Lexus-GS450H-Inverter-Controller
Opensource controller for the Lexus GS450H Hybrid inverter and gearbox.
<br>
Controls the inverter using it's own OEM logic board. No modification required.
<br>
Connects to the gearbox and controls oil pump, gear shifting and temperature monitoring.
<br>
Only requires throttle and brake inputs from the vehicle but also has CAN capabilty for communicating with modern vehicles.
<br>
Big thanks to Tom Darby for figuring out the control protocol.
<br>
Thread on the openinverter.org forum : <br>
https://openinverter.org/forum/viewtopic.php?f=14&t=205
<br>
PCB files in Designspark PCB 8.1 format.
<br>
Copyright 2019 D.Maguire and T.Darby
<br>
<br>
29/09/19 : Initial commit.PCB design in progress. 
<br>
<br>
27/10/19 : First successful test of the prototype running on the bench : https://youtu.be/F5XM6blwMlw
<br>
24/11/19 : Uploaded V3 software with reverse function. When IN1 is low motors run forward. when IN1 is pulled to +12v MG2 reverses.

21/01/2020 : Due to popular demand and a return to a no risk no reward culture, I have released the V2 design including the JLCPCB files so you can all take an arrow in back for me if I screwed up the design. In case anyone for some weird reason would like to support me why not buy a board from the EVBMW webshop : https://www.evbmw.com/index.php/evbmw-webshop

17/02/20 : WiFi interface files uploaded. Designed to run on the Olimex ESP8266 WiFi Module : <br>
https://www.olimex.com/Products/IoT/ESP8266/MOD-WIFI-ESP8266/open-source-hardware
<br>
<br>
02/06/20 : It works : https://www.youtube.com/watch?v=9eRWR5xXItc
<br>
<br>
18/01/21 : A more user friendly firmware (gs450h_v3_user.ino) with a basic serial menu interface has been tested and uploaded to aid those not familiar with programming.<br> A full description of the menu system may be found on the openinverter wiki page :
https://openinverter.org/wiki/Lexus_GS450h_Inverter


