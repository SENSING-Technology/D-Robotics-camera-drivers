###### 202608019 ######


1. Supports the following cameras:
   ```
   S36
   S56C

   SN2M4EFGD
   SG3S11AFSK
   SG8-OX08DC-5300-G2G-Hxxx
   SG8S-AR0820C-5300-G2G-Hxxx
   ```


###### Known Issue ######

1. Repeated V4L2 graph hot reload is not reliable.
   - Repeatedly running `load.sh` without rebooting may leave stale vendor
     media-entity state. Typical messages include `Entity type ... was not
     initialized`, followed by missing `/dev/video*` nodes.
   - Workaround: stop all camera applications, reboot the board, and run
     `sudo ./load.sh` once after the cold boot. Reboot after replacing a
     Sensor, Serializer, Deserializer, or MIPI driver.
