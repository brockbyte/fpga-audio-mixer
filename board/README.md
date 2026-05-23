# Board Assembly Notes

## ⚠️ Hardware Error - Manual Soldering Required

There is an error in the schematic and board layout. **PIN 36 of the 40 pin connector and PIN 13 of the MCP3008 IC should be connected**, but due to a net name mismatch in the schematic, they are not.

**You need to solder a connection manually between these two pins.**

![Board Correction](/images/Verbindung.jpg)

## Alternative connections for rotary encoders

I first tried to connect the rotary encoders via the MCP I/O expander. However, that turned out to be not working properly due to a lot issues with switch bouncing, as the MCP were not fast enough for rotating the knobs quickly. So, I connected the rotary encoders also directly to the FPGA. Luckily, the Intel Dev board had a lot of pins left I could use. Performing all the debounce stuff inside the FPGA worked really well and also the FPGA was fast enough for handling also quick rotations.

If you build the system, here is a picture of how I connected the rotary pins to the FPGA. Instead of soldering the MCP ICs, I just soldered headers:

![Board Rotary Conn A](/images/Rotary_Conn_Board_1.jpg)
![Board Rotary Conn B](/images/Rotary_Conn_Board_2.jpg)

And on the FPGA side it looks like this:

![Board Rotary FPGA](/images/Rotary_Conn_FPGA.jpg)
