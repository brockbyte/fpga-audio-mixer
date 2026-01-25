# Board Assembly Notes

## ⚠️ Hardware Error - Manual Soldering Required

There is an error in the schematic and board layout. **PIN 36 of the 40 pin connector and PIN 13 of the MCP3008 IC should be connected**, but due to a net name mismatch in the schematic, they are not.

**You need to solder a connection manually between these two pins.**

See the reference image: [../images/wire_mcp3008.jpg](../images/wire_mcp3008.jpg)
