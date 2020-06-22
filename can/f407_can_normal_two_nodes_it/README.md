## Hardware setup

Two DevEBox STM32F407VGt6 dev boards connected to MCP2551 CAN bus transceivers on a prototyping board. 
The twisted black/white wire CAN 'bus' is approximately 1m long. 5V Power supply
from JLink debugger which is used to flash both boards with identical firmware. The MCP2551 transceivers
operate at 5V, the STM32F407 CAN RX lines are 5V tolerant.

USB logic analyzer probes the CAN1_TX lines on both boards (PA12).  If you press button PA0
on a board, it starts transmitting as N1, and the other board responds as N2. External LEDs connected to
board designated as N2 in my setup.

<img src="hardware_setup.jpg"/>

## Birds eye view of Node 1 and Node 2 CAN_TX signals

Channel 0 of the logic analyzer is connected to the Node 1 CAN1_TX pin. Channel 1 is connected to 
Node 2 CAN1_TX pin. A CAN bus protocol analyzer is added for both channels  with bus clock
set to 500000. 

4 seconds of data @24Msamples/sec was logged by the analyzer. You can see all the decoded frames on Node 1 and Node 2 TX lines on the right window pane.

In the zoomed out view, you can see Node 1 transmitting the "LED control" data frame with standard message
ID 0x65D and a single byte data payload encoding the LED number. This is followed by a remote/request frame with standard message ID 0x651 requesting 2 bytes of data. Both frames are ACKed by Node N2.

After ~6.5milliseconds, Node 2 sends the response data frame for msg id 0x651 with
the two byte data payload being an incrementing counter for each response data frame. The response
data frame is ACKed by node N1.

<img src="birds_eye_view.jpg"/>

## Detail view of Node 1 data frame and remote frame transmissions.

<img src="node1_ledmsg_dataframe_and_remote_frame.jpg"/>


## Detail view of Node 2 response data frame

<img src="node2_remote_response.jpg"/>


