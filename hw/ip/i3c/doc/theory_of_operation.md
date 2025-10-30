
TODO: This is constructed in quick note form, for visibility at this early stage; correct the prose later.

# Clocking

It is intended that the I3C block be included in low power, lower performance products similar to the OpenTitan product 'Earl Grey', and it is therefore considered unacceptable to demand a high clock frequency.
At the time of writing, existing I3C blocks rely upon oversampling the received data by a large factor in order to overcome metastability issues on the input and still be able to respond promptly; the turnaround time is an important requirement in the I3C protocol, supporting immediate acceptance or rejection of the current data item.

The I3C bus supports signaling at up to 12.5MHz, with HDR-DDR supporting the transfer of data bits on both clock edges.
With this in mind, the I3C IP block expects a clock frequency of 50MHz for most of its internal logic, which naturally lends itself to setup and hold states on each of the two clock edges of a 12.5MHz I3C clock signal ('SCL').

A frequency of 50MHz does not, however, permit a sufficiently prompt response to controller signaling in the presence of the standard two-stage synchronizer approach to metastability.
Therefore, for this reason, in addition to concerns over the power consumption of the 'Always On' domain in a sleep state, the Target-side logic operates directly on the Controller-supplied clock signal 'SCL'.
As a result, once complete data units have been collected/transmitted from/to the I3C Controller, the target-side logic must transfer a data unit across clock domains to/from the main Target-side logic of the I3C IP block, operating on the 50MHz clock.

# Power considerations

A controller that has ceded the role of 'Active Controller' becomes a Secondary Controller on the I3C bus.
This is an I3C Target which has the capacity to control the bus, but it may now enter a low power state.
The I3C IP block has been designed such that it may be powered down entirely, leaving only a small amount of SCL-driven logic powered in the 'Always On' domain and capable of spotting a 'Target Reset' pattern.
Note that since I3C is a 'multi-drop' bus it is _not_ sufficient to respond to a state change at the pin or pins; rather this logic must detect a very specific pattern of signaling.

# Target-side logic

As noted above, the target-side logic, including the pattern detector that remains active in sleep states, must operate on the Controller-supplied SCL clock signal, which may be operating at up 12.5MHz average; according to the specification it may briefly appear to be 12.9MHz on account of rise/fall times, skew etc.
Since the I3C IP block supports HDR-DDR mode, it includes logic blocks that are triggered on the _negative_ edge of SCL, as well as the conventional _positive_-edge triggered logic.
To this end, the input clock signal is run through an inverter, as well as a parallel buffer, to limit skew.

An additional complication, that results from the use of I2C-compatible start (S)/stop (P) signaling in Single Data Rate (SDR) mode, is that the first I3C data line (SDA) must also be used as a clock signal to drive some of the Target-side logic.

