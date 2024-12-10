# ERRATA

## GPS Enable Polarity
The documented functionality of the GPS enable transistor (Q2) is inverted: The documentation wrongly states that setting the gate of Q2 high enables the GPS. The gate of Q2 must be set *low* to enable the GPS. 

This also affects the default setting of SB7. The GPS defaults to being off with the default configuration, unless SB2 is shorted and GPS_EN is controlled as mentioned above.

Affects: v2.0.0 - v2.0.2

Fixed in version: v2.0.3

