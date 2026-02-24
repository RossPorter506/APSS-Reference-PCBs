# ERRATA

## Battery connector silkscreen outline incorrect
The silkscreen outline for the battery connector is flipped 180 degrees from the correct orientation. The battery connector is keyed to prevent plugging the battery in reverse. In the correct orientation, the connector should be soldered such that when a battery is plugged in the red (positive) cable is next to the silkscreen label '+', and the black (negative) cable should be next to the silkscreen text '-ve'.

Affects: v2.0.0 - v2.0.5

Fixed in version: N/A 

## 'Remove/Attach Before Flight' header wrongly documented
The header that controls whether the payload switches on was originally an 'attach before flight' connector. This was changed to a 'remove before flight' header in v2.0.3, but the documentation was not updated until v2.0.6. The header behaves as an 'attach before flight' header from v2.0.0 to v2.0.2, and as a 'remove before flight' header from v2.0.3 onwards.

Affects: v2.0.3 - v2.0.5

Fixed in version: v2.0.6

## 'Disable while charging' feature does not work
The 'disable while charging' feature doesn't work correctly on the v1 PSU board. The toggle switch for this should be left in the 'Off / SW Control' position, unless the following fix is applied: Resolder Q1 such that pins 2 and 3 are swapped.

Affects: v1.*

Fixed in version: v2.0.0
