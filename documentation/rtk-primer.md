# RTK primer (short)

Centimeter GNSS uses **carrier phase** measurements (~mm noise) instead of code pseudorange (~m noise). Each satellite-carrier has an unknown **integer ambiguity** until resolved.

## Why a base or network is required

Standalone receivers cannot remove meter-level errors (ionosphere, troposphere, orbits, clocks) well enough for cm accuracy. **RTK** uses corrections from a nearby reference (NTRIP caster or network):

- Errors correlated over tens of km **cancel** when differencing base and rover observations
- **Short baselines** (<10 km) make iono/tropo negligible in the double-difference model

## Float vs fixed

| State | Accuracy | Meaning |
|-------|----------|---------|
| No RTK | ~m | Standard GNSS |
| RTK float | ~10–50 cm | Ambiguities not fully resolved |
| RTK fixed | ~1–3 cm | Integer ambiguities resolved |

The ZED-F9P reports carrier solution in logs and telemetry (`rtk`: `none`, `float`, `FIXED`).

## NTRIP on this buoy

The ESP32 downloads **RTCM3** correction data from an NTRIP caster over LTE and injects it into the ZED-F9P UART. The OpenLog records the resulting high-rate PVT independently.

## Practical limits

- Clear sky view and low multipath matter more than marginal satellite count gains
- Convergence after RTCM starts can take several minutes
- Caster distance and VRS/network geometry affect fix reliability

For full theory (differencing hierarchy, ambiguity resolution), see graduate GNSS texts or the original course primer material.
