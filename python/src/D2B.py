# This is a python library that converts stim parameters in decimal
# to binary for direct use in SPI communication with the DAC. 
################################################################################
# DAC8832(Texas Instruments) info:
# Biopolar mode 
# (MSB)1111 1111 1111 1111 = +V_REF * (32,767 / 32,768)
# (MSB)1000 0000 0000 0001 = +V_REF * (1 / 32,768)
# (MSB)1000 0000 0000 0000 = 0V
# (MSB)0111 1111 1111 1111 = -V_REF * (1 / 32,768)
# (MSB)0000 0000 0000 0000 = -V_REF * (32,767 / 32,768)
################################################################################
# Output stage: modified Howland current pump around an LT1990-10 difference amplifier.
# R5 sits between the amplifier output and the electrode, and an OPA2990 buffers the
# load-side node back into the amplifier's REF pin. That positive feedback cancels the
# load out of the transfer function, so delivered current follows the DAC voltage alone:
#
#   I = GAIN * V_DAC / R5
#
# TRANSCONDUCTANCE_A_PER_V is measured rather than derived from GAIN and R5. Substituting
# the schematic values predicted exactly half the delivered current at every amplitude
# tested (in-line current probe, 1 kOhm load), so one constant in that product is not what
# the schematic says -- most likely the reference voltage or R5 itself. Measure both and
# this can go back to being derived. Re-measure if the analog front end changes.
################################################################################
import math
from numpy import clip

V_REF = 2.5  # DAC8832 reference voltage (V)
TRANSCONDUCTANCE_A_PER_V = 0.00146627565#0.00370#0.0147  # Delivered current per volt at the DAC output (A/V)

# DAC range ceiling, not a compliance limit. The amplifier has to develop
# I * (R5 + R_load) plus whatever the DC blocking capacitor ramps to over the pulse, and
# it saturates far below full scale, so amplitudes approaching this value clip.
MAX_CURRENT = math.floor(V_REF * TRANSCONDUCTANCE_A_PER_V * 1000000)  # Maximum current in uA
# Input value is in uA
def decimal_to_binary(decimal_value, bits=16):
    V_DAC = (decimal_value / 1000000.0) / TRANSCONDUCTANCE_A_PER_V  # Convert uA to V
    DAC = V_DAC / V_REF   # Normalize to V_REF
    #print(f"V_DAC: {V_DAC} V, DAC: {DAC}")
    DAC = clip(DAC, -1,1)  # Clamp to -1 to 1
    code = int((DAC + 1) * 32768)  # Convert to 16-bit code
    code = clip(code, 0, 65535)  # Ensure code is within 16-bit range
    #print(f"Code: {code} (Decimal), Hex: {hex(code)}")
    return code


def dac_code_from_ua(amplitude_ua):
    """Full 16-bit DAC code for a signed stimulation current in uA (cathode-leading = positive)."""
    return decimal_to_binary(amplitude_ua)


def mag_code_from_ua(amplitude_ua):
    """Magnitude above mid-scale (0x8000), as used by the on-chip controller."""
    code = dac_code_from_ua(amplitude_ua)
    if code >= 0x8000:
        return int(code - 0x8000)
    return int(0x8000 - code)


def ua_from_dac_code(code):
    """Signed delivered current in uA from a 16-bit DAC code."""
    dac = (int(code) / 32768.0) - 1.0
    return dac * V_REF * TRANSCONDUCTANCE_A_PER_V * 1e6


def ua_from_mag_code(mag, positive=True):
    """Current in uA from a magnitude code (always non-negative)."""
    code = 0x8000 + int(mag) if positive else 0x8000 - int(mag)
    return ua_from_dac_code(code)