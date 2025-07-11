def to_unsigned_16bit_regs_from_signed_32bit(value):
    if value < 0:
        value += (1 << 32)
    high = (value >> 16) & 0xFFFF
    low = value & 0xFFFF
    return [high, low]

def from_unsigned_16bit_regs_to_signed_32bit(high, low):
    raw = (high << 16) | low
    return raw - 0x100000000 if raw & 0x80000000 else raw

def to_unsigned_16bit_from_signed(value):
    return value & 0xFFFF

def from_unsigned_16bit_to_signed(value):
    return value - 0x10000 if value & 0x8000 else value
