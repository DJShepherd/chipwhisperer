CONST_CLKGEN = 'clkgen'
CONST_EXTCLK = 'extclk'
CONST_DISABLED = 'disabled'

ADC_SRC_CLKGEN_X1 = 'clkgen_x1'
ADC_SRC_CLKGEN_X4 = 'clkgen_x4'
ADC_SRC_EXTCLK_X1 = 'extclk_x1'
ADC_SRC_EXTCLK_X4 = 'extclk_x4'
ADC_SRC_EXTCLK_DIR = 'extclk_dir'

CLKGEN_SRC_EXTCLK = CONST_EXTCLK
CLKGEN_SRC_SYSTEM = 'system'

HS2_SRC_DISABLE = CONST_DISABLED
HS2_SRC_CLKGEN = CONST_CLKGEN
HS2_SRC_GLITCH = 'glitch'

GPIO_NAME_TIO1 = 'tio1'
GPIO_NAME_TIO2 = 'tio2'
GPIO_NAME_TIO3 = 'tio3'
GPIO_NAME_TIO4 = 'tio4'
GPIO_NAME_PDIC = 'pdic'
GPIO_NAME_PDID = 'pdid'
GPIO_NAME_NRST = 'nrst'

TIO_NUM1 = 0
TIO_NUM2 = 1
TIO_NUM3 = 2
TIO_NUM4 = 3

TIO_MODE_DISABLE = 'gpio_disabled'
TIO_MODE_SERIAL_RX = 'serial_rx'
TIO_MODE_SERIAL_TX = 'serial_tx'
TIO_MODE_SERIAL_IO = 'serial_tx_rx'
TIO_MODE_HIGHZ = None
TIO_MODE_LOW = False
TIO_MODE_HIGH = True

# Also for NRST
PDI_MODE_HIGHZ = None 
PDI_MODE_HIGH = True
PDI_MODE_LOW = False

GLITCH_CLKSRC_TARGET = 'target'
GLITCH_CLKSRC_CLKGEN = CONST_CLKGEN
GLITCH_CLKSRC_PLL = 'pll'

VCC_GLITCH_DISABLED = CONST_DISABLED
VCC_GLITCHT_LP = 'lp'
VCC_GLITCHT_HP = 'hp'
VCC_GLITCHT_BOTH = 'both'

VCC_EGLITCHT_HP = 1 << 0
VCC_EGLITCHT_LP = 1 << 1
VCC_EGLITCHT_BOTH = VCC_EGLITCHT_LP | VCC_EGLITCHT_HP

ARM_TIMING_NO_GLITCH = 'no_glitch'
ARM_TIMING_PRE_SCOPE = 'before_scope'
ARM_TIMING_POST_SCOPE = 'after_scope'

GLITCH_OUTPUT_CLOCK = 'clock_only'
GLITCH_OUTPUT_GLITCH = 'glitch_only' # (VCC glitching)
GLITCH_OUTPUT_CLK_OR = 'clock_or' # (clock glitching)
GLITCH_OUTPUT_CLK_XOR = 'clock_xor' # (clock glitching)
GLITCH_OUTPUT_ENABLE = 'enable_only' # (VCC glitching)

GLITCH_TRIGGER_CONTINUOUS = 'continuous'
GLITCH_TRIGGER_MANUAL = 'manual'
GLITCH_TRIGGER_EXT1 = 'ext_single'
GLITCH_TRIGGER_EXT_CONT = 'ext_continuous'

FREQCTR_SRC_CLKGEN = CONST_CLKGEN
FREQCTR_SRC_EXT = CONST_EXTCLK

BASIC_MODE_LOW = 'low' # trigger = 0
BASIC_MODE_HIGH = 'high' # trigger = 1
BASIC_MODE_RISE = 'rising_edge' # trigger = 0=>1
BASIC_MODE_FALL = 'falling_edge' # trigger = 1=>0

TRIG_LOGIC_AND = 'AND'
TRIG_LOGIC_OR = 'OR'
TRIG_LOGIC_NAND = 'NAND'
#TRIG_LOGIC_XOR = 'XOR'

TRIG_SRC_TIO1 = GPIO_NAME_TIO1
TRIG_SRC_TIO2 = GPIO_NAME_TIO2
TRIG_SRC_TIO3 = GPIO_NAME_TIO3
TRIG_SRC_TIO4 = GPIO_NAME_TIO4
TRIG_SRC_NRST = GPIO_NAME_NRST
TRIG_SRC_SMA = 'sma' # CW1200 only

VCC_GLITCHT_STRS = {
    VCC_GLITCH_DISABLED: 0,
    VCC_GLITCHT_LP: VCC_EGLITCHT_LP,
    VCC_GLITCHT_HP: VCC_EGLITCHT_HP,
    VCC_GLITCHT_BOTH: VCC_EGLITCHT_BOTH,
}

def glitcht_str_mask(glitcht):
    """Converts a valid VCC_GLITCH_* string to a bitmask.

    Return:
        A VCC_EGLITCH_* bitmask representing the target VCC glitch mode.
    """
    eglitch = VCC_GLITCHT_STRS.get(glitcht, -1)
    if eglitch >= 0:
        return eglitch
    raise ValueError("Invalid glitch transistor {} must be 'both', 'hp', 'lp', or 'disabled'".format(glitcht))

def glitcht_var_mask(glitcht):
    """Ensures the input glitch value is a VCC_EGLITCH_* bitmask.

    Return:
        A VCC_EGLITCH_* bitmask representing the target VCC glitch mode.
    """
    if isinstance(glitcht, int):
        return glitcht
    return glitcht_str_mask(glitcht)