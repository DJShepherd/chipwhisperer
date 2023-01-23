#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013-2021, NewAE Technology Inc
# All rights reserved.
#
# Authors: Colin O'Flynn
#
# Find this and more at newae.com - this file is part of the chipwhisperer
# project, http://www.assembla.com/spaces/chipwhisperer
#
#    This file is part of chipwhisperer.
#
#    chipwhisperer is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    chipwhisperer is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with chipwhisperer.  If not, see <http://www.gnu.org/licenses/>.
#=================================================
import time
from . import ChipWhispererGlitch
from ....common.utils import util, consts

from ....logging import *

CODE_READ = 0x80
CODE_WRITE = 0xC0
ADDR_ADC_TRIGGER_LEVEL = 21
ADDR_DATA = 33
ADDR_LEN = 34
ADDR_BAUD = 35
ADDR_AUX_IO = 37
ADDR_EXTCLK = 38
ADDR_TRIGSRC = 39
ADDR_TRIGMOD = 40
ADDR_I2CSTATUS = 47
ADDR_I2CDATA = 48
ADDR_IOROUTE = 55
ADDR_IOREAD = 59
ADDR_EDGE_TRIGGER = 113

IOROUTE_HIGHZ = 0
IOROUTE_STX = 0b00000001
IOROUTE_SRX = 0b00000010
IOROUTE_USIO = 0b00000100
IOROUTE_USII = 0b00001000
IOROUTE_USINOUT = 0b00011000
IOROUTE_STXRX = 0b00100010
IOROUTE_GPIO_STATE = 0b01000000
IOROUTE_GPIO_MODE = 0b10000000
IOROUTE_GPIO_LOW = IOROUTE_GPIO_MODE
IOROUTE_GPIO_HIGH = IOROUTE_GPIO_MODE | IOROUTE_GPIO_STATE

GPIO_NUM1 = 0
GPIO_NUM2 = 1
GPIO_NUM3 = 2
GPIO_NUM4 = 3
GPIO_NRST = 100
GPIO_PDID = 101
GPIO_PDIC = 102

AVR_ISP_MODE = 0x1
TARGET_PWR_STATE = 0x2
TARGET_PWR_SLEW = 0x4

GLITCH_OUT_HP = 0x1
GLITCH_OUT_LP = 0x2
GLITCH_OUT_BOTH = 0x3
GLITCH_OUT_CLR = ~GLITCH_OUT_BOTH

EXTCLK_SRC_MASK = 0x7

EXTCLK_OUT_DISABLE = 0
EXTCLK_OUT_CLKGEN = 2
EXTCLK_OUT_GLITCH = 3
EXTCLK_OUT_SHIFT = 5
EXTCLK_OUT_MASK = 3 << EXTCLK_OUT_SHIFT

def invert_dict(d):
    return { d[k]: k for k in d }

def invupd_dict(d, extra):
    extra.update(invert_dict(d))
    return extra

TIO_VALID = [
    [
        IOROUTE_STX,
        IOROUTE_SRX,
        IOROUTE_USIO,
        IOROUTE_USII,
        IOROUTE_GPIO_LOW,
        IOROUTE_GPIO_HIGH,
        IOROUTE_HIGHZ,
    ], [
        IOROUTE_STX,
        IOROUTE_SRX,
        IOROUTE_USIO,
        IOROUTE_USII,
        IOROUTE_GPIO_LOW,
        IOROUTE_GPIO_HIGH,
        IOROUTE_HIGHZ,
    ], [
        IOROUTE_STX,
        IOROUTE_SRX,
        IOROUTE_STXRX,
        IOROUTE_USIO,
        IOROUTE_USII,
        IOROUTE_USINOUT,
        IOROUTE_GPIO_LOW,
        IOROUTE_GPIO_HIGH,
        IOROUTE_HIGHZ,
    ], [
        IOROUTE_STX,
        IOROUTE_GPIO_LOW,
        IOROUTE_GPIO_HIGH,
        IOROUTE_HIGHZ,
    ]
]

IOROUTE_TO_TIO_MODE = {
    IOROUTE_STX: consts.TIO_MODE_SERIAL_TX,
    IOROUTE_SRX: consts.TIO_MODE_SERIAL_RX,
    IOROUTE_STXRX: consts.TIO_MODE_SERIAL_IO,
    IOROUTE_GPIO_MODE: 'gpio_disabled',
    IOROUTE_GPIO_LOW: 'gpio_low',
    IOROUTE_GPIO_HIGH: 'gpio_high',
    IOROUTE_HIGHZ: 'high_z',
}

TIO_MODE_TO_IOROUTE = invupd_dict(IOROUTE_TO_TIO_MODE, {
    False: IOROUTE_GPIO_LOW,
    True: IOROUTE_GPIO_HIGH,
    None: IOROUTE_HIGHZ,
})

GPIO_STATE_TO_MODE = {
    None: 'high_z',
    False: 'low',
    True: 'high',
}

GPIO_MODE_TO_STATE = invupd_dict(GPIO_STATE_TO_MODE, {
    True: True,
    False: False,
    None: None,
    consts.CONST_DISABLED: None,
    'default': None,
})

HS2_DEST_TO_MODE = {
    EXTCLK_OUT_DISABLE: consts.HS2_SRC_DISABLE,
    EXTCLK_OUT_CLKGEN: consts.HS2_SRC_CLKGEN,
    EXTCLK_OUT_GLITCH: consts.HS2_SRC_GLITCH,
}

HS2_MODE_TO_DEST = invupd_dict(HS2_DEST_TO_MODE, {
    None: consts.HS2_SRC_DISABLE
})

class GPIOSettings(util.DisableNewAttr):

    def __init__(self, cwextra):
        super().__init__()
        self.cwe = cwextra

        self._is_husky = False

        self.disable_newattr()

    def _dict_repr(self):
        rtn = {}
        rtn['tio1'] = self.tio1
        rtn['tio2'] = self.tio2
        rtn['tio3'] = self.tio3
        rtn['tio4'] = self.tio4

        rtn['pdid'] = self.pdid
        rtn['pdic'] = self.pdic
        rtn['nrst'] = self.nrst

        rtn['glitch_hp'] = self.glitch_hp
        rtn['glitch_lp'] = self.glitch_lp

        rtn['extclk_src'] = self.extclk_src
        rtn['hs2'] = self.hs2

        rtn['target_pwr'] = self.target_pwr

        rtn['tio_states'] = self.tio_states

        rtn['cdc_settings'] = self.cdc_settings

        if self._is_husky:
            rtn['aux_io_mcx'] = self.aux_io_mcx
            rtn['glitch_trig_mcx'] = self.glitch_trig_mcx

        return rtn

    def __repr__(self):
        return util.dict_to_str(self._dict_repr())

    def __str__(self):
        return self.__repr__()

    def read_tio_states(self):
        """Gets all the current TIO pin state values.

        Returns:
            A tuple of the TIO pin states in integer form.
        """
        bitmask = self.cwe.read_io_pins()
        return (
            bitmask & 1,
            (bitmask >> 1) & 1,
            (bitmask >> 2) & 1,
            (bitmask >> 3) & 1
        )

    @property
    def tio_states(self):
        """
        Reads the logic level of the TIO pins (1-4) and
        returns them as a tuple of the logic levels.

        .. warning:: ChipWhisperer firmware before release 5.2.1 does not support
            reading the TIO pins!

        :getter: Read TIO states

        Returns:
            A tuple of 1's and 0's representing the logic levels
            of each TIO pin

        .. versionadded:: 5.3
            Add documented interface for the old method of reading TIO pins
        """
        return self.read_tio_states()

    def wait_for_tio(self, pin_set, timeout=None):
        """Polls TIO states until it matches a specified dict of pin:value.

        Returns:
            True if all pins match, else False for timeout
        """
        if not timeout is None:
            timeout += time.time()
        while True:
            tio = self.tio_states
            match = True
            for pin in pin_set:
                expect = bool(pin_set[pin])
                actual = bool(tio[pin])
                match = expect == actual
                if not match:
                    break
            if match:
                return True
            if (not timeout is None) and (time.time() >= timeout):
                return False

    @property
    def cdc_settings(self):
        """Check or set whether USART settings can be changed via the USB CDC connection

        i.e. whether you can change USART settings (baud rate, 8n1) via a serial client like PuTTY

        :getter: An array of length four for four possible CDC serial ports (though only one is used)

        :setter: Can set either via an integer (which sets both ports) or an array of length 4 (which sets each port)

        Returns None if using firmware before the CDC port was added
        """
        rawver = self.cwe.oa.serial.readFwVersion()
        ver = '{}.{}'.format(rawver[0], rawver[1])
        if ver < '0.30':
            return None
        return self.cwe.oa.serial.get_cdc_settings()

    @cdc_settings.setter
    def cdc_settings(self, port):
        rawver = self.cwe.oa.serial.readFwVersion()
        ver = '{}.{}'.format(rawver[0], rawver[1])
        if ver < '0.30':
            return None
        return self.cwe.oa.serial.set_cdc_settings(port)

    @property
    def aux_io_mcx(self):
        """Set the function of the AUX I/O MCX on Husky.

        Options:

        * "high_z": input: to use as a trigger (scope.trigger.triggers = 'aux') or clock (scope.clock.clkgen_src = 'extclk_aux_io').
        * "hs2": output: provide the same clock that's on HS2.
        """
        if not self._is_husky:
            raise ValueError("For CW-Husky only.")
        data = self.cwe.oa.sendMessage(CODE_READ, ADDR_AUX_IO, Validate=False, maxResp=1)[0]
        if data & 0x01:
            return "hs2"
        else:
            return "high_z"

    @aux_io_mcx.setter
    def aux_io_mcx(self, state):
        if not self._is_husky:
            raise ValueError("For CW-Husky only.")
        data = self.cwe.oa.sendMessage(CODE_READ, ADDR_AUX_IO, Validate=False, maxResp=1)[0]
        if state == 'high_z':
            data &= 0xfe
        elif state == 'hs2':
            data |= 0x01
        else:
            raise ValueError("Options: high_z, hs2")
        return self.cwe.oa.sendMessage(CODE_WRITE, ADDR_AUX_IO, [data])

    @property
    def glitch_trig_mcx(self):
        """Set the function of the Trigger/Glitch Out MCX on Husky.
        Options:

        * "glitch": glitch output (clock or voltage glitch signal, as defined by scope.glitch settings)
        * "trigger": internal trigger signal (as defined by scope.trigger)
        """
        if not self._is_husky:
            raise ValueError("For CW-Husky only.")
        data = self.cwe.oa.sendMessage(CODE_READ, ADDR_AUX_IO, Validate=False, maxResp=1)[0]
        if data & 0x02:
            return "glitch"
        else:
            return "trigger"

    @glitch_trig_mcx.setter
    def glitch_trig_mcx(self, state):
        if not self._is_husky:
            raise ValueError("For CW-Husky only.")
        data = self.cwe.oa.sendMessage(CODE_READ, ADDR_AUX_IO, Validate=False, maxResp=1)[0]
        if state == 'trigger':
            data &= 0xfd
        elif state == 'glitch':
            data |= 0x02
        else:
            raise ValueError("Options: glitch, trig")
        self.cwe.oa.sendMessage(CODE_WRITE, ADDR_AUX_IO, [data])

    @property
    def tio1(self):
        """The function of the Target IO1 pin.

        TIO1 can be used for the following functions:
         * "serial_rx": UART input
         * "serial_tx": UART output
         * "high_z" / None: High impedance input
         * "gpio_low" / False: Driven output: logic 0
         * "gpio_high" / True: Driven output: logic 1
         * "gpio_disabled": Driven output: no effect

        Default value is "serial_rx".

        :Getter:  Return one of the above strings. This shows how ChipWhisperer is 
                driving this pin; it does not show its actual logic level. Use
                scope.io.tio_states to see the actual logic level.

        :Setter: Set the Target IO1 mode.

        Raises:
           ValueError: if new value is not one of the above modes

        """
        return self._get_tio_mode(GPIO_NUM1)

    @tio1.setter
    def tio1(self, state):
        self._set_tio_mode(GPIO_NUM1, state)

    @property
    def tio2(self):
        """The function of the Target IO2 pin.

        TIO2 can be used for the following functions:
         * "serial_rx": UART input
         * "serial_tx": UART output
         * "high_z" / None: High impedance input
         * "gpio_low" / False: Driven output: logic 0
         * "gpio_high" / True: Driven output: logic 1
         * "gpio_disabled": Driven output: no effect

        Default value is "serial_tx".

        :Getter:  Return one of the above strings. This shows how ChipWhisperer is 
                driving this pin; it does not show its actual logic level. Use
                scope.io.tio_states to see the actual logic level.

        :Setter: Set the Target IO2 mode.

        Raises:
           ValueError: if new value is not one of the above modes
        """
        return self._get_tio_mode(GPIO_NUM2)

    @tio2.setter
    def tio2(self, state):
        self._set_tio_mode(GPIO_NUM2, state)

    @property
    def tio3(self):
        """The function of the Target IO3 pin.

        TIO3 can be used for the following functions:
         * "serial_rx": UART input
         * "serial_tx": UART output
         * "serial_tx_rx": UART 1-wire I/O (for smartcards)
         * "high_z" / None: High impedance input
         * "gpio_low" / False: Driven output: logic 0
         * "gpio_high" / True: Driven output: logic 1
         * "gpio_disabled": Driven output: no effect

        Default value is "high_z".

        :Getter:  Return one of the above strings. This shows how ChipWhisperer is 
                driving this pin; it does not show its actual logic level. Use
                scope.io.tio_states to see the actual logic level.

        :Setter: Set the Target IO3 mode.

        Raises:
           ValueError: if new value is not one of the above modes
        """
        return self._get_tio_mode(GPIO_NUM3)

    @tio3.setter
    def tio3(self, state):
        self._set_tio_mode(GPIO_NUM3, state)

    @property
    def tio4(self):
        """The function of the Target IO4 pin.

        TIO4 can be used for the following functions:
         * "serial_tx": UART output
         * "high_z" / None: High impedance input
         * "gpio_low" / False: Driven output: logic 0
         * "gpio_high" / True: Driven output: logic 1
         * "gpio_disabled": Driven output: no effect

        Default value is "high_z". Typically, this pin is used as a trigger
        input.

        :Getter:  Return one of the above strings. This shows how ChipWhisperer is 
                driving this pin; it does not show its actual logic level. Use
                scope.io.tio_states to see the actual logic level.

        :Setter: Set the Target IO4 mode

        Raises:
           ValueError: if new value is not one of the above modes
        """
        return self._get_tio_mode(GPIO_NUM4)

    @tio4.setter
    def tio4(self, state):
        self._set_tio_mode(GPIO_NUM4, state)

    def _get_tio_mode(self, pinnum):
        """Internal function to read the current mode of a TIO pin.

        Return:
            The API string that represents the TIO GPIO mode.
        """
        route = self.cwe.get_tio_mode(pinnum)
        mode = IOROUTE_TO_TIO_MODE[route]
        if mode is None:
            raise IOError('Invalid IO mode returned by FPGA: 0x%x', route)
        return mode

    def _set_tio_mode(self, pinnum, mode):
        """Internal function to set the current mode of a TIO pin.

        Return:
            The updated IOROUTE state.
        """
        route = TIO_MODE_TO_IOROUTE.get(mode, -1)
        if route == -1:
            raise ValueError('%s is not a valid TIO mode!' % mode)
        if not route in TIO_VALID[pinnum]:
            raise ValueError('%s is not valid for GPIO[%d]' % (mode, pinnum))
        return self.cwe.set_tio_mode(pinnum, route)

    def read_tio_pin(self, tio_num):
        """Reads the current value of a specified consts.TIO_NUM* pin.

        Return:
            True if the pin is high, else False if the pin is low.
        """
        # consts.TIO_NUM* should match to GPIO_NUM*
        return self.cwe.read_io_pin(tio_num)

    def read_tio1(self):
        """Reads the current value of TIO1.

        Return:
            True if the pin is high, else False if the pin is low.
        """
        return self.read_tio_pin(consts.TIO_NUM1)

    def read_tio2(self):
        """Reads the current value of TIO2.

        Return:
            True if the pin is high, else False if the pin is low.
        """
        return self.read_tio_pin(consts.TIO_NUM2)

    def read_tio3(self):
        """Reads the current value of TIO3.

        Return:
            True if the pin is high, else False if the pin is low.
        """
        return self.read_tio_pin(consts.TIO_NUM3)

    def read_tio4(self):
        """Reads the current value of TIO4.

        Return:
            True if the pin is high, else False if the pin is low.
        """
        return self.read_tio_pin(consts.TIO_NUM4)

    @property
    def pdic(self):
        """The function of the PDIC pin output pin.

        This is a GPIO pin. The following values are allowed:
         * "high" / True: logic 1
         * "low" / False: logic 0
         * "disabled" / "default" / "high_z" / None: undriven

        :Getter:  Return one of "high", "low", or "high_z". This shows how ChipWhisperer
                is driving this pin; it does not show its actual logic level.

        :Setter: Set the pin's state

        Raises:
        ValueError: if new state not listed above
        """
        return self._get_ctlio(GPIO_PDIC)

    @pdic.setter
    def pdic(self, state):
        self._set_ctlio(GPIO_PDIC, state)

    @property
    def pdid(self):
        """The state of the PDID pin.

        See pdic for more information."""
        return self._get_ctlio(GPIO_PDID)

    @pdid.setter
    def pdid(self, state):
        self._set_ctlio(GPIO_PDID, state)

    @property
    def nrst(self):
        """The state of the NRST pin.

        See pdic for more information."""
        return self._get_ctlio(GPIO_NRST)

    @nrst.setter
    def nrst(self, state):
        self._set_ctlio(GPIO_NRST, state)

    def _get_ctlio(self, pinnum):
        """Gets the current IO mode for CTL pins.

        Return:
            'high_z' if not driven, 'high' if driven high, else 'low' if driven low.
        """
        state = self.cwe.get_ctlio_state(pinnum)
        return GPIO_STATE_TO_MODE[state]

    def _set_ctlio(self, pinnum, level):
        """Sets the current IO mode for CTL pins.

        Return:
            The updated IOROUTE state.
        """
        state = GPIO_MODE_TO_STATE.get(level, -1)
        if state == -1:
            raise ValueError("Can't set GPIO %d to level %s (expected 'high'/True, 'low'/False, or 'disabled'/'default'/'high_z'/None)" % (pinnum, level), level)
        return self.cwe.set_ctlio_state(pinnum, state)

    @property
    def extclk_src(self):
        """The clock signal being used as input for EXTCLK.

        Currently, this can only be HS1, which is the clock from the target.
        As such, this value is read-only.
        """
        return 'hs1'

    @property
    def hs2(self):
        """The clock signal routed to the HS2 high speed output pin.

        Allowed clock signals are:
         * "clkgen": The output from the CLKGEN module
         * "glitch": The output from the glitch module
         * "disabled" / None: No clock; output driven low

        :Getter:  Return one of "clkgen", "glitch", or "disabled"

        :Setter: Set the clock to be output on HS2.

        Raises:
        ValueError: if new value not listed above
        """
        dest = self.cwe.get_clkout()
        mode = HS2_DEST_TO_MODE.get(dest, -1)
        if mode == -1:
            raise IOError("Hardware returned unknown HS2 mode: %02x" % mode)
        return mode

    @hs2.setter
    def hs2(self, mode):
        dest = HS2_MODE_TO_DEST.get(mode, -1)
        if dest == -1:
            raise ValueError("Unknown mode for HS2 pin: '%s'. Valid modes: %s" % (mode, HS2_MODE_TO_DEST.keys()), mode)
        self.cwe.set_clkout(dest)

    @property
    def target_pwr(self):
        """Whether the target board is powered by the ChipWhisperer.

        If True, the ChipWhisperer is currently supplying power to the target
        board; if False, it is not. This setting can be used to cycle power to
        the target or to help program it.

        If the target board is powered through an external supply, this setting
        may have no effect.

        :Getter:  Return the current power state of the target (True or False)

        :Setter: Turn the target power on or off.
        """
        return self.cwe.get_target_pwr_state()

    @target_pwr.setter
    def target_pwr(self, power):
        self.cwe.set_target_pwr_state(power)

    @property
    def glitch_hp(self):
        """Whether the high-power crowbar MOSFET is enabled.

        The glitch output is an SMA-connected output line that is normally
        connected to a target's power rails. If this setting is enabled, a
        high-powered MOSFET shorts the power-rail to ground when the glitch
        module's output is active.

        .. warning:: Use with caution - ensure that the glitch module is properly
            configured before enabling this setting, as it is possible to
            permanently damage hardware with this output.

        :Getter:  Return True if enabled or False if disabled

        :Setter: Turn the high-power MOSFET on or off
        """
        return self.cwe.vglitch_get_hp()

    @glitch_hp.setter
    def glitch_hp(self, active):
        self.cwe.vglitch_enable(GLITCH_OUT_HP, active)

    @property
    def glitch_lp(self):
        """Whether the low-power crowbar MOSFET is enabled.

        This is the low-power version of glitch_hp - see that documentation
        for more details.

        .. warning:: Use with caution - ensure that the glitch module is properly
            configured before enabling this setting, as it is possible to
            permanently damage hardware with this output.
        """
        return self.cwe.vglitch_get_lp()

    @glitch_lp.setter
    def glitch_lp(self, active):
        self.cwe.vglitch_enable(GLITCH_OUT_LP, active)

    def vglitch_set_mode(self, eglitcht):
        """Sets the current mode for the VCC glitch mosfets.
        """
        # consts.VCC_EGLITCHT_* should map to GLITCH_OUT_*
        self.cwe.vglitch_set_mode(eglitcht)

    def vglitch_mode_hp(self):
        """Sets the current VCC glitch mode to HP only.
        """
        self.vglitch_set_mode(consts.VCC_EGLITCHT_HP)

    def vglitch_mode_lp(self):
        """Sets the current VCC glitch mode to LP only.
        """
        self.vglitch_set_mode(consts.VCC_EGLITCHT_LP)

    def vglitch_mode_both(self):
        """Sets the current VCC glitch mode to both mosfets.
        """
        self.vglitch_set_mode(consts.VCC_EGLITCHT_BOTH)

    def vglitch_vset_mode(self, glitcht):
        """Sets the current VCC glitch mode 
        """
        self.vglitch_set_mode(consts.glitcht_var_mask(glitcht))

    def vglitch_reset(self):
        """Disables and reenables the glitch mosfets that were previously enabled.
        """
        self.cwe.vglitch_reset()

    def vglitch_disable(self):
        """Disables both glitch mosfets.
        """
        self.cwe.vglitch_clear()

    def reset_target(self, initial_state=1, reset_state=0, reset_delay=0.01, postreset_delay=0.01):
        raise NotImplementedError()

    # .. todo:: implement SCK/MOSI/MISO/CS?

    def sck(self):
        raise NotImplementedError()

    def mosi(self):
        raise NotImplementedError()

    def miso(self):
        raise NotImplementedError()

    def cs(self):
        raise NotImplementedError()

class TriggerSettings(util.DisableNewAttr):
    def __init__(self, cwextra):
        super().__init__()
        self.cwe = cwextra

        self.supported_tpins = {
            consts.TRIG_SRC_TIO1: self.cwe.PIN_RTIO1,
            consts.TRIG_SRC_TIO2: self.cwe.PIN_RTIO2,
            consts.TRIG_SRC_TIO3: self.cwe.PIN_RTIO3,
            consts.TRIG_SRC_TIO4: self.cwe.PIN_RTIO4,
            consts.TRIG_SRC_NRST: self.cwe.PIN_TNRST,
        }

        self.last_module = "basic"
        if self.cwe.hasAux:
            self.supported_tpins['sma'] = self.cwe.PIN_FPA
            self.supported_tpins['aux'] = self.cwe.PIN_FPA # alias for Husky since it's labeled 'Aux' on the sticker

        if self.cwe.hasUserio:
            self.supported_tpins['userio_d0'] = self.cwe.PIN_USERIO0
            self.supported_tpins['userio_d1'] = self.cwe.PIN_USERIO1
            self.supported_tpins['userio_d2'] = self.cwe.PIN_USERIO2
            self.supported_tpins['userio_d3'] = self.cwe.PIN_USERIO3
            self.supported_tpins['userio_d4'] = self.cwe.PIN_USERIO4
            self.supported_tpins['userio_d5'] = self.cwe.PIN_USERIO5
            self.supported_tpins['userio_d6'] = self.cwe.PIN_USERIO6
            self.supported_tpins['userio_d7'] = self.cwe.PIN_USERIO7


        self._is_husky = False

        self.disable_newattr()

    def _dict_repr(self):
        rtn = {}
        rtn['triggers'] = self.triggers
        rtn['module'] = self.module

        return rtn

    def __repr__(self):
        return util.dict_to_str(self._dict_repr())

    def __str__(self):
        return self.__repr__()

    def set_trigger_set(self, op, *src):
        triggers = None
        if len(src) > 0:
            triggers = src[0]
            for i in range(1, len(src)):
                triggers = '{} {} {}'.format(triggers, op, src[i])
        self.triggers = triggers

    @property
    def triggers(self):
        """The logical input into the trigger module.

        The trigger module uses some combination of the scope's I/O pins to
        produce a single value, which it uses for edge/level detection or UART
        triggers. This trigger output can combine 5 pins using one of 3
        different boolean operations. N/A for 'trace' trigger module (see
        scope.trace.trace_mode for how to connect trace pins.)

        Pins:
         * tio1-4: Target I/O pins 1-4. Note that these pins can be in any mode.
         * nRST: Target I/O pin nRST. Note that these pins can be in any mode.
         * sma: An auxiliary SMA input, if available (only on CW1200)

        Boolean operations:
         * OR: True if any inputs are True; False if none are
         * AND: True if all inputs are True; False if any are not
         * NAND: False if all inputs are True; True if any are not

        Note that only one boolean operation can be used over all input pins.

        Examples of acceptable trigger inputs:
         * "tio1"
         * "tio3 OR tio4"
         * "tio1 NAND tio2 NAND sma"
         * "nrst"

        Examples of unallowed trigger inputs:
         * "tio1 tio2"
         * "tio1 AND tio2 OR tio3"
         * "tio1 OR tio1"
         * "tio1 XOR tio2"
         * "serial-tx"

        :Getter:  Return a string describing the trigger mode (see examples)

        :Setter: Set the trigger mode using a string like the ones above

        Raises:
           ValueError: if string cannot be converted to a legal mode
        """
        #Get pin logic + combo mode
        if self.module == 'trace':
            return 'N/A (use scope.trace.trace_mode)'
        else:
            pins, mode = self.cwe.getPins()

            tstring = []
            if mode == self.cwe.MODE_OR: modes = "OR"
            elif mode ==  self.cwe.MODE_AND: modes = "AND"
            elif mode == self.cwe.MODE_NAND: modes = "NAND"
            else: raise IOError("Unknown mode reported by hardware: %02x" % mode)

            if pins & self.cwe.PIN_RTIO1:
                tstring.append("tio1")
                tstring.append(modes)

            if pins & self.cwe.PIN_RTIO2:
                tstring.append("tio2")
                tstring.append(modes)

            if pins & self.cwe.PIN_RTIO3:
                tstring.append("tio3")
                tstring.append(modes)

            if pins & self.cwe.PIN_RTIO4:
                tstring.append("tio4")
                tstring.append(modes)

            if pins & self.cwe.PIN_FPA:
                tstring.append("sma")
                tstring.append(modes)

            if pins & self.cwe.PIN_TNRST:
                tstring.append("nrst")
                tstring.append(modes)

            if pins & self.cwe.PIN_USERIO0:
                tstring.append("userio_d0")
                tstring.append(modes)

            if pins & self.cwe.PIN_USERIO1:
                tstring.append("userio_d1")
                tstring.append(modes)

            if pins & self.cwe.PIN_USERIO2:
                tstring.append("userio_d2")
                tstring.append(modes)

            if pins & self.cwe.PIN_USERIO3:
                tstring.append("userio_d3")
                tstring.append(modes)

            if pins & self.cwe.PIN_USERIO4:
                tstring.append("userio_d4")
                tstring.append(modes)

            if pins & self.cwe.PIN_USERIO5:
                tstring.append("userio_d5")
                tstring.append(modes)

            if pins & self.cwe.PIN_USERIO6:
                tstring.append("userio_d6")
                tstring.append(modes)

            if pins & self.cwe.PIN_USERIO7:
                tstring.append("userio_d7")
                tstring.append(modes)

            #Remove last useless combination mode
            if len(tstring) > 1:
                tstring = tstring[0:-1]

            #Return a string indicating trigger mode
            return " ".join(tstring)

    @triggers.setter
    def triggers(self, s):

        if self.module == 'trace':
            scope_logger.error('N/A for trace module. See scope.trace.trace_mode.')
        else:
            s = s.lower()

            #Split up string
            triggers = s.split()

            #Check there is only one type of combination mode
            triggerset = set(triggers)
            numcombined = int('and' in triggerset) + int('or' in triggerset) + int('nand' in triggerset)
            if numcombined > 1:
                raise ValueError("Combining multiple triggers requires same logic between each combination", s)

            if numcombined == 0 and len(triggers) > 1:
                raise ValueError("Detected more than 1 trigger pin specified, but no combination logic.", s)

            enablelogic = 0

            #Figure out enabled triggers
            for t in list(self.supported_tpins.keys()):
                if t in triggers:
                    if triggers.count(t) != 1:
                        raise ValueError("Pin '%s' appears %d times, only 1 apperance supported" % (t, triggers.count(t)), s)
                    enablelogic |= self.supported_tpins[t]

            #Find mode
            if ('or' in triggerset) or (len(triggerset) == 1):
                mode = self.cwe.MODE_OR
                modes = "or"
            elif 'and' in triggerset:
                mode = self.cwe.MODE_AND
                modes = "and"
            elif 'nand' in triggerset:
                mode = self.cwe.MODE_NAND
                modes = "nand"

            #Check mode operations in correct order, no unknown things
            expect_tpin = True
            for t in triggers:
                if expect_tpin:
                    if t not in list(self.supported_tpins.keys()):
                        raise ValueError("Error processing string at expected pin '%s'. Valid pins: %s"%(t, list(self.supported_tpins.keys())), s)
                else:
                    if t != modes:
                        raise ValueError("Unexpected combination mode '%s'. Expected %s."%(t, modes), s)
                expect_tpin ^= True

            #Finally set this thing, guess we're looking HOT
            self.cwe.setPins(enablelogic, mode)

    @property
    def module(self):
        """The trigger module in use.

        The trigger modules available depend on the hardware. On the CWLite,
        only the basic trigger module can be used; on the CW1200, the serial
        data and SAD triggers are available too.

        Available trigger modules:
         * 'basic': Trigger on a logic level or edge

        :Getter: Returns 'basic'
        """
        return "basic"

class ProTrigger(TriggerSettings):
    def _dict_repr(self):
        rtn = super()._dict_repr()
        rtn['module'] = self.module
        rtn['aux_out'] = self.aux_out
        return rtn

    @property
    def module(self):
        """The trigger module in use.

        The trigger modules available depend on the hardware. On the CWLite,
        only the basic trigger module can be used; on the CW1200, the serial
        data and SAD triggers are available too.

        Available trigger modules:
         * 'basic': Trigger on a logic level or edge
         * 'SAD':   Trigger from SAD module (CWPro only)
         * 'DECODEIO': Trigger from decode_IO module (CWPro only)

        :Getter: Return the active trigger module

        :Setter: Sets the active trigger module

        Raises:
            ValueError: module isn't one of the available strings
        """
        return self.last_module

    @module.setter
    def module(self, mode):
        if mode == "basic":
            module = self.cwe.MODULE_BASIC
        elif mode == "SAD":
            module = self.cwe.MODULE_SADPATTERN
        elif mode == "DECODEIO":
            module = self.cwe.MODULE_DECODEIO
        else:
            raise ValueError("Invalid mode {}. Must be 'basic', 'SAD', or 'DECODEIO'")

        resp = self.cwe.oa.sendMessage(CODE_READ, ADDR_TRIGMOD,
                                       Validate=False, maxResp=1)
        resp[0] &= 0xF8
        resp[0] |= module
        resp = self.cwe.oa.sendMessage(CODE_WRITE, ADDR_TRIGMOD,
                                       resp)
        self.last_module = mode

    @property
    def aux_out(self):
        """Controls AUX out on the CWPro

        CWPro only

        :Getter: Returns True for 'trigger', 'glitch' for 'glitch', 'clock' for 'clock' or False for no output.

        :Setter: Set False or 0 to disable, True or :code:`'trigger'` for trig_out,
                :code:`'glitch'` for glitch out, or :code:`'clock'` for clock_out
        """
        # resp1 = self.cwe.oa.sendMessage(CODE_READ, ADDR_EXTCLK, Validate=False, maxResp=1)
        resp = self.cwe.oa.sendMessage(CODE_READ, ADDR_TRIGMOD, Validate=False, maxResp=1)
        resp2 = self.cwe.oa.sendMessage(CODE_READ, ADDR_EXTCLK, Validate=False, maxResp=1)


        if (resp[0] & 0x08):
            return True
        elif resp2[0] & 0x10:
            return "glitch"
        elif resp2[0] & 0x08:
            return "clock"
        else:
            return False

    @aux_out.setter
    def aux_out(self, enabled):
        if enabled is True:
            enabled = "trigger"
        
        resp = self.cwe.oa.sendMessage(CODE_READ, ADDR_TRIGMOD, Validate=False, maxResp=1)
        resp2 = self.cwe.oa.sendMessage(CODE_READ, ADDR_EXTCLK, Validate=False, maxResp=1)
        resp2[0] &= 0xE7
        resp[0] &= 0xE7
        if enabled == "trigger":
            resp[0] |= 0x08
        elif enabled == "glitch":
            resp2[0] |= 0x10
        elif enabled == "clock":
            resp2[0] |= 0x08
        self.cwe.oa.sendMessage(CODE_WRITE, ADDR_TRIGMOD, resp)
        self.cwe.oa.sendMessage(CODE_WRITE, ADDR_EXTCLK, resp2)


class HuskyTrigger(TriggerSettings):
    """Husky trigger object.
    Communicates with all the trigger modules inside CW-Husky.
    Usage depends on the active trigger module.
    """
    def __init__(self, cwextra):
        self._edges = 1
        super().__init__(cwextra)
        self._is_husky = True

    def _dict_repr(self):
        rtn = {}
        rtn['module'] = self.module
        if self.module == 'ADC':
            rtn['level'] = self.level
        if self.module in ['basic', 'UART', 'edge_counter']:
            rtn['triggers'] = self.triggers
        if self.module == 'edge_counter':
            rtn['edges'] = self.edges
        return rtn

    @property
    def module(self):
        """The trigger module in use.

        The trigger modules available depend on the hardware. On the CWLite,
        only the basic trigger module can be used; on the CW1200, the serial
        data and SAD triggers are available too.

        Available trigger modules:
         * 'basic':        Trigger on a logic level or edge
         * 'ADC':          Trigger on ADC sample exceeding a threshold
         * 'SAD':          Trigger from SAD module
         * 'UART':         Trigger from UART module
         * 'edge_counter': Trigger after a number of rising/falling edges
         * 'trace':        Trigger from TraceWhisperer

        :Getter: Return the active trigger module

        :Setter: Sets the active trigger module

        Raises:
            ValueError: module isn't one of the available strings
        """
        return self.last_module

    @module.setter
    def module(self, mode):
        if mode == "basic":
            module = self.cwe.MODULE_BASIC
        elif mode == "SAD":
            module = self.cwe.MODULE_SADPATTERN
        elif mode == "UART":
            module = self.cwe.MODULE_DECODEIO
        elif mode == "trace":
            module = self.cwe.MODULE_TRACE
        elif mode == "ADC":
            module = self.cwe.MODULE_ADC
        elif mode == "edge_counter":
            module = self.cwe.MODULE_EDGE_COUNTER
        else:
            raise ValueError("Invalid mode {}. Must be 'basic', 'SAD', 'UART', 'ADC', 'trace', or 'edge_counter'")

        resp = self.cwe.oa.sendMessage(CODE_READ, ADDR_TRIGMOD,
                                       Validate=False, maxResp=1)
        resp[0] &= 0xF8
        resp[0] |= module
        resp = self.cwe.oa.sendMessage(CODE_WRITE, ADDR_TRIGMOD,
                                       resp)
        self.last_module = mode

    @property
    def level(self):
        """For triggering on ADC sample exceeding a treshold,
        when scope.trigger.module = 'ADC'.

        Sets the trigger threshold, in the range [-0.5, 0.5].

        If positive, triggers when the ADC sample exceeds this setting;
        if negative, triggers when the ADC sample is below this setting.

        Only a single trigger is issued (i.e. multiple samples exceeding
        the threshold do not each generate a trigger; cannot be used in
        conjunction with segmented capture).
        """
        offset = self.cwe.oa.offset
        raw = int.from_bytes(self.cwe.oa.sendMessage(CODE_READ, ADDR_ADC_TRIGGER_LEVEL, Validate=False, maxResp=2), byteorder='little')
        return raw / 2**12 - offset

    @level.setter
    def level(self, val):
        if not (-0.5 <= val <= 0.5):
            raise ValueError("Out of range: [-0.5, 0.5]")
        offset = self.cwe.oa.offset
        val = int((val + offset) * 2**12)
        self.cwe.oa.sendMessage(CODE_WRITE, ADDR_ADC_TRIGGER_LEVEL, list(int.to_bytes(val, length=2, byteorder='little')))

    @property
    def edges(self):
        """For triggering on edge counts, when :code:`scope.trigger.module = 'edge_counter'`.

        Sets the number of rising+falling edges on :code:`scope.trigger.triggers` that
        need to be seen for a trigger to be issued.

        Edges are sampled by the ADC sampling clock (:code:`scope.clock.adc_freq`), so
        ensure that scope.trigger.triggers does not change faster than what can
        be seen by that clock.

        Args:
            val (int): number of edges, non-zero 16-bit integer.
        """
        return self._edges

    @edges.setter
    def edges(self, val):
        if val < 1 or val > 2**16:
            raise ValueError("Out of range: [1, 2**16]")
        self._edges = val
        self.cwe.oa.sendMessage(CODE_WRITE, ADDR_EDGE_TRIGGER, list(int.to_bytes(val-1, length=2, byteorder='little')))

    @property
    def edges_seen(self):
        """Returns the number of edges seen. 
        
        Under normal operation this should
        be the same as :code:`scope.trigger.edges`. When trigger generation failed, Can
        be useful to understand why. Resets upon :code:`scope.arm()`.
        """
        return int.from_bytes(self.cwe.oa.sendMessage(CODE_READ, ADDR_EDGE_TRIGGER, Validate=False, maxResp=2), byteorder='little')



class SADTrigger(util.DisableNewAttr):
    pass


class DataTrigger(util.DisableNewAttr):
    pass


class ChipWhispererExtra(util.DisableNewAttr):
    _name = 'CW Extra'

    def __init__(self, cwtype, scope, oa):
        super().__init__()
        #self.cwADV = CWAdvTrigger()

        self.cwEXTRA = CWExtraSettings(oa, cwtype)
        #if cwtype == "cwhusky":
        self.enableGlitch = True
        if self.enableGlitch:
            self.glitch = ChipWhispererGlitch.ChipWhispererGlitch(cwtype, scope, oa)

    def armPreScope(self):
        if self.enableGlitch:
            self.glitch.armPreScope()

    def armPostScope(self):
        if self.enableGlitch:
            self.glitch.armPostScope()

    #def testPattern(self):
    #    desired_freq = 38400 * 3
    #    clk = 30E6
    #    clkdivider = (clk / (2 * desired_freq)) + 1
    #    self.cwADV.setIOPattern(strToPattern("\n"), clkdiv=clkdivider)


class CWExtraSettings:
    PIN_FPA = 0x01
    PIN_TNRST = 0x02
    PIN_RTIO1 = 0x04
    PIN_RTIO2 = 0x08
    PIN_RTIO3 = 0x10
    PIN_RTIO4 = 0x20
    MODE_OR = 0x00
    MODE_AND = 0x01
    MODE_NAND = 0x02

    PIN_USERIO0 = 0x0100
    PIN_USERIO1 = 0x0200
    PIN_USERIO2 = 0x0400
    PIN_USERIO3 = 0x0800
    PIN_USERIO4 = 0x1000
    PIN_USERIO5 = 0x2000
    PIN_USERIO6 = 0x4000
    PIN_USERIO7 = 0x8000

    MODULE_BASIC = 0x00
    MODULE_ADVPATTERN = 0x01
    MODULE_SADPATTERN = 0x02
    MODULE_DECODEIO = 0x03
    MODULE_TRACE = 0x04
    MODULE_ADC = 0x05
    MODULE_EDGE_COUNTER = 0x06

    CLOCK_FPA = 0x00
    CLOCK_FPB = 0x01
    CLOCK_PLL = 0x02
    CLOCK_RTIOIN = 0x03
    CLOCK_RTIOOUT = 0x04

    _name = "CW Extra Settings"

    def __init__(self, oa, cwtype):

        if cwtype == "cwrev2":
            hasFPAFPB = True
            hasGlitchOut = False
            hasPLL = True
            hasAux=False
            hasUserio=False
        elif cwtype == "cwlite":
            hasFPAFPB=False
            hasGlitchOut=True
            hasPLL=False
            hasAux=False
            hasUserio=False
        elif cwtype == "cw1200":
            hasFPAFPB=False
            hasGlitchOut=True
            hasPLL=False
            hasAux=True
            hasUserio=False
        elif cwtype == "cwhusky":
            hasFPAFPB=False
            hasGlitchOut=True
            hasPLL=False
            hasAux=True
            hasUserio=True
        else:
            raise ValueError("Unknown ChipWhisperer: %s" % cwtype)

        self.oa = oa
        self.hasFPAFPB = hasFPAFPB
        self.hasGlitchOut = hasGlitchOut
        self.hasPLL = hasPLL
        self.hasAux = hasAux
        self.hasUserio = hasUserio


        #Add special single-class items used as higher-level API
        self.gpiomux = GPIOSettings(self)
        self.triggermux = TriggerSettings(self)
        self.protrigger = ProTrigger(self)
        self.huskytrigger = HuskyTrigger(self)

        if cwtype == "cwhusky":
            self.gpiomux._is_husky = True
            self.triggermux._is_husky = True
            self._addr_trigsrc_size = 2
        else:
            self._addr_trigsrc_size = 1

    # IO Route API

    def read_ioroute(self):
        """Reads the IOROUTE device state.

        Return:
            bytearray representing the IOROUTE state.
        """
        return self.oa.msg_read(ADDR_IOROUTE, max_resp=8)

    def write_ioroute(self, data):
        """Writes a specified state to IOROUTE.
        """
        self.oa.msg_write(ADDR_IOROUTE, data)

    def test_ioroute_mask(self, i, mask):
        """Evaluates a non-zero flag for a specified byte in the IOROUTE state.

        Return:
            False if the mask evaluates to 0, else True.
        """
        return self.oa.msg_test_mask(ADDR_IOROUTE, i, mask, max_resp=8)

    def set_ioroute_mask(self, i, mask):
        """OR's a mask into a specified byte in the IOROUTE state.

        Return:
            The updated IOROUTE state.
        """
        return self.oa.msg_set_mask(ADDR_IOROUTE, i, mask, max_resp=8)

    def clr_ioroute_mask(self, i, mask):
        """NAND's a mask into a specified byte in the IOROUTE state.

        Return:
            The updated IOROUTE state.
        """
        return self.oa.msg_clr_mask(ADDR_IOROUTE, i, mask, max_resp=8)

    def upd_ioroute_mask(self, i, mask, set):
        """Conditionally OR's or NAND's a mask into a specified byte in the IOROUTE state.

        Return:
            The updated IOROUTE state.
        """
        return self.oa.msg_upd_mask(ADDR_IOROUTE, i, mask, set, max_resp=8)

    def and_set_ioroute_enum(self, i, mask, value):
        """Injects a value into a specified byte in the IOROUTE state.

        Return:
            The updated IOROUTE state.
        """
        return self.oa.msg_set_enum(ADDR_IOROUTE, i, mask, value, max_resp=8)

    def get_ioroute_value(self, i):
        """Gets a specified byte from the IOROUTE state.

        Return:
            The specified IOROUTE byte.
        """
        return self.oa.msg_get_value(ADDR_IOROUTE, i, max_resp=8)

    def set_ioroute_value(self, i, value):
        """Assigns a specified byte in the IOROUTE state.

        Return:
            The updated IOROUTE state.
        """
        return self.oa.msg_set_value(ADDR_IOROUTE, i, value, max_resp=8)

    def assert_tio_num(self, io_num):
        """Raises an exception if the TIO pin is invalid.
        """
        if (io_num < GPIO_NUM1) or (io_num > GPIO_NUM4):
            raise ValueError('Invalid normal IO number: %d' % io_num)

    def assert_ctlio_num(self, io_num):
        """Raises an exception if the CTL pin is invalid.
        """
        if (io_num < GPIO_NRST) or (io_num > GPIO_PDIC):
            raise ValueError('Invalid special IO number: %d' % io_num)

    def get_tio_mode(self, io_num):
        """Gets the TIO mode of the specified pin.

        Return:
            A value representing the IOROUTE_* masks.
        """
        self.assert_tio_num(io_num)
        return self.get_ioroute_value(io_num)

    def get_tio_state(self, io_num):
        """Gets the GPIO state of a TIO pin.

        Return:
            None if not in GPIO mode, True if driven high, else False if driven low.
        """
        self.assert_tio_num(io_num)
        mode = self.get_tio_mode()

        if (mode & IOROUTE_GPIOE) == 0:
            return None

        return bool(mode & IOROUTE_GPIO)

    def set_tio_mode(self, io_num, mode):
        """Sets the mode for the specified TIO pin.

        Return:
            The updated IOROUTE state.
        """
        #Sends actual IO mode to FPGA
        self.assert_tio_num(io_num)
        return self.set_ioroute_value(io_num, mode)

    def set_tio_state(self, io_num, state):
        """Sets the state of a specified TIO pin if in GPIO mode.

        Return:
            The updated IOROUTE state.
        """
        self.assert_tio_num(io_num)
        data = self.read_ioroute()

        val = data[io_num]
        if (val & IOROUTE_GPIOE) == 0:
            raise IOError('TargetIO %d is not in GPIO mode' % io_num)

        if not state is None:
            if state:
                data[io_num] = val | IOROUTE_GPIO
            else:
                data[io_num] = val & ~IOROUTE_GPIO
            self.write_ioroute(data)

        return data

    def get_ctlio_mask(self, io_num):
        """Gets a bitmask for a valid CTL IO pin for the IROUTE state.

        Return:
            A bitmask representing the CTL IO pin.
        """
        self.assert_ctlio_num(io_num)
        # Calculate bit index
        io_num = (io_num - GPIO_NRST) << 1
        # Get bitmask
        return 1 << io_num

    def get_ctlio_state(self, io_num):
        """Gets the current state of a CTL IO pin.

        Return:
            None if not driven, True if driven high, else False if driven low.
        """
        io_num = self.get_ctlio_mask(io_num)
        data = self.read_ioroute()

        if (data[6] & io_num) == 0:
            return None

        return bool(data[6] & (io_num << 1))

    def set_ctlio_state(self, io_num, state):
        """Sets the mode for the specified CTL IO pin.

        Return:
            The updated IOROUTE state.
        """
        io_num = self.get_ctlio_mask(io_num)
        data = self.read_ioroute()
        if state is None:
            # Disable GPIO mode
            data[6] &= ~io_num
        else:
            # Enable GPIO mode
            data[6] |= io_num

            # Set pin high/low
            io_num <<= 1
            if state:
                data[6] |= io_num
            else:
                data[6] &= ~io_num

        self.write_ioroute(data)
        return data

    def get_gpio_state(self, io_num):
        """Gets the state for any of the GPIO.

        Return:
            None if not driven, True if driven high, else False if driven low.
        """
        if io_num >= GPIO_NRST:
            return self.get_ctlio_state(io_num)
        return self.get_tio_state(io_num)

    def set_gpio_state(self, io_num, state):
        """Sets the GPIO state for any of the GPIO's.

        Return:
            The updated IOROUTE state.
        """
        if io_num >= GPIO_NRST:
            return self.set_ctlio_state(io_num, state)
        return self.set_tio_state(io_num, state)

    def vglitch_get(self, mask):
        """Gets a bitmask representing the VCC glitch mosfet state.

        Return:
            A GLITCH_OUT_* bitmask of the current VCC glitch mosfet state.
        """
        return self.test_ioroute_mask(4, mask & GLITCH_OUT_BOTH)

    def vglitch_get_hp(self):
        """Gets the enabled state for the HP glitch mosfet.

        Return:
            True if the HP glitch mosfet is enabled, else False.
        """
        return self.test_ioroute_mask(4, GLITCH_OUT_HP)

    def vglitch_get_lp(self):
        """Gets the enabled state for the LP glitch mosfet.

        Return:
            True if the LP glitch mosfet is enabled, else False.
        """
        return self.test_ioroute_mask(4, GLITCH_OUT_LP)

    def vglitch_set_mode(self, mask):
        """Sets the VCC glitch mosfet enabled states.

        Return:
            The updated IOROUTE state.
        """
        return self.and_set_ioroute_enum(4, GLITCH_OUT_CLR, mask & GLITCH_OUT_BOTH)

    def vglitch_enable(self, mask, enabled):
        """Conditionally enables or disables a GLITCH_OUT_* mask.

        Return:
            The updated IOROUTE state.
        """
        return self.upd_ioroute_mask(4, mask & GLITCH_OUT_BOTH, enabled)

    def vglitch_clear(self):
        """Disables both of the VCC glitch mosfets.

        Return:
            The updated IOROUTE state.
        """
        return self.clr_ioroute_mask(4, GLITCH_OUT_CLR)

    def vglitch_reset(self):
        """Disables and reenables the VCC glitch mosfets that were previously enabled.

        Return:
            The updated IOROUTE state.
        """
        data = self.read_ioroute()
        d4 = data[4]
        data[4] = d4 & GLITCH_OUT_CLR
        self.write_ioroute(data)
        data[4] = d4        
        self.write_ioroute(data)
        return data

    def get_avr_isp_mode(self):
        """Gets the current AVR ISP flag.

        Return:
            True if enabled, else False.
        """
        return self.test_ioroute_mask(5, AVR_ISP_MODE)

    def set_avr_isp_mode(self, enabled):
        """Sets the AVR ISP flag.

        Return:
            The updated IOROUTE state.
        """
        return self.upd_ioroute_mask(5, AVR_ISP_MODE, enabled)
        
    # Temporary until all references updated
    getAVRISPMode = get_avr_isp_mode
    setAVRISPMode = set_avr_isp_mode

    def get_target_pwr_state(self):
        """Gets the state of the target power pin.

        Return:
            True if the target power pin is driven high, else False if driven low.
        """
        return self.test_ioroute_mask(5, TARGET_PWR_STATE)

    def set_target_pwr_state(self, enabled):
        """Set the state of the target power pin.

        Return:
            The updated IOROUTE state.
        """
        return self.upd_ioroute_mask(5, TARGET_PWR_STATE, enabled)

    def get_target_pwr_slew(self):
        """Gets the target power SLEW flag.

        Return:
            True if fastmode, else False.
        """
        return self.test_ioroute_mask(5, TARGET_PWR_SLEW)

    def set_target_pwr_slew(self, fastmode):
        """Set the power SLEW flag.

        Return:
            The updated IOROUTE state.
        """
        return self.upd_ioroute_mask(5, TARGET_PWR_SLEW, fastmode)

    # IOREAD API

    def read_io_pins(self):
        """Read signal level of all 4 Target IOn pins synchronously.

        In most cases this is useful for low-speed digital input, hence the
        GPIO state of the Target IOn pin(s) used for digital input should be
        configured as 'High-Z'.

        Returns a bit mask where set bits indicate which of the 4 target IOn
        pins is read as high. Counting starts at bit 0, for example, bit0
        refers to tio1.
        """
        return self.oa.msg_read(ADDR_IOREAD, max_resp=1)[0]

    def read_io_pin(self, io_num):
        """Read signal level of a Target IOn pin.

        Returns True if the signal level of the Target IOn pin is high,
        otherwise False is returned.
        """
        self.assert_tio_num(io_num)
        pins = self.read_io_pins()
        return bool((pins >> io_num) & 1)

    # EXTCLK API

    def get_extclk_mask(self, mask):
        return self.oa.msg_get_mask(ADDR_EXTCLK, 0, mask, max_resp=1)

    def nand_set_extclk_enum(self, mask, value):
        return self.oa.msg_nand_set_enum(ADDR_EXTCLK, 0, mask, value, max_resp=1)

    def get_clksrc(self):
        return self.get_extclk_mask(EXTCLK_SRC_MASK)

    def set_clksrc(self, source):
        return self.nand_set_extclk_enum(EXTCLK_SRC_MASK, source & EXTCLK_SRC_MASK)

    def get_clkout(self):
        return self.get_extclk_mask(EXTCLK_OUT_MASK) >> EXTCLK_OUT_SHIFT

    def set_clkout(self, clkout):
        return self.nand_set_extclk_enum(EXTCLK_OUT_MASK, clkout << EXTCLK_OUT_SHIFT)

    def setPin(self, enabled, pin):
        current = self.getPins()

        pincur = current[0] & ~(pin)
        if enabled:
            pincur = pincur | pin

        self.setPins(pincur, current[1])

    def getPin(self, pin):
        current = self.getPins()
        current = current[0] & pin
        if current == 0:
            return False
        else:
            return True

    def setPinMode(self, mode):
        current = self.getPins()
        self.setPins(current[0], mode)

    def getPinMode(self):
        current = self.getPins()
        return current[1]

    def setPins(self, pins, mode):
        d = list(int.to_bytes((mode << 6) | pins, length=self._addr_trigsrc_size, byteorder='little'))
        self.oa.sendMessage(CODE_WRITE, ADDR_TRIGSRC, d, maxResp=self._addr_trigsrc_size)

    def getPins(self):
        resp = self.oa.sendMessage(CODE_READ, ADDR_TRIGSRC, Validate=False, maxResp=self._addr_trigsrc_size)
        pins = resp[0] & 0x3F
        if self._addr_trigsrc_size == 2:
            pins += (resp[1] << 8)
        mode = resp[0] >> 6
        return(pins, mode)

    def setTriggerModule(self, module):

        #When using special modes, force rising edge & stop user from easily changing
        resp = self.oa.sendMessage(CODE_READ, ADDR_TRIGMOD, Validate=False, maxResp=1)
        resp[0] &= 0xF8
        resp[0] |= module
        self.oa.sendMessage(CODE_WRITE, ADDR_TRIGMOD, resp)

    def getTriggerModule(self):
        resp = self.oa.sendMessage(CODE_READ, ADDR_TRIGMOD, Validate=False, maxResp=1)
        return resp[0]

    def setTrigOutAux(self, enabled):
        resp = self.oa.sendMessage(CODE_READ, ADDR_TRIGMOD, Validate=False, maxResp=1)
        resp[0] &= 0xE7
        if enabled:
            resp[0] |= 0x08
        self.oa.sendMessage(CODE_WRITE, ADDR_TRIGMOD, resp)

    def setTrigOut(self, enabled):
        resp = self.oa.sendMessage(CODE_READ, ADDR_TRIGMOD, Validate=False, maxResp=1)
        resp[0] &= 0xE7
        if enabled:
            resp[0] |= 0x08
        self.oa.sendMessage(CODE_WRITE, ADDR_TRIGMOD, resp)

    def getTrigOut(self):
        resp = self.oa.sendMessage(CODE_READ, ADDR_TRIGMOD, Validate=False, maxResp=1)
        return resp[0] & 0x08


class CWPLLDriver(object):
    def __init__(self):
        super(CWPLLDriver, self).__init__()
        self.oa = None

    def con(self, oa):
        self.oa = oa

    def isPresent(self):
        """Check for CDCE906 PLL Chip"""
        try:
            result = self.readByte(0x00)
        except IOError:
            return False
        if result & 0x0F != 0x01:
            return False
        return True

    def setupPLL(self, N, M, bypass=False, highspeed=True, num=1):
        """
        Setup PLL1.
         * For M & N:
            M =< N.
            VCOF = (Fin * N) / M
            VCOF must be in range 80-300MHz.

         * For highspeed:
           Set to 'True' if VCO freq in range 180-300 MHz. Set low if in range 80-200 MHz
        """

        if num != 1:
            raise ValueError("Only PLL1 Config Supported")

        self.writeByte(0x01, M & 0xFF)
        self.writeByte(0x02, N & 0xFF)

        b = self.readByte(0x03)
        b &= (1 << 6)|(1 << 5)
        if bypass:
            b |= 1 << 7

        b |= (M >> 8)
        b |= ((N >> 8) & 0x0F) << 1

        self.writeByte(0x03, b)

        b = self.readByte(0x06)
        b &= ~(1 << 7)
        if highspeed:
            b |= 1 << 7

        self.writeByte(0x06, b)

    def setupDivider(self, setting, clksrc, divnum=2):
        """
        setting = Divide VCOF from PLL by this value

        clksrc = 0 means PLL Bypass
        clksrc = 1 means PLL1
        clksrc = 2 means PLL2 w/ SCC etc... not supported

        divnum is divider number (0-5)
        """

        if divnum > 5:
            raise ValueError("Invalid Divider Number (0-5 allowed): %d"%divnum)

        divreg = 13 + divnum

        if (setting < 1) | (setting > 127):
            raise ValueError("Invalid Divider Setting (1-127 allowed): %d"%setting)

        self.writeByte(divreg, setting)

        if divnum == 0:
            divreg = 9
            divbits = 5
        elif divnum == 1:
            divreg = 10
            divbits = 5
        elif divnum == 2:
            divreg = 11
            divbits = 0
        elif divnum == 3:
            divreg = 11
            divbits = 3
        elif divnum == 4:
            divreg = 12
            divbits = 0
        else:
            divreg = 12
            divbits = 3

        bold = self.readByte(divreg)
        b = bold & ~(0x07<<divbits)
        b |= (clksrc & 0x07) << divbits

        if bold != b:
            self.writeByte(divreg, b)

    def setupOutput(self, outnum, inv=False, enabled=True, divsource=2, slewrate=3):
        """
        outnum is output number, 0-5
        inv = invert output?
        enable = enable output?
        divsource = divider source, 0-5
        """
        outreg = 19 + outnum
        data = 0

        if enabled:
            data |= 1 << 3

        if inv:
            data |= 1 << 6

        if divsource > 5:
            raise ValueError("Invalid Divider Source Number (0-5 allowed): %d"%divsource)

        data |= divsource
        data |= (slewrate & 0x03) << 4

        self.writeByte(outreg, data)

    def setupClockSource(self, diff=True, useIN0=False, useIN1=False):
        if diff == False:
            #Select which single-ended input to use
            if (useIN0 ^ useIN1) == False:
                raise ValueError("Only one of useIN0 or useIN1 can be True")

            bold = self.readByte(10)
            b = bold & ~(1<<4)
            if useIN1:
                b |= 1<<4

            if b != bold:
                self.writeByte(10, b)
                # print "%x, %x"%(b, self.readByte(10))

        bold = self.readByte(11)
        bnew = bold & ~((1<<6) | (1<<7))
        if diff:
            bnew |= 1<<7
        else:
            bnew |= 1<<6

        if bnew != bold:
            self.writeByte(11, bnew)

        scope_logger.debug('%x, %x' % (bnew, self.readByte(11)))

    def readByte(self, regaddr, slaveaddr=0x69):
        d = bytearray([0x00, 0x80 | 0x69, 0x80 |  regaddr])
        self.oa.sendMessage(CODE_WRITE, ADDR_I2CSTATUS, d, Validate=False)
        time.sleep(0.001)

        d = bytearray([0x04, 0x80 | 0x69, 0x80 |  regaddr])
        self.oa.sendMessage(CODE_WRITE, ADDR_I2CSTATUS, d, Validate=False)
        time.sleep(0.001)

        d = bytearray([0x00, 0x80 | 0x69, 0x80 |  regaddr])
        self.oa.sendMessage(CODE_WRITE, ADDR_I2CSTATUS, d, Validate=False)
        time.sleep(0.001)

        stat = self.oa.sendMessage(CODE_READ, ADDR_I2CSTATUS, Validate=False, maxResp=3)
        if stat[0] & 0x01:
            raise IOError("No ACK from Slave in I2C")

        stat = self.oa.sendMessage(CODE_READ, ADDR_I2CDATA, Validate=False, maxResp=1)
        return stat[0]

    def writeByte(self, regaddr, data, slaveaddr=0x69):
        d = bytearray([data])
        self.oa.sendMessage(CODE_WRITE, ADDR_I2CDATA, d, Validate=False)

        d = bytearray([0x00, 0x69, 0x80 | regaddr])
        self.oa.sendMessage(CODE_WRITE, ADDR_I2CSTATUS, d, Validate=False)
        time.sleep(0.005)

        d = bytearray([0x04, 0x69, 0x80 | regaddr])
        self.oa.sendMessage(CODE_WRITE, ADDR_I2CSTATUS, d, Validate=False)
        time.sleep(0.005)

        d = bytearray([0x00, 0x69, 0x80 | regaddr])
        self.oa.sendMessage(CODE_WRITE, ADDR_I2CSTATUS, d, Validate=False)
        time.sleep(0.005)

        stat = self.oa.sendMessage(CODE_READ, ADDR_I2CSTATUS, Validate=False, maxResp=3)
        if stat[0] & 0x01:
            raise IOError("No ACK from Slave in I2C")
