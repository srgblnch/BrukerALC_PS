#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# BrukerALC_PS.py is part of
# TANGO Device Server (http://sourceforge.net/projects/tango-ds/)
#
# tango-ds is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# tango-ds is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with tango-ds.  If not, see <http://www.gnu.org/licenses/>.

'''Control for ALBAs Bruker power supplies
controlled over a Phoenix PLC using modbus/TCP. This module includes both
the TANGO DS and the launching.
'''

META = u"""
    $URL$
    $LastChangedBy$
    $Date$
    $Rev$
    Author: Lothar Krause <lkrause@cells.es>
    License: GPL3+
""".encode('latin1')

### Imports ###
# Python Imports
import logging
import socket
import traceback
from types import StringType
from pprint import pformat
from copy import copy
from functools import wraps
from time import time

# extra imports
import PyTango as Tg
AQ = Tg.AttrQuality

import PowerSupply.standard as PS
from PowerSupply.util import *

# relative imports
from BrukerALC_ModMux import BrukerALC_ModMux, BrukerALC_ModMux_Class, ModMuxException
from modmux import DEFAULT_CORRECT, error_str

class BrukerALC_PS(PS.PowerSupply):
    '''Tango DS for ALBAs Bruker power supplies controlled over a Phoenix PLC
       using modbus/TCP.
    '''

    def __init__(self, cl, name):
        PS.PowerSupply.__init__(self, cl, name)
        self.init_device(cl, name)

    def init_device(self, cl, name):
        PS.PowerSupply.init_device(self,cl,name)
        M = self.modmux = BrukerALC_ModMux.modmux_instance()

        if not self.Channel:
            desc = "mandatory property 'Channel' not set for device %r (%s)" % (name, type(name).__name__)
            raise Exception(desc)
        try:
            self.Channel = int(self.Channel)
        except ValueError:
            desc = "property 'Channel' of device %r must be an integer, not %r." % ( name, self.Channel )
            raise Exception(desc)

        # informs modmux of channel hname
        M.name[self.Channel] = self.get_name().rpartition('/')[2].upper()

        # communicates Izero and Ilimit to modmux
        offset = self._get_attr_correction('CurrentSetpoint', dflt=DEFAULT_CORRECT)['Offset']
        M.Izero[self.Channel] = int(offset)

        Ilimit = self._correct2('Current', self.RegulationPrecision, True, dflt=DEFAULT_CORRECT)
        M.Ilimit[self.Channel] = int(Ilimit)
        self.STAT.INITIALIZED()

    def is_power_on(self):
        return self.modmux.is_channel_on(self.Channel)

    def is_busy(self):
        if self.get_state()!=Tg.DevState.MOVING:
            return False
        # checks whether too far away from setpoint
        av = self._read('CurrentDelta')
        if av.quality==PS.AQ_VALID and self.is_power_on():
            return abs(av.value) > self.RegulationPrecision
        else:
            return False

    def update_busy(self):
        """Updates state,status when new current setpoint has been set.
        """
        if self.get_state() not in (Tg.DevState.ON, Tg.DevState.MOVING):
            return
        self.STAT.CURRENT_ADJUST()

    ### Commands ###
    @PS.CommandExc
    def UpdateState(self):
        # faults are sticky
        if self.get_state()==Tg.DevState.FAULT:
            return self.get_state()
        elif self.get_state()==Tg.DevState.ALARM and self.alarms:
            return self.get_state()

        MOD = self.modmux

        # setup some alias to render code more readable
        ch = self.Channel
        READY = MOD.st_ready
        ON = self.is_power_on()

        if MOD.comm_fail:
            self.STAT.COMM_ERROR(MOD.comm_fail)

        elif READY is None:
            s = BrukerALC_ModMux.stat_instance()
            self.STAT.set_stat2(s.state, s.status)

        elif not READY:
            self.STAT.NOT_READY()
        # READY guranteed after this line

        elif self.is_busy():
            self.update_busy()

        elif not MOD.st_on:
            self.STAT.OFF(what='power supply')

        elif ON:
            code = MOD.Imeas_state[self.Channel]
            if code is None:
                if MOD.Iref[self.Channel] is None:
                    self.STAT.ALARM('power on but setpoint unknown!')
                else:
                    self.STAT.ON_CURRENT()
            else:
                status = ''.join(error_str(code))
                self.STAT.ALARM(status)


        elif ON is None:
            self.STAT.UNKNOWN('ready')

        elif not ON:
            self.STAT.OFF()

        # otherwise the programmar made a mistake
        else:
            self.STAT.SYNTH_FAULT("ON=%r,READY=%r" % (ON, READY))


    @PS.CommandExc
    def read_Ready(self, attr):
        attr.set_value(bool(self.modmux.st_ready))

    @PS.CommandExc
    def read_Fault(self, attr):
        attr.set_value(bool(not self.modmux.st_ready) )

    @PS.CommandExc
    def On(self):
        try:
            self.modmux.switch_channel_on(self.Channel, force=True)
            self.set_state(Tg.DevState.MOVING)
            self.STAT.SWITCH_ON()
            self.update_busy()
        except ModMuxException, exc:
            self.STAT.ALARM(str(exc))
            raise PS.PS_Exception(exc)

    @PS.CommandExc
    def Off(self):
        self.modmux.switch_channel_off(self.Channel)

    ### Current related ###
    # low-level attribute
    @PS.AttrExc
    def read_RemoteMode(self, attr):
        if self.modmux.comm_t:
            attr.set_value_date_quality(True, self.modmux.comm_t, PS.AQ_VALID)
        else:
            attr.set_quality(PS.AQ_INVALID)

    @PS.AttrExc
    def read_I(self, attr):
        val = self.modmux.Imeas[self.Channel]
        if val is None:
            attr.set_quality(AQ.ATTR_INVALID)
        else:
            val = int(val)
            attr.set_value(val)

        Iref = self.modmux.Iref[self.Channel]
        if not Iref is None:
            attr.set_write_value(Iref)

    @PS.AttrExc
    def write_I(self, attr):
        self.modmux.set_Iref(self.Channel, attr.get_write_value())

    @PS.AttrExc
    def read_Current(self, attr):
        meas = self.modmux.Imeas[self.Channel]
        cor = self._correct2('Current', meas, dflt=DEFAULT_CORRECT)
        if cor is None:
            attr.set_quality(PS.AQ_INVALID)
            return

        # sets quality to warning when too far away from setpoint
        Iset_aval = self._read('CurrentSetpoint')
        if Iset_aval.quality==PS.AQ_VALID and abs(Iset_aval.value-cor) > self.RegulationPrecision and self.get_state() == Tg.DevState.ON:
            attr.set_value_date_quality(cor, time(), PS.AQ_WARNING)
        else:
            attr.set_value(cor)

    @PS.AttrExc
    def read_CurrentSetpoint(self, attr):
        Iref = self.modmux.Iref[self.Channel]
        v = self._correct_attr('CurrentSetpoint', attr, Iref, inv=True, dflt=DEFAULT_CORRECT)
        if not v is None:
          attr.set_write_value(v)

    @PS.AttrExc
    def write_CurrentSetpoint(self, attr):
        Iset = attr.get_write_value()
        corrected = self._correct('CurrentSetpoint', Iset, dflt=DEFAULT_CORRECT)
        self.modmux.set_Iref(self.Channel, corrected)
        self.update_busy()

    ### Voltage related ###
    @PS.AttrExc
    def read_Voltage(self, attr):
        Vmeas = self.modmux.Vmeas[self.Channel]
        if Vmeas is None:
            attr.set_quality(AQ.ATTR_INVALID)
        else:
            V = self._correct('Voltage', Vmeas, inv=True, dflt=DEFAULT_CORRECT)
            attr.set_value(V)

    @PS.AttrExc
    def read_V(self, attr):
        V = self.modmux.Vmeas[self.Channel]
        if V is None:
            attr.set_quality(AQ.ATTR_INVALID)
        else:
            attr.set_value(V)

    @PS.AttrExc
    def read_ErrorCode(self, attr):
      state1 = self.modmux.Imeas_state[self.Channel] or 0
      state2 = self.modmux.Vmeas_state[self.Channel] or 0
      attr.set_value( (state1<<8) | state2 )


class BrukerALC_PS_Class(PS.PowerSupply_Class):
    class_property_list = PS.gen_property_list( ("RegulationPrecision",))
    class_property_list['RegulationPrecision'][2] = 0.01 # 5e-3

    device_property_list = PS.gen_property_list( ("Channel",), XI=4, cpl=class_property_list)

    attr_opt = ('Current','CurrentSetpoint', 'Voltage')
    attr_list = PS.gen_attr_list(opt=attr_opt, max_err=100)
    POLL_PERIOD = 500
    attr_list['Current'][1]['polling period'] = POLL_PERIOD
    I_FMT = '%6.4f'
    attr_list['Current'][1]['format'] = I_FMT
    attr_list['CurrentSetpoint'][1]['format'] = I_FMT

    attr_list['Voltage'][1]['polling period'] = POLL_PERIOD
    attr_list['I'] = [ [ Tg.DevShort, Tg.SCALAR, Tg.READ_WRITE ], dict() ]
    attr_list['V'] = [ [ Tg.DevShort, Tg.SCALAR, Tg.READ ], dict() ]
    attr_list['ErrorCode'] = [ [ Tg.DevLong, Tg.SCALAR, Tg.READ ], dict() ]

    cmd_opt = ( 'UpdateState', )
    cmd_list = PS.gen_cmd_list(opt=cmd_opt)
    cmd_list['UpdateState'][2]['polling period'] = 500


if __name__ == '__main__':
    PS.tango_main('BrukerALC_PS', (BrukerALC_PS, BrukerALC_ModMux) )
