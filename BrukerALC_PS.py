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
    $URL: https://tango-ds.svn.sourceforge.net/svnroot/tango-ds/Servers/PowerSupply/BrukerEC_PS/1.9/BrukerEC_PS.py $
    $LastChangedBy: lkrause2 $
    $Date: 2010-07-20 14:38:59 +0200 (mar 20 de jul de 2010) $
    $Rev: 1911 $
    Author: Lothar Krause <lkrause@cells.es>
    License: GPL3+ $
""".encode('latin1')

### Imports ###
# Python Imports
import sys
import logging
import socket
import traceback
from types import StringType
from pprint import pformat
from copy import copy
from functools import wraps

# extra imports
import PyTango as Tg
AQ = Tg.AttrQuality
import ps_standard as PS
from ps_util import *

# relative imports
from BrukerALC_ModMux import BrukerALC_ModMux, BrukerALC_ModMux_Class, ModMuxException
from modmux import DEFAULT_CORRECT

class BrukerALC_PS(PS.PowerSupply):
    '''Tango DS for ALBAs Bruker power supplies controlled over a Phoenix PLC
using modbus/TCP.
    '''

    # assigned later in main program

    def __init__(self, cl, name):
        PS.PowerSupply.__init__(self, cl, name)
        self.init_device(cl, name)

    def init_device(self, cl, name):
        PS.PowerSupply.init_device(self,cl,name)
        self.modmux = BrukerALC_ModMux.modmux_instance()

        # also catches defect #2658088 in PyTango
        if not self.Channel:
            desc = "mandatory property 'Channel' not set for device %r" % name
            raise Exception(desc)
        try:
            self.Channel = int(self.Channel)
        except ValueError:
            desc = "property 'Channel' of device %r must be an integer, not %r." % ( name, self.Channel )
            raise Exception(desc)

        # sets up event handling
        self.set_change_event("Status", True, True)
        self.set_change_event("State", True, True)

        self.STAT.INITIALIZED()

    ### Commands ###
    @PS.CommandExc
    def UpdateState(self):
        # faults are sticky
        if self.get_state()==Tg.DevState.FAULT:
            return self.get_state()

        MOD = self.modmux

        # setup some alias to render code more readable
        ch = self.Channel
        READY = MOD.Ready
        ON = MOD.channel_on[self.Channel]

        if READY is None:
            s = BrukerALC_ModMux.stat_instance()
            self.STAT.set_stat2(s.state, s.status)

        elif not READY:
            self.STAT.NOT_READY()
        # READY guranteed after this line

        elif ON:
            p = PS.PSEUDO_ATTR
            self.read_Current(p)
            if p.quality == AQ.ATTR_VALID:
                self.STAT.ON_CURRENT()
            else:
                self.STAT.ON_CURRENT()

        elif not MOD.On:
            self.STAT.OFF()

        elif ON is None:
            self.STAT.UNKNOWN('ready')

        # the correctors have no real "OFF" state so we use "STANDBY"
        # - for the moment
        elif not ON:
            extra = None
            try:
                aval = self._read('CurrentSetpoint')
                extra = str(aval.value)+" A"
            except Exception, exc:
                self.log.debug(exc)
            self.STAT.STANDBY(extra)

        # otherwise the programmar made a mistake
        else:
            self.STAT.SYNTH_FAULT("ON=%r,READY=%r" % (ON, READY))

    @PS.CommandExc
    def read_Ready(self, attr):
        attr.set_value(bool(self.modmux.Ready))

    @PS.CommandExc
    def read_Fault(self, attr):
        attr.set_value( bool(not self.modmux.Ready) )

    @PS.CommandExc
    def On(self):
        try:
            M = self.modmux
            offset = self._get_attr_correction('CurrentSetpoint')['Offset']
            if M.Iref[self.Channel] is None:
                self.modmux.set_Iref(self.Channel, -offset)
            self.modmux.switch_channel_on(self.Channel)
        except ModMuxException, exc:
            raise PS.PS_Exception(exc)

    @PS.CommandExc
    def Off(self):
        offset = self._get_attr_correction('CurrentSetpoint')['Offset']
        self.modmux.switch_channel_off(self.Channel, -offset)


    ### Current related ###
    # low-level attribute
    @PS.AttrExc
    def read_RemoteMode(self, attr):
        attr.set_value(True)

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
            attr.set_write_value(int(val))

    @PS.AttrExc
    def write_I(self, attr):
        Iref = int(attr.get_write_value())
        self.modmux.set_Iref(self.Channel, Iref)

    # corrected values
    @PS.AttrExc
    def read_Current(self, attr):
        meas = self.modmux.Imeas[self.Channel]
        v = self._correct_attr('CurrentSetpoint', attr, meas, inv=True, dflt=DEFAULT_CORRECT)

    @PS.AttrExc
    def read_CurrentStatus(self, attr):
        ist = self.modmux.Imeas_state[self.Channel]
        status = ''.join(modmux.error_str(ist))
        attr.set_value(status)

    @PS.AttrExc
    def read_CurrentSetpoint(self, attr):
        Iref = self.modmux.Iref[self.Channel]
        v = self._correct_attr('CurrentSetpoint', attr, Iref, inv=True, dflt=DEFAULT_CORRECT)
        attr.set_write_value(v)

    @PS.AttrExc
    def write_CurrentSetpoint(self, attr):
        data = []
        attr.get_write_value(data)
        corrected = self._correct('CurrentSetpoint', data[0], dflt=DEFAULT_CORRECT)
        self.modmux.set_Iref(self.Channel, corrected)

    ### Voltage related ###
    @PS.AttrExc
    def read_Voltage(self, attr):
        Umeas = self.modmux.Umeas[self.Channel]
        if Umeas is None:
            attr.set_quality(AQ.ATTR_INVALID)
        else:
            U = self._correct('Voltage', Umeas, inv=True, dflt=DEFAULT_CORRECT)
            attr.set_value(U)


    @PS.AttrExc
    def read_V(self, attr):
        val = self.modmux.Umeas[self.Channel]
        if val is None:
            attr.set_quality(AQ.ATTR_INVALID)
        else:
            val = int(val)
            attr.set_value(val)


class BrukerALC_PS_Class(PS.PowerSupply_Class):
    class_property_list = {
    }

    device_property_list = PS.gen_property_list( ("Channel",), XI=4)

    attr_list = PS.gen_attr_list(opt=('Current','CurrentSetpoint', 'Voltage'), max_err=100)
    POLL_PERIOD = 500
    attr_list['Current'][1]['polling period'] = POLL_PERIOD
    I_FMT = '%6.4f'
    attr_list['Current'][1]['format'] = I_FMT
    attr_list['CurrentSetpoint'][1]['format'] = I_FMT

    attr_list['Voltage'][1]['polling period'] = POLL_PERIOD
    attr_list['I'] = [ [ Tg.DevShort, Tg.SCALAR, Tg.READ_WRITE ], dict() ]
    attr_list['V'] = [ [ Tg.DevShort, Tg.SCALAR, Tg.READ ], dict() ]

    cmd_opt = ( 'UpdateState',)
    cmd_list = PS.gen_cmd_list(opt=cmd_opt)
    cmd_list['UpdateState'][2]['polling period'] = 500


if __name__ == '__main__':
  PS.tango_main('BrukerALC_PS', sys.argv, (BrukerALC_PS, BrukerALC_ModMux) )
