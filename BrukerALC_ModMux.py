#!/usr/bin/env python2.5
# -*- coding: utf-8 -*-
#
# BrukerALC_ModMux.py is part of tango-ds
# http://sourceforge.net/projects/tango-ds/
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
#

'''Control for Bruker powersupplies for the correctors in ALBAs
   Linac-to-booster Transferline (ALC).
   The ALC are controlled over a Phoenix PLC using modbus/TCP.
   This module contains a device server class for functionality effecting
   all power supply channels
'''

from time import time
import PyTango as Tg
# extra imports
import PowerSupply.standard as PS
from PowerSupply.util import *
# local imports
from modmux import ModMux, ModMuxException

# after how many seconds to wait before tryin to read again from a modbus
# device that timed out. More than 3 seconds since the default timeout is 3
# seconds.
REPEAT_DELAY = 9

class BrukerALC_ModMux(PS.PowerSupply):
    '''Control for Bruker powersupplies for the correctors in ALBAs
       Linac-to-booster Transferline (ALC).
       The ALC are controlled over a Phoenix PLC using modbus/TCP.
       This module contains a device server class for functionality effecting
       all power supply channels.
    '''

    def __init__(self, cl, name):
        PS.PowerSupply.__init__(self, cl, name)
        self.init_device(cl, name)

        # for pychecker
        if False:
            self.ModbusDevice = None
            self.ModbusTimeout = None

    def init_device(self, cl, name):
        PS.PowerSupply.init_device(self,cl,name)
        BrukerALC_ModMux.__STAT = self.STAT

        self.modmux = self.modmux_instance()
        if not self.ModbusDevice:
            desc = "mandatory property 'Modbus' not set for device %r." % name
            raise Exception(desc)

        self.modmux.connect(self.ModbusDevice)
        self.modmux.set_timeout_millis_df(self.ModbusTimeout)
        self._next_update = 0
        self.STAT.INITIALIZED()
        try:
            PS.CommandExc(self.__class__.On)(self)
        except Exception, exc:
            self.log.error('failure in init_device {0}'.format(exc))


    __MODMUX = None
    @classmethod
    def modmux_instance(cls):
        '''Returns server-wide shared instance of Modmux class.
        '''
        if cls.__MODMUX is None:
            cls.__MODMUX = ModMux()
        return cls.__MODMUX

    __STAT = None
    @classmethod
    def stat_instance(cls):
        '''Returns server-wide shared instance of BrukerALC_Mux class
           in order to allow access to state and status.
        '''
        return cls.__STAT


    ### Commands ###

    @PS.CommandExc
    def UpdateState(self):

        # delays update if Modbus calls timeout, ensures responsiveness
        # even if client is not responding
        if self._next_update > time():
            return self.get_state()

        M = self.modmux

        # reads status information from hardware
        try:
            M.update()

        # guards for handling communication errors
        except Tg.CommunicationFailed, flt:
            self._next_update = time()+REPEAT_DELAY
            reason = flt[-1].reason
            desc = flt[-1].desc
            if reason == 'API_DeviceTimedOut':
                self.STAT.ALARM('modbus %r timeout' % self.ModbusDevice)
                # possible DevFailed exception are returned and ignored
                self.modmux.set_timeout_millis_df(self.ModbusTimeout)
            else:
                self.STAT.ALARM('communication: %s' % desc)

        except Tg.DevFailed, flt:
            self._next_update = time()+REPEAT_DELAY
            reason = flt[-1].reason
            desc = flt[0].desc
            self.STAT.ALARM('%s' % desc)

        else:
            # process updated information
            R = M.st_ready
            O = M.st_on

            if R is None:
                self.STAT.UNKNOWN("no data read")

            elif R and O:
                self.STAT.ON_CURRENT()

            elif R:
                self.STAT.OFF(what='power supply')

            elif not R:
                self.STAT.INTERLOCK("")

            else:
                self.STAT.SYNTH_FAULT( (R,O) )


    @PS.CommandExc
    def On(self):
        self.modmux.poweron()

    @PS.CommandExc
    def Off(self):
        self.modmux.poweroff()

    @PS.ExceptionHandler
    def Summary(self):
        self.__summary = self.modmux.summary()
        return self.__summary

    ### Attributes ###
    @PS.ExceptionHandler
    def read_Ready(self, attr):
        attr.set_value(bool(self.modmux.st_ready))

    @PS.ExceptionHandler
    def read_RemoteMode(self, attr):
        if self.modmux.comm_t:
            attr.set_value_date_quality(True, self.modmux.comm_t, PS.AQ_VALID)
        else:
            attr.set_quality(PS.AQ_INVALID)


class BrukerALC_ModMux_Class(PS.PowerSupply_Class):

    class_property_list = {
    }

    device_property_list = PS.gen_property_list( opt=('ModbusDevice','ModbusTimeout') )

    attr_list = PS.gen_attr_list(max_err=16)


    cmd_opt = ('UpdateState', )
    cmd_list = PS.gen_cmd_list(opt=cmd_opt)
    cmd_list['Summary'] = [
        [ Tg.DevVoid, "" ],
        [ Tg.DevString, "returns textual summary of different channels" ],
        {} ]

