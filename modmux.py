#!/usr/bin/env python
# -*- coding: utf-8 -*-

# modmux.py - modbus interface to Bruker LT corrector PS.
# This file is part of tango-ds (http://sourceforge.net/projects/tango-ds/)
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

'''Modbus interface for Bruker LT power supplies.
'''

# NEAT: use PCP for both reading & writing (can not mix)
# MAYB: redesign modmux to use channel info class with Imeas, Iref, Izero,
#       update_t, state
# NEAT: tuning read_AIN() loop so that is automatically adjusts to timing
# NEAT: run continously read_AIN in background thread, pushing change events
# NEAT: optimize configuration step
# NEAT: tune configure() step to consist of one write, one read step (for checking)
# NEAT: measure how long take Modbus calls in comparison with wait.
# NEAT: remove / beautify ctrlword() function
# NEAT: tango-cs bug #2801558: setting the timeout even when the device is not exported

from time import time, sleep
import ctypes
import logging
from functools import partial
import PyTango as Tg
from PowerSupply.util import int2bin

CHANNEL_NUM = 12

### Analog Input Modules ###
# information about analog inputs (AIN) and their modules
AIN_WRITE_BASE = 586
AIN_READ_BASE = 202
AIN_MODULE_NUM = 3
AIN_PER_MODULE_NUM = 8
AIN_TOTAL_NUM = AIN_MODULE_NUM * AIN_PER_MODULE_NUM
AIN_MODULE_SIZE = 2
AIN_TOTAL_SIZE = AIN_MODULE_SIZE * AIN_MODULE_NUM

# AIN configuration,
AIN_MEAS_PM10V   =    0x1 #< sets measurement range to +-10V
AIN_FORMAT_IB_IL =  0x000
AIN_FILTER_AVG16 = 0x0000
AIN_FILTER_NONE  = 0x1000
AIN_FILTER_AVG4  = 0x2000
AIN_FILTER_AVG32 = 0x3000
AIN_CONFIG_OPTIONS = AIN_MEAS_PM10V | AIN_FORMAT_IB_IL | AIN_FILTER_AVG32
#   6000 - configures all channels
AIN_CONTROL_CONFIG = [ 0x6000, AIN_CONFIG_OPTIONS ]


### Analog Output Modules ###
# information about analog outputs (AOUT) and their modules
AOUT_WRITE_BASE = 576
AOUT_READ_BASE = 192
AOUT_MODULE_NUM = 2
AOUT_PER_MODULE = 8
AOUT_GROUP_PER_MODULE = 2
AOUT_PER_GROUP = 4
AOUT_MODULE_SIZE = 5
AOUT_MAX_VALUE = +32512 # 0x7f00
AOUT_MIN_VALUE = -32512 # 0x8100
AOUT_GROUP_CODE = [ 0x100, 0x900 ]
AOUT_TOTAL_SIZE = AOUT_MODULE_SIZE * AOUT_MODULE_NUM

# AOUT configuration
AOUT_RANGE_PM10V =   0x1 #< sets output range to +- 10 V
AOUT_FORMAT_IB_IL = 0x00 #< IB IL format
AOUT_CONFIG_OPTIONS = AOUT_RANGE_PM10V | AOUT_FORMAT_IB_IL
# parameters for AOUT configuration
AOUT_CONTROL_CONFIG = 0x6000 | AOUT_CONFIG_OPTIONS
AOUT_CONTROL_RECALL = 0x0200

### Digital Inputs and Outputs ###
# information about digital inputs and outputs
DIN_BASE = 0
DOUT_BASE = 384
COIL_OUT = 0

# Phoenix PCP communication
# used for analog out
PCP2_ADDR_COMM = 6020
PCP2_ADDR_CONF = 6021

PCP3_ADDR_COMM = 6030
PCP3_ADDR_CONF = 6031

PCP_CONF = 0x80
PCP_AOUT = 0x85

# approximate conversion factor for converting integer values to real values
# these factors are meant as an aid in testing, the TANGO DS allows to
# configure individual conversion factors for each channel.

# per Bruker documentation, for readback
# 2.0 / 30000 # A / unit
# 2.0 / 30000 # V / unit

# useful defaults
CONV_in2U = 2.0 / 30000
OFF_in2U = 0

CONV_in2I = 2.0 / 30000
OFF_in2I = 0

CONV_I2out = 30000.0 / 2
OFF_I2out = 0

DEFAULT_CORRECT = {
    'ScalingFactor' : 15000.0,
    'Offset' : 0.0
}

DEFAULT_ILIMIT = 25

MASK_ERROR = 0x8000

# time to wait after modbus write to select AIN channels
# and AIN being ready to be read.
# 1.0 / 120 s is a experimental value that
# offers a good compromise between rarely having to redo
# the call, and not waiting too much.
READ_DELAY = 1.0 / 100

# maximum delay until configuration is ready, measurement data available
MAX_DELAY = 0.1


class ModMuxException(Exception):
    pass

def ctrlword(groupidx, recall=False):
    '''Looks up which control word must be configured to select the
       AOUT group belonging to 'groupidx'.
    '''
    g = int(groupidx)
    if not 0 <= g < AOUT_GROUP_PER_MODULE:
        raise Exception('invalid a-out group index %d' % g)
    a = 0x0900 if g else 0x0100
    if recall:
        a |= AOUT_CONTROL_RECALL
    return a

def groupidx(ctrlword):
    '''Looks up which control word must be configured to select the
       AOUT group belonging to 'groupidx'.
    '''
    c = int(ctrlword)
    if c==0x0900:
        return 1
    elif c==0x0100:
        return 0
    else:
        raise ModMuxException('ctrlword %x does not select a group' % c)

def extract_ain_status(data, sep=","):
    return [ sep.join(error_str(c)) for c in extract_ain_code(data) ]

def extract_ain_code(ain_values):
    '''Returns tuple with error codes described for the
       Phoenix IB IL AI/8 SF format or None if no error occured.
    '''
    def uv(a):
        if a is None:
            return None
        u = ctypes.c_uint16(a).value
        if  0x8000 < u < 0x8100:
            return u^0x8000
    return tuple(uv(a) for a in ain_values)

# Errors following Phoenix IB IL AI/8 SF format
ERROR_MASK = ERROR_MIN = 0x8000
ERROR_MAX = 0x8100
ERROR_MSG = {
    0x01 : 'overrange',
    0x02 : 'open circuit',
    0x04 : 'invalid measure (channel configured?)',
    0x10 : 'I/O supply voltage failure',
    0x40 : 'faulty analog input module',
    0x80 : 'underrange'
}

def error_str(code):
    '''Returns a list of strings describing the errors code.
       If the code does not describe an error, ['ok'] is returned,
       so in any case the list returned contains at least 1 element.
    '''
    if code is None or code==0:
        return [ 'ok' ]
    strings = [ msg for mask,msg in ERROR_MSG.iteritems() if code & mask  ]
    return strings

def nullify(values, errors, NAN=0):
    '''Returns tuple of values replacing elements for which the corresponding
       element in errors is True by NAN.
    '''
    return tuple(NAN if err else val for val,err in zip(values,errors) )

class Channel(object):
    state = None

    Izero = 0
    Iref = None
    Imeas = None
    Imeas_state = 0

    Vmeas = None
    Vmeas_state = 0

class ModMux(object):
    ''' Master of Data class.object for multiplexing the modbus device for the
        different channels. Responsible for reading and writing modbus hardware,
        converting data and returning different channels.
    '''

    mb_read = mb_write = mb_coil =  None


    def __init__(self):
        self.log = logging.getLogger('ModMux')

        self.need_config = False
        self.pcp_configured = False

        # channel names
        self.name = [ 'channel %s' % ch for ch in range(CHANNEL_NUM) ]

        # channel settings
        self.channel_on = [ None ] * CHANNEL_NUM
        self.Izero = [ 0.0 ] * CHANNEL_NUM
        self.Ilimit = [ DEFAULT_ILIMIT ] * CHANNEL_NUM
        self.Iref = [  None ] * CHANNEL_NUM

        # read back from PLC
        self.Imeas = [ None ] * CHANNEL_NUM
        self.Vmeas = [ None ] * CHANNEL_NUM

        # state of power supply
        self.st_ready = None
        self.st_on = None

        # needed for something
        self.pcp_invoke_id = -2

        # counters
        self.jiffies = 0
        self.read_count = 0
        self.read_repeat_count = 0
        self.write_count = 0

    def connect(self, name):
        self.device_name = name
        self.reconnect()

    def reconnect(self):
        self.modbus = Tg.DeviceProxy(self.device_name)
        self.mb_write = self.modbus.PresetMultipleRegisters
        self.mb_read = self.modbus.ReadInputRegisters
        self.mb_coils = self.modbus.ForceMultipleCoils

    def set_timeout_millis_df(self, value):
        try:
            self.modbus.set_timeout_millis(value)
        except Tg.DevFailed, df:
            desc = df[-1]['desc']
            self.log.error('Could not set timeout because device %s failed %s.', self.device_name, desc)
            return df

    def configure(self):
        '''Configures analog terminals for reading / writing analog outputs.
        '''

        self.log.debug('configuring analog I/O...')

        # configure all analog inputs
        ain_cfg = AIN_CONTROL_CONFIG*AIN_MODULE_NUM
        arg_cfg = [ AIN_WRITE_BASE, AIN_TOTAL_SIZE ] + ain_cfg
        self.mb_write(arg_cfg)

        # configures all analog outputs
        cfg1 = [ AOUT_CONTROL_CONFIG] + [ 0 ] * AOUT_PER_GROUP
        arg_cfg = [ AOUT_WRITE_BASE, AOUT_TOTAL_SIZE ] +  cfg1 * AOUT_MODULE_NUM
        self.mb_write( arg_cfg )

        # waits for changes to take effect
        sleep(MAX_DELAY)

        # checks whether configuration was applied successfully
        status = self.mb_read([AIN_READ_BASE, AIN_TOTAL_SIZE ])
        assert all(status == ain_cfg), "failure to configure analog input module(s)! should be %s is %s." % (ain_cfg, status)
        self.log.debug('configured analog input modules %s', map(hex,AIN_CONTROL_CONFIG))

        # checks whether AOUT configuration has been accepted
        aout_state = self.mb_read( [AOUT_READ_BASE, AOUT_TOTAL_SIZE ])[::AOUT_MODULE_SIZE]
        aout_cfg = [ AOUT_CONTROL_CONFIG] * AOUT_MODULE_NUM
        assert all(aout_state==aout_cfg), "could not configure analog output module(s)\nShould be %s is %s." % (aout_cfg, aout_state)
        self.log.debug('configured analog output modules %x', AOUT_CONTROL_CONFIG)

        # reconfiguring the channels has the unfortunate effect of
        # switching them off
        self.channel_on = [ False ] * CHANNEL_NUM

    def configure_pcp(self):
        # configures AOUT for both reading and writing
        args = [PCP2_ADDR_CONF, 3, PCP_CONF, 9, self.pcp_invoke_id]
        self.mb_write(args)
        self.pcp_invoke_id+=1

        args = [PCP2_ADDR_COMM, 1, 2]
        self.mb_write(args)

        # configures AOUT for reading
        args = [PCP2_ADDR_CONF, 3, PCP_AOUT, 0, self.pcp_invoke_id]
        self.mb_write(args)
        self.pcp_invoke_id+=1

    def update(self):
        '''Updates status of modmux interface from modbus device.
        '''
        if self.need_config:
            self.configure()
            self.need_config = False
            return

        ain = self.read_AIN()
        self.Imeas = ain[CHANNEL_NUM:2*CHANNEL_NUM]
        self.Imeas_state = extract_ain_code(self.Imeas)

        self.Vmeas = ain[:CHANNEL_NUM]
        self.Vmeas_state = extract_ain_code(self.Vmeas)
        self.st_ready, self.st_on = self.read_state()

        self.jiffies += 1

    def read_state(self):
        state = self.mb_read( [DIN_BASE, 1] )
        bits = int2bin(state[0])
        return bits[0], bits[1]

    def poweron(self):
        self._power(1)

    def poweroff(self):
        # marks all channels as unknown
        self.channel_on = [ None ] * CHANNEL_NUM
        self._write_all()
        self._power(0)

    def is_channel_on(self, ch):
        """Indicates whether this channel is switched on.
           It is possible that None is returned to indicate
           that one can not be sure one way nor the other.
        """
        if not self.st_on:
            return False
        o = self.channel_on[ch]

        # if not determined yet, the guessing begins
        if o is None:
            Izero = self.Izero[ch]
            Ilimit = self.Ilimit[ch]
            Iref = self.Iref[ch]
            Imeas = self.Imeas[ch]

            if Izero is None:
                o = None

            elif not Iref is None:
                o = Iref!=Izero

            elif Imeas is None or Ilimit is None:
                o = None

            else:
                o = abs(Imeas-Izero) > Ilimit

            self.channel_on[ch] = o
        return o


    def _power(self, flag):
        flag = int(bool(flag))
        self.mb_coils( [COIL_OUT, 1, flag] )

    def switch_channel_off(self, ch0):
        self._write(ch0)
        self.channel_on[ch0] = False

    def switch_channel_on(self, ch0, force=False):
        '''Switches channel 'ch0' on. If no reference current was set an
           ModMuxException will be thrown.
        '''
        if force:
            if not self.st_on:
                self.poweron()

        elif not self.st_on:
                raise ModMuxException("power supply is off")

        elif not self.st_ready:
                raise ModMuxException("power supply not ready")

        if self.Iref[ch0] is None:
            self.Iref[ch0] = self.Izero[ch0]
        r = self._write(ch0)
        self.channel_on[ch0] = True
        return r

    def write_ack(self, addr, words, idx=(0,)):
        read = self.mb_read
        self.mb_write( [ addr, len(words) ] + words )
        max_t = time()+MAX_DELAY
        while time() < max_t:
            self.read_count += 1
            dat = read( [addr, len(words)] )
            if all(dat[i]==words[i] for i in idx):
                break
            else:
                self.read_repeat_count += 1
            sleep(READ_DELAY)
        return dat

    def read_AOUT(self):
        # for the first 8 channel PCP reading is used
        data = self.mb_read( [PCP2_ADDR_COMM, AOUT_PER_MODULE] )
        ## data += self.mb_read( [PCP3_ADDR_COMM, CHANNEL_NUM-len(data)] )
        self.Iref[:AOUT_PER_MODULE] = (int(d) for d in data)

        # for the latter 4 channels the "classic" method is used.
        # this relies that the group is never switched,
        # that is the first group not the latter four
        args = [AOUT_READ_BASE + AOUT_MODULE_SIZE, 1+AOUT_PER_GROUP]
        dat2 = self.mb_read( args )
        word0 = dat2[0]
        assert word0==ctrlword(0), 'aout module 2 not switched to first group!'
        self.Iref[AOUT_PER_MODULE:CHANNEL_NUM] =  (int(d) for d in dat2[1:])

    def read_AIN(self):
        '''Reads all analog inputs.
        '''
        # this assures that we do not receive too much or too little inputs
        assert CHANNEL_NUM * 2 == AIN_TOTAL_NUM
        write = self.mb_write
        read = self.mb_read

        # analog input values
        aiva = [ None ] * AIN_TOTAL_NUM
        arg_select = [ AIN_WRITE_BASE, AIN_TOTAL_SIZE ] + \
            [None,0] * AIN_MODULE_NUM
        arg_read = [ AIN_READ_BASE, AIN_TOTAL_SIZE]
        for ch0 in xrange(0, AIN_PER_MODULE_NUM):
            ch_code = ch0*0x100
            arg_select[2::AIN_MODULE_SIZE] = (ch_code,) * AIN_MODULE_NUM
            # selects ch0 of all modules in a single write
            write(arg_select)

            # waits until data is very likely readys
            sleep(READ_DELAY)

            # gets data of all modules in a single read
            data = read(arg_read)
            status_words = data[::AIN_MODULE_SIZE]
            meas = data[1::AIN_MODULE_SIZE]
            while any(s!=ch_code for s in status_words):
                # repeats the read if the status word has not been updated
                if any(t & MASK_ERROR for t in status_words):
                    # checks wether configuration was bad
                    dx = [ "%x" % d for d in data ]
                    self.log.error('error %s while reading modbus - reconfiguration required.', dx)
                    self.need_config = True
                    return aiva

                self.read_repeat_count += 1
                data = read(arg_read)
                meas = data[1::AIN_MODULE_SIZE]
                status_words = data[::AIN_MODULE_SIZE]

            aiva[ch0::AIN_PER_MODULE_NUM] = meas

        return aiva

    def set_Iref(self, ch0, value):
        '''Sets (and writes) a new setpoint for channel 'ch0'.
        '''
        assert AOUT_MIN_VALUE <= value <= AOUT_MAX_VALUE, 'current setpoint %s out of allowed range' % value
        # records setpoint
        self.Iref[ch0] = int(value)
        if self.channel_on[ch0]:
            self._write(ch0)

    def _write_all(self):
        """Causes writes setpoints of all channels
        """
        for ch in range(0, CHANNEL_NUM, AOUT_PER_GROUP):
            self._write(ch)


    def _write(self, ch0):
        '''Causes setpoint of ch0 to be writtenś
        '''
        # pre-conditions
        assert 0 <= ch0 < CHANNEL_NUM, "channel number must be between 0 and %s" % CHANNEL_NUM
        self.write_count += 1

        # indices determining what to write, which ...
        modidx = ch0 / AOUT_PER_MODULE   #< output module
        aoutidx = ch0 % AOUT_PER_MODULE  #< output of that module
        groupidx = aoutidx / AOUT_PER_GROUP  #< group of that module
        # ALL outputs of that group have to be written!

        # where control word and the aout values must be written
        addr = AOUT_WRITE_BASE + modidx*AOUT_MODULE_SIZE

        # which channels have to be written
        ch_start = modidx*AOUT_PER_MODULE + groupidx*AOUT_PER_GROUP
        ch_end = ch_start+AOUT_PER_GROUP
        assert ch_start <= ch0 < ch_end
        Iref, Izero, on = self.Iref, self.Izero, self.channel_on
        values = [ (Iref[ch] if on[ch] else Izero[ch]) for ch in range(ch_start,ch_end) ]
        missing_channels = [ self.name[ch] for ch,v in enumerate(values) if v is None]
        if missing_channels:
            name = self.name[ch0]
            miss = ', '.join(missing_channels)
            raise ModMuxException('can not write %s, missing setpoints for %s' % (name, miss))
        args = [ addr, 1+AOUT_PER_GROUP, AOUT_GROUP_CODE[groupidx] ] + values
        self.log.debug('writing aout %r',args)
        self.mb_write( args )

    def summary(self):
        if self.st_on:
            status = 'ON'
        elif self.st_ready and self.need_config:
            status= "READY but not configured"
        elif self.st_ready:
            status = 'READY'
        else:
            status = 'FAULT'

        FMT = "%2s %6s %6s %-9s %6s %-9s\n"
        FMTi = "%2d "+FMT
        HEAD =  "   "+FMT % ('on', "Iref", "Imeas", "status", "Vmeas", "status" )
        ist = extract_ain_status(self.Imeas)
        ust = extract_ain_status(self.Vmeas)
        oo = [ '?' if o is None else int(o) for o in self.channel_on ]
        z = zip(range(0, CHANNEL_NUM), oo, self.Iref, self.Imeas, ist, self.Vmeas, ust)
        ch_status = ( (FMTi % t) for t in z )
        return status + '\n' + \
            HEAD + "".join(ch_status) + "--\n" + \
            "%d updates, %d reads repeated, %d writes  \n" % \
                (self.jiffies, self.read_repeat_count, self.write_count)


if __name__ == "__main__":
    log_fmt = '%(name)s %(levelname)-.1s%(levelname)-.1s %(message)s'
    logging.basicConfig(level=logging.DEBUG, format=log_fmt)
    log_name = "modmux"
    log = logging.getLogger(log_name)
    log.setLevel(logging.INFO)
    device_name = 'lt01/ct/pc-cor-modbus'
    modmux = ModMux()
    modmux.connect(device_name)
    modmux.set_timeout_millis_df(2000)
#    modmux.fetch()
#    nap()
#     modmux.configure()
#    modmux.power(True)
    for c in range(0,CHANNEL_NUM):
        modmux.set_Iref(c, c*1000)
        modmux.switch_channel_on(c)
    sleep(1)
    modmux._switch_power(1)
    modmux.update()
#    modmux.powermode = PM_STANDBY
#    modmux.set_Iref(0, 15*1000)
#    modmux.switch_channel_on(0)
#    sleep(3)
#    modmux.update()
#    for i in range(0,10):
#        modmux.update()
    print modmux.summary()


