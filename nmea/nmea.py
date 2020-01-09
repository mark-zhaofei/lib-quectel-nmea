"""
.. module:: nmea

***********
NMEA Module
***********

This module implements a Zerynth driver and parser for a generic NMEA GNSS receiver.


The following functionalities are implemented:

    * retrieve the current location fix if present
    * retrieve the current UTC time

The driver supports reading NMEA sentences from a serial port only.

Location fixes are obtained by parsing NMEA sentences of type RMC and GGA, and optionally GSA.
Obtaining a fix or UTC time are thread safe operations.

    """

import streams
import threading

@c_native("_nmea_readline",["csrc/nmea.c"])
def readline(serial,buffer,timeout=5000):
    """
.. method:: readline(serial,buffer,timeout=5000)

    Wait for a full NMEA sentence from the specified *serial* interface and copy it
    to the specified *buffer* (bytearray), with optional *timeout* (in milliseconds).

    Returns the length of the NMEA sentence or a negative error code:

    * *-1*, if the line does not start with NMEA header (missing :samp:`'$'`)
    * *-2*, if the line has incomplete NMEA sentence (missing  :samp:`'*'`)
    * *-3*, if the line has invalid or mismatching NMEA checksum

    """
    pass

@c_native("_nmea_parseline",["csrc/nmea.c"])
def parseline(buffer,length,tm,fix):
    """
.. method:: parseline(buffer,length,tm,fix)

    Parse the content of the specified line *buffer* (bytearray) up to *length* bytes
    and fill the two sequences *tm* and *fix* with date/time and fix data if available.

    Returned value is *0* if the line does not have valid data, or a combination (sum) of:
    
    * *4*, if the *tm* sequence (7 items) has been filled (from RMC sentence)
    * *1, 2 or 3*, if the *fix* sequence (9 items) has been filled (from RMC, GGA or GSA respectively)

    """
    pass

class NMEA_Receiver():
    """

.. class:: NMEA_Receiver()

    This class is meant to be used as a base class to provide a uniform interface for GNSS receivers.

    Instances of this class are fed with NMEA sentences using the :any:`parse` method and can be
    queried for UTC time and location data. The parser can be disabled to clear acquired data and
    to prevent further updates until it is enabled again (to avoid stale data).

    """

    def __init__(self):
        self.debug = False
        self._tm = [0,0,0,0,0,0,0]
        self._lat = 0
        self._lon = 0
        self._spd = 0
        self._cog = 0
        self._nsat = 0
        self._hdop = 0
        self._vdop = 0
        self._pdop = 0
        self._alt  = 0
        self._cfix = [None]*10
        self._time = None
        self._fix = None
        self._rmc = False
        self._gga = False
        self._gsa = False
        self._utc = False
        self._lock = threading.Lock()
        self._enabled = True


    def print_d(self,*args):
        if self.debug:
            print('>', *args)


    def fix(self):
        """
.. method:: fix()

        Return the current fix or *None* if not available.
        A fix is a tuple with the following elements:

            * latitude in decimal format (-89.9999 - 89.9999)
            * longitude in decimal format (-179.9999 - 179.9999)
            * altitude in meters
            * speed in Km/h
            * course over ground as degrees from true north
            * number of satellites for this fix
            * horizontal dilution of precision (0.5 - 99.9)
            * vertical dilution of precision (0.5 - 99.9)
            * positional dilution of precision (0.5 - 99.9)
            * UTC time as a tuple (yyyy,MM,dd,hh,mm,ss,microseconds)

        """
        self._lock.acquire()
        r = self._fix
        self._fix = None
        self._lock.release()
        return r

    def has_fix(self):
        """
.. method:: has_fix()
    
        Return *True* if a fix is available

        """
        self._lock.acquire()
        r = self._fix
        self._lock.release()
        return r is not None

    def utc(self):
        """
.. method:: utc()

        Return the current UTC time or *None* if not available.
        A UTC time is a tuple of (yyyy,MM,dd,hh,mm,ss,microseconds).

        UTC time can be wrong if no fix has ever been obtained.
        """
        r= None
        self._lock.acquire()
        if self._time:
            r = self._time
            self._time = None
        self._lock.release()
        return r

    def has_utc(self):
        """
.. method:: has_utc()
    
        Return *True* if a UTC time is available

        """
        self._lock.acquire()
        r = self._time
        self._lock.release()
        return r is not None

    ##################### Private

    def enable(self,state):
        """
.. method:: enable(state)
    
        Enable or disable the NMEA parser. Also clear any acquired position fix or UTC data when disabled.

        """
        self._lock.acquire()
        self._enable = state
        if not state:
            self._time = None
            self._fix = None
        self._lock.release()

    def parse(self,buffer,count):
        """
.. method:: parse(buffer,count)
    
        Parse *count* bytes from the specified *buffer* (bytearray) and updates the internal state
        from valid NMEA sentences found (when enabled).

        """
        r = parseline(buffer,count,self._tm,self._cfix)
        #print("R",r)
        if r&4:
            # we have time
            self._lock.acquire()
            if self._enable:
                self._time = (self._tm[0],self._tm[1],self._tm[2],self._tm[3],self._tm[4],self._tm[5],self._tm[6])
            self._lock.release()
        r=r&3
        if r==1:
            self._rmc=True
        elif r==2:
            self._gga=True
        elif r==3:
            self._gsa=True

        #print(self._rmc,self._gga,self._gsa,r)
        if self._rmc and self._gga:
            # we have a fix
            if not self._gsa:
                # optional data VDOP, PDOP
                self._cfix[7] = None
                self._cfix[8] = None
            self._lock.acquire()
            if self._enable:
                self._fix = (
                        self._cfix[0],
                        self._cfix[1],
                        self._cfix[2],
                        self._cfix[3],
                        self._cfix[4],
                        self._cfix[5],
                        self._cfix[6],
                        self._cfix[7],
                        self._cfix[8],
                        (self._tm[0],self._tm[1],self._tm[2],self._tm[3],self._tm[4],self._tm[5],self._tm[6])
                        )
            self._rmc = False
            self._gga = False
            self._gsa = False
            self._lock.release()
