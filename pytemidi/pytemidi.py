import ctypes
import os
import struct
import threading
import time
import weakref
from comtypes import GUID
import inspect
import logging
from ctypes import wintypes as wintypes

log = logging.getLogger(__name__)

py_bitness = struct.calcsize("P") * 8
libName = "teVirtualMIDI64.dll" if py_bitness == 64 else "teVirtualMIDI.dll"
try:
    midiDll = ctypes.WinDLL(os.path.join(os.environ['WINDIR'], "system32", libName))
except FileNotFoundError as e:
    logging.exception("  DLL not found, ensure you've installed loopMIDI or rtpMIDI from\n"
                      "http://www.tobias-erichsen.de/software/\n"
                      "  This library is dependent on it.")

# Bits in Mask to enable logging for specific areas
LOGGING_MISC = 1  # log internal stuff (port enable, disable...)
LOGGING_RX = 2  # log data received from the driver
LOGGING_TX = 4  # log data sent to the driver

def realaddr(pointer):
    return ctypes.cast(pointer, ctypes.c_void_p).value

# This is the size of the buffer that is being used for communication
# with the driver when instanciating a port with the old, deprecated
# "virtualMIDICreatePort" function.  This value is currently 128kb - 1,
# but may change anytime in the future.  This value also limits the
# maximum size of received sysex-data due to the parser in the merger-
# module in the driver.
DEFAULT_BUFFER_SIZE = 0x1fffe

# Bits in Mask to virtualMIDICreatePortEx2
FLAGS_PARSE_RX = 1  # tells the driver to always provide valid preparsed MIDI-commands
FLAGS_PARSE_TX = 2  # tells the driver to parse all data received via virtualMIDISendData
FLAGS_INSTANTIATE_RX_ONLY = 4  # Only the "midi-out" part of the port is created
FLAGS_INSTANTIATE_TX_ONLY = 8  # Only the "midi-in" part of the port is created
FLAGS_INSTANTIATE_BOTH = 12  # a bidirectional port is created

FLAGS_SUPPORTED = FLAGS_PARSE_RX | FLAGS_PARSE_TX | FLAGS_INSTANTIATE_RX_ONLY | FLAGS_INSTANTIATE_TX_ONLY


class MIDIPort(ctypes.Structure):
    """Wrapper for the VM_MIDI_PORT C struct."""
    pass


MIDI_PORT = ctypes.POINTER(MIDIPort)  # Just a pointer, whatever is underneath, we don't care

# Callback interface.  This callback is called by the driver/interface-dll for a packet of MIDI-data that is received
# from the driver by the application using the virtual MIDI-port.
#
# This callback is called in an arbitrary thread-context - so make sure you have all your locking in order!
#
# If you have created the virtual-MIDI-port and specified TE_VM_FLAGS_PARSE_RX in the flags parameter, you will
# receive a fully valid, preparsed MIDI-command with each callback.  The maximum size of data will be the amount
# you specified in maxSysexLength.  Invalid commands or Sysex-commands with a length in excess of maxSysexLength
# will be discarded and not forwarded to you.  Realtime-MIDI-commands will never be "intermingled" with other
# commands (either normal or Sysex) in this mode.  If a realtime-MIDI-command is detected, it is sent to the
# application before the command that it was intermingled with.
#
# In case of the driver being deactivated, the callback is called one time with a midiDataBytes==NULL and
# length==zero, either the driver has been disabled, or another application using the driver has started
# the installation of a newer driver-version
#
# You can throttle the speed of your virtualMIDI-port by not returning immediately from
# this callback after you have taken care of the data received.
#
# If you want to throttle to 31250 bps for example, you need to place this line
# before you return from your callback-function:
# Sleep( length * 10 * 1000) / 31250 );
#
# LPVM_MIDI_DATA_CB(LPVM_MIDI_PORT, midiDataBytes, length, dwCallbackInstance)
bytep = ctypes.POINTER(ctypes.c_uint8)
MIDI_DATA_CB = ctypes.WINFUNCTYPE(None, MIDI_PORT, bytep, wintypes.DWORD, wintypes.PDWORD)

# virtualMIDICreatePortEx2 - this is the current intended function to create a virtual MIDI-port.
#
# You can specify a name for the device to be created. Each named port can only exist once on a system.
#
# When the application terminates, the port will be deleted (or if the public front-end of the port is already in use by
# a DAW-application, it will become inactive - giving back apropriate errors to the application using this port.
#
# In addition to the name, you can supply a callback-interface, which will be called for all MIDI-data received by the
# virtual-midi port. You can also provide instance-data, which will also be handed back within the callback, to have the
# ability to reference port-specific data-structures within your application.
#
# If you specify "NULL" for the callback function, you will not receive any callback, but can call the blocking function
# "virtualMIDIGetData" to retrieve received MIDI-data/commands.  This is especially useful if one wants to interface
# this library to managed code like .NET or Java, where callbacks into managed code are potentially complex or
# dangerous.  A call to virtualMIDIGetData when a callback has been set during the creation will return with
# "ERROR_INVALID_FUNCTION".
#
# If you specified TE_VM_FLAGS_PARSE_RX in the flags parameter, you will always get one fully valid, preparsed
# MIDI-command in each callback. In maxSysexLength you should specify a value that is large enough for the maximum size
# of Sysex that you expect to receive.  Sysex-commands larger than the value specified here will be discarded and not
# sent to the user.  Realtime-MIDI-commands will never be "intermingled" with other commands (either normal or Sysex)
# in this mode.  If a realtime-MIDI-command is detected, it is sent to the application before the command that it was
# intermingled with.
#
# If you specify a maxSysexLength smaller than 2, you will receive fully valid preparsed MIDI-commands, but no
# Sysex-commands, since a Sysex-command must be at least composed of 0xf0 + 0xf7 (start and end of sysex).  Since the
# parser will never be able to construct a valid Sysex, you will receive none - but all other MIDI-commands will be
# parsed out and sent to you.
#
# When a NULL-pointer is handed back to the application, creation failed.  You can check GetLastError() to find out the
# specific problem  why the port could not be created.
# LPVM_MIDI_PORT CALLBACK virtualMIDICreatePortEx2( LPCWSTR portName, LPVM_MIDI_DATA_CB callback,
# DWORD_PTR dwCallbackInstance, DWORD maxSysexLength, DWORD flags );

# virtualMIDICreatePortEx3
#
# This version of the function adds the ability to provide two pointers to GUIDs that can be used to set the
# manufacturer and product guids that an application using the public port can retrieve using midiInGetDevCaps or
# midiOutGetDevCaps with the extended device-capability-structures (MIDIINCAPS2 and MIDIOUTCAPS2).  If those pointers
# are set to NULL, the default-guids of the teVirtualMIDI-driver will be used.
#
# LPVM_MIDI_PORT CALLBACK virtualMIDICreatePortEx3( LPCWSTR portName, LPVM_MIDI_DATA_CB callback,
# DWORD_PTR dwCallbackInstance, DWORD maxSysexLength, DWORD flags, GUID *manufacturer, GUID *product );
virtualMIDICreatePortEx3 = midiDll.virtualMIDICreatePortEx3
virtualMIDICreatePortEx3.restype = MIDI_PORT
virtualMIDICreatePortEx3.argtypes = [wintypes.LPCWSTR,
                                     MIDI_DATA_CB,
                                     wintypes.PDWORD,
                                     wintypes.DWORD,
                                     wintypes.DWORD,
                                     ctypes.POINTER(GUID),
                                     ctypes.POINTER(GUID)]


# With this function, you can close a virtual MIDI-port again, after you have instanciated it.
#
# After the return from this function, no more callbacks will be received.
#
# Beware: do not call this function from within the midi-port-data-callback.  This may result in a deadlock!

virtualMIDIClosePort = midiDll.virtualMIDIClosePort
virtualMIDIClosePort.restype = None
virtualMIDIClosePort.argtypes = [MIDI_PORT]

# With this function you can send a buffer of MIDI-data to the driver / the application that opened the
# virtual-MIDI-port. If this function returns false, you may check GetLastError() to find out what caused the problem.
#
# This function should always be called with a single complete and valid midi-command (1-3 octets, or possibly more
# for sysex).  Sysex-commands should not be split!  Realtime-MIDI-commands shall not be intermingled with other MIDI-
# commands, but sent seperately!
#
# The data-size that can be used to send data to the virtual ports may be limited in size to prevent
# an erratic application to allocate too much of the limited kernel-memory thus interfering with
# system-stability.  The current limit is 512kb.
#
# BOOL CALLBACK virtualMIDISendData( LPVM_MIDI_PORT midiPort, LPBYTE midiDataBytes, DWORD length );

virtualMIDISendData = midiDll.virtualMIDISendData
virtualMIDISendData.restype = wintypes.BOOL
virtualMIDISendData.argtypes = [MIDI_PORT, wintypes.LPBYTE, wintypes.DWORD]

# With this function you can use virtualMIDI without usage of callbacks.  This is especially interesting
# if you want to interface the DLL to managed environments like Java or .NET where callbacks from native
# to managed code are more complex.
#
# To use it, you need to open a virtualMIDI-port specifying NULL as callback.  If you have specified a
# callback when opening the port, this function will fail - you cannot mix callbacks & reading via this
# function.
#
# You need to provide a buffer large enough to retrieve the amount of data available.  Otherwise the
# function will fail and return to you the necessary size in the length parameter.  If you specify
# midiDataBytes to be NULL, the function will succeed but only return the size of buffer necessary
# to retrieve the next MIDI-packet.
#
# virtualMIDIGetData will block until a complete block of data is available.  Depending on the fact if
# you have specified to parse data into valid commands or just chunks of unprocessed data, you will
# either receive the unparsed chunk (possibly containing multiple MIDI-commands), or a single, fully
# valid MIDI-command.  In both cases, the length parameter will be filled with the length of data retrieved.
#
# You may only call virtualMIDIGetData once concurrently.  A call to this function will fail if another
# call to this function is still not completed.
#
# BOOL CALLBACK virtualMIDIGetData( LPVM_MIDI_PORT midiPort, LPBYTE midiDataBytes, PDWORD length );



# With this function an application can find out the process-ids of all applications
# that are currently using this virtual MIDI port
# A pointer to an array of ULONG64s must be supplied.  Currently no more than 16 process ids are returned
# Before calling the length is the size of the buffer provided by the application in bytes
# After calling the length is the number of process-ids returned times sizeof(ULONG64)
#
# BOOL CALLBACK virtualMIDIGetProcesses( LPVM_MIDI_PORT midiPort, ULONG64 *processIds, PDWORD length );

virtualMIDIGetProcesses = midiDll.virtualMIDIGetProcesses
virtualMIDIGetProcesses.restype = wintypes.BOOL
virtualMIDIGetProcesses.argtypes = [MIDI_PORT, ctypes.POINTER(ctypes.c_uint64), wintypes.PDWORD]

# With this function you can abort the created midiport.  This may be useful in case you want
# to use the virtualMIDIGetData function which is blocking until it gets data.  After this
# call has been issued, the port will be shut-down and any further call (other than virtualMIDIClosePort)
# will fail
#
# BOOL CALLBACK virtualMIDIShutdown( LPVM_MIDI_PORT midiPort );
virtualMIDIShutdown = midiDll.virtualMIDIShutdown
virtualMIDIShutdown.restype = None
virtualMIDIShutdown.argtypes = [MIDI_PORT]


class DriverError(IOError):
    def __init__(self, errno, additional=""):
        super().__init__(self, f"ERROR({errno}): {additional}")


def shutdown(_id):
    pointer = ctypes.c_void_p(_id)
    port = ctypes.cast(pointer, MIDI_PORT)
    virtualMIDIClosePort(port)
    return virtualMIDIShutdown(port)


GCTimer = 30 * 60  # 30 min


class GCThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True, name="PyTeMIDI.GC")
        self.collection = {}
        self.running = False
        self.unified_callback_ptr = MIDI_DATA_CB(self.unified_callback)

    def collect(self):
        for _id,ref in self.collection.items():
            if ref() is None:
                shutdown(_id)
        self.collection = {k: v for k,v in self.collection.items() if v is not None}

    def add(self, _id, dev):
        self.collection[_id] = weakref.ref(dev)
        if not self.running:
            self.start()

    def remove(self, _id):
        shutdown(_id)
        del self.collection[_id]

    def close_all(self):
        for _id in self.collection:
            shutdown(_id)

    def unified_callback(self, port, midi_bytes, length, additional):
        port_addr = realaddr(port)
        dev = self.collection[port_addr]()
        if dev is not None:
            try:
                dev.call_callbacks(bytes(midi_bytes[:length]) if midi_bytes is not None else None)
            except Exception:
                log.exception("Callback exception encountered")

    def run(self) -> None:
        self.running = True
        import atexit
        atexit.register(self.close_all)
        while True:
            time.sleep(GCTimer)
            self.collect()


GC = GCThread()


class Device(object):
    def __init__(self, name="PyTeVirtualMIDI", manufacturer=None, product=None, raw_stream=False, no_output=False,
                 no_input=False, sysex_size=DEFAULT_BUFFER_SIZE, raw_callback=None):
        self._name = name
        self._manufacturer = manufacturer
        self._product = product
        self._raw_stream = raw_stream
        self._no_input = no_input
        self._no_output = no_output
        self._sysex_size = sysex_size
        self._cb_functions = weakref.WeakSet()
        self._cb_methods = weakref.WeakKeyDictionary()
        self._cb_lk = threading.RLock()
        self._raw_callback = raw_callback
        self._id = None
        self._id_addr = 0
        self._cb = None

    def close(self):
        GC.remove(self._id_addr)
        self._id = None
        self._id_addr = 0

    def send(self, _bytes):
        c_buf = (ctypes.c_byte * len(_bytes)).from_buffer(_bytes)
        ret = virtualMIDISendData(self._id, c_buf, len(_bytes))
        if ret == 0:
            raise DriverError(ctypes.GetLastError(), "couldn't send data")

    def get_clients(self):
        buf = (ctypes.c_uint64*16)(*([0] * 16))
        len = wintypes.DWORD(16*64)
        ret = virtualMIDIGetProcesses(self._id, buf, ctypes.byref(len))
        if ret.value:
            return buf.value[:len.value/64]
        else:
            raise DriverError(ctypes.GetLastError(), "couldn't get client PIDs")

    def call_callbacks(self, midi_bytes):
        with self._cb_lk:
            # Call handler functions
            for func in self._cb_functions:
                func(self, midi_bytes)

            # Call handler methods
            for obj, funcs in self._cb_methods.items():
                for func in funcs:
                    func(obj, self, midi_bytes)

    def register_callback(self, callback):
        """
        Connect a callback to this device if it is not connected already.
        """
        with self._cb_lk:
            if not self.is_registered(callback):
                if inspect.ismethod(callback):
                    method_self = callback.__self__
                    if method_self not in self._cb_methods:
                        self._cb_methods[method_self] = set()

                    self._cb_methods[method_self].add(callback.__func__)
                else:
                    self._cb_functions.add(callback)

    def is_registered(self, callback):
        """
        Check if a callback is connected to this device.
        """
        with self._cb_lk:
            if inspect.ismethod(callback):
                method_self = callback.__self__
                method_func = callback.__func__
                if method_self in self._cb_methods and method_func in self._cb_methods[method_self]:
                    return True
                return False
            return callback in self._cb_functions

    def unregister_callback(self, callback):
        """
        Disconnect a callback from a device if it is connected else do nothing.
        """
        with self._cb_lk:
            if self.is_registered(callback):
                if inspect.ismethod(callback):
                    self._cb_methods[callback.__self__].remove(callback.__func__)
                else:
                    self._cb_functions.remove(callback)

    def started(self):
        return self._id is not None

    def has_output_open(self):
        return not self._no_output

    def has_input_open(self):
        return not self._no_input

    def is_raw(self):
        return self._raw_stream

    def name(self):
        return self._name

    def create(self):
        flags = 0
        if not self._no_input:
            flags |= FLAGS_INSTANTIATE_RX_ONLY
            if not self._raw_stream:
                flags |= FLAGS_PARSE_RX
        if not self._no_output:
            flags |= FLAGS_INSTANTIATE_TX_ONLY
            if not self._raw_stream:
                flags |= FLAGS_PARSE_TX
        muid = None
        if self._manufacturer is not None:
            muid = ctypes.byref(self._manufacturer)
        puid = None
        if self._product is not None:
            puid = ctypes.byref(self._product)
        if self._raw_callback is not None:
            def cb_py(port, midi_bytes, length, additional):
                try:
                    self._raw_callback(midi_bytes[:length] if midi_bytes is not None else None)
                except Exception:
                    log.exception("Callback exception encountered")
            cb = MIDI_DATA_CB(cb_py)
            self._cb = cb
        else:
            cb = GC.unified_callback_ptr
        _id = virtualMIDICreatePortEx3(self._name, cb, None, self._sysex_size, flags, muid, puid)
        if _id is None:
            raise DriverError(ctypes.GetLastError(), "couldn't create the device")
        self._id = _id
        self._id_addr = realaddr(_id)
        GC.add(self._id_addr, self)


# With this function you can retrieve the version of the driver that you are using.
# In addition you will receive the version-number as a wide-string constant as return-value.
#
# LPCWSTR CALLBACK virtualMIDIGetVersion( PWORD major, PWORD minor, PWORD release, PWORD build );
virtualMIDIGetVersion = midiDll.virtualMIDIGetVersion
virtualMIDIGetVersion.restype = wintypes.LPCWSTR
virtualMIDIGetVersion.argtypes = [
    wintypes.PWORD,  # major
    wintypes.PWORD,  # minor
    wintypes.PWORD,  # release
    wintypes.PWORD]  # build


def get_library_version(split=False):
    if split:
        major = wintypes.WORD(0)
        minor = wintypes.WORD(0)
        release = wintypes.WORD(0)
        build = wintypes.WORD(0)
        ret = virtualMIDIGetVersion(ctypes.byref(major),
                                    ctypes.byref(minor),
                                    ctypes.byref(release),
                                    ctypes.byref(build))
        return (major.value,
                minor.value,
                release.value,
                build.value)
    ret = virtualMIDIGetVersion(None,None,None,None)
    return ret


# With this function you can retrieve the version of the driver that you are using.
# In addition you will receive the version-number as a wide-string constant as return-value.
#
# LPCWSTR CALLBACK virtualMIDIGetDriverVersion( PWORD major, PWORD minor, PWORD release, PWORD build );
virtualMIDIGetDriverVersion = midiDll.virtualMIDIGetVersion
virtualMIDIGetDriverVersion.restype = wintypes.LPCWSTR
virtualMIDIGetDriverVersion.argtypes = [wintypes.PWORD,wintypes.PWORD,wintypes.PWORD,wintypes.PWORD]


def get_driver_version(split=False):
    if split:
        major = wintypes.WORD(0)
        minor = wintypes.WORD(0)
        release = wintypes.WORD(0)
        build = wintypes.WORD(0)
        ret = virtualMIDIGetDriverVersion(ctypes.byref(major),
                                          ctypes.byref(minor),
                                          ctypes.byref(release),
                                          ctypes.byref(build))
        return (major.value,
                minor.value,
                release.value,
                build.value)
    ret = virtualMIDIGetDriverVersion(None,None,None,None)
    return ret


# With this function logging can be activated into DbgView.
# Please specify a bitmask made up form binary "or"ed values from TE_VM_LOGGING_XXX
#
# DWORD CALLBACK virtualMIDILogging( DWORD logMask );
virtualMIDILogging = midiDll.virtualMIDILogging
virtualMIDILogging.restype = wintypes.DWORD
virtualMIDILogging.argtypes = [wintypes.DWORD]


def logging(bitmask=0):
    return virtualMIDILogging(bitmask)


if __name__ == "__main__":
    import mido

    def test_cb(_, b):
        msg = mido.Message.from_bytes(b)
        print(msg)
    print(get_driver_version())
    print(get_library_version())
    dev = Device("test")
    dev.register_callback(test_cb)
    logging(LOGGING_MISC|LOGGING_RX|LOGGING_TX)
    dev.create()
    dev.send(mido.Message("note_on", channel=0, note=36, velocity=127).bin())
    dev.close()

