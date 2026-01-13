import sys
import ctypes
from ctypes import *
from ctypes import wintypes

sys.path.append(
    r"D:\Desktop\RoboMaster\RoboticArm\ROKAE\Hik-Camera\GrabImage"
)

from MvCameraControl_class import *

class HikCamera:
    def __init__(self, dev_index=0):
        MvCamera.MV_CC_Initialize()

        self.device_list = MV_CC_DEVICE_INFO_LIST()
        tlayerType = (MV_GIGE_DEVICE | MV_USB_DEVICE |
                      MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE)

        ret = MvCamera.MV_CC_EnumDevices(tlayerType, self.device_list)
        if ret != 0 or self.device_list.nDeviceNum == 0:
            raise RuntimeError("Enum devices failed or no device found, ret=0x%x" % ret)

        print("Found %d devices, use index %d" % (self.device_list.nDeviceNum, dev_index))

        self.cam = MvCamera()
        stDeviceList = cast(self.device_list.pDeviceInfo[dev_index],
                            POINTER(MV_CC_DEVICE_INFO)).contents

        ret = self.cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            raise RuntimeError("CreateHandle failed, ret=0x%x" % ret)

        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise RuntimeError("OpenDevice failed, ret=0x%x" % ret)

        if stDeviceList.nTLayerType == MV_GIGE_DEVICE or stDeviceList.nTLayerType == MV_GENTL_GIGE_DEVICE:
            nPacketSize = self.cam.MV_CC_GetOptimalPacketSize()
            if int(nPacketSize) > 0:
                self.cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)

        self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_ON)
        self.cam.MV_CC_SetEnumValue("TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE)

        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise RuntimeError("StartGrabbing failed, ret=0x%x" % ret)

        self._display_hwnd = None
        self._display_stop = False
        self._wndproc = None

        print("HikCamera initialized OK.")

    def _create_display_window(self, width, height, title):
        user32 = ctypes.windll.user32
        hinstance = ctypes.windll.kernel32.GetModuleHandleW(None)
        class_name = "HikCamDisplayWindow"

        WNDPROCTYPE = ctypes.WINFUNCTYPE(ctypes.c_long, wintypes.HWND, ctypes.c_uint, wintypes.WPARAM, wintypes.LPARAM)

        def _wndproc(hwnd, msg, wparam, lparam):
            if msg == 0x0010:
                user32.DestroyWindow(hwnd)
                return 0
            if msg == 0x0002:
                user32.PostQuitMessage(0)
                return 0
            return user32.DefWindowProcW(hwnd, msg, wparam, lparam)

        self._wndproc = WNDPROCTYPE(_wndproc)

        class WNDCLASS(ctypes.Structure):
            _fields_ = [
                ("style", ctypes.c_uint),
                ("lpfnWndProc", WNDPROCTYPE),
                ("cbClsExtra", ctypes.c_int),
                ("cbWndExtra", ctypes.c_int),
                ("hInstance", wintypes.HINSTANCE),
                ("hIcon", wintypes.HANDLE),
                ("hCursor", wintypes.HANDLE),
                ("hbrBackground", wintypes.HANDLE),
                ("lpszMenuName", wintypes.LPCWSTR),
                ("lpszClassName", wintypes.LPCWSTR),
            ]

        wndclass = WNDCLASS()
        wndclass.style = 0
        wndclass.lpfnWndProc = self._wndproc
        wndclass.cbClsExtra = 0
        wndclass.cbWndExtra = 0
        wndclass.hInstance = hinstance
        wndclass.hIcon = None
        wndclass.hCursor = user32.LoadCursorW(None, 32512)
        wndclass.hbrBackground = 5
        wndclass.lpszMenuName = None
        wndclass.lpszClassName = class_name

        user32.RegisterClassW(ctypes.byref(wndclass))

        hwnd = user32.CreateWindowExW(
            0,
            class_name,
            title,
            0x00CF0000,
            100,
            100,
            width,
            height,
            None,
            None,
            hinstance,
            None,
        )
        if not hwnd:
            raise RuntimeError("CreateWindowExW failed")

        user32.ShowWindow(hwnd, 1)
        user32.UpdateWindow(hwnd)
        return hwnd

    def display_stream(self, width=800, height=600, title="HikCamera Display"):
        self._display_stop = False
        self._display_hwnd = self._create_display_window(width, height, title)

        self.cam.MV_CC_StopGrabbing()
        self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise RuntimeError("StartGrabbing failed, ret=0x%x" % ret)

        user32 = ctypes.windll.user32
        msg = wintypes.MSG()
        stOutFrame = MV_FRAME_OUT()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))

        while not self._display_stop:
            ret = self.cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
            if ret == 0 and bool(stOutFrame.pBufAddr):
                info = stOutFrame.stFrameInfo
                stDisplayParam = MV_DISPLAY_FRAME_INFO()
                memset(byref(stDisplayParam), 0, sizeof(stDisplayParam))
                stDisplayParam.hWnd = int(self._display_hwnd)
                stDisplayParam.nWidth = info.nWidth
                stDisplayParam.nHeight = info.nHeight
                stDisplayParam.enPixelType = info.enPixelType
                stDisplayParam.pData = stOutFrame.pBufAddr
                stDisplayParam.nDataLen = info.nFrameLen
                self.cam.MV_CC_DisplayOneFrame(stDisplayParam)
                self.cam.MV_CC_FreeImageBuffer(stOutFrame)

            while user32.PeekMessageW(ctypes.byref(msg), 0, 0, 0, 1):
                if msg.message == 0x0012:
                    self._display_stop = True
                    break
                user32.TranslateMessage(ctypes.byref(msg))
                user32.DispatchMessageW(ctypes.byref(msg))

        if self._display_hwnd:
            user32.DestroyWindow(self._display_hwnd)
            self._display_hwnd = None

        self.cam.MV_CC_StopGrabbing()
        self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_ON)
        self.cam.MV_CC_SetEnumValue("TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE)
        self.cam.MV_CC_StartGrabbing()

    def set_trigger_mode(self, on):
        self.cam.MV_CC_StopGrabbing()
        self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_ON if on else MV_TRIGGER_MODE_OFF)
        if on:
            self.cam.MV_CC_SetEnumValue("TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE)
        self.cam.MV_CC_StartGrabbing()

    def _grab_frame(self, trigger):
        import numpy as np
        import cv2
        from ctypes import byref, sizeof, c_ubyte, memset

        if trigger:
            ret = self.cam.MV_CC_SetCommandValue("TriggerSoftware")
            if ret != 0:
                print("Trigger failed:", ret)
                return None

        stOutFrame = MV_FRAME_OUT()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))

        ret = self.cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
        if ret != 0:
            print("GetImageBuffer failed:", ret)
            return None

        try:
            buf_addr = ctypes.cast(stOutFrame.pBufAddr, ctypes.c_void_p).value
            if not buf_addr:
                print("Empty buffer address")
                return None
            width = stOutFrame.stFrameInfo.nWidth
            height = stOutFrame.stFrameInfo.nHeight
            data_len = stOutFrame.stFrameInfo.nFrameLen
            pixel_type = stOutFrame.stFrameInfo.enPixelType

            ctypes_data = (c_ubyte * data_len).from_address(buf_addr)
            img_data = np.frombuffer(ctypes_data, dtype=np.uint8)

            if pixel_type == 0x01080001:
                image = img_data.reshape((height, width))
            elif pixel_type == 0x01080009:
                raw = img_data.reshape((height, width))
                image = cv2.cvtColor(raw, cv2.COLOR_BayerRG2BGR)
            elif pixel_type == 0x02180014:
                raw = img_data.reshape((height, width, 3))
                image = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
            else:
                print("Unknown pixel type: %s, try Mono reshape" % hex(int(pixel_type)))
                try:
                    image = img_data.reshape((height, width))
                except Exception as e:
                    print("Reshape failed:", e)
                    image = None

            return image
        finally:
            self.cam.MV_CC_FreeImageBuffer(stOutFrame)

    def capture_once(self):
        return self._grab_frame(trigger=True)

    def grab_frame(self):
        return self._grab_frame(trigger=False)

    def close(self):
        try:
            self.cam.MV_CC_StopGrabbing()
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
        finally:
            MvCamera.MV_CC_Finalize()
            print("HikCamera closed.")
