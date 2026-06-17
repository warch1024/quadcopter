
import hu_m40
from micropython import const

HU_K1 = const(0x0001)
HU_K2 = const(0x0002)
HU_K3 = const(0x0004)
HU_K4 = const(0x0008)
HU_K5 = const(0x0010)
HU_K6 = const(0x0020)
HU_RX = const(4)
HU_RY = const(5)
HU_LX = const(6)
HU_LY = const(7)

g_hu_m40 = hu_m40.HU_M40()

g_hu_m40.init()

while True:
    if g_hu_m40.read()==1 :
        if g_hu_m40.button(HU_K1):
            print("hu_m40_button: K1")

        if g_hu_m40.button(HU_K2):
            print("hu_m40_button: K2")

        if g_hu_m40.button(HU_K3):
            print("hu_m40_button: K3")

        if g_hu_m40.button(HU_K4):
            print("hu_m40_button: K4")

        if g_hu_m40.button(HU_K5):
            print("hu_m40_button: K5")

        if g_hu_m40.button(HU_K6):
            print("hu_m40_button: K6")

        print("hu_m40_rx: ",g_hu_m40.analog(HU_RX))
        print("hu_m40_ry: ",g_hu_m40.analog(HU_RY))
        print("hu_m40_lx: ",g_hu_m40.analog(HU_LX))
        print("hu_m40_ly: ",g_hu_m40.analog(HU_LY))
