#/***********************************************************************
# * 
# * 文件名   hu_m40.py
# * 作者     hche
# * E-mail   
# * 日期     2025/06/30
# * 版本     V0.1.0
# * 说明     HU遥控手柄解码
# * 
# ************************************************************************
# * 金华市核芯风暴智能科技有限公司
# ************************************************************************/

"""HU_M40 driver for MicroPython"""

from machine import Pin, SPI
import nrf24l01
import utime
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


HU_M40_TIMEOUT_CNT = const(30000)   # 根据不同平台配置通信超时时间，一般配置成2秒
HU_M40_PAYLOAD_WIDTH = const(16)



class HU_M40:

    def __init__(self):

        self.HU_M40_ADDR_DEF = (b"HXFB0")
        self.rx_data = bytearray(16)
        self.last_buttons=0
        self.buttons=0
        self.rx_addr = (b"HXFB1") # 地址码，有冲突时修改此地址
        self.tx_addr=self.HU_M40_ADDR_DEF
        self.ticks=0
        self.nrf=0
    
#/*-------------------------------------------
#
#         以下根据不同平台自行实现  
#         
#----------------------------------------------*/
    
    def init_hw(self):
        CE_PIN = 7
        CSN_PIN = 15
        SCK_PIN = 17
        MOSI_PIN = 16
        MISO_PIN = 18
        
        # 初始化 SPI 接口
        spi = SPI(2, baudrate=1000000, polarity=0, phase=0, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))


        csn = Pin(CSN_PIN, mode=Pin.OUT, value=1)  # Chip Select Not
        ce = Pin(CE_PIN, mode=Pin.OUT, value=0)   # Chip Enable

        # 初始化 NRF24L01
        self.nrf = nrf24l01.NRF24L01(spi, csn, ce, channel=25, payload_size=HU_M40_PAYLOAD_WIDTH)
        self.nrf.reg_write(0x04, (8 << 4) | 8)
        
        
    def tx_mode(self,nrf,txaddr,rxaddr):
        nrf.open_tx_pipe(txaddr)
        nrf.open_rx_pipe(0, rxaddr)
        
    def rx_mode(self,nrf,txaddr,rxaddr):
        nrf.open_tx_pipe(txaddr)
        nrf.open_rx_pipe(0, rxaddr)
        nrf.start_listening()

    def send_data(self,nrf,buf,length):
        nrf.send(buf,200)
        #nrf.send_start(buf)
        #start = utime.ticks_ms()
        #result = None
        #while result is None and utime.ticks_diff(utime.ticks_ms(), start) < 100:
        #    result = nrf.send_done()  # 1 == success, 2 == fail


    def received_data(self,nrf,buf,length):
        if nrf.any():
            buf1=nrf.recv() # 接收数据
            nrf.stop_listening()
            for i in range(16):
                buf[i] = buf1[i]
            
            return 1
        return 0
    
#/*-------------------------------------------
#
#         一般情况下以下不用修改
#         
#----------------------------------------------*/
    def button(self, button):
        return ((self.buttons & button) > 0);
        
    def analog(self, button):
        return self.rx_data[button]
        
    def check_sum(self,buf, length):
        sum=0
        for i in range(length):
            sum += buf[i]
        
        return sum&0xFF
        
        
    def init(self):
        #print("hu_m40_init")
        self.init_hw()
        nrf = self.nrf
        
        self.rx_mode(nrf,self.HU_M40_ADDR_DEF,self.HU_M40_ADDR_DEF)
        self.tx_addr=self.HU_M40_ADDR_DEF
        
        for i in range(16):
            self.rx_data[i]=0
        
        self.rx_data[HU_RX] = 0x80;
        self.rx_data[HU_RY] = 0x80;
        self.rx_data[HU_LX] = 0x80;
        self.rx_data[HU_LY] = 0x80;
        
        self.buttons = 0;
        
        
        
    def read(self):
        
        nrf = self.nrf
        res = 0
        dat = bytearray(16)
        
        
        self.ticks+=1
        
        if self.ticks > HU_M40_TIMEOUT_CNT:  # 通信超时
        
            self.ticks = 0;

            self.init() # 重新初始化

            res = 0
        

        if self.received_data( self.nrf, dat, HU_M40_PAYLOAD_WIDTH ) :
            #print("收到数据:", dat)
            #print("dat[0] == 1:", dat[0] == 1)
            #print("dat[1] == 3:", dat[1] == 3)
            #print("dat[15]:", dat[15])
            #print("self.check_sum( dat, 15 ):", self.check_sum( dat, 15 ))
            
            if dat[0] == 1 and dat[1] == 3 and dat[15] == self.check_sum( dat, 15 ) : # 校验成功
            
              
                  for i in range(16):
                  
                      self.rx_data[i] = dat[i];
                  

                  self.buttons = self.rx_data[8];
                  
                  res = 1;
                  self.ticks = 0;
                  
                  # 返回数据到遥控器
                  dat[0] = 0x01; # 固定为1
                  dat[1] = 0x83; # 功能码
                  dat[2] = dat[2]; # 序号
                  dat[3] = 11; # 长度
                  dat[4] = self.rx_addr[0];
                  dat[5] = self.rx_addr[1];
                  dat[6] = self.rx_addr[2];
                  dat[7] = self.rx_addr[3];
                  dat[8] = self.rx_addr[4];
                  dat[15] = self.check_sum( dat, 15 );
                  
                  #print("tx_addr:", self.tx_addr)
                  self.tx_mode( nrf, self.tx_addr, self.tx_addr );
                  self.send_data(nrf, dat, HU_M40_PAYLOAD_WIDTH);         #发送数据
                  #print("发送数据:", dat)
                  
                  
                  self.rx_mode( nrf, self.rx_addr, self.rx_addr );
                  self.tx_addr = self.rx_addr;
        return res




__version__ = '0.1.0'


