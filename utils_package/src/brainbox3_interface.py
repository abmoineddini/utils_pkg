#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import String, Int16
import numpy as np
import sys
import socket
import select
import time


class AsciiIo:
    """Example class for communication with Brainboxes ED-range products
      Tested with Python 2.7.9 and 3.4.3 on Windows, and 2.7.6 and 3.4.0 on Linux
    """
    
    def __init__(self, ipaddr, port=9500, timeout=5.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ipaddr, port))
        self.timeout = timeout
        self.recv_chunk_size = 32

    def command_noresponse(self, txmessage):
        self._send(txmessage)

    def command_response(self, txmessage):
        self._send(txmessage)
        return self._receive()
        
    def _send(self, txmessage):
        txmessage = txmessage + b"\r"
        totalsent = 0
        while totalsent < len(txmessage):
            bytes_sent = self.sock.send(txmessage[totalsent:])
            if bytes_sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + bytes_sent

    def _receive(self):
        tstart = time.time()
        data = b''
        endpos = -1
        while endpos < 0:
            tleft = max(0.0, tstart + self.timeout - time.time())
            ready_to_read, ready_to_write, in_error = select.select([self.sock], [], [self.sock], tleft)
            if len(in_error) > 0:
                raise RuntimeError("error on socket connection")
            if len(ready_to_read) > 0:
                chunk = self.sock.recv(self.recv_chunk_size)
                if chunk == b'':
                    raise RuntimeError("socket connection broken")
                data = data + chunk
                endpos = data.find(b'\r')
            elif tleft == 0.0:
                # raise RuntimeError("timeout on socket receive")
                return None
        return data[:endpos]
            
    def _destructor(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
        except:
            # ignore any failures to shutdown and close the socket - it's probably already closed
            pass

    def __del__(self):
        self._destructor()

    def __enter__(self):
        return self
        
    def __exit__(self, type, value, traceback):
        self._destructor() # __del__ gets called anyway, so does this help at all?  Does it get the socket closed sooner?


class BrainboxInpu():
    def __init__(self):
        self.io = AsciiIo(ipaddr='10.5.0.13', port=9500, timeout=1.0)
        # self.pub_Din0 = rospy.Publisher("brainbox3_input/sensor_input0/Estop", Int16, queue_size=10)
        self.pub_Din1 = rospy.Publisher("brainbox3_input/sensor_input1/breakbeam2", Int16, queue_size=10)
        self.pub_Din2 = rospy.Publisher("brainbox3_input/sensor_input2/breakbeam3", Int16, queue_size=10)
        self.pub_Din3 = rospy.Publisher("brainbox3_input/sensor_input3/reset_system", Int16, queue_size=10)
        self.pub_Din4 = rospy.Publisher("brainbox3_input/sensor_input4/breakbeam4", Int16, queue_size=10)
        # self.pub_Din5 = rospy.Publisher("brainbox3_input/sensor_input5/...", Int16, queue_size=10)
        # self.pub_Din6 = rospy.Publisher("brainbox3_input/sensor_input6/...", Int16, queue_size=10)
        # self.pub_Din7 = rospy.Publisher("brainbox3_input/sensor_input7/...", Int16, queue_size=10)
        # self.pub_Din8 = rospy.Publisher("brainbox3_input/sensor_input8/...", Int16, queue_size=10)
        self.pub_conv1 = rospy.Publisher("brainbox3_input/sensor/conv1", Int16, queue_size=10)
        self.pub_conv2 = rospy.Publisher("brainbox3_input/sensor/conv2", Int16, queue_size=10)
        self.pub_conv3 = rospy.Publisher("brainbox3_input/sensor/conv3", Int16, queue_size=10)
        self.pub_conv4 = rospy.Publisher("brainbox3_input/sensor/conv4", Int16, queue_size=10)

        self.function_change = False
        self.msg_Dout_control = ['0','0','0','0','0','0','0','0']
        self.res_Dout_feedback = '00000000'
        
    def callback_brainbox_COM(self, event):
            if self.function_change == False:
                msgin = '@01'
                msg = msgin.encode()

                rxdata = self.io.command_response(msg)
                if rxdata is None:
                    print("  No response received!")
                else:
                    rxdataString = rxdata.decode()
                    if rxdataString != ">":
                        res_Din = "{0:08b}".format(int(rxdataString[3:], 16))
                        self.res_Dout_feedback = "{0:08b}".format(int(rxdataString[1:3], 16))
                        
                        self.pub_Din1.publish(int(res_Din[::-1][1]))
                        self.pub_Din2.publish(int(res_Din[::-1][2]))
                        self.pub_Din3.publish(int(res_Din[::-1][3]))
                        self.pub_Din4.publish(int(res_Din[::-1][4]))
                        # print(int(res_Din[::-1]))
                        self.pub_conv1.publish(int(self.res_Dout_feedback[::-1][0]))
                        self.pub_conv2.publish(int(self.res_Dout_feedback[::-1][1]))
                        self.pub_conv3.publish(int(self.res_Dout_feedback[::-1][2]))
                        self.pub_conv4.publish(int(self.res_Dout_feedback[::-1][3]))
                        
                        # print(int(res_Din[::-1][0]))
                        self.function_change = not self.function_change

            else:
                join_msg_out = '' .join(self.msg_Dout_control)[::-1]
                msg_out = '#0100' + "0x{:02x}".format(int(join_msg_out, 2))[2:]
                msg_out = msg_out.encode()

                rxdata_Dout = self.io.command_response(msg_out)
                if rxdata_Dout is None:
                    print("  No response received!")
                else:
                    rxdataString = rxdata_Dout.decode()
                    self.function_change = not self.function_change

    def callback_Dout0(self, msg):
        message1 = str(msg.data)
        if message1 != self.res_Dout_feedback[7]:
            self.msg_Dout_control[0] = message1

    def callback_Dout1(self, msg):
        message2 = str(msg.data)
        if self.res_Dout_feedback[6] != message2:
            self.msg_Dout_control[1] = message2

    def callback_Dout2(self, msg):
        message3 = str(msg.data)
        if self.res_Dout_feedback[5] != message3:
            self.msg_Dout_control[2] = message3

    def callback_Dout3(self, msg):
        message3 = str(msg.data)
        if self.res_Dout_feedback[4] != message3:
            self.msg_Dout_control[3] = message3
            self.callback_Dout5(message3)   
    
    def callback_Dout4(self, msg):
        message4 = str(msg.data)
        if self.res_Dout_feedback[3] != message4:
            self.msg_Dout_control[4] = message4

    def callback_Dout5(self, msg):
        message6 = msg
        if self.res_Dout_feedback[2] != message6:
            self.msg_Dout_control[5] = message6

    # def callback_Dout6(self, msg):
    #     message = str(msg.data)
    #     if self.res_Dout_feedback[1] != message:
    #         self.msg_Dout_control[6] = message

    # def callback_Dout7(self, msg):
    #     message = str(msg.data)
    #     if self.res_Dout_feedback[0] != message:
    #         self.msg_Dout_control[7] = message


if __name__ == '__main__':
    rospy.init_node("brainbox3_interface_node", anonymous=False)
    bbox_input = BrainboxInpu()
    rospy.loginfo("brainbox3 operational")
    rospy.Timer(rospy.Duration(1/100), bbox_input.callback_brainbox_COM)
    bbox_input_sub0 = rospy.Subscriber('brainbox3_control/Dout0/conv_1' , Int16, bbox_input.callback_Dout0)
    bbox_input_sub1 = rospy.Subscriber('brainbox3_control/Dout1/conv_2' , Int16, bbox_input.callback_Dout1)
    bbox_input_sub2 = rospy.Subscriber('brainbox3_control/Dout2/conv_3' , Int16, bbox_input.callback_Dout2)
    bbox_input_sub3 = rospy.Subscriber('brainbox3_control/Dout3/conv_4' , Int16, bbox_input.callback_Dout3)
    bbox_input_sub4 = rospy.Subscriber('brainbox3_control/Dout4/conv_6' , Int16, bbox_input.callback_Dout4)
    # bbox_input_sub5 = rospy.Subscriber('brainbox3_control/Dout5/conv_3_blower' , Int16, bbox_input.callback_Dout5)
    # bbox_input_sub6 = rospy.Subscriber('brainbox3_control/Dout6/6' , Int16, bbox_input.callback_Dout6)
    # bbox_input_sub7 = rospy.Subscriber('brainbox3_control/Dout7/7' , Int16, bbox_input.callback_Dout7)

    
    rospy.spin()
