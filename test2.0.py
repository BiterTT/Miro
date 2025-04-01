#!/usr/bin/python3
#
#	@section COPYRIGHT
#	Copyright (C) 2023 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, and a complete
#	definition of "The Software", see LICENSE in the MDK root
#	directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	"The Software" in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of "The
#	Software" to you; the license granted to you is not a sale.
#	This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of "The
#	Software".
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#	

# create node
import rospy
rospy.init_node("client_minimal", anonymous=True)

################################################################

# to use in MIROcode, copy everything below this line into the
# MIROcode Python editor.
#
# vvvvvv vvvvvv vvvvvv vvvvvv

################################################################

import os
import sys
import time
import numpy as np
import math

import miro2 as miro
import geometry_msgs
from geometry_msgs import *
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import UInt8, UInt16, UInt32, Float32MultiArray, UInt16MultiArray, UInt32MultiArray


import rospy
from sensor_msgs.msg import JointState
from vosk import Model, KaldiRecognizer
import sounddevice as sd
import json
import threading
################################################################

class OfflineKeywordListener:

    def __init__(self,contoller):
  

        self.robot_name = rospy.get_param("~robot_name", "miro")

        # 初始化头部控制发布器
        self.pub_head = rospy.Publisher(
            f"/{self.robot_name}/control/kinematic_joints",
            JointState, queue_size=0
        )

        # 初始化尾巴控制发布器（可与头部共用话题）
        self.pub_tail = self.pub_head

        # 加载离线语音识别模型（路径根据你本地情况修改）
        self.model = Model(r"/home/mima123/vosk-model")
        self.rec = KaldiRecognizer(self.model, 16000)

        self.triggered = False

        # 关键词到动作的映射  
        self.keyword_actions = {
            "hello": lambda:contoller.audio_judge("hello"),
            # "mirror": self.action_shake_head,
            # "hello": self.action_nod,
            # "shake": self.action_shake_tail,
        }

        # 配置音频输入流
        self.stream = sd.RawInputStream(
            samplerate=16000,
            blocksize=8000,
            dtype='int16',
            channels=1,
            callback=self.audio_callback
        )

        rospy.loginfo("🎤 离线语音识别已启动，关键词有：%s", ", ".join(self.keyword_actions.keys()))

    def run(self):
        with self.stream:
            rospy.spin()

    def audio_callback(self, indata, frames, time, status):
        if self.rec.AcceptWaveform(bytes(indata)):
            result = self.rec.Result()
            text = json.loads(result).get("text", "").lower()
            rospy.loginfo("🧪 识别文本: %s", text)

            if not self.triggered:
                for keyword, action_fn in self.keyword_actions.items():
                    if keyword in text:
                        rospy.loginfo(f"🗣️ 识别到关键词：{keyword}")
                        self.triggered = True
                        action_fn()
                        rospy.Timer(rospy.Duration(3.0), self.reset_trigger, oneshot=True)
                        break

    def reset_trigger(self, event):
        self.triggered = False



class controller:

    # def callback_package(self, msg):

    # 	# report
    # 	vbat = np.round(np.array(msg.battery.voltage) * 100.0) / 100.0
    # 	if not vbat == self.vbat:
    # 		self.vbat = vbat
    # 		print ("battery", vbat)


    def callback_package(self, msg):
        # store for processing in update_gui
        self.input_package = msg


    def loop(self):

        # state

        msg_wheels = TwistStamped()
        self.stop_spin_flag = True

        # loop
        while self.t_now < 1000.0 and not rospy.core.is_shutdown():

            self.Get_msg_package = self.input_package
            self.xk = math.sin(self.t_now * self.f_kin * 2 * math.pi)
            self.xc = math.sin(self.t_now * self.f_cos * 2 * math.pi)
            self.xcc = math.cos(self.t_now * self.f_cos * 2 * math.pi)
            self.xc2 = math.sin(self.t_now * self.f_cos * 1 * math.pi)

            #touch perception
            self.BodyTouch_Flag, self.HeadTouch_Flag = self.Gain_Touch_flag(self.Get_msg_package)


            #self.happy_dance(xk, xc, xc2, xcc)
            self.touch_feel(self.xk, self.xc, self.xc2, self.xcc)

            
            if (self.wheel_debug):
                stright_stop_flag = True
                speed = 0.4
                self.Wheel_Move_Straight_Forward(self.msg_wheels, stright_stop_flag, speed)
            if(self.illum_debug ):
                shine_flag = True
                self.illum_Shine(self.xcc, shine_flag)
            if(self.dance_debug):
                head_flag = True
                vertical = 0.5
                horizontal = 0.5
                self.dance(self.msg_push, self.xk, vertical, horizontal, head_flag)
            if(self.head_debug):
                self.Shake_heads(self.xk, "normal")
            if(self.contorl_eyes_debug):
                self.eye_control("blink", self.xc, 0.5)
            if(self.control_ears_debug):
                self.ear_control("normal",self.xc, 0.5)
            if(self.control_tails_debug):
                self.tail_control("wagdroop", self.xc, self.xc2)
            # if(self.t_now< 100):
            #     # self.Spin(self.msg_spin, "spin_angles", 0.25, self.t_now, self.spin_T_flag)
            #     self.Spin(self.msg_spin, "dance_roll", 0.25, self.t_now, self.spin_T_flag)

            #self.happy_dance(xk, xc, xc2, xcc)

            if(self.audio_head_nod):
                self.audio_head_time =  self.audio_head_time + 1
                if self.audio_head_time < 200:
                    self.Shake_heads(self.xk, "lift_bow_head")
                else:
                    self.audio_head_time = 0
                    self.audio_head_nod = False
            # state
            time.sleep(0.02)
            self.t_now = self.t_now + 0.02
            self.t_control_now = self.t_control_now + 0.02



    def Gain_Touch_flag(self, Get_msg_package):
        # body touch
        Body_Msg = Get_msg_package.touch_body.data
        self.BodyTouch_Flag = Body_Msg
        #print(Body_Msg)

        # update head touch
        Head_Msg = Get_msg_package.touch_head.data
        self.HeadTouch_Flag  = Head_Msg
        #print(Head_Msg)

        return self.BodyTouch_Flag, self.HeadTouch_Flag

    #stright forward and stop---------------speed
    def Wheel_Move_Straight_Forward(self, msg_wheels, stright_stop_flag, speed):
        if stright_stop_flag == False:
            if not speed is None:
                msg_wheels.twist.linear.x = speed
                msg_wheels.twist.angular.z = 0.0
                self.pub_wheels.publish(msg_wheels)
        else:
            msg_wheels.twist.linear.x = 0.0
            msg_wheels.twist.angular.z = 0.0
            self.pub_wheels.publish(msg_wheels)

    #spin specific angles
    def Spin(self, msg_wheels, spin_mode, spin_numbers):
        if spin_mode == "spin_angles":
            v = 4
            msg_wheels.twist.linear.x = 0.0
            msg_wheels.twist.angular.z = v * 6.2832 * spin_numbers	
            self.pub_wheels.publish(msg_wheels)
        else:
            msg_wheels.twist.linear.x = 0.0
            msg_wheels.twist.angular.z = 0.0
            self.pub_wheels.publish(msg_wheels)
        if spin_mode == "dance_roll":
            v = 4
            msg_wheels.twist.linear.x = 0.0
            msg_wheels.twist.angular.z = v * 6.2832 * spin_numbers	
            self.pub_wheels.publish(msg_wheels)
        if spin_mode == "stop":
            msg_wheels.twist.linear.x = 0.0
            msg_wheels.twist.angular.z = 0.0
            self.pub_wheels.publish(msg_wheels)
    # def Spin(self, msg_wheels, spin_mode, spin_numbers, t_now, spin_T_flag):
    #     if spin_mode == "spin_angles":
    #         if spin_T_flag:
    #             self.T = t_now + self.spin_duration
    #             self.spin_T_flag = False
    #         if self.t_control_now < self.T:
    #             v = 4
    #             msg_wheels.twist.linear.x = 0.0
    #             msg_wheels.twist.angular.z = v * 6.2832 * spin_numbers	
    #             self.pub_wheels.publish(msg_wheels)
    #         else:
    #             self.spin_T_flag = True
    #             msg_wheels.twist.linear.x = 0.0
    #             msg_wheels.twist.angular.z = 0.0
    #             self.pub_wheels.publish(msg_wheels)
    #     if spin_mode == "dance_roll":
    #         self.spin_T_flag = True
    #         v = 4
    #         msg_wheels.twist.linear.x = 0.0
    #         msg_wheels.twist.angular.z = v * 6.2832 * spin_numbers	
    #         self.pub_wheels.publish(msg_wheels)
    #     if spin_mode == "stop":
    #         self.spin_T_flag = True
    #         msg_wheels.twist.linear.x = 0.0
    #         msg_wheels.twist.angular.z = 0.0
    #         self.pub_wheels.publish(msg_wheels)

    def illum_Shine(self, xcc, shine_flag):
        if shine_flag == True:
            q = int(xcc * -127 + 128)

            for i in range(0, 3):
                self.msg_illum.data[i] = (q << ((2-i) * 8)) | 0xFF000000
            for i in range(3, 6):
                self.msg_illum.data[i] = (q << ((i-3) * 8)) | 0xFF000000
        else:
            # 不闪烁时设为全黑（只有 alpha 通道）
            for i in range(6):
                self.msg_illum.data[i] = 0xFF000000
        self.pub_illum.publish(self.msg_illum)

    def dance(self,msg_push, xk, vertical, horizontal, head_flag):
        if head_flag == True:
            
            msg_push.link = miro.constants.LINK_HEAD
            
            msg_push.flags = miro.constants.PUSH_FLAG_VELOCITY
            msg_push.pushpos = geometry_msgs.msg.Vector3(miro.constants.LOC_NOSE_TIP_X, miro.constants.LOC_NOSE_TIP_Y, miro.constants.LOC_NOSE_TIP_Z)
            msg_push.pushvec = geometry_msgs.msg.Vector3(vertical * xk, horizontal * xk, 0.0 * xk)
            self.pub_push.publish(msg_push)
            
    def Shake_heads(self, xk, head_mode):
        if head_mode == "normal":
            self.msg_kin.position[1] = np.radians(0.0)
            self.msg_kin.position[2] = np.radians(0.0)
            self.msg_kin.position[3] = np.radians(0.0) 
        if head_mode == "lift_head":  
            self.msg_kin.position[1] = np.radians(0.0)
            self.msg_kin.position[2] = np.radians(0.0)
            self.msg_kin.position[3] = np.radians(-30.0) 
        if head_mode == "bow_head":  
            self.msg_kin.position[1] = np.radians(0.0)
            self.msg_kin.position[2] = np.radians(0.0)
            self.msg_kin.position[3] = np.radians(50.0) 
        if head_mode == "lift_bow_head":
            self.msg_kin.position[1] = xk * np.radians(20.0) + np.radians(30.0)
        if head_mode == "left_head":
            self.msg_kin.position[1] = np.radians(0.0)
            self.msg_kin.position[2] = np.radians(50.0)
            self.msg_kin.position[3] = np.radians(0.0) 
        if head_mode == "right_head":
            self.msg_kin.position[1] = np.radians(0.0)
            self.msg_kin.position[2] = np.radians(-50.0)
            self.msg_kin.position[3] = np.radians(0.0) 
        if head_mode == "left_right_head":
            t = xk * np.radians(45.0)
            if False:
                # this branch is used to measure YAW_COUNTS_PER_RAD
                t = (xk + 0.5) * np.radians(45.0)
                t = np.clip(t, 0.0, np.radians(45.0))
            self.msg_kin.position[2] = t
        if head_mode == "nod_head":
            self.msg_kin.position[1] = np.radians(20.0)
            self.msg_kin.position[2] = np.radians(0.0)
            self.msg_kin.position[3] = xk * np.radians(20.0) + np.radians(-10.0)
            
        self.pub_kin.publish(self.msg_kin)

    def print_mv(self):
        print("fucking idiot")
        
    def control_sensors(self, control_mode, xc, sc):
        sc = 0.5
        if control_mode == "head_all":
            for i in range(2, 6):
                self.msg_cos.data[i] = xc * sc + 0.5
        if control_mode == "left_eye_ear":
            for i in [2, 4]:
                self.msg_cos.data[i] = xc * sc + 0.5
        if control_mode == "right_eye_ear":
            for i in [3, 5]:
                self.msg_cos.data[i] = xc * sc + 0.5
        self.pub_cos.publish(self.msg_cos)

    def eye_control(self, eye_mode, xc, sc):
        if eye_mode == "blink":
            for i in [2, 3]:
                self.msg_cos.data[i] = xc * sc + 0.5
            self.pub_cos.publish(self.msg_cos)
        if eye_mode =="open":
            for i in [2, 3]:
                self.msg_cos.data[i] = 1.0
            self.pub_cos.publish(self.msg_cos)

    def ear_control(self, ear_mode, xc, sc):
        if ear_mode == "normal":
            for i in [4, 5]:
                self.msg_cos.data[i] = 0.5
        if ear_mode == "inverse":
            self.msg_cos.data[4] = xc * sc + 0.5
            self.msg_cos.data[5] = (xc * sc + 0.5)
        self.pub_cos.publish(self.msg_cos)
        
    def tail_control(self, tail_mode, xc, xc2):
        if tail_mode == "wag":
            self.msg_cos.data[1] = xc * 0.5 + 0.5
        if tail_mode == "droof":
            self.msg_cos.data[0] = xc * 0.5 + 0.5
        if tail_mode == "wagdroop":
            if xc2 >= 0:
                self.msg_cos.data[1] = xc * 0.5 + 0.5
            else:
                self.msg_cos.data[0] = xc * 0.5 + 0.5
        if tail_mode == "normal":
            self.msg_cos.data[1] = 0.5
            self.msg_cos.data[0] = 0.5
        self.pub_cos.publish(self.msg_cos)

    def happy_dance(self, xk, xc, xc2, xcc):
        self.Shake_heads(xk, "lift_bow_head")
        self.tail_control("wagdroop",xc, xc2)
        self.ear_control("inverse", xc, 0.5)
        self.eye_control("blink", xc, 0.5)
        self.illum_Shine(xcc, True)
        self.Spin(self.msg_spin, "dance_roll", 0.25)

    def touch_feel(self, xk, xc, xc2, xcc):
        self.BodyTouch_Flag, self.HeadTouch_Flag = self.Gain_Touch_flag(self.Get_msg_package)
        # touch head
        if self.HeadTouch_Flag > 0:
            self.Shake_heads(xk, "bow_head")
            self.tail_control("wag",xc, xc2)
            self.ear_control("inverse", xc, 0.5)
        else:
            self.Shake_heads(xk, "normal")
            self.tail_control("normal",xc, xc2)
            self.tail_control("normal",xc, xc2)
        
        # touch body
        if self.BodyTouch_Flag > 0:
            self.illum_Shine(xcc, True)
            self.tail_control("droop",xc, xc2)
            self.ear_control("inverse", xc, 0.5)

        # touch left body
        if (self.BodyTouch_Flag > 0) & (self.BodyTouch_Flag < 16383):
            self.left_touch = True
        if (self.BodyTouch_Flag > 16383) & (self.BodyTouch_Flag < 32767):
            self.right_touch = True
        if(self.left_touch):
            self.touch_time = self.touch_time + 1
            if self.touch_time < 30:
                self.Spin(self.msg_spin, "spin_angles", 0.2)
                print(self.touch_time)
            else:
                self.touch_time = 0
                self.left_touch = False
                print(self.left_touch)
                self.Spin(self.msg_spin, "spin_angles", 0.0)
        # touch right body
        if(self.right_touch):
            self.touch_time = self.touch_time + 1
            if self.touch_time < 30:
                self.Spin(self.msg_spin, "spin_angles", -0.2)
            else:
                self.touch_time = 0
                self.right_touch = False

    def audio_judge(self, audio_judge_flag):
        if audio_judge_flag == "hello":
            self.audio_head_nod = True
    
    def __init__(self, args):

        self.Audio = OfflineKeywordListener(self)
        self.audio_thread = threading.Thread(target=self.Audio.run)
        self.audio_thread.daemon = True  # 守护线程，主程序退出时自动结束
        self.audio_thread.start()


        self.xk = None
        self.xc = None
        self.xcc = None
        self.xc2 = None

        self.audio_head_nod = False
        self.audio_head_time = 0
        
        self.active = True
        self.t_now = 0.0
        self.t_control_now = 0.0

        # state
        self.vbat = 0
        self.Get_msg_package = None

        #Get Touch sensors
        self.BodyTouch_Flag = []
        self.HeadTouch_Flag = []
        self.right_touch = False
        self.left_touch  =True

        #wheels parameters
        self.wheels = None  #wheels velocity
        self.stop_spin_flag = False
        self.spin_duration = 0.5
        self.spin_T_flag = True
        self.T = 0
        #stright move
        self.wheelsf = None
        #Get wheel sensors
        self.msg_wheels = TwistStamped()
        self.msg_spin = TwistStamped()

        #illum states
        self.illum = False	
        self.f_kin = 0.25
        self.f_cos = 1.0
        
        #heads mode
        self.kin = ""
        

        #cosl, cosr, eyes, ears, wag, droop, wagdroop
        self.cos = ""
        self.msg_cos = Float32MultiArray()
        self.msg_cos.data = [0.5, 0.5, 0.0, 0.0, 0.5, 0.5]
        self.a = 0

        #dance msg
        self.msg_push = miro.msg.push()

        #head msg
        self.msg_kin = JointState()
        self.msg_kin.position = [0.0, np.radians(30.0), 0.0, 0.0]
        #sensor msg
        

        #tail mode
        self.tail_wag_mode = "wag"
        self.tail_droof_mode = "droof"
        self.tail_wagdroop_mode = "wagdroop"

        #debug
        self.touch_debug = 0
        self.spin_debug = 1
        self.wheel_debug = 0
        self.illum_debug = 0
        self.dance_debug = 0
        self.head_debug = 1

        self.yaw_head_debug = 0
        self.pitch_head_debug = 0
        self.stop_head_debug = 0

        self.control_left_eye_ear_debug = 0
        self.control_right_eye_ear_debug = 0
        self.contorl_eyes_debug = 1
        self.control_ears_debug = 1
        self.control_tails_debug = 1
        self.tail_mode = None


        self.msg_illum = UInt32MultiArray()
        self.msg_illum.data = [0, 0, 0, 0, 0, 0]
        
        self.touch_time = 0

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        # publish
        topic = topic_base_name + "/control/cmd_vel"
        print ("publish", topic)
        self.pub_wheels = rospy.Publisher(topic, geometry_msgs.msg.TwistStamped, queue_size=0)

        # subscribe
        topic = topic_base_name + "/sensors/package"
        print ("subscribe", topic)
        self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package,
                    self.callback_package, queue_size=1, tcp_nodelay=True)

        # publish
        topic = topic_base_name + "/control/illum"
        print ("publish", topic)
        self.pub_illum = rospy.Publisher(topic, UInt32MultiArray, queue_size=0)

        topic = topic_base_name + "/core/mpg/push"
        print ("publish", topic)
        self.pub_push = rospy.Publisher(topic, miro.msg.push, queue_size=0)

        # publish
        topic = topic_base_name + "/control/kinematic_joints"
        print ("publish", topic)
        self.pub_kin = rospy.Publisher(topic, JointState, queue_size=0)

        # publish
        topic = topic_base_name + "/control/cosmetic_joints"
        print ("publish", topic)
        self.pub_cos = rospy.Publisher(topic, Float32MultiArray, queue_size=0)


        #initial state
        self.ear_control("normal",0, 0.5)
        self.tail_control("normal", 0, 0)
        self.Shake_heads(0, "normal")
        self.eye_control("open", 0, 0)
        self.Spin(self.msg_spin, "stop", 0.25)
        self.illum_Shine(0, False) 

        # wait for connect
        print ("wait for connect...")
        time.sleep(1)




if __name__ == "__main__":

    # normal singular invocation
    main = controller(sys.argv[1:])
    main.loop()

