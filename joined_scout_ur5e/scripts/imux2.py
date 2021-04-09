#!/usr/bin/env python

import scipy.io
import numpy as np
import rospy
import matplotlib.pyplot as plt
import math
import serial
import binascii
import time
from collections import deque
from std_msgs.msg import Int32

##################################################################################

def  calculation(data):
	if data >= 23768:
		data -= 65536
	data = data * 0.01
	data = round(data , 2)
	return data
class control():
    def __init__(self, port_num):


        ALIVE = 0

        self.port_num = port_num

        self.DATA = bytearray(8)

        self.DATA[0] = 60
        self.DATA[1] = ord('0')
        self.DATA[2] = ord('0')
        self.DATA[3] = ord('l')
        self.DATA[4] = ord('p')
        self.DATA[5] = ord('f')
        self.DATA[6] = ord('5')
        self.DATA[7] = 62


    def open_serial(self):
        self.ser = serial.Serial(
            port=self.port_num,
            baudrate=921600,
        )

    def send_data(self):

        self.ser.write(bytes(self.DATA))



    def receive_data(self):
        hex_result = [0 for i in range(20)]
        for i in range(20):
            result = self.ser.read()
            hex_result[i] = binascii.b2a_hex(result)
        imu_id = int(hex_result[3],16)
        roll = int(hex_result[4] + hex_result[5],16)
        pitch = int(hex_result[6] + hex_result[7],16)
        yaw = int(hex_result[8] + hex_result[9],16)
        acc_x = int(hex_result[10] + hex_result[11],16)
        acc_y = int(hex_result[12] + hex_result[13],16)
        acc_z = int(hex_result[14] + hex_result[15],16)
        return imu_id, calculation(roll),calculation(pitch),calculation(yaw),calculation(acc_x),calculation(acc_y),calculation(acc_z)

def make_average_euler(roll_hand_array,pitch_hand_array,yaw_hand_array,roll_wrist_array,pitch_wrist_array,yaw_wrist_array):	
    roll_hand = 0
    pitch_hand = 0
    yaw_hand = 0
    roll_wrist = 0
    pitch_wrist = 0
    yaw_wrist = 0
    for i in range(len(roll_hand_array)):
        roll_hand += roll_hand_array[i]
        pitch_hand += pitch_hand_array[i]
        yaw_hand += yaw_hand_array[i]
        roll_wrist += roll_wrist_array[i]
        pitch_wrist += pitch_wrist_array[i]
        yaw_wrist += yaw_wrist_array[i]
    roll_hand = roll_hand / len(roll_hand_array)
    pitch_hand = pitch_hand / len(pitch_hand_array)
    yaw_hand = yaw_hand / len(yaw_hand_array)
    roll_wrist = roll_wrist / len(roll_wrist_array)
    pitch_wrist = pitch_wrist / len(pitch_wrist_array)
    yaw_wrist = yaw_wrist / len(yaw_wrist_array)
    return roll_hand,pitch_hand,yaw_hand,roll_wrist,pitch_wrist,yaw_wrist


def make_matrix_R_b(roll_hand,pitch_hand,yaw_hand,roll_wrist,pitch_wrist,yaw_wrist):
	roll = np.deg2rad(roll_hand)	
	pitch = np.deg2rad(pitch_hand)	
	yaw = np.deg2rad(yaw_hand)			
	R_hand = np.array([[math.cos(yaw)*math.cos(pitch),math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll),math.cos(yaw)*math.sin(pitch)*math.cos(roll)+math.sin(yaw)*math.sin(roll)],
		[math.sin(yaw)*math.cos(pitch),math.sin(yaw)*math.sin(pitch)*math.sin(roll)+math.cos(yaw)*math.cos(roll),math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll)],
		[-math.sin(pitch),math.cos(pitch)*math.sin(roll),math.cos(pitch)*math.cos(roll)]])
	roll = np.deg2rad(roll_wrist)	
	pitch = np.deg2rad(pitch_wrist)	
	yaw = np.deg2rad(yaw_wrist)
	R_wrist = np.array([[math.cos(yaw)*math.cos(pitch),math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll),math.cos(yaw)*math.sin(pitch)*math.cos(roll)+math.sin(yaw)*math.sin(roll)],
		[math.sin(yaw)*math.cos(pitch),math.sin(yaw)*math.sin(pitch)*math.sin(roll)+math.cos(yaw)*math.cos(roll),math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll)],
		[-math.sin(pitch),math.cos(pitch)*math.sin(roll),math.cos(pitch)*math.cos(roll)]])
	R_b_f = np.dot(R_wrist,R_hand.T)
	return R_b_f


def trans_hand_to_wrist(roll_hand,pitch_hand,yaw_hand,R_trans):
    roll = np.deg2rad(roll_hand)	
    pitch = np.deg2rad(pitch_hand)	
    yaw = np.deg2rad(yaw_hand)			
    R_hand = np.array([[math.cos(yaw)*math.cos(pitch),math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll),math.cos(yaw)*math.sin(pitch)*math.cos(roll)+math.sin(yaw)*math.sin(roll)],
		[math.sin(yaw)*math.cos(pitch),math.sin(yaw)*math.sin(pitch)*math.sin(roll)+math.cos(yaw)*math.cos(roll),math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll)],
		[-math.sin(pitch),math.cos(pitch)*math.sin(roll),math.cos(pitch)*math.cos(roll)]])
    R = np.dot(R_trans,R_hand)
    pitch = np.rad2deg(math.atan2(-R[2][0],math.sqrt(math.pow(R[0][0],2)+math.pow(R[1][0],2))))
    yaw = np.rad2deg(math.atan2(R[1][0]/math.cos(np.deg2rad(pitch)),R[0][0]/math.cos(np.deg2rad(pitch))))
    roll = np.rad2deg(math.atan2(R[2][1]/math.cos(np.deg2rad(pitch)),R[2][2]/math.cos(np.deg2rad(pitch))))
    return roll,pitch,yaw




def make_different(roll_hand_deq,pitch_hand_deq,yaw_hand_deq,roll_wrist_deq,pitch_wrist_deq,yaw_wrist_deq,acc_x_hand_deq,acc_y_hand_deq,acc_z_hand_deq,acc_x_wrist_deq,acc_y_wrist_deq,acc_z_wrist_deq):
    roll_hand = 0
    pitch_hand = 0
    yaw_hand = 0
    acc_x_hand = 0
    acc_y_hand = 0
    acc_z_hand = 0
    roll_wrist = 0
    pitch_wrist = 0
    yaw_wrist = 0
    acc_x_wrist = 0
    acc_y_wrist = 0
    acc_z_wrist = 0 
    different_euler_sum = 0
    different_acc_sum = 0
    different_roll = [0,0]
    different_pitch = [0,0]
    different_yaw = [0,0]
    different_acc_x_only_hand = 0
    different_acc_y_only_hand = 0
    different_acc_z_only_hand = 0
    move = 0
    for i in range(len(roll_hand_deq)):
        different_roll[i] = roll_hand_deq[i] - roll_wrist_deq[i]
        different_pitch[i] = pitch_hand_deq[i] - pitch_wrist_deq[i]
        different_yaw[i] = yaw_wrist_deq[i] - yaw_hand_deq[i]
    different_acc_x_only_hand = acc_x_hand_deq[1] - acc_x_hand_deq[0]
    different_acc_y_only_hand = acc_y_hand_deq[1] - acc_y_hand_deq[0]
    different_acc_z_only_hand = acc_z_hand_deq[1] - acc_z_hand_deq[0]
    if abs(different_roll[1]-different_roll[0]) >5 or abs(different_pitch[1]-different_pitch[0]) >5 or abs(different_yaw[1]-different_yaw[0]) >5:
        if abs(different_acc_x_only_hand) > 1 or abs(different_acc_y_only_hand) > 1 or abs(different_acc_z_only_hand) > 1:
            move = 1
        else:
            move = 0
    return move, different_acc_x_only_hand, different_acc_y_only_hand, different_acc_z_only_hand

def select(different_acc_x_only_hand, different_acc_y_only_hand,  different_acc_z_only_hand):
    x = np.array([abs(different_acc_x_only_hand), abs(different_acc_y_only_hand),  abs(different_acc_z_only_hand)])
    return np.argmax(x)

def different_array(acc):
    acc_sum = 0
    for i in range(len(acc)-1):
        acc_sum += abs(acc[i+1] - acc[i])
    if acc_sum > 0.1:
        move = 1
    else:
        move = 0
    return move

def softmax(x):
    e_x = np.exp(x - np.max(x))
    return e_x / e_x.sum()

    
if __name__=='__main__':
    rospy.init_node('imux2', anonymous=True)
    pub = rospy.Publisher('cmd_ur5e', Int32, queue_size=1)
    #f = open('/home/irol/catkin_ws/src/imu/test_imu_2.txt','w')
    imu = control('/dev/ttyUSB0')
    imu.open_serial()
    count = 0
    move_during = 0
    i = 0
    move_end = 0
    first = 0
    roll_wrist_array = []
    pitch_wrist_array = []
    yaw_wrist_array = []
    acc_x_wrist_array = []
    acc_y_wrist_array = []
    acc_z_wrist_array = []
    roll_hand_array = []
    pitch_hand_array = []
    yaw_hand_array = []
    acc_x_hand_array = []
    acc_y_hand_array = []
    acc_z_hand_array = []
    roll_hand_deq = deque(maxlen=2)
    pitch_hand_deq = deque(maxlen=2)
    yaw_hand_deq = deque(maxlen=2)
    acc_x_hand_deq = deque(maxlen=2)
    acc_y_hand_deq = deque(maxlen=2)
    acc_z_hand_deq = deque(maxlen=2)
    roll_wrist_deq = deque(maxlen=2)
    pitch_wrist_deq = deque(maxlen=2)
    yaw_wrist_deq = deque(maxlen=2)
    acc_x_wrist_deq = deque(maxlen=2)
    acc_y_wrist_deq = deque(maxlen=2)
    acc_z_wrist_deq = deque(maxlen=2)
    acc_x_hand_deq_for_different = deque(maxlen=5)
    acc_y_hand_deq_for_different = deque(maxlen=5)
    acc_z_hand_deq_for_different = deque(maxlen=5)
    first_weight = scipy.io.loadmat("/home/irol/catkin_ws/src/joined_scout_ur5e/scripts/first_layer_weight.mat")
    first_weight = first_weight['LayerWeights']
    second_weight = scipy.io.loadmat("/home/irol/catkin_ws/src/joined_scout_ur5e/scripts/second_layer_weight.mat")
    second_weight = second_weight['LayerWeights1']
    first_bias = scipy.io.loadmat("/home/irol/catkin_ws/src/joined_scout_ur5e/scripts/first_layer_bias.mat")
    first_bias = first_bias['LayerBiases']
    second_bias = scipy.io.loadmat("/home/irol/catkin_ws/src/joined_scout_ur5e/scripts/second_layer_bias.mat")
    second_bias = second_bias['LayerBiases1']
    first_layer = [0 for i in range(25)]
    second_layer = [0 for i in range(6)]
    for i in range(5):
        acc_x_hand_deq_for_different.append(i)
        acc_y_hand_deq_for_different.append(i)
        acc_z_hand_deq_for_different.append(i)
    i = 0
    acc_x_hand_deq.append(0)
    acc_y_hand_deq.append(0)
    acc_z_hand_deq.append(0)
    while not rospy.is_shutdown():
        imu_id, roll, pitch, yaw, acc_x, acc_y, acc_z = imu.receive_data()
        if imu_id == 0:
            roll_wrist = roll
            pitch_wrist = pitch
            yaw_wrist = yaw
            acc_x_wrist = acc_x
            acc_y_wrist = acc_y
            acc_z_wrist = acc_z
        elif imu_id == 1:
            roll_hand = roll
            pitch_hand = pitch
            yaw_hand = yaw
            acc_x_hand = acc_x
            acc_y_hand = acc_y
            acc_z_hand = acc_z
        i += 1
        if i < 1000 and i > 2 and first == 0:
            roll_wrist_array.append(roll_wrist)
            pitch_wrist_array.append(pitch_wrist)
            yaw_wrist_array.append(yaw_wrist)
            roll_hand_array.append(roll_hand)
            pitch_hand_array.append(pitch_hand)
            yaw_hand_array.append(yaw_hand)
        elif i>1000 and first == 0:
            avr_roll_hand,avr_pitch_hand,avr_yaw_hand,avr_roll_wrist,avr_pitch_wrist,avr_yaw_wrist = make_average_euler(roll_hand_array,pitch_hand_array,yaw_hand_array,roll_wrist_array,pitch_wrist_array,yaw_wrist_array)
            R_trans = make_matrix_R_b(avr_roll_hand,avr_pitch_hand,avr_yaw_hand,avr_roll_wrist,avr_pitch_wrist,avr_yaw_wrist)
            first = 1
            print("trans_R")
        if first == 1 :
            trans_roll_hand, trans_pitch_hand, trans_yaw_hand = trans_hand_to_wrist(roll_hand,pitch_hand,yaw_hand,R_trans)  
            count += 1
            if imu_id == 0:
                roll_wrist_deq.append(roll_wrist)
                pitch_wrist_deq.append(pitch_wrist)
                yaw_wrist_deq.append(yaw_wrist)
                acc_x_wrist_deq.append(acc_x_wrist)
                acc_y_wrist_deq.append(acc_y_wrist)
                acc_z_wrist_deq.append(acc_z_wrist)
            elif imu_id == 1:
                roll_hand_deq.append(trans_roll_hand)
                pitch_hand_deq.append(trans_pitch_hand)
                yaw_hand_deq.append(trans_yaw_hand)
                acc_x_hand_deq.append(acc_x_hand)
                acc_y_hand_deq.append(acc_y_hand)
                acc_z_hand_deq.append(acc_x_hand)
            if count >5:
                move, different_acc_x_only_hand, different_acc_y_only_hand,  different_acc_z_only_hand= make_different(roll_hand_deq,pitch_hand_deq,yaw_hand_deq,roll_wrist_deq,pitch_wrist_deq,yaw_wrist_deq,acc_x_hand_deq,acc_y_hand_deq,acc_z_hand_deq,acc_x_wrist_deq,acc_y_wrist_deq,acc_z_wrist_deq)
                if move == 1 and move_during ==0 :
                    move_during = 1
                    roll_hand_array = []
                    pitch_hand_array = []
                    yaw_hand_array = []
                    acc_x_hand_array = []
                    acc_y_hand_array = []
                    acc_z_hand_array = []
                    roll_wrist_array = []
                    pitch_wrist_array = []
                    yaw_wrist_array = []
                    acc_x_wrist_array = []
                    acc_y_wrist_array = []
                    acc_z_wrist_array = []
                    first_roll_hand = trans_roll_hand
                    first_pitch_hand = trans_pitch_hand
                    first_yaw_hand = trans_yaw_hand
                    roll_trans_negative_positive = 0
                    pitch_trans_negative_positive = 0
                    yaw_trans_negative_positive = 0
                    print("move start",count)
                if move_during == 1:
                    mode = select(different_acc_x_only_hand, different_acc_y_only_hand,  different_acc_z_only_hand)
                    acc_x_hand_deq_for_different.append(acc_x_hand)
                    acc_y_hand_deq_for_different.append(acc_y_hand)
                    acc_z_hand_deq_for_different.append(acc_z_hand)
                    if first_roll_hand >= 0:
                        roll_negaive_positive = 1
                    else:
                        roll_negaive_positive = 0
                    if first_pitch_hand >0:
                        pitch_negaive_positive = 1
                    else:
                        pitch_negaive_positive = 0     
                    if first_yaw_hand >= 0:     
                        yaw_negaive_positive = 1
                    else:
                        yaw_negaive_positive = 0              
                    if imu_id == 1:  
                        if roll_trans_negative_positive == 0:           
                            if roll_negaive_positive == 1:
                                if trans_roll_hand < -160 :
                                    roll_trans_negative_positive = 1
                            else:
                                if trans_roll_hand > 160:
                                    roll_trans_negative_positive = 2
                        if roll_trans_negative_positive == 1:
                            if trans_roll_hand < 0:
                                trans_roll_hand = 360 + trans_roll_hand
                        elif roll_trans_negative_positive == 2:
                            if trans_roll_hand >=0:
                                trans_roll_hand = -360 + trans_roll_hand

                        if pitch_trans_negative_positive == 0:
                            if pitch_negaive_positive == 1:
                                if trans_pitch_hand < -70 :
                                    pitch_trans_negative_positive = 1
                                    trans_pitch_hand = 180 - trans_pitch_hand
                            else:
                                if trans_pitch_hand >= 70:
                                    pitch_trans_negative_positive = 2
                        if pitch_trans_negative_positive == 1:
                            if trans_pitch_hand < 0:
                                trans_pitch_hand = 180 + trans_pitch_hand
                        elif pitch_trans_negative_positive == 2:
                            if trans_pitch_hand >= 0:
                                trans_pitch_hand = -180 + trans_pitch_hand

                        if yaw_trans_negative_positive == 0:
                            if yaw_negaive_positive == 1:
                                if trans_yaw_hand < -160 : 
                                    yaw_trans_negative_positive = 1
                            else:
                                if trans_yaw_hand >= 160:
                                    yaw_trans_negative_positive = 2
                        if yaw_trans_negative_positive == 1:
                            if trans_yaw_hand < 0:
                                trans_yaw_hand = 360 + trans_yaw_hand
                        elif yaw_trans_negative_positive == 2:
                            if trans_yaw_hand >= 0:
                                trans_yaw_hand = -360 + trans_yaw_hand
                        roll_hand_array.append(trans_roll_hand)
                        pitch_hand_array.append(trans_pitch_hand)
                        yaw_hand_array.append(trans_yaw_hand)
                        acc_x_hand_array.append(acc_x_hand)
                        acc_y_hand_array.append(acc_y_hand)
                        acc_z_hand_array.append(acc_x_hand)
                    if mode == 0:
                        if different_array(acc_x_hand_deq_for_different) == 1 :
                            move_during = 1
                        else:
                            move_during = 0
                            move_end = 1
                            print("move_end",count)
                    elif mode == 1:
                        if different_array(acc_y_hand_deq_for_different) == 1 :
                            move_during = 1
                        else:
                            move_during = 0
                            move_end = 1
                            print("move_end",count)
                    elif mode == 2:
                        if different_array(acc_z_hand_deq_for_different) == 1 :
                            move_during = 1
                        else:
                            move_during = 0
                            move_end = 1
                            print("move end",count)
                elif move_end == 1:
                    move_end = 0
                    num = []
                    for i in range(len(roll_hand_array)):
                        num.append(i)
                    roll_fit = np.polyfit(num,roll_hand_array,1)
                    pitch_fit = np.polyfit(num,pitch_hand_array,1)
                    yaw_fit = np.polyfit(num,yaw_hand_array,1)
                    input_data = [roll_fit[0], pitch_fit[0],yaw_fit[0]]
                    for i in range (25):
                        sum = 0
                        for j in range(3):
                            sum += input_data[j] * first_weight[i][j]
                        first_layer[i] = sum + first_bias[i]

                    for i in range (6):
                        sum = 0
                        for j in range(25):
                            sum += first_layer[j] * second_weight[i][j]
                        second_layer[i] = sum + second_bias[i]

                    result = softmax(second_layer)
                    result = np.array(result)
                    result = result.tolist()
                    result = result.index(max(result)) + 1
                    print(result)
                    pub.publish(result)
                    """
                    f.write(str(roll_fit[0]))
                    f.write("\t\t")	
                    f.write(str(pitch_fit[0]))
                    f.write("\t\t")
                    f.write(str(yaw_fit[0]))
                    f.write("\n")
                    roll_fit = np.polyfit(num,roll_hand_array,3)
                    pitch_fit = np.polyfit(num,pitch_hand_array,3)
                    yaw_fit = np.polyfit(num,yaw_hand_array,3)import scipy.io
                    print(pitch_fit)
                    print(yaw_fit)
                    print(max_min_euler)
                    if max_min_euler.index(max(max_min_euler)) == 0:
                        if roll_fit[0] < 0:
                            print("3 gesture")
                        else:
                            print("3 reverse gesture")
                    elif max_min_euler.index(max(max_min_euler)) == 1:
                        if pitch_fit[0] > 0:
                            print("2 gesutre")
                        else:
                            print("2 reverse gesture")
                    elif max_min_euler.index(max(max_min_euler)) == 2:
                        if yaw_fit[0] > 0:
                            print("1 gesture")
                        else:
                            print("1 reverse gesture")
                    """
                    #print(yaw_hand_array)
                    #print(len(yaw_hand_array))
                    
            """
            f.write(str(trans_roll_hand))
            f.write("\t\t")
            f.write("\t\t")	
            f.write(str(trans_pitch_hand))
            f.write("\t\t")
            f.write("\t\t")	
            f.write(str(trans_yaw_hand))
            f.write("\t\t")
            f.write("\t\t")	
            f.write(str(acc_x_hand))
            f.write("\t\t")
            f.write(str(acc_y_hand))
            f.write("\t\t")
            f.write(str(acc_z_hand))
            f.write("\t\t")
            f.write(str(roll_wrist))
            f.write("\t\t")
            f.write(str(pitch_wrist))
            f.write("\t\t")	
            f.write(str(yaw_wrist))
            f.write("\t\t")	
            f.write(str(acc_x_wrist))
            f.write("\t\t")
            f.write(str(acc_y_wrist))
            f.write("\t\t")
            f.write(str(acc_z_wrist))
            f.write("\n")
            """
    #f.close()


            
