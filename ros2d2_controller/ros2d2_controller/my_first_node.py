#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial, time
import tkinter as tk
import math
import numpy as np
from numpy import linalg
        

class BluetoothPublisher(Node):
    
    def __init__(self):                             # Constructor
        super().__init__("bluetooth_publisher")
        #self.publisher_ = self.create_publisher(String, "bluetooth_topic", 10)
        self.get_logger().info("ROS")      #Write a log


    def send_command(self, angulo, motor):    #Primero se ingresa el valor del angulo en rad, luego T o t para el motor
        msg = String()
        msg = motor + angulo + "\n"
        #self.publisher_.publish(msg)
        self.bluetooth_connection.write(msg.encode())


    def start_bt_connection(self):
        try:
            self.bluetooth_connection = serial.Serial("/dev/rfcomm0",115200)
            print("Conexion exitosa")
        except:
            print("Error de conexion")


class App(BluetoothPublisher):
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=400, height=400)
        self.canvas.pack()
        self.canvas.create_line(200, 0, 200, 400, width=1) # eje y
        self.canvas.create_text(210, 10, text='y') # label eje x
        self.canvas.create_line(0, 200, 400, 200, width=1) # eje x
        self.canvas.create_text(390, 190, text='x') # label eje y
        self.canvas.bind("<Button-1>", self.on_click)
        self.points = []
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0
        self.vn_i = []
        self.vn_o = []
        self.R_robot = float()
        self.r_wheel = float()
        self.R_robot = 28.5/2
        self.r_wheel = 15.5/2
        self.wheel_robot_disp = 0.0
        self.left_wheel_rotation = 0.0
        self.right_wheel_rotation = 0.0


    def calculate_magnitude (self): 
        if len(self.points) > 1:
            self.magnitude = np.sqrt((self.x2-self.x1)**2 + (self.y2-self.y1)**2)
        else:
            self.magnitude = np.sqrt((self.x1-0)**2 + (self.y1-0)**2)

    def calculate_robot_displacement (self):
        self.wheel_robot_disp = (self.magnitude/self.r_wheel)
        self.left_wheel_rotation = self.left_wheel_rotation + self.wheel_robot_disp
        self.right_wheel_rotation = self.right_wheel_rotation + self.wheel_robot_disp

    def calculate_robot_rotation (self): 
        if len(self.points) > 1:
            self.vn_o = np.array([(self.x2-self.x1),(self.y2-self.y1)])
            self.robot_rot = math.acos((np.dot(self.vn_i,self.vn_o))/((np.linalg.norm(self.vn_o))*(np.linalg.norm(self.vn_i))))*180/3.141516
        else:
            self.vn_i = np.array([(self.x1-0),(self.y1-0)])
            self.robot_rot = math.acos((np.dot(self.vn_i,np.array([0,1])))/((np.linalg.norm(self.vn_i))*(np.linalg.norm([0,1]))))*180/3.141516

    def calculate_wheel_rotation (self):
        self.wheel_rotation= ((self.R_robot * self.robot_rot)/self.r_wheel)*math.pi/180


    def calculate_direction (self):
        if len(self.points) > 1:
            crossProduct = np.cross(self.vn_i, self.vn_o)
            if crossProduct < 0:
                print("Sentido de giro: horario")
                self.left_wheel_rotation = self.left_wheel_rotation + self.wheel_rotation
                self.right_wheel_rotation = self.right_wheel_rotation - self.wheel_rotation
            elif crossProduct > 0:
                print("Sentido de giro: antihorario")
                self.left_wheel_rotation = self.left_wheel_rotation - self.wheel_rotation
                self.right_wheel_rotation = self.right_wheel_rotation + self.wheel_rotation
            self.vn_i = self.vn_o
        else:
            crossProduct = np.cross([0,1],self.vn_i)
            if crossProduct < 0:
                print("Sentido de giro: horario")
                self.left_wheel_rotation = self.left_wheel_rotation + self.wheel_rotation
                self.right_wheel_rotation = self.right_wheel_rotation - self.wheel_rotation
            elif crossProduct > 0:
                print("Sentido de giro: antihorario")
                self.left_wheel_rotation = self.left_wheel_rotation - self.wheel_rotation
                self.right_wheel_rotation = self.right_wheel_rotation + self.wheel_rotation
        
    def on_click(self, event):
        x, y = event.x, event.y
        self.points.append((x, y))
        self.canvas.create_oval(x-5, y-5, x+5, y+5, fill='red')
        if len(self.points) > 1:
            self.x1, self.y1 = self.points[-2]
            self.x2, self.y2 = self.points[-1]
            self.canvas.create_line(self.x1, self.y1, self.x2, self.y2, arrow=tk.LAST, width=1)
            self.x1 = self.x1 -200
            self.y1 = 200 - self.y1
            self.x2 = self.x2 -200
            self.y2 = 200 - self.y2
        else:
            self.x1, self.y1 = self.points[0]
            self.canvas.create_line(200, 200, self.x1, self.y1, arrow=tk.LAST, width=1)
            self.x1 = self.x1 -200
            self.y1 = 200 - self.y1

        print("---------------------------------------------------")
        self.calculate_robot_rotation()
        self.calculate_wheel_rotation()
        self.calculate_direction()
        print("Rotation [°]: {}".format(self.robot_rot))
        print("Wheel rotation for rotation [rad]: {}".format(self.wheel_rotation))
        print("Left wheel rotation for rotation [rad]: {}".format(self.left_wheel_rotation))
        print("Right wheel rotation for rotation [rad]: {}".format(self.right_wheel_rotation))

        print(str(self.left_wheel_rotation))
        self.send_command(str(self.left_wheel_rotation), "T")
        self.send_command(str(self.right_wheel_rotation), "t")
        time.sleep(5)
        

        self.calculate_magnitude()
        self.calculate_robot_displacement()

        print("Coordenadas del punto: ({}, {})".format(x-200,200-y))
        print("Magnitude [cm]: {}".format(self.magnitude))
        print("Wheel rotation for displacement [rad]: {}".format(self.wheel_robot_disp))

        print("Left wheel rotation for rotation [rad]: {}".format(self.left_wheel_rotation))
        print("Right wheel rotation for rotation [rad]: {}".format(self.right_wheel_rotation))
        

        self.send_command(str(self.left_wheel_rotation), "T")
        self.send_command(str(self.right_wheel_rotation), "t")
        #time.sleep(1)


def main(args=None):
    rclpy.init(args=args)     
    # Everything youll write goes in between these lines. This is a NODE

    root = tk.Tk()
    root.title("ROS2D2 mapping displacement")
    app = App(root)
    app.start_bt_connection()
    root.mainloop()
    #node = BluetoothPublisher() # Creates the instance node that inherits from Node
    #node.start_bt_connection()
    rclpy.spin(app)         

    # Everything youll write goes in between these lines. This is a NODE
    rclpy.shutdown()         


if __name__ == '__main__':
    main()


