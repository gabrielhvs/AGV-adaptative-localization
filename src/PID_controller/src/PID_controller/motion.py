#!/usr/bin/env python

# ROS packages (python)
import rospy
import math

from geometry_msgs.msg import Twist

error_last = 0.0

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.velocity = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vel_lin_ant = 0
        self.vel_lin = 0

    def compute(self, error, dt=0.01):
            if dt == 0:
                dt = 0.01
            P_out = self.Kp * error
            self.integral += error * dt
            I_out = self.Ki * self.integral
            derivative = (error - self.previous_error) / dt
            D_out = self.Kd * derivative
            self.previous_error = error
            return P_out + I_out + D_out

    def move(self, error, dt, forceBreak):
        self.velocity.angular.z = self.compute(error, dt)
        
        # 1. Calcula a velocidade ideal (sua lógica original otimizada)
        vel_alvo = 0.6 * max(0.0, 1.0 - (abs(error)/1.5))

        # 2. Define a aceleração máxima permitida (ex: 1.0 m/s²)
        aceleracao_maxima = 1.0 
        variacao_maxima_permitida = aceleracao_maxima * dt

        # 3. Calcula o quanto a velocidade quer mudar
        delta_v = vel_alvo - self.vel_lin_ant

        # 4. Trava a variação dentro do limite físico
        delta_v = max(min(delta_v, variacao_maxima_permitida), -variacao_maxima_permitida)

        # 5. Aplica a variação suavizada
        vel_lin = self.vel_lin_ant + delta_v
        self.vel_lin_ant = vel_lin

        self.velocity.linear.x = vel_lin

        if(forceBreak):
             self.velocity.linear.x = 0

        self.publisher.publish(self.velocity)