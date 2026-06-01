#!/usr/bin/python3

class PIDController():

    def __init__(self, kp=0, ki=0, kd=0):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.target = 0
        self.rev = 0

        self.error = 0
        self.last_error = 0

        self.p_out = 0
        self.i_out = 0
        self.d_out = 0
        self.pid_out = 0

        self.i_out_lim = 100000
        self.pid_out_lim = 180000

                            
    def pid_calcul(self,target,rev):

        self.target = target 
        self.rev = rev

        self.error = self.target - self.rev

        self.p_out = self.kp * self.error
        self.i_out = self.ki * self.error + self.i_out
        self.i_out = self.limit(self.i_out, self.i_out_lim)  # 限制积分输出
        self.d_out = self.kd *(self.error-self.last_error)

        self.pid_out = self.p_out + self.i_out + self.d_out
        self.pid_out = self.limit(self.pid_out, self.pid_out_lim)  # 限制PID总输出
        self.last_error = self.error

        return self.pid_out
    
    def limit(self,target_vel, limit_vel):
        if target_vel > 0 and target_vel > limit_vel:
            return limit_vel
        elif target_vel < 0 and target_vel < -limit_vel:
            return -limit_vel
        else:
            return target_vel
    
    def pid_config(self,kp ,ki ,kd):

        self.kp = kp
        self.ki = ki
        self.kd = kd

    def pid_clear(self):

        self.target = 0
        self.rev = 0

        self.error = 0
        self.last_error = 0

        self.p_out = 0
        self.i_out = 0
        self.d_out = 0
        self.pid_out = 0

