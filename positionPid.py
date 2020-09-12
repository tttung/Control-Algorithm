#!/usr/bin/env python
# -*- coding: utf-8 -*-

class positionPid():
    """这里定义了一个关于PID的类"""
    def __init__(self, exp_val, kp, ki, kd):
        self.KP = kp
        self.KI = ki
        self.KD = kd
        self.exp_val = exp_val
        self.now_val = 0
        self.sum_err = 0
        self.now_err = 0
        self.last_err = 0
    
    def pid(self):
        self.last_err = self.now_err
        self.now_err = self.exp_val - self.now_val
        self.sum_err += self.now_err
        # 这一块是严格按照公式来写的
        self.now_val = self.KP * (self.exp_val - self.now_val) + self.KI * self.sum_err + self.KD * (self.now_err - self.last_err)
        return self.now_val

if __name__ == '__main__':

