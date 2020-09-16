#!/usr/bin/env python
# -*- coding: utf-8 -*-

class incrementalPid():
    def __init__(self, exp_val, kp, ki, kd):
        self.KP = kp
        self.KI = ki
        self.KD = kd
        self.exp_val = exp_val
        self.now_val = 0
        self.now_err = 0
        self.last_err = 0
        self.last_last_err = 0
        self.change_val = 0
    
    def pid(self):
        self.last_last_err = self.last_err
        self.last_err = self.now_err
        self.now_err = self.exp_val - self.now_val
        self.change_val = self.KP * (self.now_err - self.last_err) + self.KI * self.now_err + self.KD * (self.now_err - 2 * self.last_err + self.last_last_err)
        self.now_val += self.change_val
        return self.now_val

if __name__ == '__main__':
