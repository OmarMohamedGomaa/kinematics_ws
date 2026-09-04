import math

class Kinematics:
    def __init__(self, L, W, R):
        self.L = L
        self.W = W
        self.R = R
 
    def inverse(self, vx, vy, wz):
        raise NotImplementedError
 
    def forward(self, w):
        raise NotImplementedError