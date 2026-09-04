import math

class Kinematics:
    def __init__(self, L, W, R):
        self.L = L #base length
        self.W = W #track width
        self.R = R #radius
 
    def inverse(self, vx, vy, wz):
        raise NotImplementedError
 
    def forward(self, w):
        raise NotImplementedError

class MecanumKinematics(Kinematics):
    def inverse(self, vx, vy, wz):
        k = self.L + self.W #scale factor
 
        w_fl = (vx - vy - k * wz) / self.R
        w_fr = (vx + vy + k * wz) / self.R
        w_rl = (vx + vy - k * wz) / self.R
        w_rr = (vx - vy + k * wz) / self.R
 
        return [w_fl, w_fr, w_rl, w_rr]
 
    def forward(self, w):
        w_fl, w_fr, w_rl, w_rr = w
        k = self.L + self.W
 
        vx = (self.R / 4.0) * (w_fl + w_fr + w_rl + w_rr)
        vy = (self.R / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        wz = (self.R / (4.0 * k)) * (-w_fl + w_fr - w_rl + w_rr)
 
        return vx, vy, wz