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

class ThreeWheelOmniKinematics(Kinematics):

    def inverse(self, vx, vy, wz):

        w1 = 1/self.R *(vy + self.L* wz)
        w2 = 1/self.R *((-math.sqrt(3)/2)*vx - 1/2*vy + self.L* wz)
        w3 = 1/self.R *((math.sqrt(3)/2)*vx - 1/2*vy + self.L* wz)


        return w1 , w2 , w3


    def forward(self, w):
        w1, w2, w3 = w

        vx = self.R * (math.sqrt(3)/3 * (-w2 + w3))
        vy = self.R * (1/3 * (w1 - 1/2*w2 - 1/2*w3))
        wz = self.R * (1/(3*self.L) * (w1 + w2 + w3))

        return vx, vy, wz   


class FourWheelOmniKinematics(Kinematics):

    def inverse(self, vx, vy, wz):
        w1 = 1/self.R * (vx - vy - self.L* wz)
        w2 = 1/self.R * (vx + vy + self.L* wz)
        w3 = 1/self.R * (vx - vy + self.L* wz)
        w4 = 1/self.R * (vx + vy - self.L* wz)

        return w1 , w2 , w3 , w4


    def forward(self, w):
        w1, w2, w3, w4 = w

        vx = self.R * (1/4 * (w1 + w2 + w3 + w4))
        vy = self.R * (-1/4 * (w1 - w2 + w3 - w4))
        wz = self.R * (-1/(4*self.L) * (w1 - w2 - w3 + w4))

        return vx, vy, wz

class DiffDriveKinematics(Kinematics):
    def inverse(self, vx, vy, wz):
        v_left = vx - wz * self.L
        v_right = vx + wz * self.L
 
        w_left = v_left / self.R
        w_right = v_right / self.R
 
        return [w_left, w_right, w_left, w_right]
 
    def forward(self, w):
        w_left = (w[0] + w[2]) / 2.0
        w_right = (w[1] + w[3]) / 2.0
 
        v_left = w_left * self.R
        v_right = w_right * self.R
 
        vx = (v_left + v_right) / 2.0
        vy = 0.0
        wz = (v_right - v_left) / (2.0 * self.L)
 
        return vx, vy, wz 