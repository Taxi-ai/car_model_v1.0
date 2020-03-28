import numpy as np
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg
class Bicycle():
    def __init__(self):
       # The bicycle begins with zero initial conditions
        self.xc = 0              # x coordinate
        self.yc = 0              # y coord
        self.theta = 0           #orientation angle
        self.delta = 0           #steering angle
        self.beta = 0            # angular difference between v at cg and the heading
        #begins with a maximum turning rate w_max 
        self.L = 2               # distance between front axle and the rear (a wheelbase length)
        self.lr = 1.2            # distance between  cg and rear axle
        self.w_max = 1.22        # steering angle rate of change (maximum turning rate)
        self.w_min = -1.22
        self.sample_time = 0.01  #sample time for used for update at every time step
        
    def reset(self):             #for reset model vars
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0

    def step(self, v, w):
        # ==================================
        #  Implement kinematic model here
        # ==================================
        
        # max turn rate
        if w > 0:
            w = min(w, self.w_max)
        else:
            w = max(w, self.w_min)
        
        self.beta = np.arctan(self.lr * np.tan(self.delta) / self.L)
        
        # get step discreate difference
        xc_d = v * np.cos(self.theta + self.beta)                     # velocity at x dir
        yc_d = v * np.sin(self.theta + self.beta)                     # velocity at y dir
        theta_d = v * np.cos(self.beta) * np.tan(self.delta) / self.L #orientation rate of change
        delta_d = w                                                   #steering angle rate of change
        t_d = 1e-2                                                    # time sample
        
        # update
        self.xc += xc_d * t_d
        self.yc += yc_d * t_d
        self.theta += theta_d * t_d
        self.delta += delta_d * t_d


model = Bicycle()
def Do_circle(R, L= model.L, sample_time = model.sample_time, time_end ):

    model.reset()

    t_data = np.arange(0,time_end,sample_time)
    x_data = np.zeros_like(t_data)
    y_data = np.zeros_like(t_data)
    V = 2*np.pi*R / time_end

    for i in range(t_data.shape[0]):
        x_data[i] = model.xc
        y_data[i] = model.yc
    
        if model.delta < np.arctan(L/R):
            model.step(V, model.w_max)
        else:
            model.step(V, 0)
        
       

    #plt.axis('equal'),
    #plt.plot(x_data, y_data,label='Learner Model')
    
    #plt.legend()
    #plt.show()