import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import obtain_trajectory_data as trajec_generator
import copy

class double_pendulum:
    def __init__(self):
        # paramteres of the double pendulum model
        self.l1, self.l2 = 0.32, 0.4
        self.lc1, self.lc2 = 0.14, 0.17
        self.m1, self.m2 = 5.7, 2.65
        self.I1, self.I2 = 0.061, 0.038
        self.c1, self.c2 = 2.5, 1.2
        self.g = 9.81

        self.states = np.array([0,0,0,0])
        self.Torque = np.array([[0],[0]])
        self.step_size = 0.01

    def model_dynamics(self):
        th1, th2, w1, w2 = self.states[0], self.states[1], self.states[2], self.states[3]
        K1 = m1 * lc1 ** 2 + m2 * l1 ** 2 + I1
        K2 = m2 * lc2 * l1
        K3 = m2 * lc2 ** 2 + I2
        K4 = (m1 * lc1 + m2 * l1) * g
        K5 = m2 * lc2 * g

        # The dynamics of the leg model is given by M(X)*X'' + C(X,X')*X' + G(X) = T
        # Here X = [theta1, theta_2], X' = [theta1_dot, theta2_dot]
        # The equations were derived using Lagrange's motion equations
        M = np.array([[K1, K2 * np.cos(th1 - th2)], [K2 * np.cos(th1 - th2), K3]])
        C = np.array([[c2 + c1, K2 * np.sin(th1 - th2) * w2 - c2], [-K2 * np.sin(th1 - th2) * w1 - c2, c2]])
        G = np.array([[K4 * np.sin(th1)], [K5 * np.sin(th2)]])

        return M, C, G

    def state_space(self,_states):
        th1, th2, w1, w2 = _states[0], _states[1], _states[2], _states[3]

        M, C, G = self.model_dynamics()

        # state space equations
        th1_dot = w1
        th2_dot = w2
        w1_dot = np.dot(np.linalg.inv(M), self.Torque - np.dot(C, np.array([[w1], [w2]])) - G)[0][0]
        w2_dot = np.dot(np.linalg.inv(M), self.Torque - np.dot(C, np.array([[w1], [w2]])) - G)[1][0]

        return np.array([th1_dot, th2_dot, w1_dot, w2_dot])

    def forward_dynamics(self): # runge_kutta fourth order

        self.Torque = self.determine_controller_torque()

        k1 = self.step_size*self.state_space(self.states)
        k2 = self.step_size*self.state_space(self.states + k1/2)
        k3 = self.step_size*self.state_space(self.states + k2/2)
        k4 = self.step_size*self.state_space(self.states + k3)

        self.states += k1/6 + k2/3 + k3/3 + k4/6

    def determine_controller_torque(self):
        T_assist = np.array([[0], [0]])
        current_target = np.zeros(6)
        for state_num in range(6):
            current_target[state_num] = np.interp(t_sim[current_step],t_data,target_states[:,state_num])
        T = self.inv_dyn_controller(current_target)

        if assistance:
            T_assist = self.assistive_controller()
            assisted_torques[:, current_step] = T_assist.reshape(2) + T.reshape(2)

        if desired:
            desired_torques[:, current_step] = T.reshape(2)
        else:
            undesired_torques[:, current_step] = T.reshape(2)

        return T + T_assist

    def inv_dyn_controller(self,_states):

        Md, Cd, Gd = self.model_dynamics()

        q2 = np.array([[_states[4]], [_states[5]]])
        q1 = np.array([[_states[2]], [_states[3]]])

        T = np.dot(Md, q2) + np.dot(Cd, q1) + Gd

        return T

    def assistive_controller(self):
        x = l1 * np.sin(self.states[0]) + l2 * np.sin(self.states[1])
        y = -l1 * np.cos(self.states[0]) - l2 * np.cos(self.states[1])

        _x = l1 * np.sin(desired_state_memory[current_step, 0]) + l2 * np.sin(desired_state_memory[current_step, 1])
        _y = -l1 * np.cos(desired_state_memory[current_step, 0]) - l2 * np.cos(desired_state_memory[current_step, 1])

        dist = np.sum((np.array([_x, _y]) - np.array([x, y])) ** 2) ** 0.5
        dist = max(dist - 0.005, 0)
        error_memory[current_step] = dist

        Kp = np.array([30000, 30000])
        Kd = np.array([100, 100])

        if current_step > 0:
            current_e = desired_state_memory[current_step, 0:2] - state_memory[current_step, 0:2]
            last_e = desired_state_memory[current_step, 0:2] - state_memory[current_step - 1, 0:2]
        else:
            current_e = desired_state_memory[current_step, 0:2] - state_memory[current_step, 0:2]
            last_e = np.zeros(2)
        error_rate = (current_e - last_e) / self.step_size
        T = dist ** 2 * (Kp * (current_e) + Kd * error_rate)

        return T.reshape(2, 1)

def state_plotter():
    colors = ['k','r','b']
    conditions = ['desired', 'undesired', 'assisted']
    Y_all = [desired_state_memory,undesired_state_memory,assisted_state_memory]
    fig1, ax = plt.subplots(1, 2)
    fig2, ankle_plot = plt.subplots(1, 2)
    for i,Y in enumerate(Y_all):
        ax[0].plot(t_sim,Y[:,0]*180/np.pi,colors[i], label = conditions[i]+'_hip')
        ax[0].plot(t_sim,Y[:,1]*180/np.pi, colors[i]+'--',label = conditions[i]+'_knee')
        ax[0].set_xlabel('Time (s)')
        ax[0].set_ylabel('Joint angle (deg)')

        ax[1].plot(t_sim,Y[:,2]*180/np.pi, colors[i])
        ax[1].plot(t_sim,Y[:,3]*180/np.pi, colors[i]+'--')
        ax[1].set_xlabel('Time (s)')
        ax[1].set_ylabel('Joint angluar velocity (deg/s)')

        ankle_x = l1 * np.sin(Y[:,0]) + l2 * np.sin(Y[:,1])
        ankle_y = -l1 * np.cos(Y[:,0]) - l2 * np.cos(Y[:,1])
        ankle_plot[0].plot(t_sim,ankle_x,colors[i],label = conditions[i])
        ankle_plot[0].set_xlabel('Time (s)')
        ankle_plot[0].set_ylabel('Ankle x')
        ankle_plot[1].plot(t_sim,ankle_y, colors[i],)
        ankle_plot[1].set_xlabel('Time (s)')
        ankle_plot[1].set_ylabel('Ankle y')

    ax[0].legend()
    ankle_plot[0].legend()

def torque_plotter():
    colors = ['k','r','b']
    conditions = ['desired', 'undesired', 'assisted']
    T_all = [desired_torques,undesired_torques,assisted_torques]
    fig1, torque_plot = plt.subplots(1, 2)
    for i,T in enumerate(T_all):
        torque_plot[0].plot(t_sim,T[0,:],colors[i], label = conditions[i])
        torque_plot[0].set_xlabel('Time (s)')
        torque_plot[0].set_ylabel('Hip torques')

        torque_plot[1].plot(t_sim,T[1,:], colors[i])
        torque_plot[1].set_xlabel('Time (s)')
        torque_plot[1].set_ylabel('Knee torques')

    torque_plot[0].legend()

def animator(states):
    anim_figure = plt.figure()
    anim_ax = plt.axes(xlim = (-.5,0.5), ylim = (-1,0))
    #anim_ax.axis('equal')

    thigh_plot, = anim_ax.plot([],[], 'r', linewidth = 3)
    shank_plot, = anim_ax.plot([],[], 'b', linewidth = 3)

    def update(frame):
        thigh_link = [0, l1 * np.sin(states[frame, 0]), 0, -l1 * np.cos(states[frame, 0])]
        shank_link = [l1 * np.sin(states[frame, 0]), l1 * np.sin(states[frame, 0]) + l2 * np.sin(states[frame, 1]), -l1 * np.cos(states[frame, 0]),
                      -l1 * np.cos(states[frame, 0]) - l2 * np.cos(states[frame, 1])]
        thigh_plot.set_data(thigh_link[0:2],thigh_link[-2:])
        shank_plot.set_data(shank_link[0:2],shank_link[-2:])

        return thigh_plot,shank_plot,

    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

    ani = animation.FuncAnimation(anim_figure, update, frames=len(t_sim), interval=10, blit=True)
    ani.save('tracking.mp4', writer = writer)

l1,l2  = 0.32,0.4
lc1,lc2 = 0.14,0.17
m1,m2 = 5.7,2.65
I1,I2 = 0.061,0.038
c1,c2 = 2.5,1.2
g = 9.81

# loading the desired and undesired data from file
t_data,desired_states = trajec_generator.desired_trajectory()
undesired_states = trajec_generator.gen_undesired_trajectory()

#running simulation of desired behavior
desired = True
assistance = False
target_states = copy.copy(desired_states)
pend_model = double_pendulum()
pend_model.states = copy.copy(desired_states[0, :-2])
pend_model.step_size = 0.005
t_sim = np.linspace(t_data[0],t_data[-1],(t_data[-1]-t_data[0])/pend_model.step_size)
desired_torques = np.zeros((2,len(t_sim)))
state_memory = np.zeros((len(t_sim),4))
for current_step in range(len(t_sim)-1):
    state_memory[current_step, :] = pend_model.states
    pend_model.forward_dynamics()

state_memory[-1, :] = state_memory[-2, :]
desired_state_memory = copy.copy(state_memory)

#running simulation of undesired behavior
desired = False
assistance = False
target_states = copy.copy(undesired_states)
pend_model.states = copy.copy(undesired_states[0, :-2])
pend_model.Torque = np.array([[0], [0]])
state_memory = np.zeros((len(t_sim),4))
undesired_torques = np.zeros((2,len(t_sim)))
for current_step in range(len(t_sim)-1):
    state_memory[current_step, :] = pend_model.states
    pend_model.forward_dynamics()

state_memory[-1, :] = state_memory[-2, :]
undesired_state_memory = copy.copy(state_memory)

#running simulation of assisted behavior
desired = False
assistance = True
target_states = copy.copy(undesired_states)
pend_model.states = copy.copy(undesired_states[0, :-2])
pend_model.Torque = np.array([[0], [0]])
state_memory = np.zeros((len(t_sim),4))
assisted_torques = np.zeros((2,len(t_sim)))
error_memory = np.zeros(len(t_sim))
for current_step in range(len(t_sim)-1):
    state_memory[current_step, :] = pend_model.states
    pend_model.forward_dynamics()

state_memory[-1, :] = state_memory[-2, :]
assisted_state_memory = copy.copy(state_memory)

# plotting results
state_plotter()
torque_plotter()
plt.show()
#animator(assisted_state_memory)