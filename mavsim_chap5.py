"""
mavsim_python
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/2/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from viewers.drone_viewer import MavViewer
from viewers.data_viewer import DataViewer
from models.drone_dynamics import MavDynamics
#from chap4.wind_simulation import WindSimulation
from models.trim import compute_trim
from models.compute_models import compute_model
from tools.signals import Signals

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import VideoWriter
    video = VideoWriter(video_name="chap5_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
#wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)

# use compute_trim function to compute trim state and trim input
Va = 25.
gamma = 0.*np.pi/180.
trim_state, trim_input = compute_trim(mav, Va, gamma)
mav._state = trim_state  # set the initial state of the mav to the trim state
delta = trim_input  # set input to constant constant trim input

# # compute the state space model linearized about trim
compute_model(mav, trim_state, trim_input)

# this signal will be used to excite modes
#input_signal = Signals(amplitude=.05,
                       #duration=0.01,
                       #start_time=2.0)

# initialize the simulation time
sim_time = SIM.start_time
plot_time = sim_time


# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------set control surfaces-------------
    #delta_e, delta_a, delta_r, delta_t = controlJoystick.getInputs()
    #delta.from_array(np.array([[delta_e, delta_a, delta_r, delta_t]]).T)

    # -------physical system-------------
    #current_wind = wind.update()  # get the new wind vector
    mav.update(delta,wind=np.zeros((6,1)))  # propagate the MAV dynamics

    # -------update viewer-------------
    if sim_time-plot_time > SIM.ts_plotting:
        mav_view.update(mav.true_state)  # plot body of MAV
        plot_time = sim_time
    data_view.update(mav.true_state,  # true states
                     mav.true_state,  # estimated states
                     mav.true_state,  # commanded states
                     delta,  # inputs to aircraft
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




