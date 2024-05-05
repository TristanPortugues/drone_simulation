"""
mavsimPy
    - Chapter 3 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        12/18/2018 - RWB
        1/14/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM


from viewers.drone_viewer import MavViewer
from viewers.data_viewer import DataViewer
from models.drone_dynamics import MavDynamics
from message_types.msg_delta import MsgDelta

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import VideoWriter
    video = VideoWriter(video_name="chap3_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()




#NO WIND YET
wind = np.array([[0.], [0.], [0.]])



# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:

    # -------vary forces and moments to check dynamics-------------
    fx = 100
    fy = 10  # 10
    fz = 10  # 10
    Mx = 0.1  # 0.1
    My = 0.1  # 0.1
    Mz = 0.1  # 0.1

    forces_moments = np.array([[fx, fy, fz, Mx, My, Mz]]).T



    # -------physical system-------------
    mav.update(delta,wind)  # propagate the MAV dynamics

    # -------update viewer-------------
    mav_view.update(mav.true_state)  # plot body of MAV
    data_view.update(mav.true_state,  # true states
                     mav.true_state,  # estimated states
                     mav.true_state,  # commanded states
                     delta,  # inputs to the aircraft
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




