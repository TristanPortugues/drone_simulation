"""
mavsimPy
    - Chapter 4 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        12/27/2018 - RWB
        1/17/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from viewers.drone_viewer import MavViewer
from viewers.data_viewer import DataViewer
from models.drone_dynamics import MavDynamics
#from chap4.wind_simulation import WindSimulation
from message_types.msg_delta import MsgDelta
#import joystick

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import VideoWriter
    video = VideoWriter(video_name="chap4_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
#wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)


delta = MsgDelta()

# initialize the simulation time
sim_time = SIM.start_time
plot_time = sim_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------set control surfaces-------------
    delta.elevator = 0. #-0.1248
    delta.aileron = 0. #0.001836
    delta.rudder = 0. #-0.0003026
    delta.throttle = 0. #0.6768
    #delta_e, delta_a, delta_r, delta_t = controlJoystick.getInputs()
    #delta.from_array(np.array([[delta_e, delta_a, delta_r, delta_t]]).T)

    # -------physical system-------------
    #current_wind = wind.update()  # get the new wind vector

    print("begining update at",sim_time)
    mav.update(delta, """current_wind""")

    print("update at ",sim_time)  # propagate the MAV dynamics

    # -------update viewer-------------
    if sim_time-plot_time > SIM.ts_plotting:

        print("begining view update at",sim_time)
        mav_view.update(mav.true_state)

        print("update done") # plot body of MAV
        plot_time = sim_time

        print("plot_done")


    print("yo")
    data_view.update(mav.true_state,  # true states
                     mav.true_state,  # estimated states
                     mav.true_state,  # commanded states
                     delta,  # inputs to aircraft
                     SIM.ts_simulation)

    print("update_done")
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




