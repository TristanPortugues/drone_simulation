"""
mavsim_python
    - Chapter 6 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/5/2019 - RWB
        2/24/2020 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from viewers.drone_viewer import MavViewer
from viewers.data_viewer import DataViewer
from models.drone_dynamics import MavDynamics
#from chap4.wind_simulation import WindSimulation
from controls.autopilot import Autopilot
#from chap6.autopilot_tecs import Autopilot
from tools.signals import Signals

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="chap6_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
#wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)

# autopilot commands
from message_types.msg_autopilot import MsgAutopilot
commands = MsgAutopilot()
#Va_command = Signals(dc_offset=25.0,amplitude=10.0,start_time=2.0,frequency=0.01)
#altitude_command = Signals(dc_offset=100.0,amplitude=1000.0,start_time=0.0,frequency=0.02)
#course_command = Signals(dc_offset=np.radians(180),amplitude=np.radians(180),start_time=2.0,frequency=0.01)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:

    # -------autopilot commands-------------
    #commands.airspeed_command = 25
    #commands.course_command = course_command.square(sim_time)
    #commands.altitude_command = altitude_command.sinusoid(sim_time)

    commands.airspeed_command = 25
    commands.course_command = np.radians(0)
    commands.altitude_command = 0

    # -------autopilot-------------
    estimated_state = mav.true_state  # uses true states in the control
    delta, commanded_state = autopilot.update(commands, estimated_state)

    # -------physical system-------------
    #current_wind = wind.update()  # get the new wind vector
    mav.update(delta, np.zeros((6,1)) )  # propagate the MAV dynamics

    # -------update viewer-------------
    mav_view.update(mav.true_state)  # plot body of MAV
    data_view.update(mav.true_state,  # true states
                     estimated_state,  # estimated states
                     commanded_state,  # commanded states
                     delta,  # input to aircraft
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




