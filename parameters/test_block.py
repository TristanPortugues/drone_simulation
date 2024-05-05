import matplotlib.pyplot as plt
import control as ctrl

# System transfer function
b0 = 1
a1 = 2
a0 = 1
system_tf = ctrl.TransferFunction([b0], [1, a1, a0])

# Proportional controller transfer function
Kp = 2
kp_controller = ctrl.TransferFunction([Kp], [1])

# Integral controller transfer function
Ki = 1
ki_controller = ctrl.TransferFunction([Ki], [1, 0])

# Create parallel block diagram with proportional and integral controllers
parallel_system = ctrl.parallel(system_tf * kp_controller, ki_controller)

# Plot the block diagram
plt.figure()
ctrl.matplotlib.use('TkAgg')
ctrl.matplotlib.pyplot.bode_plot(parallel_system, dB=True)
plt.show()
