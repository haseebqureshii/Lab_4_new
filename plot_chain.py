from imports import plt
from imports import plot_utils

#%matplotlib widget
def plot_robot_chain(joint_angles, robot_chain, target_position):
    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)  
    fig.set_figwidth(13)  
    robot_chain.plot(joint_angles, ax, target=target_position)
    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.6)
    plt.ion()