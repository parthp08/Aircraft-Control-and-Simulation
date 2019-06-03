import math
import time
import numpy as np
from numpy import rad2deg

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D, Poly3DCollection
from matplotlib.collections import PolyCollection
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from scipy.io import loadmat

"""
file from : https://github.com/stanleybak/AeroBenchVVPython.git
"""

##----------------------------------------------------------------------
def scale3d(pts, scale_list):
    'scale a 3d ndarray of points, and return the new ndarray'

    assert len(scale_list) == 3

    rv = np.zeros(pts.shape)

    for i in range(pts.shape[0]):
        for d in range(3):
            rv[i, d] = scale_list[d] * pts[i, d]

    return rv

##----------------------------------------------------------------------
def rotate3d(pts, theta, psi, phi):
    'rotates an ndarray of 3d points, returns new list'

    sinTheta = math.sin(theta)
    cosTheta = math.cos(theta)
    sinPsi = math.sin(psi)
    cosPsi = math.cos(psi)
    sinPhi = math.sin(phi)
    cosPhi = math.cos(phi)

    transform_matrix = np.array([ \
        [cosPsi * cosTheta, -sinPsi * cosTheta, sinTheta], \
        [cosPsi * sinTheta * sinPhi + sinPsi * cosPhi, \
        -sinPsi * sinTheta * sinPhi + cosPsi * cosPhi, \
        -cosTheta * sinPhi], \
        [-cosPsi * sinTheta * cosPhi + sinPsi * sinPhi, \
        sinPsi * sinTheta * cosPhi + cosPsi * sinPhi, \
        cosTheta * cosPhi]], dtype=float)

    rv = np.zeros(pts.shape)

    for i in range(pts.shape[0]):
        rv[i] = np.dot(pts[i], transform_matrix)

    return rv

##----------------------------------------------------------------------
def plot3d_anim(times, states, skip=1, filename=None): #, modes, ps_list, Nz_list):
    '''
    make a 3d plot of the GCAS maneuver
    '''

    full_plot = True

    if filename == '': # plot to the screen
        filename = None
        skip = 20
        full_plot = False
    elif filename.endswith('.gif'):
        skip = 20  #5      # PARTH     # changing skip rate to see if it makes processing time faster
    else:
        skip = 1 # plot every frame

    assert len(times) == len(states)

    start = time.time()

    times = times[0::skip]          # PARTH #what to change in time array?? ot remains same
    states = states[0::skip]
    # modes = modes[0::skip]
    # ps_list = ps_list[0::skip]
    # Nz_list = Nz_list[0::skip]

    fig = plt.figure(figsize=(16,9)) #(16,9))#(6, 5))            # PARTH # changed fig size
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(0,90)#(30,60)#(0, 90) #(30, 45)       # PARTH  # change view point

    pos_xs = [pt[9] for pt in states]
    pos_ys = [pt[10] for pt in states]
    pos_zs = [pt[11] for pt in states]

    trail_line, = ax.plot([], [], [], color='r', lw=1)

    data = loadmat('f-16.mat')
    f16_pts = data['V']
    f16_faces = data['F']

    plane_polys = Poly3DCollection([], color=None if full_plot else 'k')
    ax.add_collection3d(plane_polys)

    ax.set_xlim([min(pos_xs)+950, max(pos_xs)-950])     # PARTH # how to convert height into 3 component
    ax.set_ylim([min(pos_ys)+950, max(pos_xs)-950])     # PARTH # added 750 # remove if required
    ax.set_zlim([min(pos_zs)+980, max(pos_zs)-980])     # PARTH # maybe change the limits as our plot is 2d

    ax.set_xlabel('X [ft]')
    ax.set_ylabel('Y [ft]')
    ax.set_zlabel('Altitude [ft] ')
    frames = len(times)

    ##text
    fontsize = 14
    time_text = ax.text2D(0.05, 1.07, "", transform=ax.transAxes, fontsize=fontsize)
    mode_text = ax.text2D(0.95, 1.07, "", transform=ax.transAxes, fontsize=fontsize, horizontalalignment='right')

    alt_text = ax.text2D(0.05, 1.00, "", transform=ax.transAxes, fontsize=fontsize)
    v_text = ax.text2D(0.95, 1.00, "", transform=ax.transAxes, fontsize=fontsize, horizontalalignment='right')

    alpha_text = ax.text2D(0.05, 0.93, "", transform=ax.transAxes, fontsize=fontsize)
    beta_text = ax.text2D(0.95, 0.93, "", transform=ax.transAxes, fontsize=fontsize, horizontalalignment='right')

    nz_text = ax.text2D(0.05, 0.86, "", transform=ax.transAxes, fontsize=fontsize)
    ps_text = ax.text2D(0.95, 0.86, "", transform=ax.transAxes, fontsize=fontsize, horizontalalignment='right')

    ang_text = ax.text2D(0.5, 0.79, "", transform=ax.transAxes, fontsize=fontsize, horizontalalignment='center')

    def anim_func(frame):
        'updates for the animation frame'

        speed = states[frame][0]
        alpha = states[frame][1]
        beta = states[frame][2]
        alt = states[frame][11]
        phi = states[frame][3]
        theta = states[frame][4]
        psi = states[frame][5]          # PARTH # ignoring p, q, r
        dx = states[frame][9]
        dy = states[frame][10]
        dz = states[frame][11]

        time_text.set_text('t = {:.2f} sec'.format(times[frame]))
        # mode_text.set_text('Mode: {}'.format(modes[frame]))

        alt_text.set_text('h = {:.2f} ft'.format(alt))
        v_text.set_text('V = {:.2f} ft/sec'.format(speed))

        alpha_text.set_text('$\\alpha$ = {:.2f} deg'.format(rad2deg(alpha)))
        beta_text.set_text('$\\beta$ = {:.2f} deg'.format(rad2deg(beta)))

        # nz_text.set_text('$N_z$ = {:.2f} g'.format(Nz_list[frame]))
        # ps_text.set_text('$p_s$ = {:.2f} deg/sec'.format(rad2deg(ps_list[frame])))

        ang_text.set_text('[$\\phi$, $\\theta$, $\\psi$] = [{:.2f}, {:.2f}, {:.2f}] deg'.format(\
          rad2deg(phi), rad2deg(theta), rad2deg(psi)))

        # do trail
        trail_len = 50 / skip #200 / skip
        start_index = max(0, frame-trail_len)     
        start_index = int(start_index)         # PARTH     # added this line # because throwing error
        trail_line.set_data(pos_xs[start_index:frame], pos_ys[start_index:frame])
        trail_line.set_3d_properties(pos_zs[start_index:frame])

        scale = 1 #25     # PARTH # reduced scale to 10
        pts = scale3d(f16_pts, [-scale, scale, scale])

        pts = rotate3d(pts, theta, -psi, phi)

        size = 1000
        minx = dx - size
        maxx = dx + size
        miny = dy - size
        maxy = dy + size
        minz = dz - size
        maxz = dz + size

        ax.set_xlim([minx+950, maxx-950])       # PARTH # maybe change limits
        ax.set_ylim([miny+950, maxy-950])       # added 200 + or -
        ax.set_zlim([minz+980, maxz-980])

        verts = []
        fc = []
        count = 0

        for face in f16_faces:
            face_pts = []

            count = count + 1

            if not full_plot and count % 10 != 0:
                continue

            for index in face:
                face_pts.append((pts[index-1][0] + dx, \
                    pts[index-1][1] + dy, \
                    pts[index-1][2] + dz))

            verts.append(face_pts)
            fc.append('k')

        # # draw ground
        # if minz <= 0 and maxz >= 0:
        #     z = 0
        #     verts.append([(minx, miny, z), (maxx, miny, z), (maxx, maxy, z), (minx, maxy, z)])
        #     fc.append('0.8')

        plane_polys.set_verts(verts)
        plane_polys.set_facecolors(fc)

        return None

    anim_obj = animation.FuncAnimation(fig, anim_func, frames, interval=30, \
        blit=False, repeat=True)

    if filename is not None:

        if filename.endswith('.gif'):
            print(f"\nSaving animation to '{filename}' using 'imagemagick'...") #.format(filename)
            # anim_obj.save(filename, dpi=80, writer='imagemagick')
            anim_obj.save(filename, dpi=30, writer='imagemagick')       # PARTH # reduced dpi to reduce size
            print(f"Finished saving to {filename} in {time.time() - start:.1f} sec") #.format(filename, time.time() - start)
        else:
            # fps = 50
            fps = 30        # PARTH # reduced fps to reduce size
            codec = 'libx264'

            print(f"\nSaving '{filename}' at {fps:.2f} fps using ffmpeg with codec '{codec}'.") #.format(filename, fps, codec)

            # if this fails do: 'sudo apt-get install ffmpeg'
            try:
                extra_args = []

                if codec is not None:
                    extra_args += ['-vcodec', str(codec)]

                anim_obj.save(filename, fps=fps, extra_args=extra_args)
                print(f"Finished saving to {filename} in {time.time() - start:.1f} sec") #.format(filename, time.time() - start)

            except AttributeError:
                traceback.print_exc()
                print("\nSaving video file failed! Is ffmpeg installed? Can you run 'ffmpeg' in the terminal?")
    
    else:
        plt.show()

##----------------------------------------------------------------------

if __name__ == "__main__":
    # from plot_parth import plot3d_anim
    plot3d_anim(times, states)      # time and state == array
