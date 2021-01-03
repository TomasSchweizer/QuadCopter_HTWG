"""
Program to give a 3d attitude of the IMU sensor, represented as a 3d cube
"""

import numpy as np
import scipy.io
import math


import pygame as pg
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *


import frames as fr
import usb_bulk


# global defines
USB_RAW_SENSOR_DATA = 1
USB_FUSED_ANGLES = 2
USB_QUATERNIONS = 3

# set send data
USB_DATA = USB_QUATERNIONS

# arrays for matlab
roll_array = np.empty([])
pitch_array = np.empty([])
yaw_array = np.empty([])
quat_array = np.array([1, 0, 0, 0])
samples_array = np.zeros([])

# array for coordinate frame rotation
rot_coordinate_frame_x = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, math.cos(math.pi), -math.sin(math.pi), 0.0],
                                 [0.0, math.sin(math.pi), math.cos(math.pi), 0.0], [0.0, 0.0, 0.0, 1.0]])
rot_coordinate_frame_y = np.array([[math.cos(math.pi), 0.0, math.sin(math.pi), 0.0], [0.0, 1.0, 0.0, 0.0],
                                 [math.sin(math.pi), 0.0, math.cos(math.pi), 0.0], [0.0, 0.0, 0.0, 1.0]])
rot_coordinate_frame_z = np.array([[math.cos(math.pi), -math.sin(math.pi), 0.0, 0.0], [math.sin(math.pi), math.cos(math.pi), 0.0, 0.0],
                                  [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])



# Display 3D object on pygame Screen with PyOpenGl
class View3D:
    def __init__(self, width, height, wireframe, reference_frame, usb_device):
        self.width = width
        self.height = height
        self.wireframe = wireframe
        self.reference_frame = reference_frame
        self.usb_device = usb_device
        pg.init()
        self.screen = pg.display.set_mode((width, height), DOUBLEBUF | OPENGL)
        pg.display.set_caption('Attitude Determination Quadcopter')
        self.background = (10, 10, 50)
        self.clock = pg.time.Clock()
        pg.font.init()
        self.myfont = pg.font.SysFont('Comic Sans MS', 30)

        # set view
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        #init
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


    # Create the pygame screen until closed and main loop
    def run(self):

        running = True

        while running:
            for event in pg.event.get():
                if event.type == pg.QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    # while closing save roll, pitch, yaw, quat data as matlab file
                    scipy.io.savemat('Quadcopter_Measurements.mat',
                                     dict(roll_array=roll_array, pitch_array=pitch_array,
                                          yaw_array=yaw_array, quat_array=quat_array, samples=samples_array))
                    pg.quit()
                    running = False
                    quit()

            data = self.usb_device.usb_read_data()
            print(data)
            self.wireframe.quaternion.q = data
            # inverse rotation around th y axis
            #self.wireframe.quaternion.q *= np.array([1.0, 1.0, -1.0, 1.0])


            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
            glTranslatef(0.0, 0.0, -10.0)

            self.drawText((-4.5, 2.5, 2), "Quadcopter 3D-Visualization", 18)
            self.drawText((2.5, -2.5, 2), "Press Escape to exit.", 16)

            roll, pitch, yaw = self.wireframe.get_attitude()

            self.drawText((-4.5, -2.3, 2), "YAW: %f" % (yaw), 16)
            self.drawText((-4.5, -2.5, 2), "PITCH: %f" % (pitch), 16)
            self.drawText((-4.5, -2.7, 2), "ROLL: %f" % (roll), 16)

            glPushMatrix()
            self.display_reference_frame()
            glPopMatrix()

            glPushMatrix()
            glMultMatrixf(self.wireframe.rotation_mat())
            self.display_block()
            glPopMatrix()
            self.save_values()

            pg.display.flip()
            pg.time.wait(10)

    def drawText(self, position, text_string, size):
        font = pg.font.SysFont("Courier", size, True)
        text_surface = font.render(text_string, True, (255, 255, 255, 255), (0, 0, 0, 255))
        textData = pg.image.tostring(text_surface, "RGBA", True)
        glRasterPos3d(*position)
        glDrawPixels(text_surface.get_width(), text_surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


    # save roll, pitch, yaw and quaternion in numpy arrays
    def save_values(self):

        global roll_array, pitch_array, yaw_array, quat_array, samples_array

        #yaw, pitch, roll = self.wireframe.get_attitude()

        # TODO: ( test later in c code) find true north woth addition of declination

        quat = self.wireframe.quaternion.q


        #roll_array = np.append(roll_array, roll)
        #pitch_array = np.append(pitch_array, pitch)
        #yaw_array = np.append(yaw_array, yaw)

        quat_array = np.append(quat_array, [quat])
        #samples_array = np.append(samples_array, len(roll_array) - 1)

    # Draw block with PyOpenGl
    def display_block(self):

        # draw the Block / Cube
        glBegin(GL_QUADS)
        for face in self.wireframe.faces:

            glColor3fv(face.color)

            for node_index in face.nodes_indices:

                node = self.wireframe.nodes[node_index]
                node_vec = (node.x, node.y, node.z)

                glVertex3fv(node_vec)
        glEnd()

        glBegin(GL_LINES)

        for edge in self.wireframe.edges:

            glColor3fv(edge.color)

            for node_index in edge.nodes_indices:

                node = self.wireframe.nodes[node_index]
                node_vec = (node.x, node.y, node.z)

                glVertex3fv(node_vec)
        glEnd()


        glBegin(GL_LINES)

        for axis in self.wireframe.axes:

            glColor3fv(axis.color)

            for node_index in axis.nodes_indices:
                node = self.wireframe.axes_nodes[node_index]
                node_vec = (node.x, node.y, node.z)

                glVertex3fv(node_vec)

        glEnd()


    # draw reference frame with PyOpenGl
    def display_reference_frame(self):

        glBegin(GL_LINES)

        for axis in self.reference_frame.axes:

            glColor3fv(axis.color)

            for node_index in axis.nodes_indices:
                node = self.reference_frame.nodes[node_index]
                node_vec = (node.x, node.y, node.z)

                glVertex3fv(node_vec)

        glEnd()


        self.drawText((3.25, 0.0, 0.0), "N", 18)
        self.drawText((0.0, 3.25, 0.0), "E", 18)
        #self.drawText((0.0, 0.0, 3.25), "U", 18)


# initialize the block/rectangular cube with values
def init_block():

    # create wireframe instance block
    block = fr.Wireframe()

    # create starting nodes/ colors of the block
    block_nodes = [(x, y, z) for x in (-1.5, 1.5) for y in (-1, 1) for z in (-0.1, 0.1)]
    node_colors = [(255, 255, 255)] * len(block_nodes)

    # add and print nodes
    block.add_nodes(block_nodes, node_colors)
    block.output_nodes()

    # create faces of the block by adding 4 nodes to 1 face and add colors to faces
    """
       face[0] = 0,2,4,6 -> backside
       face[1] = 0,1,3,2 -> leftside
       face[2] = 1,3,7,5 -> frontside
       face[3] = 4,5,7,6 -> rightside
       face[4] = 2,3,7,6 -> topside
       face[5] = 0,1,5,4 -> bottomside        
       """
    block_faces = [(0, 2, 6, 4), (0, 1, 3, 2), (1, 3, 7, 5), (4, 5, 7, 6), (2, 3, 7, 6), (0, 1, 5, 4)]
    faces_colors = [(255, 0, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 255, 0)]

    # add and print faces
    block.add_faces(block_faces, faces_colors)
    block.output_faces()

    # create edges of the block by adding to nodes to 1 edge and add colors to edges
    block_edges = [(0, 2), (2, 6), (6, 4), (4, 0), (0, 1), (1, 3),
                   (3, 2), (3, 7), (7, 5), (5, 1), (4, 5), (7, 6)]
    edges_colors = [(0, 0, 0)] * len(block_edges)

    # add and print edges
    block.add_edges(block_edges, edges_colors)
    block.output_edges()

    # create pointer of quadcopter
    # create starting nodes/ colors of the reference frame
    block_axes_nodes = [(3.0, 0.0, 0.0), (0.0, 0.0, 0.0), (3.0, 0.0, 0.0), (2.825, 0.125, 0.0), (3.0, 0.0, 0.0), (2.825, -0.125, 0.0),
                             (0.0, 3.0, 0.0), (0.0, 0.0, 0.0), (0.0, 3.0, 0.0), (0.125, 2.825, 0.0), (0.0, 3.0, 0.0), (-0.125, 2.825, 0.0),
                             (0.0, 0.0, 3.0), (0.0, 0.0, 0.0), (0.0, 0.0, 3.0), (0.0, 0.125, 2.825), (0.0, 0.0, 3.0), (0.0, -0.125, 2.825)]
    axes_nodes_colors = [(255, 255, 255)] * len(block_axes_nodes)

    # add and print all nodes to reference frame
    block.add_axes_nodes(block_axes_nodes, axes_nodes_colors)
    block.output_axes_nodes()

    # create axes of the reference frame
    block_axes = [(0, 1, 2, 3, 4, 5), (6, 7, 8, 9, 10, 11), (12, 13, 14, 15, 16, 17)]
    axes_colors = [(255, 0, 0), (0, 0, 255), (100, 255, 100)]

    # add and print all axes to reference frame
    block.add_axes(block_axes, axes_colors)
    block.output_axes()

    return block

# init reference frame
def init_reference_frame():

    reference_frame = fr.ReferenceFrame()

    # create starting nodes/ colors of the reference frame
    reference_frame_nodes = [(-3.0, 0.0, 0.0), (3.0, 0.0, 0.0), (3.0, 0.0, 0.0), (2.825, 0.125, 0.0), (3.0, 0.0, 0.0), (2.825, -0.125, 0.0),
                             (0.0, -3.0, 0.0), (0.0, 3.0, 0.0), (0.0, 3.0, 0.0), (0.125, 2.825, 0.0), (0.0, 3.0, 0.0), (-0.125, 2.825, 0.0),
                             (0.0, 0.0, -3.0), (0.0, 0.0, 3.0), (0.0, 0.0, 3.0), (0.0, 0.125, 2.825), (0.0, 0.0, 3.0), (0.0, -0.125, 2.825)]
    rf_nodes_colors = [(255, 255, 255)] * len(reference_frame_nodes)

    # add and print all nodes to reference frame
    reference_frame.add_nodes(reference_frame_nodes, rf_nodes_colors)
    reference_frame.output_nodes()

    # create axes of the reference frame
    reference_frame_axes = [(0, 1, 2, 3, 4, 5), (6, 7, 8, 9, 10, 11), (12, 13, 14, 15, 16, 17)]
    rf_axes_colors = [(255, 0, 0), (0, 0, 255), (0, 255, 0)]

    # add and print all axes to reference frame
    reference_frame.add_axes(reference_frame_axes, rf_axes_colors)
    reference_frame.output_axes()

    return reference_frame


if __name__ == '__main__':

    usb_bulk_device = usb_bulk.USBBulkDevice(id_vendor=0x1cbe, id_product=0x0003, USB_DATA=USB_DATA)

    block = init_block()
    reference_frame = init_reference_frame()

    view_3D = View3D(1080, 720, block, reference_frame, usb_bulk_device)

    view_3D.run()
