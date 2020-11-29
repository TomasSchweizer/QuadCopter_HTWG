"""
Program to give a 3d attitude of the IMU sensor, represented as a 3d cube
"""

import numpy as np
import scipy.io

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
quat_array = np.empty([1, 1, 1, 1])
samples_array = np.zeros([])

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


        # Using depth test to make sure closer colors are shown over further ones
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        # start view
        glMatrixMode(GL_PROJECTION)
        gluPerspective(45.0, (width / height), 0.1, 40.0)
        glTranslatef(0.0, 0.0, -10.0)

        # to rotate that earth frame is down delete to have sight from top
        glRotated(-90.0, 1.0, 0.0, 0)

        glScalef(-1.0, -1.0, 1.0)

    # Create the pygame screen until closed and main loop
    def run(self):

        running = True

        while running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    # while closing save roll, pitch, yaw, quat data as matlab file
                    scipy.io.savemat('Quadcopter_Measurements.mat',
                                     dict(roll_array=roll_array, pitch_array=pitch_array,
                                          yaw_array=yaw_array, quat_array=quat_array, samples=samples_array))
                    pg.quit()
                    running = False
                    quit()

            data = np.array(self.usb_device.usb_read_data())
            #print(data)
            self.wireframe.quaternion.q = data
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glMatrixMode(GL_MODELVIEW)

            glLoadMatrixf(self.wireframe.rotation_mat())

            self.display_reference_frame()
            self.display_block()
            self.save_values()

            pg.display.flip()
            pg.time.wait(10)

    # save roll, pitch, yaw and quaternion in numpy arrays
    def save_values(self):

        global roll_array, pitch_array, yaw_array, quat_array, samples_array

        yaw, pitch, roll = self.wireframe.get_attitude()
        quat = self.wireframe.quaternion.q

        print(roll)
        roll_array = np.append(roll_array, roll)
        pitch_array = np.append(pitch_array, pitch)
        yaw_array = np.append(yaw_array, yaw)



        quat_array = np.append(quat_array, [quat])


        samples_array = np.append(samples_array, len(roll_array) - 1)

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

    return block

# init reference frame
def init_reference_frame():

    reference_frame = fr.ReferenceFrame()

    # create starting nodes/ colors of the reference frame
    reference_frame_nodes = [ (-7.5, 0.0, 0.0), ( 7.5, 0.0, 0.0), ( 0.0,-7.5, 0.0),
                              ( 0.0, 7.5, 0.0), ( 0.0, 0.0,-7.5), ( 0.0, 0.0, 7.5)]
    rf_nodes_colors = [(255, 255, 255)] * len(reference_frame_nodes)

    # add and print all nodes to reference frame
    reference_frame.add_nodes(reference_frame_nodes, rf_nodes_colors)
    reference_frame.output_nodes()

    # create axes of the reference frame
    reference_frame_axes = [(0, 1), (2, 3), (4, 5)]
    rf_axes_colors = [(255, 255, 255)] * len(reference_frame_axes)

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
