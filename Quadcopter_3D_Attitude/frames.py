"""
Create the Wireframe and the 6 faces of the block:
    
    1. Node: xyz coordinates of a block
    2. Face: 4 coordinates which define a face of the block
    3. Wireframe: stores the information of the cube and acts as an interface to Pygame
"""
import quaternion as quat

# Node to store each pont of the block
class Node:
    def __init__(self, coordinates, color):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]
        self.color = color


# Face stores 4 nodes which make up a face of the block
class Face:
    def __init__(self, nodes, color):
        self.nodes_indices = nodes
        self.color = color


# Stores 2 nodes for each edge
class Edge:
    def __init__(self, nodes, color):
        self.nodes_indices = nodes
        self.color = color

# Axes for the reference frame
class Axis:
    def __init__(self, nodes, color):
        self.nodes_indices = nodes
        self.color = color

# Wireframe stores the details of the block
class Wireframe:
    def __init__(self):
        self.nodes = []
        self.edges = []
        self.faces = []
        self.quaternion = quat.Quaternion()

    # add all nodes to the wireframe
    def add_nodes(self, node_list, color_list):
        for node, color in zip(node_list, color_list):
            self.nodes.append(Node(node, color))

    # add all faces to the wireframe
    def add_faces(self, face_list, color_list):
        for index, color in zip(face_list, color_list):
            self.faces.append(Face(index, color))

    # add all edges to the wireframe
    def add_edges(self, edge_list, color_list):
        for index, color in zip(edge_list, color_list):
            self.edges.append(Edge(index, color))

    # output all points of the Nodes
    def output_nodes(self):
        print("\n --- Nodes --- ")
        for i, node in enumerate(self.nodes):
            print(" %d: (%.2f, %.2f, %.2f) \t Color: (%d, %d, %d)" %
                  (i, node.x, node.y, node.z, node.color[0], node.color[1], node.color[2]))

    # output all faces and color and which nodes belong to which face
    def output_faces(self):
        print("\n --- Faces --- ")
        for i, face in enumerate(self.faces):
            print("Face %d:" % i)
            print("Color: (%d, %d, %d)" % (face.color[0], face.color[1], face.color[2]))
            for node_index in face.nodes_indices:
                print("\tNode %d" % node_index)

    # output all edges and color
    def output_edges(self):
        print("\n --- Edges --- ")
        for i, edge in enumerate(self.edges):
            print("Edge %d:" % i)
            print("Color: (%d, %d, %d)" % (edge.color[0], edge.color[1], edge.color[2]))
            for node_index in edge.nodes_indices:
                print("\tNode %d" % node_index)

    # calls quaternion rotat
    def quat_rotate(self, w, dt):
        self.quaternion.rotate(w, dt)

    def rotation_mat(self):
        rotation_mat = quat.get_rot_mat(self.quaternion.q)
        return rotation_mat

    def get_attitude(self):
        return quat.get_euler_angles(self.quaternion.q)

class ReferenceFrame:
    def __init__(self):
        self.nodes = []
        self.axes = []

    # add all nodes to the reference frame
    def add_nodes(self, node_list, color_list):
        for node, color in zip(node_list, color_list):
            self.nodes.append(Node(node, color))

    # add all axes to the reference frame
    def add_axes(self, axis_list, color_list):
        for index, color in zip(axis_list, color_list):
            self.axes.append(Face(index, color))

    # output all points of the Nodes
    def output_nodes(self):
        print("\n --- Nodes --- ")
        for i, node in enumerate(self.nodes):
            print(" %d: (%.2f, %.2f, %.2f) \t Color: (%d, %d, %d)" %
                  (i, node.x, node.y, node.z, node.color[0], node.color[1], node.color[2]))

    # output all axes and color
    def output_axes(self):
        print("\n --- Axes --- ")
        for i, axis in enumerate(self.axes):
            print("Axis %d:" % i)
            print("Color: (%d, %d, %d)" % (axis.color[0], axis.color[1], axis.color[2]))
            for node_index in axis.nodes_indices:
                print("\tNode %d" % node_index)


