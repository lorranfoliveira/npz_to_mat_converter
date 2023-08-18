from scipy.spatial import KDTree
from scipy.io import savemat
import numpy as np
import json


class Node:
    def __init__(self, cg, sup, x, y):
        self.cg = cg
        self.sup = sup
        self.x = x
        self.y = y

    def distance(self, other):
        return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)


class Element:
    def __init__(self, nodes, E, A, L):
        self.nodes = nodes
        self.E = E
        self.A = A

    def length(self):
        return self.nodes[0].distance(self.nodes[1])


class Data:
    def __init__(self, npz_filename):
        self.data = np.load(npz_filename)
        self.points = self.data['arr_0']
        self.elements = self.data['arr_1']
        self.kd_three = KDTree(self.points)
        self.nodes_sup = [self.kd_three.query(x)[1] for x in self.data['arr_2']]
        self.nodes_load = [self.kd_three.query(x)[1] for x in self.data['arr_3']]


class Fem:
    def __init__(self, npz_filename, initial_area=1e-4, volume=1.0, young=1.0, fx=1.0, fy=1.0):
        """Constructor

        :param npz_filename: Path to npz file
        :param volume: Total volume of the structure
        :param young: Young's modulus
        :param fx: Reference force in x direction
        :param fy: Reference force in y direction
        """
        self.nodes: list[Node] = []
        self.elements: list[Element] = []
        self.filename = npz_filename
        self.initial_area = initial_area
        self.volume = volume
        self.young = young
        self.fx = fx
        self.fy = fy
        self.data = Data(npz_filename)

        self.generate_nodes_and_elements()

    def generate_nodes_and_elements(self):
        print('Generating nodes and elements...')
        for i in range(len(self.data.points)):
            cg = [self.fx, self.fy] if i in self.data.nodes_load else [0, 0]
            sup = [1, 1] if i in self.data.nodes_sup else [0, 0]
            x, y = self.data.points[i]
            self.nodes.append(Node(cg, sup, x, y))

        for i in range(len(self.data.elements)):
            node1, node2 = [self.nodes[j] for j in self.data.elements[i]]
            self.elements.append(Element([node1, node2], self.young, self.initial_area, node1.distance(node2)))

    def generate_dict(self):
        mat_file = {'fem': {'NNode': len(self.nodes),
                            'NElem': len(self.elements),
                            'Vol': self.volume}}

        for i in range(len(self.nodes)):
            try:
                mat_file['fem']['Node'].append({'cg': self.nodes[i].cg,
                                                'sup': self.nodes[i].sup,
                                                'x': self.nodes[i].x,
                                                'y': self.nodes[i].y})
            except KeyError:
                mat_file['fem']['Node'] = [{'cg': self.nodes[i].cg,
                                            'sup': self.nodes[i].sup,
                                            'x': self.nodes[i].x,
                                            'y': self.nodes[i].y}]

        for i in range(len(self.elements)):
            try:
                mat_file['fem']['Element'].append({'nodes': [int(j) for j in self.data.elements[i] + 1],
                                                   'E': self.elements[i].E,
                                                   'A': self.elements[i].A,
                                                   'L': self.elements[i].length()})
            except KeyError:
                mat_file['fem']['Element'] = [{'nodes': [int(j) for j in self.data.elements[i] + 1],
                                               'E': self.elements[i].E,
                                               'A': self.elements[i].A,
                                               'L': self.elements[i].length()}]
        return mat_file

    def new_file_name(self):
        return (f'{self.filename.replace(".npz", "")}__fx-{str(self.fx).replace(".", "_")}__fy-'
                f'{str(self.fy).replace(".", "_")}')

    def save_json(self):
        print('Generating .json file...')
        with open(f'{self.new_file_name()}.json', 'w') as outfile:
            json.dump(self.generate_dict(), outfile)
        print('Done!')

    def save_mat(self):
        print('Generating .mat file...')
        savemat(f'{self.new_file_name()}.mat', self.generate_dict())

        print('Done!')


if __name__ == '__main__':
    files = ['disc_2_loads/disc_2_loads.npz', 'disc_4_loads/disc_4_loads.npz']
    forces = [0.05, 0.25, 0.5, 0.75, 1.0]

    for file in files:
        for force in forces:
            fem1 = Fem(file, initial_area=1e-4, volume=1.0, young=1.0, fx=force, fy=1.0)
            fem1.save_json()
            fem1.save_mat()

            fem2 = Fem(file, initial_area=1e-4, volume=1.0, young=1.0, fx=1.0, fy=force)
            fem2.save_json()
            fem2.save_mat()
