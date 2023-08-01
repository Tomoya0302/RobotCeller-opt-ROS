import rectangle_packing_solver as rps
import random
import numpy as np
import yaml
from socket import socket, AF_INET, SOCK_STREAM
from threading import Thread
import pickle
import matplotlib.patches as patches
from matplotlib import pylab as plt
import dill
import time

DUMMY = 0
RECTANGLE = [{"width": 240, "height": 240, "rotatable": True},  # robot {"width": 240, "height": 240, "rotatable": True}
             {"width": 100, "height": 100, "rotatable": True},  # table
             {"width":  50, "height":  30, "rotatable": True},  # parts 1
             {"width":  50, "height":  30, "rotatable": True},  # parts 2
             {"width":  50, "height":  40, "rotatable": True},  # parts 3
             {"width":  50, "height":  40, "rotatable": True},  # parts 4
             {"width":  50, "height":  50, "rotatable": True},  # parts 5
             {"width":  50, "height":  50, "rotatable": True},  # parts 6
             {"width":  50, "height":  60, "rotatable": True},  # parts 7
             {"width":  50, "height":  60, "rotatable": True},  # parts 8
             {"width":  50, "height":  70, "rotatable": True},  # parts 9
             {"width":  50, "height":  70, "rotatable": True},  # parts 10
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            #  {"width":  25, "height":  25, "rotatable": True},  # dummy
            ]
EXPANSION = 0.001 # mm -> m
WIDTH_LIMIT = 600
HEIGHT_LIMIT = 600

MAX_ITERATION = 2

SHOW_FIG = False # show and save layout every time or not

# Socket
HOST = 'localhost'
PORT = 51001
MAX_MESSAGE = 2048
NUM_THREAD = 1

class Layout:
    def __init__(self):
        self.rectangle = RECTANGLE
        self.color = ['royalblue', 'gold', 'brown', 'orangered', 'lightsalmon', 'greenyellow', 'limegreen', 'green', 'darkcyan', 'slategrey', 'lightpink', 'indigo',
                      'black','black','black','black','black','black','black','black','black','black','black','black','black','black','black','black','black','black','black','black']
        # Define a problem
        self.problem = rps.Problem(self.rectangle) 

    def generate(self, iter):
        gp = random.sample(list(range(self.problem.n)), k=self.problem.n) 
        gn = random.sample(list(range(self.problem.n)), k=self.problem.n)
        rot = [0 for _ in range(self.problem.n)]
        seqpair = rps.SequencePair(pair=(gp, gn))
        floorplan = seqpair.decode(problem=self.problem, rotations=rot)
        solution = rps.Solution(seqpair,floorplan)
        floorplan_dict = self.upload(solution, iter)
        if SHOW_FIG:
            self.visualize(floorplan_dict, solution, iter)
        return floorplan_dict

    def area(self, gp, gn, rot, sol, iter, show):
        seqpair = rps.SequencePair(pair=(gp, gn))
        floorplan = seqpair.decode(problem=self.problem, rotations=rot)
        solution = rps.Solution(seqpair,floorplan)
        floorplan_dict = self.upload(solution, sol, iter)
        if show:
            self.visualize(floorplan_dict, solution, sol, iter)
        return solution.floorplan.area, floorplan_dict

    def upload(self, solution, sol, iter):
        # upload the floorplan
        floorplan_dict = {}
        for i in range(len(solution.floorplan.positions)):
            pos_dict = solution.floorplan.positions[i]
            id = pos_dict['id']
            del pos_dict['id']
            floorplan_dict[id] = pos_dict
        if sol:
            file_name = './floorplan_log/floorplan_solution_' + str(iter) + '.yaml'
            with open(file_name, 'w') as fp:
                yaml.dump(floorplan_dict, fp)

        return floorplan_dict

    def visualize(self, floorplan, solution, sol, iter):
        plt.rcParams["font.size"] = 14
        positions = solution.floorplan.positions
        bounding_box = solution.floorplan.bounding_box

        # Figure settings
        bb_width = bounding_box[0]
        bb_height = bounding_box[1]
        fig = plt.figure(figsize=(10, 10 * bb_height / bb_width + 0.5))
        ax = plt.axes()
        ax.set_aspect("equal")
        plt.xlim([0, bb_width])
        plt.ylim([0, bb_height])
        plt.xlabel("X")
        plt.ylabel("Y")

        # Plot every rectangle
        for i, rectangle in enumerate(positions):
            r = patches.Rectangle(
                xy=(rectangle["x"], rectangle["y"]),
                width=rectangle["width"],
                height=rectangle["height"],
                edgecolor="#000000",
                facecolor=self.color[i],
                alpha=1.0,
                fill=True,
            )
            ax.add_patch(r)

            # Add text label
            centering_offset = 0.011
            center_x = rectangle["x"] + rectangle["width"] / 2 - bb_width * centering_offset
            center_y = rectangle["y"] + rectangle["height"] / 2 - bb_height * centering_offset
            ax.text(x=center_x, y=center_y, s=str(i), fontsize=16, color='black')

        plt.show()
        if not sol:
            file_name = './floorplan_log/floorplan_' + str(iter) + '.png'
        else:
            file_name = './floorplan_log/solution_' + str(iter) + '.png'
        fig.savefig(file_name)

class Robot:
    def __init__(self, replay):
        self.dummy = DUMMY
        self.layout = Layout()
        self.host = HOST
        self.port = PORT
        self.num_thread = NUM_THREAD
        self.expansion = EXPANSION
        self.replay = replay
        self.clients = []
        self.z_rot = [0.2, 0.0, 1.57, 0.0]
        self.finish_flag = False
        self.count = 0
        self.max_iteration = MAX_ITERATION
        self.elapsed_time_list = []
        self.output_file = open('./floorplan_log/result.txt', 'w')
        self.output_file.write('Iter.\tArea\tOperatingTime\tFeasibility\tTime\n')
        self.output_file_ = open('./floorplan_log/result_.txt', 'w')
        self.output_file_.write('Iter.\tArea\tOperatingTime\tTime\n')

        with socket(AF_INET, SOCK_STREAM) as sock:
            sock.bind(('', self.port))
            sock.listen(NUM_THREAD)
            print('Waiting ...')

            try:
                con, addr = sock.accept()
            except KeyboardInterrupt:
                print('ERROR')
            print('[CONNECT] {}'.format(addr))
            self.clients.append((con, addr))

            if self.replay == None:
                self.simulation = 7777
            else:
                self.simulation = 8888


    def move(self):
        # load
        with open('./floorplan_log/floorplan_solution_' + str(self.replay) + '.yaml', 'r') as input:
            self.floorplan = yaml.safe_load(input)
        # Robot position
        self.robot_x = (self.floorplan[0]['x'] + self.floorplan[0]['width']  / 2) * self.expansion
        self.robot_y = (self.floorplan[0]['y'] + self.floorplan[0]['height'] / 2) * self.expansion

        self.move_joint([self.simulation]) # simulation
        self.sub_th = Thread(target=self.main, args=(), daemon=True)
        self.sub_th.start()

        while True:
            if self.finish_flag:
                self.finish_flag = False
                self.count += 1
                break

    def trajectory_generate(self, area, floorplan):
        self.floorplan = floorplan
        self.area = area
        # Robot position
        self.robot_x = (self.floorplan[0]['x'] + self.floorplan[0]['width']  / 2) * self.expansion
        self.robot_y = (self.floorplan[0]['y'] + self.floorplan[0]['height'] / 2) * self.expansion
        self.feasibility = True

        for i in range(len(self.floorplan)-1-self.dummy):
            x = (self.floorplan[i+1]['x'] + self.floorplan[i+1]['width']  / 2) * self.expansion - self.robot_x 
            y = (self.floorplan[i+1]['y'] + self.floorplan[i+1]['height'] / 2) * self.expansion - self.robot_y
        
        if self.feasibility:
            self.move_joint([self.simulation]) # simulation
            self.sub_th = Thread(target=self.main, args=(), daemon=True)
            self.sub_th.start()

            while True:
                if self.finish_flag:
                    self.finish_flag = False
                    self.count += 1
                    break

        return self.ros_time, self.feasibility

    def close_connection(self):
        self.move_joint([9999]) # finish
        self.output_file.close()
        self.output_file_.close()

    def move_joint(self, data):
        data = pickle.dumps(data, protocol=2) # if python2 -> protocol=2
        self.clients[0][0].sendto(data, self.clients[0][1])

    def main(self):
        self.feasibility = True
        time_list = np.zeros(len(self.floorplan)-2-self.dummy, dtype=float)
        data = self.clients[0][0].recv(MAX_MESSAGE)
        # Convert Python 2 "ObjectType" to Python 3 object
        dill._dill._reverse_typemap["ObjectType"] = object
        data = pickle.loads(data, encoding='bytes')

        for i in range(len(self.floorplan)-2-self.dummy):
            # move to parts
            target = [(self.floorplan[i+2]['x'] + self.floorplan[i+2]['width']  / 2) * self.expansion - self.robot_x, 
                        (self.floorplan[i+2]['y'] + self.floorplan[i+2]['height'] / 2) * self.expansion - self.robot_y,
                        self.z_rot[0],
                        self.z_rot[1],
                        self.z_rot[2],
                        self.z_rot[3],
                        (self.floorplan[1]['x'] + self.floorplan[1]['width']  / 2) * self.expansion - self.robot_x, 
                        (self.floorplan[1]['y'] + self.floorplan[1]['height'] / 2) * self.expansion - self.robot_y,
                        self.z_rot[0],
                        self.z_rot[1],
                        self.z_rot[2],
                        self.z_rot[3],
                        int(i/(len(self.floorplan)-3-self.dummy))]
            self.move_joint(target)

            # ROS time
            data = self.clients[0][0].recv(MAX_MESSAGE)
            # Convert Python 2 "ObjectType" to Python 3 object
            dill._dill._reverse_typemap["ObjectType"] = object
            data = pickle.loads(data, encoding='bytes')
            if data[0] == float('inf'):
                self.feasibility = False
            time_list[i] = data[0]
        
        self.ros_time = round(np.sum(time_list), 9)
        tm = time.time()
        if self.replay == None:
            print('Iteration :', self.count, ', Area :', self.area, ', OperatingTime :', self.ros_time, ', Feasibility :', self.feasibility, ', Time :', tm)
            self.output_file.write(str(self.count) + '\t' + str(self.area) + '\t' + str(self.ros_time) + '\t' + str(self.feasibility) + '\t' + str(tm) + '\n')
            if self.feasibility:
                self.output_file_.write(str(self.count) + '\t' + str(self.area) + '\t' + str(self.ros_time) + '\t' + str(tm) + '\n')
        self.finish_flag = True