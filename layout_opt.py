from layout_generation import Layout
from layout_generation import Robot
import numpy as np
import matplotlib.pyplot as plt
from pymoo.core.problem import ElementwiseProblem
from pymoo.core.sampling import Sampling
from pymoo.core.crossover import Crossover
from pymoo.core.mutation import Mutation
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.visualization.scatter import Scatter

class RobotLayout(ElementwiseProblem):
    
    def __init__(self):
        self.n_rect = 12
        super().__init__(
            n_var=self.n_rect * 3, 
            n_obj=2, 
            n_constr=1
            )
        self.iter = 0
        self.result = np.zeros((1, 2), dtype=float)

    def _evaluate(self, x, out, *args, **kwargs):
        gp = x[0:self.n_rect].tolist()
        gn = x[self.n_rect: 2*self.n_rect].tolist()
        rot = x[2*self.n_rect:3*self.n_rect].tolist()

        layout = Layout()
        area, floorplan = layout.area(gp, gn, rot, False, self.iter, False)
        ros_time, feasibility = robot.trajectory_generate(area, floorplan)

        f1 = area
        f2 = ros_time
        g1 = not feasibility # feasibility (g1<=0, 0 is feasible, 1 is not feasible)

        if feasibility:
            self.result = np.append(self.result, [[f1, f2]], axis=0)

        out["F"] = np.column_stack([f1, f2])
        out["G"] = g1

        self.iter += 1

    def visualize(self):
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        ax.scatter(self.result[1:, 0], self.result[1:, 1], marker='.')
        ax.set_xlabel('f1')
        ax.set_ylabel('f2')
        plt.show()

class LayoutSampling(Sampling):

    def _do(self, problem, n_samples, **kwargs):
        n_rect = problem.n_var // 3
        x1 = np.zeros((n_samples, n_rect))
        x2 = np.zeros((n_samples, n_rect))
        x3 = np.zeros((n_samples, n_rect))
        for i in range(n_samples):
            x1[i, :] = np.random.permutation(n_rect)
            x2[i, :] = np.random.permutation(n_rect)
        x3 = np.random.randint(2, size=(n_samples, n_rect))
        x = np.hstack((x1,x2,x3))
        return x

class LayoutOXCrossover(Crossover):
    def __init__(self):

        # define the crossover: number of parents and number of offsprings
        super().__init__(2, 2)

    def OrderCrossover(self, receiver, donor, start, end):
        shift = False

        # the donation and a set of it to allow a quick lookup
        donation = np.copy(donor[start:end + 1])
        donation_as_set = set(donation)

        # the final value to be returned
        y = []

        for k in range(len(receiver)):

            # do the shift starting from the swapped sequence - as proposed in the paper
            i = k if not shift else (start + k) % len(receiver)
            v = receiver[i]

            if v not in donation_as_set:
                y.append(v)

        # now insert the donation at the right place
        y = np.concatenate([y[:start], donation, y[start:]]).astype(copy=False, dtype=int)

        return y


    def _do(self, problem, X, **kwargs):

        # The input of has the following shape (n_parents, n_matings, n_var)
        _, n_matings, n_var = X.shape

        Y = np.full((self.n_offsprings, n_matings, n_var), -1, dtype=int)

        # for each mating provided
        for i in range(n_matings):
            a, b = X[:, i, :]
            n = int(len(a)/3)

            a_dict = {A: B for A, B in zip(a[0:n], a[2*n:3*n])}
            b_dict = {A: B for A, B in zip(b[0:n], b[2*n:3*n])}

            start, end = np.sort(np.random.choice(n, 2, replace=False))

            for j in range(2):
                Y[0, i, j*n:(j+1)*n] = self.OrderCrossover(
                    a[j*n:(j+1)*n], b[j*n:(j+1)*n], start, end)
                Y[1, i, j*n:(j+1)*n] = self.OrderCrossover(
                    b[j*n:(j+1)*n], a[j*n:(j+1)*n], start, end)
            
            for k in range(n):
                Y[0, i, 2*n + k] = a_dict[Y[0, i, k]]
                Y[1, i, 2*n + k] = b_dict[Y[1, i, k]]

        return Y

class LayoutMutation(Mutation):

    def __init__(self, prob=1.0):
        super().__init__()
        self.prob = prob

    def mutation(self, y, start, end, inplace=True):
        y = y if inplace else np.copy(y)
        y[start:end + 1] = np.flip(y[start:end + 1])
        return y

    def _do(self, problem, X, **kwargs):
        Y = X.copy()
        _, n = Y.shape
        n = int(n/3)
        for i, y in enumerate(X):
            if np.random.random() < self.prob:
                start, end = np.sort(np.random.choice(n, 2, replace=False))
                for j in range(3):
                    Y[i, j*n:(j+1)*n] = self.mutation(y[j*n:(j+1)*n], start, end, inplace=True)

        return Y

alg = 'NSGA2' 
REPLAY = None # None or int (Setting the number activates the robot.)
robot = Robot(replay=REPLAY)

if REPLAY == None:

    robot_ = RobotLayout()

    if alg == 'NSGA2':
        algorithm = NSGA2(
            pop_size=100,
            sampling=LayoutSampling(),
            crossover=LayoutOXCrossover(),
            mutation=LayoutMutation(),
            eliminate_duplicates=True
        )
        res = minimize(robot_,
                    algorithm,
                    ('n_gen', 500),
                    seed=1,
                    verbose=True)

    print(res.F)
    print(res.X)

    Scatter().add(res.F).show()

    SHOW_FIG = True
    n_solution, n_var = res.X.shape
    n_box = n_var//3
    for i in range(n_solution):
        x = res.X[i, :]
        gp = x[0:n_box].tolist()
        gn = x[n_box: 2*n_box].tolist()
        rot = x[2*n_box:3*n_box].tolist()

        layout = Layout()
        layout.area(gp, gn, rot, True, i, SHOW_FIG)

    robot_.visualize()

else:
    robot.move()

robot.close_connection()