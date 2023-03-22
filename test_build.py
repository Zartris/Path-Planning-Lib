# go to build folder and run this script:
# make && cp ../test_build.py . && python test_build.py

import numpy as np
from path_planning_lib import MAPFProblemDefinition as MPD
from path_planning_lib import PIBTSolver, Plan


def make_test_problem():
    grid_map = np.zeros((40, 40), dtype=int)
    grid_map[20, :-2] = 1
    np.random.seed(0)
    edge_cost = np.random.randint(0, 10, size=(40, 40), dtype=int).tolist()
    print("test constructor")
    problem = MPD(
        _instance_name="test",
        _seed=0,
        _max_comp_time=100,
        _max_timestep=100,
        _num_agents=4,
        grid_map=grid_map.tolist(),
        edge_cost_moving_up=edge_cost,
        edge_cost_moving_down=edge_cost,
        edge_cost_moving_left=edge_cost,
        edge_cost_moving_right=edge_cost
    )
    print("test constructor done")
    print("test setConfig")
    problem.setConfig(
        start_pos=np.array([[0, 0], [0, 1], [0, 2], [0, 3]], dtype=int).tolist(),
        goal_pos=np.array([[39, 39], [39, 38], [39, 37], [39, 36]], dtype=int).tolist(),
        priority=np.array([0, 1, 2, 3], dtype=int).tolist()
    )
    print("test setConfig done")
    return problem


problem = make_test_problem()
print("test setWellFormedInstance")
problem.setWellFormedInstance()
print("test setWellFormedInstance done")
print("test setRandomStartsGoals")
problem.setRandomStartsGoals()
print("test setRandomStartsGoals done")
print("test setMaxCompTime")
problem.setMaxCompTime(100)
print("test setMaxCompTime done")

problem = make_test_problem()
print("test pibt solver consturctor")
solver = PIBTSolver(problem)
print("test pibt solver consturctor done")
print("test pibt solver solve")
solver.solve()
print("test pibt solver solve done")
solver.printResult()

sol: Plan = solver.getSolution()
print("Test solver.getPath\n", sol.getPath(0), "\n Done")
print("Test solver.getPathXY\n", sol.getPathXY(0), "\n Done")
print("Test getAllPaths\n", sol.getAllPaths(), "\n Done")
print("Test getAllPathsXY\n", sol.getAllPathsXY(), "\n Done")

print("==============================================")
print("===============RECONFIG=====================")
print("==============================================")
solver.setConfig(
    start_pos=np.array([[0, 0], [0, 1], [0, 2], [0, 3]], dtype=int).tolist(),
    goal_pos=np.array([[22, 22], [23, 22], [24, 22], [25, 22]], dtype=int).tolist(),
    priority=np.array([0, 1, 2, 3], dtype=int).tolist()
)
solver.solve()
sol: Plan = solver.getSolution()
print("Test solver.getPath\n", sol.getPath(1), "\n Done")
print("Test solver.getPathXY\n", sol.getPathXY(1), "\n Done")
print("Test getAllPaths\n", sol.getAllPaths(), "\n Done")
print("Test getAllPathsXY\n", sol.getAllPathsXY(), "\n Done")
print("Test solver.getPathToGoal\n", sol.getPathToGoal(1), "\n Done")
print("Test solver.getPathToGoalXY\n", sol.getPathToGoalXY(1), "\n Done")