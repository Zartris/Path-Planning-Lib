from path_planning_lib import MAPFPersistentProblemDefinition
import numpy as np

grid_map = np.zeros((40, 40), dtype=int)
grid_map[20, :] = 1
np.random.seed(0)
edge_cost = np.random.randint(0, 10, size=(40, 40), dtype=int).tolist()
print("test constructor")
problem = MAPFPersistentProblemDefinition(
    _instance_name="test",
    _seed=0,
    _max_comp_time=100,
    _max_timestep=100,
    _num_agents=4,
    _grid_with_speed=True,
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
print("test setWellFormedInstance")
problem.setWellFormedInstance()
print("test setWellFormedInstance done")
print("test setRandomStartsGoals")
problem.setRandomStartsGoals()
print("test setRandomStartsGoals done")
print("test setMaxCompTime")
problem.setMaxCompTime(100)
print("test setMaxCompTime done")
