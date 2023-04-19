# bicyclemodel_mpc

### Author : Dinh Ngoc Duc
### Created : 2023/04/19

### Memo 

### Execute code :
Run "src/mpc_execute.m"


## MATLAB : Bicycle model guidance

class Model
=> model
=> <constructor> (...)
=> state = updateModel(input)

class Controller
=> controller
=> controller.model = model
=> <constructor> (...)
=> state = execute(reference, measurement)

class Trajectory
=> read

class Measurement
=> addNoise

class Simulation
=> simulation
=> simulation.controller = controller
=> simulation.trajectory = trajectory
=> simulation.measurement = measurement
=> reference = trajectory.read
=> for <state = execute(reference, measurement)> <measurement.addNoise()> <states = [states; state] end

class Animation
=> annimation
=> annimation.simulation = simulation
=> animate(states)
