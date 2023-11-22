#!/usr/bin/env python

from pydrake.all import *
import numpy as np

from optitrack_drake import OptiTrack
from controller import TemplateController

# Hardware parameters
hw_experiment_time = 5.0  # total for hardware experiment
realtime_rate = 1.0        # speed of simulation relative to real time

# Simulation parameters
config = MultibodyPlantConfig()
config.time_step = 1e-2
config.discrete_contact_solver = "sap"
config.contact_model = "point"

# Setup the Drake system diagram
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlant(config, builder)

# Add a flat ground with friction
ground_props = ProximityProperties()
AddContactMaterial(
        friction=CoulombFriction(static_friction=1.0, dynamic_friction=0.7),
        dissipation=0,
        properties=ground_props)
AddCompliantHydroelasticPropertiesForHalfSpace(
        slab_thickness=0.1,
        hydroelastic_modulus=1e7,
        properties=ground_props)
plant.RegisterCollisionGeometry(
        plant.world_body(),
        RigidTransform(),
        HalfSpace(),
        "ground_collision",
        ground_props)

# start meshcat
meshcat = StartMeshcat()

# Add the harpy robot
model_file = "./models/urdf/harpy_planar.urdf"
harpy = Parser(plant).AddModels(model_file)[0]

# add gravity to the sim
plant.gravity_field().set_gravity_vector([0, 0, 0])

plant.AddDistanceConstraint(
    plant.GetBodyByName("BallTarsusLeft"), [0, 0, 0],
    plant.GetBodyByName("BallFemurLeft"), [0, 0, 0],
    0.32)
plant.AddDistanceConstraint(
    plant.GetBodyByName("BallTarsusRight"), [0, 0, 0],
    plant.GetBodyByName("BallFemurRight"), [0, 0, 0],
    0.32)

# simepl PD controller for joint positions
Kp = 450 * np.ones(plant.num_actuators())
Kd = 50 * np.ones(plant.num_actuators())
actuator_indices = [JointActuatorIndex(i) for i in range(plant.num_actuators())]
for actuator_index, Kp, Kd in zip(actuator_indices, Kp, Kd):
    plant.get_joint_actuator(actuator_index).set_controller_gains(
        PdControllerGains(p=Kp, d=Kd)
    )
plant.Finalize()

# Add thrusters
left_thruster = builder.AddSystem(
        Propeller(plant.GetBodyByName("ThrusterLeft").index()))
right_thruster = builder.AddSystem(
        Propeller(plant.GetBodyByName("ThrusterRight").index()))

# combines forces of both thrusters
spatial_force_multiplexer = builder.AddSystem(  
        ExternallyAppliedSpatialForceMultiplexer(2))

# 
builder.Connect(
        plant.get_body_poses_output_port(),
        left_thruster.get_body_poses_input_port())
builder.Connect(
        plant.get_body_poses_output_port(),
        right_thruster.get_body_poses_input_port())

builder.Connect(
        left_thruster.get_spatial_forces_output_port(),
        spatial_force_multiplexer.get_input_port(0))
builder.Connect(
        right_thruster.get_spatial_forces_output_port(),
        spatial_force_multiplexer.get_input_port(1))
builder.Connect(
        spatial_force_multiplexer.get_output_port(),
        plant.get_applied_spatial_force_input_port())

thruster_demux = builder.AddSystem(
    Demultiplexer(2, 1))
builder.Connect(
        thruster_demux.get_output_port(0),
        left_thruster.get_command_input_port())
builder.Connect(
        thruster_demux.get_output_port(1),
        right_thruster.get_command_input_port())

# Add the controller
controller = builder.AddSystem(TemplateController())
opt = builder.AddSystem(OptiTrack())
# opt.stream_pose()
builder.Connect(
        opt.GetOutputPort("position"),
        controller.GetInputPort("position"))
builder.Connect(
        opt.GetOutputPort("quaternion"),
        controller.GetInputPort("quaternion"))
builder.Connect(
        plant.get_state_output_port(),
        controller.GetInputPort("x_hat"))
builder.Connect(
        controller.GetOutputPort("tau_ff"),
        plant.get_actuation_input_port())
builder.Connect(
        controller.GetOutputPort("x_nom"),
        plant.get_desired_state_input_port(harpy))
builder.Connect(
        controller.GetOutputPort("thrust"),
        thruster_demux.get_input_port())

AddDefaultVisualization(builder, meshcat)
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

# Set the initial condition
q0 = np.array([0, 0.514,     # base position (514 mm default height)
               0,            # base orientation
               0, 0,         # thrusters
               0, 0, 0, 0,   # right leg
               0, 0, 0, 0])  # left leg
v0 = np.array([0.0,0,          # base velocity
               0,            # base angular velocity
               0,0,          # thrusters angular velocity
               0,0,0,0,      # right leg joint angular velocity
               0,0,0,0])     # left leg joint  angular velocity
plant.SetPositions(plant_context, q0)
plant.SetVelocities(plant_context, v0)

# intialize the simulation
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(realtime_rate)
simulator.Initialize()

# run the simulation
meshcat.StartRecording()
simulator.AdvanceTo(hw_experiment_time)
meshcat.PublishRecording()


