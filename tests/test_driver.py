import numpy as np
import time
from pydrake.all import (
    PiecewisePolynomial,
    DiagramBuilder,
    TrajectorySource,
    LcmPublisherSystem,
    DrakeLcm,
    AbstractValue,
    Simulator,
    ZeroOrderHold,
    TriggerType,
)
import lcm
from drake_ur_driver import lcmt_ur_command, lcmt_ur_status
from pydrake.all import LeafSystem, BasicVector


class TrajectoryToUrCommand(LeafSystem):
    """
    A system that converts a trajectory to a lcmt_ur_command message.
    """

    def __init__(self, num_joints):
        LeafSystem.__init__(self)
        self.num_joints = num_joints
        self.DeclareVectorInputPort("trajectory", BasicVector(self.num_joints))
        self.DeclareAbstractOutputPort(
            "ur_command", lambda: AbstractValue.Make(lcmt_ur_command()), self.CalcOutput
        )

    def CalcOutput(self, context, output):
        trajectory = self.get_input_port().Eval(context)
        command = lcmt_ur_command()
        command.utime = int(time.time() * 1e6)
        command.joint_position = trajectory
        command.control_mode_expected = lcmt_ur_command.kPosition
        output.set_value(command)

ur_status = None
def my_handler(channel, data):
    global ur_status
    ur_status = lcmt_ur_status.decode(data)
    
lc = lcm.LCM("")
sub = lc.subscribe("UR_STATUS", my_handler)
lc.handle()

# Define the times and control points for the trajectory.
times = [0.0, 15.0, 30.0, 45.0]
# The values at each time point.
# Each column represents a different time. Each row is a different joint.
control_points = np.array(
    [
        [np.pi / 2, -np.pi / 2, np.pi / 2],  # Joint 1 angle
        [np.pi / 2, -np.pi / 2, np.pi / 2],  # Joint 2 angle
        [2 * np.pi / 3, -2 * np.pi / 3, 2 * np.pi / 3],  # Joint 3 angle
        [np.pi / 4, -np.pi / 4, np.pi / 4],  # Joint 4 angle
        [np.pi / 2, 0, np.pi / 2],  # Joint 5 angle
        [0.0, -np.pi / 4, 0.0],  # Joint 6 angle
    ]
)
control_points = np.hstack((np.array(ur_status.joint_position).reshape((6, 1)), control_points))

# Create the trajectory object.
trajectory = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
    times, control_points
)

# Create a source that outputs the trajectory.
builder = DiagramBuilder()
traj_source = builder.AddSystem(TrajectorySource(trajectory))
ur_command = builder.AddSystem(TrajectoryToUrCommand(num_joints=6))
builder.Connect(traj_source.get_output_port(), ur_command.get_input_port())
l = DrakeLcm()
pub = LcmPublisherSystem.Make(
    channel="UR_COMMAND",
    lcm_type=lcmt_ur_command,
    lcm=l,
    publish_triggers={TriggerType.kPeriodic},
    publish_period=1/125.0,
    # use_cpp_serializer=True,
)

# Add a ZeroOrderHold system for state updates.
position_zero_order_hold = builder.AddSystem(ZeroOrderHold(1 / 125.0, 6))
builder.Connect(
    traj_source.get_output_port(), position_zero_order_hold.get_input_port()
)

builder.AddSystem(pub)
builder.Connect(ur_command.get_output_port(), pub.get_input_port())

diagram = builder.Build()

# Create a default context for the diagram.
diagram_context = diagram.CreateDefaultContext()

simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.Initialize()
simulator.AdvanceTo(45.0)
