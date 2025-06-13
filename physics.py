#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples


from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from wpimath.system.plant import DCMotor
from wpimath.system.plant import DCMotor
from rev import SparkMaxSim, SparkAbsoluteEncoderSim
import typing
from wpilib import Mechanism2d, SmartDashboard, Color8Bit
from wpilib.simulation import (
    SingleJointedArmSim,
)
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from util import FalconSim

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller

        """
        This creates the simulation motors with the Sim class so all you have to do
        is call it with the motor from robot
        """

        self.left_drive_motor_sim = SparkMaxSim(robot.left1, DCMotor.NEO(2))
        self.right_drive_motor_sim = SparkMaxSim(
            robot.right1, DCMotor.NEO(2)
        )

        """allows importation of vars in robot.py anywhere in the class"""
        self.robot = robot

        bumper_width = 3.25


        """Create the tank drive( drive train on the bot) sim object"""
        self.drive_sim = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM, # motor configuration
            110, # This is an estimate 
            10.71, # drivetrain gear ratio
            2,  # motors per side
            22, # robot wheelbase which is the distance between the wheels
            23 + bumper_width * 2, # robot width with bumper width but x2 cause bumpers on both sides
            32 + bumper_width * 2, # robot length
            6 ,# wheel diameter
            timestep=20 # Normal loop time for bot which is time it takes to run all code in 
                        # one loop should take less than 0.02 sec or 20 mil sec
        )

        """Creates the gearbox for the arm sim"""
        self.arm_gearbox = DCMotor.falcon500(1)

        """Creates the sim object for arm sim"""
        self.arm_sim = SingleJointedArmSim(            
            self.arm_gearbox,
            73,
            0.1,  # Moment of enerta (ngl i dk how to calc cause i have not taken physics)
            0.6096,  # arm length 
            0.0, # min angle
            0.838, # Max angle
            True, # Bool to sim gravity or not
            0.0, # Starting angle
            [0, 0])


        """arm sim  componet creattion"""
        self.arm_encoder_sim = SparkAbsoluteEncoderSim(robot.right1)
        """Falcon sim is a custom lib in lemon lib but for this i just put in a util file"""
        self.arm_motor_sim = FalconSim(robot.arm, 0.1, 73)

        """Creates visuization elements for sim gui"""
        self.arm_sim_vis = Mechanism2d(20, 50)
        self.arm_root = self.arm_sim_vis.getRoot("Arm Root", 10, 0)
        self.arm_ligament = self.arm_root.appendLigament(
            "Arm", 10, 0, color=Color8Bit(0, 150, 0)
        )

        # Put Mechanism to SmartDashboard
        SmartDashboard.putData("Arm Sim", self.arm_sim_vis)

    def update_sim(self, now: float, tm_diff: float) -> None:

        """Drives the bot"""
        transform = self.drive_sim.calculate(self.left_drive_motor_sim, self.right_drive_motor_sim, tm_diff)
        pose = self.physics_controller.move_robot(transform)

        """Moves arm"""
        self.arm_sim.setInput(0, self.arm_motor_sim.getSetpoint())
        self.arm_sim.update(tm_diff)
        self.arm_encoder_sim.setPosition(self.arm_sim.getAngleDegrees())

        self.arm_ligament.setAngle(self.arm_sim.getAngleDegrees())