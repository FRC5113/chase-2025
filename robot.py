import wpilib
import wpilib.drive
import rev
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix5 import TalonSRX 
from phoenix5 import NeutralMode
from navx import AHRS
import math

class myRobot(wpilib.TimedRobot):
    def robotInit(self):
        BRUSHLESS = rev.SparkMax.MotorType.kBrushless
        self.talon_config = TalonFXConfiguration()
        self.left1 = rev.SparkMax(12, BRUSHLESS)
        self.left2 = rev.SparkMax(14, BRUSHLESS)
        self.right1 = rev.SparkMax(11, BRUSHLESS)
        self.right2 = rev.SparkMax(13, BRUSHLESS)
        self.arm = TalonFX(12) #make sure can ID is correct
        self.spinner = TalonSRX(40) #make sure CAN ID is correct

        self.configureMotor(self.left1)
        self.configureMotor(self.left2, self.left1)
        self.configureMotor(self.right1)
        self.configureMotor(self.right2, self.right1)

        self.xbox = wpilib.XboxController(0)

        self.drive = wpilib.drive.DifferentialDrive(self.left1, self.right1)

        self.gyro = AHRS.create_spi()
        
    def teleopInit(self):
        self.configureMotor(self.left1, coast = False)
        self.configureMotor(self.left2, coast = False)
        self.configureMotor(self.right1, coast = False)
        self.configureMotor(self.right2, coast = False)
        self.configureMotor(self.arm, coast = False)
        self.configureMotor(self.spinner, coast = False)

        #Set current yaw to zero
        self.gyro.zeroYaw()

    def disabledExit(self):
        self.configureMotor(self.left1)
        self.configureMotor(self.left2)
        self.configureMotor(self.right1)
        self.configureMotor(self.right2)
    # 21 and 54 are right
    # 52 and 51 are left
    def configureMotor(self, motor: rev.SparkMax|TalonFX|TalonSRX, follow: rev.SparkMax = None,coast:bool = True) -> None:
        if isinstance(motor,rev.SparkMax):
            if follow is not None:
                motor.configure(
                    rev.SparkMaxConfig().follow(follow.getDeviceId()),
                    rev.SparkMax.ResetMode.kResetSafeParameters,
                    rev.SparkMax.PersistMode.kPersistParameters,
                )
            if coast:
                motor.configure(
                    rev.SparkMaxConfig().setIdleMode(rev.SparkMaxConfig.IdleMode.kCoast),
                    rev.SparkMax.ResetMode.kResetSafeParameters,
                    rev.SparkMax.PersistMode.kPersistParameters,
                )
            else:
                motor.configure(
                    rev.SparkMaxConfig().setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake),
                    rev.SparkMax.ResetMode.kResetSafeParameters,
                    rev.SparkMax.PersistMode.kPersistParameters,
                )
        elif isinstance(motor, TalonFX) or isinstance(motor, TalonSRX):
            if follow is not None:
                print("Chase, you are a moron")
            #Ryan, is this code correct
            if coast:
                motor.setNeutralMode(NeutralMode.Coast)
            else:
                motor.setNeutralMode(NeutralMode.Brake)
    def calcDrive(self, x: float, y: float) -> [float, float]:
        current_angle = self.gyro.getAngle() % 360

        x = math.

    def teleopPeriodic(self):
        #Set drive
        #self.drive.arcadeDrive(-self.xbox.getLeftY(), self.xbox.getRightX())

        #Set Arm
        lt = self.xbox.getLeftTriggerAxis()
        rt = self.xbox.getRightTriggerAxis()
        if(lt > rt and lt > -1):
            self.arm.set(lt)
        elif(rt > lt and rt > -1):
            self.arm.set(-rt)

        #Set spinner
        lb = self.xbox.getLeftBumper()
        rb = self.xbox.getRightBumper()

        if lb:
            self.spinner.set(value = 1)
        elif rb:
            self.spinner.set(value = -1)
            
        
