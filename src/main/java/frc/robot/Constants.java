package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final int armContinuousCurrentLimit = 35;
    public static final int armPeakCurrentLimit = 50;
    public static final double armPeakCurrentDuration = 0.1;
    public static final boolean armEnableCurrentLimit = true;

    public static final int ARM_STALL_CURRENT_LIMIT = 30;
    public static final int ARM_FREE_CURRENT_LIMIT = 35;

    public static final class GripperConstants{
        public static final int gripperMotorPort = 11;

        public static final int solenoidOpenID = 6;
        public static final int solenoidCloseID = 3;
        public static final int PCMID = 44;

        public static final int isGripperOpenID = 2;
    }

    public static final class BigConstants{
        //bigArm PID values
        public static final double bigArmKV = 0.0;
        public static final double bigArmKP = 0.002;
        public static final double bigArmKI = 0.0;
        public static final double bigArmKD = 0.0002;
        public static final double bigArmKS = 0.0002;

        //bigArm feedforward values
        public static final double feedForwardVelocity = 0.0;


        public static final double bigArmGearRatio = 125 / 1;

        public static final int bigArmMotorID = 16;
        public static final int bigArmEncoderID = 5;

        public static final double ticksTolerance = 500;

        public static final String bigArmPreferencesKey = "bigArmEncoder" ;

        public static final double coneUpperSetPoint = -78853;//-88152;
        public static final double coneMiddleSetPoint = -47773;


        public static final double cubeUpperSetPoint = -72463;

        public static final double cubeMiddleSetPoint = -56326;

        public static final double intakeSetPoint = 2838;//-1711;
        public static final double intakeReturnSetPoint = 37000;
        public static final double intakeReturnDeadZone = 48839;

        public static final double highIntakeSetpoint = -28557;

        public static final double middleOfRobotSetPoint = 0.0;
        public static final double groundSetPoint = 9083;//-84986;
        public static final double groundSetPoint2 = 10000;

        public static final double homeSpeed = 0.1;
        public static final double homeSetPoint = 0.0;
        public static final int homeSwitchID = 1;

        public static final double maxSpeed = 0;
        public static final double maxAcceleration = 0;
        public static final double maxJerk = 0;

        public static final int kTimeoutMs = 30;
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
    }

    public static final class SwerveConstants {
        /* Gyro ID (Changes Per Robot) */
        public static final int pigeonID = 18;

        /* Invert gyro if necessary (Changes Per Robot) */
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants (Changes Per Robot) */
        public static final double trackWidth = 0.61;
        public static final double wheelBase = 0.61;
    

                                                                                  
        /* Wheel Constants (Changes Per Robot's Wheels) */ 
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final ClosedLoopRampsConfigs closedLoopRamp = new ClosedLoopRampsConfigs();

        /* Gear Ratios (Changes Per Module) */
        public static final double driveGearRatio = (8.14 / 1.0); 
        public static final double angleGearRatio = (150 / 7.0) / 1.0; 

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 50;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values (Changes Per Module) */
        public static final double angleKP = 0.1;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values (Changes Per Robot) */
        public static final double driveKP = 0.3;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values (Changes Per Robot) */
        public static final double driveKS = (0.22754/ 12); // divided by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.6777 / 12);
        public static final double driveKA = (0.45944 / 12);


        /* Swerve Profiling Values (Changes Per Robot) - usually set by preference */
        /** meters / second */
        public static final double maxSpeed = 5;
        /** Percent output to motors. */
        public static final double maxPercentVelocity = 1; 
        /** radians / second */
        public static final double maxAngularVelocity = 5;

        public static double voltageComp = 12;

        /* Swerve Slow Mode Reduce Values */
        public static final double slowModeSpeed = maxSpeed / 4;
        public static final double slowModeAngularVelocity = 2;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts - Ensure that motors are CCW+!!! (Changes Per Module)*/
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert - Ensure that CANCoders are CCW+!! (Changes Per Module) */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants (Changes Per Robot) */
        /* Front Left Module - Module 0 */
        public static final class FrontLeftModule {
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 15;
            public static final int canCoderID = 14;
            public static final double angleOffset = 315.43;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, false);
        }
        /* Front Right Module - Module 1  */
        public static final class FrontRightModule {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 13;
            public static final double angleOffset = 325.8;
            public static final SwerveModuleConstants constants = 

                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, false);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 11;
            public static final double angleOffset = 306.562;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, false);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 0;
            public static final double angleOffset = 257.7;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, false);
        }

        //new charge station align attempt


        public static final double otherSideSpeed = 1.8;

        /* Charge Align Station Constants */
        public static final double ALIGN_STATION_SPEED = .5;
        public static final double STATION_PITCH_ANGLE_TOLERANCE = 2.5;

        /* where to align Constants */

        public static final double[] blueAligningYAxis = {20, 42.19, 64, 86, 108.19, 130, 152, 174.19, 196};
        public static final double[] redAligningYAxis = {196, 174.19, 152, 130, 108.19, 86, 64, 42.19, 20};

        public static final double blueAligningX = 0;
        public static final double redAligningX = 0;

        public static final double howCloseWeNeedToBe = 1.541526;
    }

    public static final class Misc {
        /* Gyro Align Robot PID Values */
        public static final double gyroAlignKP = 0.3;
        public static final double gyroAlignKI = 0.0;
        public static final double gyroAlignKD = 0.0;
        public static final double gyroAlignKF = 0.0;
        
        // Deadband for joysticks. To deny too small of joystick values.
        public static final double stickDeadband = 0.1;
    }

    public static final class LilConstants {
        /* LilArm PID Values */
        public static final double lilArmKS = 0.01;
        public static final double lilArmKV = 0.01;
        public static final double lilArmKP = 0.01;
        public static final double lilArmKI = 0.0;
        public static final double lilArmKD = 0.0002;

        /* LilArm Motors ID */
        public static final int lilArmMotorID = 18;

        public static final int lilArmEncoderID = 3;

        /* LilArm Motors Gear Ratio */
        public static final double lilMotorGearRatio = 50 / 1;

        /* LilArm Solenoid Ports */
        public static final int PCMID = 44;
        public static final int lilArmSolenoidID = 4;

        public static final double ticksTolerance = 20;

        public static final double coneUpperSetPoint = -1738;
        public static final double coneMiddleSetPoint = -1187;
        public static final double cubeUpperSetPoint = -1420;
        public static final double cubeMiddleSetPoint = -940;
        public static final double middleOfRobotSetPoint = -184;
        public static final double highIntakeSetpoint = -1010;
        public static final double groundSetPoint = -3302;
        public static final double autoStartPoint = -136;

        public static final double resetPoint = -30;
        
        public static final double maxSpeed = 0;
        public static final double maxAcceleration = 0;
        public static final double maxJerk = 0;

        public static int kTimeoutMs = 30;
        public static int kSlotIdx = 0;
        public static int kPIDLoopIdx = 0;

    }

    public static final class TelescopicArmConstants {
        public static final int teleMotorID = 0;

        // TelescopicArm PID values
        public static final double TelesKV = 0.0;
        public static final double TelesKS = 0.0;
        public static final double TelesKP = 0.002;
        public static final double TelesKI = 0.0;
        public static final double TelesKD = 0.0002;
        public static final PIDController TelesPID = new PIDController(TelesKP, TelesKI, TelesKD);

        // TelescopicArm feedforward values
        public static final double feedForwardVelocity = 0.0;

        public static final double TelesGearRatio = 125 / 1;

        public static final int TelesEncoderID = 5;

        public static final double ticksTolerance = 500;

        public static final int homeSwitchID = 2;

        public static final double maxSpeed = 0;
        public static final double maxAcceleration = 0;
        public static final double maxJerk = 0;

        public static int kTimeoutMs = 30;
        public static int kSlotIdx = 0;
        public static int kPIDLoopIdx = 0;

        public static final double coneUpperSetPoint = -78853;
        public static final double coneMiddleSetPoint = -47773;

        public static final double cubeUpperSetPoint = -72463;
        public static final double cubeMiddleSetPoint = -56326;

        public static final double highIntakeSetpoint = -28557;
        public static final double middleOfRobotSetPoint = 0.0;
        public static final double groundSetPoint = 9083;
    }
   
    public static final class VisionConstants {
        // Camera's name in Photon's NetworkTable (Set in the PhotonVision UI)
        public static final String rightCameraName = "Arducam_OV9281_USB_Camera";
        public static final String leftCameraName = "Arducam_OV9281_USB_Camera";

        /**
         * Camera's relative location to the center of the robot.
         * Translation in meters,
         * Rotation is in radians.
         * (Changes Per Robot)
         */
        public static final Transform3d rightRobotToCam =
                new Transform3d(
                        new Translation3d( 0.31,-0.205, 0.23),
                        new Rotation3d(
                                0, 0,
                                Units.degreesToRadians(2)));
                                
        public static final Transform3d leftRobotToCam =
        new Transform3d(
                new Translation3d( 0.31,-0.205, 0.23),
                new Rotation3d(
                        0, 0,
                        Units.degreesToRadians(2)));
    }

    public static final class AutoConstants {
        // Self-explanatory, Limits for speed (linear velocity) and speed of rotation (angular velocity)
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared =3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared =  3* Math.PI;
        public static final PathConstraints pathConstraints = new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1.3;//2.5;
    
        // Basically a PID controller that also uses the physical limits of the swerve (max speed & max acceleration) to not go over the maximum values
        // Currently is unused in PathPlanner trajectories. Might be deprecated for auto path following use.
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final boolean tuningMode = true;
}
