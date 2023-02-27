package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final class GripperConstants{
        public static final double gripperKP = 0.0;//TODO change the PID values
        public static final double gripperKI = 0.0;
        public static final double gripperKD = 0.0;
        public static final PIDController gripperPID = new PIDController(gripperKP, gripperKI, gripperKD);

        public static final double gripperKs = 0.0;//TODO change the feed forward values
        public static final double gripperKv = 0.0;
        public static final SimpleMotorFeedforward gripperFeedforward = new SimpleMotorFeedforward(gripperKs, gripperKv);

        public static final double gripperGearRatio = 100 / 1;

        public static final int gripperMotorPort = 11;

        public static final double cubeGrabingSetPoint = 0.0;//TODO change the set points
        public static final double coneGrabingSetPoint = 0.0;
        public static final double openGripperSetPoint = 0.0;

        public static final double grippingTolarance = 0.0;

        public static final int solenoidOpenID = 6;
        public static final int solenoidCloseID = 5;
        public static final int PCMID = 1;

        public static final int isGripperOpenID = 2;//TODO
    }

    public static final class BigArmConstants{
        //bigArm PID values
        private static final double bigArmKP = 0.002;//TODO change the PID values
        private static final double bigArmKI = 0.0;
        private static final double bigArmKD = 0.0001;
        public static final PIDController bigArmPID = new PIDController(bigArmKP, bigArmKI, bigArmKD);

        //bigArm feedforward values
        public static final double feedForwardVelocity = 0.0;

        // TODO: Update these to the arm's real physical constraints.
        private static final double bigArmMaxSpeed = 0;
        private static final double bigArmMaxAccel = 0;
        public static final TrapezoidProfile.Constraints bigArmProfileConstraints = new TrapezoidProfile.Constraints(bigArmMaxSpeed, bigArmMaxAccel);
        private static final double bigArmKs = 0.0;//TODO change th feed forward values
        private static final double bigArmKg = 0.0;
        private static final double bigArmKv = 0.0;
        public static final ArmFeedforward bigArmFeedforward = new ArmFeedforward(bigArmKs, bigArmKg, bigArmKv);

        public static final double bigArmGearRatio = 3000 / 1;// (60 Gear to 50 gear = 60 * 50 = 3000 - god knows why) 

        public static final int bigArmMotorPort = 4;

        public static final double ticksTolerance = 1000;

        public static final String bigArmPreferencesKey = "bigArmEncoder" ;

        //TODO: add setPoints
        public static final double coneUpperSetPoint = 0;
        public static final double coneMiddleSetPoint = 1666;

        public static final double cubeUpperFirstSetPoint = 44000;
        public static final double cubeUpperSecondSetPoint = 25000;
        public static final double cubeUpperFinalSetPoint = 47800;

        public static final double cubeMiddleSetPoint = -17668;

        public static final double intakeSetPoint = 16424;
        public static final double intakeReturnSetPoint = 37000;
        public static final double intakeReturnDeadZone = 48839;

        public static final double middleOfRobotSetPoint = 0.0;
        public static final double groundSetPoint = -18105;//-10626;

        public static final double honeSpeed = 0.4;
        public static final double honeSetPoint = 0.0;
        public static final int honeSwitchID = 1;
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
        public static final double closedLoopRamp = 0.0;

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
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values (Changes Per Module) */
        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values (Changes Per Robot) */
        public static final double driveKP = 0.15;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values (Changes Per Robot) */
        public static final double driveKS = (0.63964 / 12); // divided by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.6223 / 12);
        public static final double driveKA = (0.47062 / 12);


        /* Swerve Profiling Values (Changes Per Robot) - usually set by preference */
        /** meters / second */
        public static final double maxSpeed = 3;
        /** Percent output to motors. */
        public static final double maxPercentVelocity = 0.85; 
        /** radians / second */
        public static final double maxAngularVelocity = 5; 

        /* Swerve Slow Mode Reduce Values */
        public static final double slowModeSpeed = maxSpeed / 4;
        public static final double slowModeAngularVelocity = maxAngularVelocity / 2;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
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

        /* Charge Align Station Constants */
        public static final double ALIGN_STATION_SPEED = .2;
        public static final double STATION_PITCH_ANGLE_TOLERANCE = 2.5;

        /* where to align Constants */
        public static final double alignmentToleranceMeters = 0.0;//TODO: decied what is conciderd close enough

        // 3 scoring locations on each side.
        public static final List<Translation2d> cubeScoringLocations = List.of(new Translation2d[]{
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d()
        });

        // 6 scoring locations per side.
        public static final List<Translation2d> coneScoringLocations = List.of(new Translation2d[]{
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
        });
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

    public static final class LilArmConstants {
        /* LilArm PID Values */ 
        private static final double lilArmMotorsKP = 0.1;//TODO change the PID values
        private static final double lilArmMotorsKI = 0.0;
        private static final double lilArmMotorsKD = 0.0;
        
        public static final PIDController lilArmPID = new PIDController(lilArmMotorsKP, lilArmMotorsKI, lilArmMotorsKD);
    
        /* LilArm Feedforward Values */ 
        // TODO: Update these to the arm's real physical constraints.
        private static final double lilArmMaxSpeed = 0;
        private static final double lilArmMaxAccel = 0;
        public static final TrapezoidProfile.Constraints lilArmProfileConstraints = new TrapezoidProfile.Constraints(lilArmMaxSpeed, lilArmMaxAccel);

        public static final double lilArmMotorsKs = 0.0;//TODO change the feedforward values
        public static final double lilArmMotorsKg = 0.0;
        public static final double lilArmMotorsKv = 0.0;
        public static final ArmFeedforward lilArmFeedforward = new ArmFeedforward(lilArmMotorsKs, lilArmMotorsKg, lilArmMotorsKv);
    
        public static final String lilArmPreferencesKey = "lilArmEncoder";

        /* LilArm Motors ID */ 
        public static final int lilArmMotorID = 18;

        public static final int lilArmEncoderID = 3;

        /* LilArm Motors Gear Ratio */
        public static final double lilMotorGearRatio = 6000 / 1;//TODO change the values

        /* LilArm Solenoid Ports */
        public static final int PCMID = 1;
        public static final int lilArmSolenoidID = 3;

        public static final double ticksTolerance = 5;

                //TODO: add setPoints
        public static final double coneUpperSetPoint = 0.0;
        public static final double coneMiddleSetPoint = 1080;


        public static final double cubeUpperFirstSetPoint = -700;
        public static final double cubeUpperFinalSetPoint = -1740;

        public static final double cubeMiddleSetPoint = 1111;

        public static final double intakeSetPoint = 95;
        public static final double intakeReturnDeadZone =2345;

        public static final double middleOfRobotSetPoint = 0.0;
        public static final double groundSetPoint = 433;//449;

        public static final double resetPoint = -30;

    }

  
    public static final class VisionConstants {
        // Camera's name in Photon's NetworkTable (Set in the PhotonVision UI)
        public static final String cameraName = "limelightCam";

        /**
         * Camera's relative location to the center of the robot.
         * Translation in meters,
         * Rotation is in radians.
         * (Changes Per Robot)
         */
        public static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(0.115, 0.0, 0.367),
                        new Rotation3d(
                                0, 0,
                                0));
    }

    public static final class AutoConstants {
        // Self-explanatory, Limits for speed (linear velocity) and speed of rotation (angular velocity)
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
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

}
