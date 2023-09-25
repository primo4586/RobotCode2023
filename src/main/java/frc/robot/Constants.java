package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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

    public static final SupplyCurrentLimitConfiguration ARM_MOTOR_SUPPLY_CONFIG = new SupplyCurrentLimitConfiguration(true, 30, 35, 0.1); 

    public static final int ARM_STALL_CURRENT_LIMIT = 30;
    public static final int ARM_FREE_CURRENT_LIMIT = 35;

    public static final class BigConstants{
        public static final int bigArmMotorID = 5;

        public static final int homeSwitchID = 1;
        public static final double homeSpeed = -0.1;
        public static final double homeSetPoint = 0.0;

        //bigArm PID values
        public static final double bigArmKF = (58.3333*1023)/(2.29*25*2048);
        public static final double bigArmKP = 0.8002;
        public static final double bigArmKI = 0.0;
        public static final double bigArmKD = 0.29;
        public static final PIDController bigArmPID = new PIDController(bigArmKP, bigArmKI, bigArmKD);

        //bigArm feedforward values
        public static final double feedForwardVelocity = 0.0;


        public static final double bigArmGearRatio = 125 / 1;

        public static final double ticksTolerance = 500;

        public static final String bigArmPreferencesKey = "bigArmEncoder" ;

        public static final double coneUpperSetPoint = 41969;
        public static final double coneMiddleSetPoint = 20006;
        public static final double coneLowerSetPoint = -56326;


        public static final double cubeUpperSetPoint = 36823;
        public static final double cubeMiddleSetPoint = 23077;
        public static final double cubeLowerSetPoint = -56326;

        public static final double intakeSetPoint = 2325;
        public static final double intakeReturnSetPoint = 37000;
        public static final double intakeReturnDeadZone = 48839;

        public static final double highIntakeSetpoint = 13626;

        public static final double middleOfRobotSetPoint = 0.0;
        public static final double groundSetPoint = -12082;
        
        public static final double softLimitForward = 58982;
        public static final double softLimitReverse = -17166;

        public static int kTimeoutMs = 30;
        public static int kSlotIdx = 0;
        public static int kPIDLoopIdx = 0;


        public static final double ks =0.61094;
        public static final double kv =2.4998;
        public static final double ka =0.13149;
        
        public static final double maxSpeed = 3.42*25*2048/10;
        public static final double maxAcceleration = (((12) / ka)*25*2048)/10;
    }

    public static final class SwerveConstants {
        /* Gyro ID (Changes Per Robot) */
        public static final int pigeonID = 10;

        /* Invert gyro if necessary (Changes Per Robot) */
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants (Changes Per Robot) */
        public static final double trackWidth = 0.615;
        public static final double wheelBase = 0.615;
    

                                                                                  
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
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));//TODO: check if data is correct

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 50;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
        public static final SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                driveEnableCurrentLimit,
                driveContinuousCurrentLimit,
                drivePeakCurrentLimit,
                drivePeakCurrentDuration);

        /* Angle Motor PID Values (Changes Per Module) */
        public static final double angleKP = 0.1;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values (Changes Per Robot) */
        public static final double driveKP =  0.032;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values (Changes Per Robot) */
        public static final double driveKS = (0.12716);/// 12); // divided by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (3.1);// / 12);
        public static final double driveKA = (1.7636);// / 12);


        /* Swerve Profiling Values (Changes Per Robot) - usually set by preference */
        /** meters / second */
        public static final double maxSpeed = 4.117261723402446;
        /** Percent output to motors. */
        public static final double maxPercentVelocity = 1; 
        /** radians / second */
        public static final double maxAngularVelocity = Units.rotationsPerMinuteToRadiansPerSecond(3.80249338852179/2.732373*60);

        public static final double maxAcceleration = 7.6923076924;

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
        /* Front Right Module - Module 1  */
        public static final class FrontRightModule {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 1;
            public static final double angleOffset = 126.3;
            public static final SwerveModuleConstants constants = 

                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, false);
        }
        /* Front Left Module - Module 0 */
        public static final class FrontLeftModule {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 18;
            public static final int canCoderID = 2;
            public static final double angleOffset = 237.3;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, false);
        }

        
        /* Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 19;
            public static final int canCoderID = 3;
            public static final double angleOffset = 227.3;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, false);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int driveMotorID = 4;
            public static final int angleMotorID =20;
            public static final int canCoderID = 4;
            public static final double angleOffset = 257.3;
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

        public static final double blueAligningX = 1.83;
        public static final double redAligningX = 14.71;

        public static final double howCloseWeNeedToBe = 2.67;

        public static final double minAutoCollectRotation = 0.5;
        public static final double minAutoCollectSpeed = 0.5;
        
        public static final double trajAccuracy = 0.1; //TODO: change to a real number
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
        public static final double lilArmMotorsKF = (58.3333*1023)/(0.251*250*2048);
        public static final double lilArmMotorsKP = 0.96756;
        public static final double lilArmMotorsKI = 0.0;
        public static final double lilArmMotorsKD = 0.078266;//0.01278;

        /* LilArm Motors ID */
        public static final int lilArmMotorID = 6;

        public static final int lilArmEncoderID = 11;

        /* LilArm Motors Gear Ratio */
        public static final double lilMotorGearRatio = 250 / 1;

        public static final double ticksTolerance = 20;

        public static final double coneUpperSetPoint = 19560;
        public static final double coneMiddleSetPoint = 82021;
        public static final double coneLowerSetPoint = -56326;

        public static final double cubeUpperSetPoint = 66076;
        public static final double cubeMiddleSetPoint = 128809;
        public static final double cubeLowerSetPoint = -56326;

        public static final double middleOfRobotSetPoint = 175000;
        public static final double highIntakeSetpoint = 65037;
        public static final double groundSetPoint =  -185997;
        public static final double autoStartPoint = -136;

        public static final double resetPoint = -30;
        
        public static final double softLimitForward = 213000;
        public static final double softLimitReverse = -242499;

        public static int kTimeoutMs = 30;
        public static int kSlotIdx = 0;
        public static int kPIDLoopIdx = 0;

        public static final double ks =0.18105;
        public static final double kv =5.3859;
        public static final double ka =0.086304;
        
        public static final double maxSpeed =(1.23*250*2048*2)/10;//(1.23*250*2048*2)/10;
        public static final double maxAcceleration = (80*50*2048)/10;//(80*50*2048)/10;


    }

    public static final class TelescopicArmConstants {
        public static final int teleMotorID = 7;

        // TelescopicArm PID values
        public static final double TelesKF = 0;//(41.58333*1023)/(4.37*10*2048);
        public static final double TelesKP = 0.23562;
        public static final double TelesKI = 0.0;
        public static final double TelesKD = 0.049226;

        // TelescopicArm feedforward values
        public static final double feedForwardVelocity = 0.0;

        public static final double TelesGearRatio = 10 / 1;

        public static final int TelesEncoderID = 5;

        public static final double ticksTolerance = 500;

        public static final int homeSwitchID = 0;

        public static int kTimeoutMs = 30;
        public static int kSlotIdx = 0;
        public static int kPIDLoopIdx = 0;

        public static final double coneUpperSetPoint = 41280;
        public static final double coneMiddleSetPoint = 32531;
        public static final double coneLowerSetPoint = -56326;

        public static final double cubeUpperSetPoint = 35505;
        public static final double cubeMiddleSetPoint = 24239;
        public static final double cubeLowerSetPoint = -56326;

        public static final double highIntakeSetpoint = 28099;
        public static final double middleOfRobotSetPoint = 1100;
        public static final double groundSetPoint = 22728;

        public static final double softLimitForward = 48000;
        public static final double softLimitReverse = 1000;

        public static final double ks =0.064344;
        public static final double kv =1.0843;
        public static final double ka =0.012619;
        
        public static final double maxSpeed =(4.37*10*2048)/10;
        public static final double maxAcceleration = (388*10*2048)/10;
    }
   
    public static final class VisionConstants {
        // Camera's name in Photon's NetworkTable (Set in the PhotonVision UI)
        public static final String rightCameraName = "rightCam";
        public static final String leftCameraName = "leftCam";
        public static final String limeLightCameraName = "OV5647";

        /**
         * Camera's relative location to the center of the robot.
         * Translation in meters,
         * Rotation is in radians.
         * (Changes Per Robot)
         */
        public static final Transform3d rightRobotToCam =
                new Transform3d(
                        new Translation3d( 0.317,-217, 0.235),
                        new Rotation3d(
                                0, 0,
                                Units.degreesToRadians(4)));
                                
        public static final Transform3d leftRobotToCam =
        new Transform3d(
                new Translation3d( 0.309,0.205, 0.235),
                new Rotation3d(
                        0, 0,
                        Units.degreesToRadians(7)));


        public static final Transform3d limeLightRobotToCam =
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
}
