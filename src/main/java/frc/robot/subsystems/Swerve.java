package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Misc;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.VisionPoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

    // Swerve Basic Components
    private SwerveModule[] mSwerveMods; // Order for modules is always FL, FR, BL, BR
    private PigeonIMU gyro;

    // To avoid gyro resetting messing with odometry data, during teleop the driver
    // resets the offset to the current gyro angle, rather then resetting the gyro
    // itself
    private double teleopRotationOffset = 0;

    // Field visualizer for debug purposes
    private Field2d field2d = new Field2d();

    // Uses vision data to estimate the robot's position on the field.
    private VisionPoseEstimator visionPoseEstimator;

    // Uses the swerve's encoders to estimate it's position on the field, given a
    // starting position. (Like a Odometer on cars, but instead of counting kms,
    // this counts the offset from a starting position)
    private SwerveDriveOdometry swerveOdometry;

    // Uses the odometry & vision pose estimator, to fuse their data together in
    // order to better estimate the robot's position on the field.
    private final SwerveDrivePoseEstimator poseEstimation;

    public Swerve() {

        gyro = new PigeonIMU(new TalonSRX(SwerveConstants.pigeonID));
        gyro.configFactoryDefault();
        zeroGyro();

        SmartDashboard.putData(field2d);
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, SwerveConstants.FrontLeftModule.constants),
                new SwerveModule(1, SwerveConstants.FrontRightModule.constants),
                new SwerveModule(2, SwerveConstants.BackLeftModule.constants),
                new SwerveModule(3, SwerveConstants.BackRightModule.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());
        poseEstimation = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(),
                getPositions(), swerveOdometry.getPoseMeters());
        /**
         * Sets how much does the SwervePoseEstimator trusts vision data. See WPILib
         * documentation & the SwervePoseEstimator class for more.
         */
        poseEstimation.setVisionMeasurementStdDevs(VecBuilder.fill(0.3, 0.3, 0.3));
    }

    /**
     * Drives the robot (mostly) during teleop control.
     * 
     * @see TeleopSwerve
     * 
     * @param translation   Movement of the robot on the X & Y plane (meters /
     *                      second)
     * @param rotation      Movement in rotation. (radians / second)
     * @param fieldRelative If the robot should move relative to the field or the
     *                      robot.
     * @param isOpenLoop    If the robot should use PID & FF to correct itself and
     *                      be more accurate
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds chasisSpeeds;

        // Gets the current
        if (fieldRelative)
            chasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    isOpenLoop ? getTeleopYaw() : getYaw());
        else
            chasisSpeeds = new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation);

        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics
                .toSwerveModuleStates(chasisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Stops the modules in place
     * (Sets 0 m/s as setpoints and 0 degrees for the rotation setpoint using
     * OpenLoop)
     */
    public void stopModules() {
        for (SwerveModule module : mSwerveMods) {
            module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
        }
    }

    /**
     * Sets the desired states for each module. Using closed loop control.
     * 
     * @param desiredStates Desired states for each module as a SwerveModuleState
     *                      array.
     */
    public void setModuleStatesClosedLoop(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }


    public void setModulesStatesClosedLoopReveresed(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredStateReveresed(desiredStates[mod.moduleNumber]);
        }
    }

    /**
     * Gets the current robot's estimated position on the field.
     * If VisionPoseEstimator is not set, returns only the position using odometry
     * data,
     * otherwise the position is fused from both of them in SwervePoseEstimator.
     * 
     * @return The current robot's position on the field, as a Pose2d object.
     */
    public Pose2d getPose() {
        return poseEstimation.getEstimatedPosition();
    }

    /**
     * Resets the current odometry/pose estimator's position to the given pose.
     * 
     * @param pose pose to reset the odometry / pose estimator to.
     */
    public void resetPose(Pose2d pose) {
        poseEstimation.resetPosition(getYaw(), getPositions(), pose);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    /**
     * Zeros the teleop gyro offset to be the same as the gyro's current angle.
     */
    public void zeroTeleopGyro() {
        teleopRotationOffset = getYaw().getDegrees();
    }

    /**
     * Returns the current gyro's yaw angle.
     * 
     * @return The current gyro's yaw angle in a Rotation2d object.
     */
    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0])
                : Rotation2d.fromDegrees(ypr[0]);
    }

    public double getPitch() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return ypr[1];
    }

    /**
     * Returns the current gyro's angle offset by the teleop rotation offset.
     *
     * To avoid gyro resetting messing with odometry data, during teleop the driver
     * resets the offset to the current gyro angle, rather then resetting the gyro
     * itself. Therefore having a teleop gyro value and a normal gyro value.
     *
     * @return "Teleop Gyro"'s current angle in a Rotation2d object.
     */
    public Rotation2d getTeleopYaw() {
        return getYaw().minus(Rotation2d.fromDegrees(teleopRotationOffset));
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    @Override
    public void periodic() {
        updateOdometry();

        SmartDashboard.putNumber("Gyro", getYaw().getDegrees());
        SmartDashboard.putNumber("Roll", getRoll());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Teleop Gyro", getTeleopYaw().getDegrees());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    public void updateOdometry() {
        poseEstimation.update(getYaw(), getPositions());
        swerveOdometry.update(getYaw(), getPositions());
        field2d.getObject("Odometry").setPose(swerveOdometry.getPoseMeters());

        if (visionPoseEstimator != null) {
            var result = visionPoseEstimator
                    .getEstimatedGlobalPose(poseEstimation.getEstimatedPosition());

            if (result != null) {
                poseEstimation.addVisionMeasurement(result.estimatedPose.toPose2d(), result.timestampSeconds);
                field2d.getObject("Vision Position").setPose(result.estimatedPose.toPose2d());
            }
        }

        field2d.setRobotPose(getPose());
    }

    public Command driveForwardUntilMeters(double driveSpeed, double metersSetpoint) {
        return run(() -> {
            drive(new Translation2d(driveSpeed, 0), metersSetpoint, true, false);

        }).until(() -> swerveOdometry.getPoseMeters().getTranslation().getX() == metersSetpoint);
    }

    /**
     * Sets the pose estimator to use.
     * Setting it to null means the swerve won't utilize the vision pose estimator.
     * 
     * @param visionPoseEstimator vision pose estimator to go off of.
     */
    public void setVisionPoseEstimator(VisionPoseEstimator visionPoseEstimator) {
        this.visionPoseEstimator = visionPoseEstimator;
    }

    /**
     * Returns modules' encoder positions (Drive Motors) and the module's angle -
     * used for Odometry & Pose Estimator
     * 
     * @return An array of SwerveModulePosition objects containing that data
     * @see SwerveModulePosition
     */
    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                mSwerveMods[0].getPostion(),
                mSwerveMods[1].getPostion(),
                mSwerveMods[2].getPostion(),
                mSwerveMods[3].getPostion(),
        };
    }

    
    /**
     * Follows a given trajectory from PathPlanner
     * 
     * @param trajectory          Trajectory to follow
     * @param shouldResetOdometry Should odometry be reset before following the
     *                            trajectory or not
     * @return Returns a sequential command group to reset odometry, and follow the
     *         path, according to the arguments.
     * 
     * @apiNote In 2023, the field is rotationally asymetric,
     *          as a cause of that trajectories have to flip if
     *          you're on the red side.
     *          To have the trajectory work for both sides, use
     *          the overload method with the useAllianceColor argument instead of
     *          this one.
     */
    public Command followTrajectory(PathPlannerTrajectory trajectory, boolean shouldResetOdometry) {

        PPSwerveControllerCommand followTrajecotryControllerCommand = new PPSwerveControllerCommand(
                trajectory,
                this::getPose,
                SwerveConstants.swerveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                new PIDController(AutoConstants.kPThetaController, 0, 0),
                this::setModuleStatesClosedLoop,
                false, // Assuming we don't need to flip the trajectory, we set this to false.
                this);

        InstantCommand resetOdometryBeforePath = new InstantCommand(() -> {
            if (shouldResetOdometry)
                resetPose(trajectory.getInitialHolonomicPose());
        }, this);
        return resetOdometryBeforePath.andThen(followTrajecotryControllerCommand);

    }

    /**
     * Follows a given trajectory from PathPlanner
     * In 2023, the field is rotationally asymetric,
     * as a cause of that trajectories have to flip if
     * you're on the red side. This follow trajectory
     * command accounts for that.
     * 
     * @param trajectory          Trajectory to follow
     * @param shouldResetOdometry Should odometry be reset before following the
     *                            trajectory or not
     * 
     * @return ProxyCommand which regenerates the command so that on
     *         the start auto, we would have the Driver Station data to know
     *         what alliance we are, and flip the trajectory if necessary.
     */
    public Command followTrajectoryModifiedToAlliance(PathPlannerTrajectory trajectory, boolean shouldResetOdometry, boolean reverseInputs) {

        Supplier<Command> followCommandSupplier = () -> {
            // Modifies trajectory in case we need to because of our alliance
            PathPlannerTrajectory modifiedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory,
                    DriverStation.getAlliance());

            return reverseInputs ? followTrajectoryWithReveresedInputs(modifiedTrajectory, shouldResetOdometry) : followTrajectory(modifiedTrajectory, shouldResetOdometry);
        };

        return new ProxyCommand(followCommandSupplier);
    }

    public Command driveForTimeAtSpeed(Translation2d speed, double timeSeconds) {
        Timer timer = new Timer();
        return run(() -> {
            drive(speed, 0, true, true);
        })
        .beforeStarting(() -> timer.start())
        .until(() -> timer.hasElapsed(timeSeconds));
    }

    public Command driveUntilPitchChangeAtSpeed(double speed, double angleDeltaTolerance, double delay) {
        double[] startingPitch = {0};
        Timer timer = new Timer();
        return runOnce(() -> {
            startingPitch[0] = getRoll();
            timer.start();
        }).andThen(
            run(() -> 
                drive(new Translation2d(speed, 0), 0, true, true)
            )
        ).until(() -> {
            double deltaAngle = startingPitch[0] - getRoll();
            return timer.hasElapsed(delay) && deltaAngle >= angleDeltaTolerance;
        });
    }


    public Command driveUntilPitchChangeAtSpeed(double speed, double angleDeltaTolerance) {
        double[] startingPitch = {0};
        return runOnce(() -> {
            startingPitch[0] = getRoll();
        }).andThen(
            run(() -> 
                drive(new Translation2d(speed, 0), 0, true, false)
            )
        ).until(() -> {
            double deltaAngle = startingPitch[0] - getRoll();
            return Math.abs(deltaAngle) >= angleDeltaTolerance;
        });
    }

    public Command driveUntilPitchAtSpeed(double speed, double targetPitch) {
        double[] startingPitch = {0};
        return runOnce(() -> {
            startingPitch[0] = getRoll();
        }).andThen(
            run(() -> 
                drive(new Translation2d(speed, 0), 0, true, false)
            )
        ).until(() -> {
            return Math.abs(getRoll()) >= targetPitch;
        });
    }


    public Command driveUntilPitchAtSpeedLower(double speed, double targetPitch) {
        double[] startingPitch = {0};
        return runOnce(() -> {
            startingPitch[0] = getRoll();
        }).andThen(
            run(() -> 
                drive(new Translation2d(speed, 0), 0, true, false)
            )
        ).until(() -> {
            return Math.abs(getRoll()) <= targetPitch;
        });
    }
    public Command followTrajectoryWithReveresedInputs(PathPlannerTrajectory trajectory, boolean shouldResetOdometry) {

        PPSwerveControllerCommand followTrajecotryControllerCommand = new PPSwerveControllerCommand(
                trajectory,
                this::getPose,
                SwerveConstants.swerveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                new PIDController(AutoConstants.kPThetaController, 0, 0),
                this::setModulesStatesClosedLoopReveresed,
                false, // Assuming we don't need to flip the trajectory, we set this to false.
                this);

        InstantCommand resetOdometryBeforePath = new InstantCommand(() -> {
            if (shouldResetOdometry)
                resetPose(trajectory.getInitialHolonomicPose());
        }, this);
        return resetOdometryBeforePath.andThen(followTrajecotryControllerCommand);

    }


    /**
     * Auto-aligns the robot to a given setpoint degree
     * 
     * @param degrees degree to align the robot to
     * @return a command that aligns the robot until its' aligned.
     */
    public Command gyroAlignCommand(double degrees) {

        PIDController pid = new PIDController(Misc.gyroAlignKP, 0, 0);

        return run(() -> {
            drive(
                    new Translation2d(),
                    pid.calculate(getYaw().getDegrees(), degrees),
                    false,
                    false);
        })
                .until(() -> pid.atSetpoint())
                .finallyDo((interrupted) -> pid.close());
    }

    public Command chargeStationAlign() {
        return run(() -> {
            Translation2d moveDirection = new Translation2d();

            if (getPitch() > SwerveConstants.STATION_PITCH_ANGLE_TOLERANCE) {
                moveDirection = new Translation2d(SwerveConstants.ALIGN_STATION_SPEED, 0.0); // Vroom vroom positive
            } else if (getPitch() < -SwerveConstants.STATION_PITCH_ANGLE_TOLERANCE) {
                moveDirection = new Translation2d(-SwerveConstants.ALIGN_STATION_SPEED, 0.0); // Vroom vroom negative

            }

            drive(
                    moveDirection,
                    0,
                    true,
                    false);
        })
                .until(() -> Math.abs(getPitch()) < SwerveConstants.STATION_PITCH_ANGLE_TOLERANCE);
    }
    
    public void lockWheelsChargeStation() {
        for (SwerveModule module : mSwerveMods) {
            module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        }
    }

    public Command LockWheelsChargeStation(){
        return runOnce(()->{
            lockWheelsChargeStation();
        });
    }        
}