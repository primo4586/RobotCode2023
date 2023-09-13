package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.vision.VisionPoseEstimator;
import frc.robot.Constants;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  
  private double[] swerveState = new double[8];

  double maxSpeed = 0,
  currentVelocity = 0;


  public Swerve() {
    gyro = new PigeonIMU(new TalonSRX(SwerveConstants.pigeonID));
    gyro.configFactoryDefault();
    setGyro(0);

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

  @Override
  public void periodic() {
    putStates();

    currentVelocity = mSwerveMods[0].getVelocity();

    for (SwerveModule module : mSwerveMods) {
      if(module.getVelocity()>maxSpeed)
        maxSpeed = module.getVelocity();
    }
    SmartDashboard.putNumber("swerve max speed", maxSpeed);
    SmartDashboard.putNumber("swerve current speed", currentVelocity);
  }

  // functions and commands

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

  public Command driveUntilMeters(double driveSpeed, double metersSetpoint, boolean forward) {
    return run(() -> {
      drive(new Translation2d(forward ? driveSpeed : -driveSpeed, 0), metersSetpoint, true, false);

    }).until(() -> swerveOdometry.getPoseMeters().getTranslation().getX() == metersSetpoint);
  }
  

    // PathPlanner

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
  public ProxyCommand followTrajectory(PathPlannerTrajectory trajectory, boolean shouldResetOdometry,
      boolean useAllianceColor) {
    PPSwerveControllerCommand followTrajecotryControllerCommand = new PPSwerveControllerCommand(
        trajectory,
        this::getPose,
        SwerveConstants.swerveKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        new PIDController(AutoConstants.kPThetaController, 0, 0),
        this::setModuleStatesClosedLoop,
        useAllianceColor,
        this);
    InstantCommand resetOdometryBeforePath = new InstantCommand(() -> {
      if (shouldResetOdometry)
        resetPose(trajectory.getInitialHolonomicPose());
    }, this);
    return resetOdometryBeforePath.andThen(followTrajecotryControllerCommand).asProxy();

  }
  
  public ProxyCommand followTrajectory(PathPlannerTrajectory trajectory, boolean shouldResetOdometry){
    return followTrajectory(trajectory, shouldResetOdometry, false);
  }

  public PathPlannerTrajectory generateTrajectoryToAligmentPose(Translation2d targetPose) {
    if (targetPose == null)
      return new PathPlannerTrajectory(); // Empty trajectory - 0 seconds duration.

    PathPoint robotPose = new PathPoint(getPose().getTranslation(), getYaw(), getStates()[0].speedMetersPerSecond);
    PathPoint endPoint = new PathPoint(targetPose, Rotation2d.fromDegrees(180));

    PathPlannerTrajectory trajectory = PathPlanner
        .generatePath(new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared), robotPose, endPoint);

    field2d.getObject("traj").setTrajectory(trajectory);

    return trajectory;
  }

  public PathPlannerTrajectory generateTrajectoryToPoseList(List<PathPoint> trajectoryPoints) {
    if (trajectoryPoints == null)
      return new PathPlannerTrajectory(); // Empty trajectory - 0 seconds duration.

    PathPlannerTrajectory trajectory = PathPlanner
        .generatePath(new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared), trajectoryPoints);

    field2d.getObject("traj").setTrajectory(trajectory);

    return trajectory;

  }

  // utils/sets/gets

  public boolean areWeCloseEnough() {// TODO: find a good distance
    if (DriverStation.getAlliance() == Alliance.Blue) {
      return (poseEstimation.getEstimatedPosition().getX() < Units.inchesToMeters(SwerveConstants.blueAligningX)
          + SwerveConstants.howCloseWeNeedToBe);
    }

    return (poseEstimation.getEstimatedPosition().getX() > Units.inchesToMeters(SwerveConstants.blueAligningX)
        - SwerveConstants.howCloseWeNeedToBe);
  }

  //make sure wheels are all looking forword for the start of the game
  public void checkWheelsAlignment() {
    boolean alignCheck = true;

    for (SwerveModule module : mSwerveMods) {
      SmartDashboard.putBoolean("wheel" + module.moduleNumber + "alligned", module.areWheelsAligned());
      if (!module.areWheelsAligned()) {
        alignCheck = false;
      }
    }
    SmartDashboard.putBoolean("wheels aligned", alignCheck);
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

  public Command wheelsInX() {
    return runOnce(() -> {
      for (SwerveModule module : mSwerveMods) {
        module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
      }
    });
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

  public void putStates() {
    int i = 0;
    for (SwerveModule module : mSwerveMods) {
        swerveState[i] = module.getAngle().getDegrees();
        swerveState[i + 1] = module.getVelocity();
        i += 2;
    }
    SmartDashboard.putNumberArray("swerveState", swerveState);
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

  public void setGyro(double gyroVal) {
    gyro.setYaw(gyroVal);
  }

  public void zeroGyroForAutoEnd() {

    gyro.setYaw(gyro.getYaw() + 180);
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
}
