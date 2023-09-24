
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.Objective;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.*;
import frc.robot.commands.actions.CoolScore;
import frc.robot.commands.actions.EmergencyStopArms;
import frc.robot.commands.actions.FullEStop;
import frc.robot.commands.actions.HighIntake;
import frc.robot.commands.actions.Ground;
import frc.robot.commands.actions.MiddleOfBot;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.actions.gripper.Collect;
import frc.robot.commands.actions.gripper.Eject;
import frc.robot.commands.actions.gripper.Hold;
import frc.robot.subsystems.*;
import frc.robot.vision.VisionPoseEstimator;
import frc.robot.vision.VisionPoseEstimatorLimeLight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotContainer {

  private Objective objective;
  private Alliance alliance = DriverStation.getAlliance();

  Translation2d endPointTranslatio2d;

  PathPlannerTrajectory traj;

  PathPoint blueUpperAlignToEnterCommunity = new PathPoint(new Translation2d(5.42, 4.7), new Rotation2d(0));
  PathPoint blueUpperEnterCommunity = new PathPoint(new Translation2d(2.3, 4.7), new Rotation2d(0));
  
  PathPoint blueLowerAlignToEnterCommunity = new PathPoint(new Translation2d(5.42, 0.9), new Rotation2d(0));
  PathPoint blueLowerEnterCommunity = new PathPoint(new Translation2d(2.3, 0.9), new Rotation2d(0));


  /* Controllers */
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Subsystems */
  private Swerve swerve;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Swerve swerve, Gripper gripper, LilArm lilArm, BigArm bigArm, Objective objective,
      TelescopicArm telescopicArm) {

    this.swerve = swerve;
    boolean fieldRelative = true;
    boolean openLoop = true;

    VisionPoseEstimatorLimeLight visionPoseEstimator = new VisionPoseEstimatorLimeLight(PoseStrategy.LOWEST_AMBIGUITY);
    swerve.setVisionPoseEstimator(visionPoseEstimator);

    bigArm.setDefaultCommand(bigArm.setMotorSpeed(() -> operatorController.getRightY() * -0.4));
    lilArm.setDefaultCommand(lilArm.setMotorSpeed(() -> operatorController.getLeftY() * 0.5));

    telescopicArm.setDefaultCommand(telescopicArm.stop());
    
    swerve.setDefaultCommand(new TeleopSwerve(swerve, driverController, translationAxis, strafeAxis, rotationAxis,
        fieldRelative, openLoop, () -> driverController.getRightTriggerAxis() > 0.5));

    // Configure the button bindings
    configureButtonBindings(swerve, gripper, lilArm, bigArm, objective, telescopicArm);
    //if (Robot.isReal()) {
    //buildCameras();
    //}
  }

  public void traj() {
    double aligningX = (alliance == Alliance.Blue) ? SwerveConstants.blueAligningX : SwerveConstants.redAligningX;
    double aligningY = Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]);
    endPointTranslatio2d = new Translation2d(aligningX, aligningY);
    PathPoint endPoint = new PathPoint(endPointTranslatio2d, new Rotation2d(0));

    Pose2d pose = swerve.getPose();
    SmartDashboard.putNumber("pose", pose.getX());
    PathPoint robotPose = new PathPoint(
        pose.getTranslation(), swerve.getYaw(), swerve.getStates()[0].speedMetersPerSecond);

    List<PathPoint> pointsList = null;

    if((Math.abs(swerve.getPose().getX() - endPointTranslatio2d.getX()) > SwerveConstants.trajAccuracy ||
        Math.abs(swerve.getPose().getY() - endPointTranslatio2d.getY()) > SwerveConstants.trajAccuracy)){
      if (alliance == Alliance.Blue) {
        if (pose.getY() > 3.35) {
          if (pose.getX() > 5.42) {
            pointsList = List.of(robotPose, blueUpperAlignToEnterCommunity, blueUpperEnterCommunity, endPoint);
          } else if (pose.getX() > 2.47) {
            pointsList = List.of(robotPose, blueUpperEnterCommunity, endPoint);
          } else {
            pointsList = List.of(robotPose, endPoint);
          }
        } else {
          if (pose.getX() > 5.42) {
            pointsList = List.of(robotPose, blueLowerAlignToEnterCommunity, blueLowerEnterCommunity);
          } else if (pose.getX() > 3.3) {
            pointsList = List.of(robotPose, blueLowerEnterCommunity);
          } else if (pose.getX() > 2.47) {
            pointsList = List.of(robotPose, blueLowerEnterCommunity, endPoint);
          } else {
            pointsList = List.of(robotPose, endPoint);
          }
        }
      } else {
        if (pose.getY() > 3.35) {
          if (pose.getX() > 11.13) {
            pointsList = List.of(robotPose, blueUpperAlignToEnterCommunity, blueUpperEnterCommunity, endPoint);
          } else if (pose.getX() > 14.08) {
            pointsList = List.of(robotPose, blueUpperEnterCommunity, endPoint);
          } else {
            pointsList = List.of(robotPose, endPoint);
          }
        } else {
          if (pose.getX() > 11.13) {
            pointsList = List.of(robotPose, blueLowerAlignToEnterCommunity, blueLowerEnterCommunity);
          } else if (pose.getX() > 13.25) {
            pointsList = List.of(robotPose, blueLowerEnterCommunity);
          } else if (pose.getX() > 14.08) {
            pointsList = List.of(robotPose, blueLowerEnterCommunity, endPoint);
          } else {
            pointsList = List.of(robotPose, endPoint);
          }
        }
      }
      
    }
    
    traj = swerve.generateTrajectoryToPoseList(pointsList);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(Swerve swerve, Gripper gripper, LilArm lilArm, BigArm bigArm,
      Objective objective, TelescopicArm telescopicArm) {

    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm);
    PutItemInTheMiddle putItemInTheMiddle =  new PutItemInTheMiddle(lilArm, bigArm, gripper, telescopicArm);
    HighIntake highIntake = new HighIntake(bigArm, lilArm, gripper, telescopicArm);
    Ground groundIntake = new Ground(gripper, lilArm, bigArm, telescopicArm);

    /* Driver Buttons */
    driverController.y().onTrue(new InstantCommand(() -> swerve.zeroTeleopGyro(), swerve));
    //driverController.start().onTrue(lilArm.TurnLilArmToSetpoint(LilConstants.autoStartPoint));
    driverController.back().onTrue(lilArm.zeroLilArm());
    driverController.x().whileTrue(new Eject(gripper));
    //driverController.b().onTrue(new CoolScore(swerve, bigArm, lilArm, gripper, objective, telescopicArm).asProxy());
    driverController.b().onTrue(swerve.followTrajectory(traj, false));
    
    driverController.povCenter().onTrue(swerve.stopModulescCommand());
    driverController.povDown().onTrue(swerve.stopModulescCommand());
    driverController.povDownLeft().onTrue(swerve.stopModulescCommand());
    driverController.povDownRight().onTrue(swerve.stopModulescCommand());
    driverController.povLeft().onTrue(swerve.stopModulescCommand());
    driverController.povRight().onTrue(swerve.stopModulescCommand());
    driverController.povUp().onTrue(swerve.stopModulescCommand());
    driverController.povUpLeft().onTrue(swerve.stopModulescCommand());
    driverController.povUpRight().onTrue(swerve.stopModulescCommand());
    
    /* Operator Buttons */

    operatorController.rightBumper().onTrue(gripper.changeWhatWeGrip());
    operatorController.leftBumper().onTrue(new Collect(gripper));
    operatorController.y().onTrue(putItemInTheUpper);
    operatorController.a().onTrue(putItemInTheMiddle);
    operatorController.b().onTrue(new MiddleOfBot(lilArm, bigArm, telescopicArm, gripper));
    operatorController.x().onTrue(highIntake);
    operatorController.start().onTrue(new EmergencyStopArms(lilArm, bigArm, telescopicArm, gripper));
    operatorController.back().onTrue(lilArm.zeroLilArm().andThen(telescopicArm.Home()).andThen(bigArm.Home()));//(bigArm.Home());

    operatorController.povCenter().onTrue(groundIntake);
    operatorController.povDown().onTrue(groundIntake);
    operatorController.povDownLeft().onTrue(groundIntake);
    operatorController.povDownRight().onTrue(groundIntake);
    operatorController.povLeft().onTrue(groundIntake);
    operatorController.povRight().onTrue(groundIntake);
    operatorController.povUp().onTrue(groundIntake);
    operatorController.povUpLeft().onTrue(groundIntake);
    operatorController.povUpRight().onTrue(groundIntake);

    operatorController.leftTrigger().whileTrue(telescopicArm.setMotorSpeed(-0.1));
    operatorController.rightTrigger().whileTrue(telescopicArm.setMotorSpeed(0.1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    PathConstraints constraints = new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    return swerve.followTrajectory(PathPlanner.loadPath("One Meter Forward", constraints), true);
  }

  public double getOperatorLeftStick(){
    return operatorController.getLeftY();
  }
}
