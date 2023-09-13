
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.Objective;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.commands.*;
import frc.robot.commands.actions.CoolScore;
import frc.robot.commands.actions.CoolScoreDrive;
import frc.robot.commands.actions.EmergencyStopArms;
import frc.robot.commands.actions.FullEStop;
import frc.robot.commands.actions.HighIntake;
import frc.robot.commands.actions.Ground;
import frc.robot.commands.actions.MiddleOfBot;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.actions.gripper.Collect;
import frc.robot.commands.actions.gripper.Eject;
import frc.robot.subsystems.*;
import frc.robot.vision.VisionPoseEstimator;

public class RobotContainer {

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

    VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(PoseStrategy.LOWEST_AMBIGUITY);
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
    MiddleOfBot middleOfBot = new MiddleOfBot(lilArm, bigArm, telescopicArm, gripper);
    Eject eject = new Eject(gripper);
    Collect collect = new Collect(gripper);


    /* Driver Buttons */
    driverController.y().onTrue(new InstantCommand(() -> swerve.zeroTeleopGyro(), swerve));
    driverController.start().onTrue(lilArm.TurnLilArmToSetpoint(LilConstants.autoStartPoint));
    driverController.back().onTrue(lilArm.zeroLilArm());
    driverController.x().whileTrue(eject);
    driverController.a().onTrue(new ConditionalCommand(new CoolScore(swerve, bigArm, lilArm, gripper, objective, telescopicArm).asProxy(), Commands.none(), () -> swerve.areWeCloseEnough()));
    //driverController.b().onTrue(swerve.maxSpeed());
    driverController.pov(0).onTrue(new CoolScoreDrive(swerve, objective));
    
    driverController.povCenter().onTrue(new FullEStop(lilArm, bigArm, telescopicArm, swerve));
    driverController.povDown().onTrue(new FullEStop(lilArm, bigArm, telescopicArm, swerve));
    driverController.povDownLeft().onTrue(new FullEStop(lilArm, bigArm, telescopicArm, swerve));
    driverController.povDownRight().onTrue(new FullEStop(lilArm, bigArm, telescopicArm, swerve));
    driverController.povLeft().onTrue(new FullEStop(lilArm, bigArm, telescopicArm, swerve));
    driverController.povRight().onTrue(new FullEStop(lilArm, bigArm, telescopicArm, swerve));
    driverController.povUp().onTrue(new FullEStop(lilArm, bigArm, telescopicArm, swerve));
    driverController.povUpLeft().onTrue(new FullEStop(lilArm, bigArm, telescopicArm, swerve));
    driverController.povUpRight().onTrue(new FullEStop(lilArm, bigArm, telescopicArm, swerve));
    
    /* Operator Buttons */

    operatorController.rightBumper().onTrue(gripper.changeWhatWeGrip());
    operatorController.leftBumper().onTrue(collect);
    operatorController.y().onTrue(putItemInTheUpper);
    operatorController.a().onTrue(putItemInTheMiddle);
    operatorController.b().onTrue(middleOfBot);
    operatorController.x().onTrue(highIntake);
    operatorController.start().onTrue(new EmergencyStopArms(lilArm, bigArm, telescopicArm));
    operatorController.back().onTrue(bigArm.Home());

    operatorController.povCenter().onTrue(groundIntake);
    operatorController.povDown().onTrue(groundIntake);
    operatorController.povDownLeft().onTrue(groundIntake);
    operatorController.povDownRight().onTrue(groundIntake);
    operatorController.povLeft().onTrue(groundIntake);
    operatorController.povRight().onTrue(groundIntake);
    operatorController.povUp().onTrue(groundIntake);
    operatorController.povUpLeft().onTrue(groundIntake);
    operatorController.povUpRight().onTrue(groundIntake);

    operatorController.leftTrigger().onTrue(telescopicArm.setMotorSpeed(-0.1));
    operatorController.rightTrigger().onTrue(telescopicArm.setMotorSpeed(0.1));
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
