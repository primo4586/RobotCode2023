
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.*;
import frc.robot.commands.actions.IntakeSequential;
import frc.robot.commands.actions.MiddleOfBot;
import frc.robot.commands.actions.CoolScore;
import frc.robot.commands.actions.EmergencyStop;
import frc.robot.commands.actions.GrabItemFromHighIntake;
import frc.robot.commands.actions.GroundTele;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.subsystems.*;

public class RobotContainer {

  /* Controllers */
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private UsbCamera driverCamera;

  /* Subsystems */
  private Swerve swerve;
  private CameraHandler handler;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Swerve swerve, Gripper gripper, LilArm lilArm, BigArm bigArm, DaulArmSim daulArmSim, Objective objective) {
    this.swerve = swerve;
    boolean fieldRelative = true;
    boolean openLoop = true;

    bigArm.setDefaultCommand(bigArm.setMotorSpeed(() -> operatorController.getRawAxis(0)*0.7));
    lilArm.setDefaultCommand(lilArm.setMotorSpeed(() -> operatorController.getRawAxis(1)*0.5));

    swerve.setDefaultCommand(new TeleopSwerve(swerve, driverController, translationAxis, strafeAxis, rotationAxis,
        fieldRelative, openLoop, () -> driverController.getRightTriggerAxis() > 0.5));

    // Configure the button bindings
    configureButtonBindings(swerve, gripper, lilArm, bigArm, objective);
    buildCameras();
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
      Objective objective) {

    CoolScore coolScore = new CoolScore(swerve, bigArm, lilArm, gripper, objective);

    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper);
    PutItemInTheMiddle putItemInTheMiddle =  new PutItemInTheMiddle(lilArm, bigArm, gripper);
    GrabItemFromHighIntake highIntake = new GrabItemFromHighIntake(bigArm, lilArm);
    MiddleOfBot middleOfBot = new MiddleOfBot(lilArm, bigArm);
    GroundTele groundIntake = new GroundTele(gripper, lilArm, bigArm);


    /* Driver Buttons */
    driverController.y().onTrue(new InstantCommand(() -> swerve.zeroTeleopGyro(), swerve));
    driverController.x().onTrue(gripper.toggleGripper());
    driverController.start().onTrue(lilArm.TurnLilArmToSetpoint(LilArmConstants.autoStartPoint));
    driverController.back().onTrue(lilArm.zeroLilArm());
    driverController.b().onTrue(new ConditionalCommand(coolScore, Commands.none(), () -> swerve.areWeCloseEnough()));

    /* Operator Buttons */

    operatorController.leftBumper().onTrue(gripper.changeWhatWeGrip());
    operatorController.rightBumper().onTrue(lilArm.closeLilArmSolenoid());
    operatorController.rightTrigger().onTrue(lilArm.openLilArmSolenoid());
    operatorController.y().onTrue(putItemInTheUpper);
    operatorController.a().onTrue(putItemInTheMiddle);
    operatorController.b().onTrue(middleOfBot);
    operatorController.x().onTrue(highIntake);
    operatorController.start().onTrue(new EmergencyStop(lilArm, bigArm));
    operatorController.back().onTrue(bigArm.Hone());

    operatorController.povCenter().onTrue(groundIntake);
    operatorController.povDown().onTrue(groundIntake);
    operatorController.povDownLeft().onTrue(groundIntake);
    operatorController.povDownRight().onTrue(groundIntake);
    operatorController.povLeft().onTrue(groundIntake);
    operatorController.povRight().onTrue(groundIntake);
    operatorController.povUp().onTrue(groundIntake);
    operatorController.povUpLeft().onTrue(groundIntake);
    operatorController.povUpRight().onTrue(groundIntake);
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

    public void buildCameras() {
      driverCamera = CameraServer.startAutomaticCapture("Forward", 0);
      //driverCamera.setVideoMode(PixelFormat.kYUYV, 320, 240, 10);

  }

  public UsbCamera getDriverCamera() {
    return driverCamera;
  }
}
