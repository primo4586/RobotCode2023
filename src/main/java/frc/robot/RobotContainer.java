
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.*;
import frc.robot.commands.actions.GrabItemFromIntake;
import frc.robot.commands.actions.GrabItemFromIntakeNoOpen;
import frc.robot.commands.actions.MoveArmsToSetPointsLilFirst;
import frc.robot.commands.actions.EmergencyStop;
import frc.robot.commands.actions.GrabItemFromHighIntake;
import frc.robot.commands.actions.MoveArmsToTheGround;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
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
  private CameraHandler handler;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Swerve swerve, Gripper gripper, LilArm lilArm, BigArm bigArm) {
    this.swerve = swerve;
    boolean fieldRelative = true;
    boolean openLoop = true;

    bigArm.setDefaultCommand(bigArm.setMotorSpeed(() -> operatorController.getRightY()));
    lilArm.setDefaultCommand(lilArm.setMotorSpeed(() -> operatorController.getLeftY()));

    swerve.setDefaultCommand(new TeleopSwerve(swerve, driverController, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop, () ->driverController.getRightTriggerAxis() > 0.5));

    // Configure the button bindings
    configureButtonBindings(gripper, lilArm, bigArm);
    buildCameras();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(Gripper gripper, LilArm lilArm, BigArm bigArm) {
  
  GrabItemFromIntake grabItemFromIntake = new GrabItemFromIntake(lilArm, bigArm, gripper);
  GrabItemFromIntakeNoOpen grabItemFromIntakeNoOpen = new GrabItemFromIntakeNoOpen(lilArm, bigArm, gripper);
  ConditionalCommand intake = new ConditionalCommand(grabItemFromIntakeNoOpen, grabItemFromIntake,()-> gripper.getFakeIsGripperOpen());


    /* Driver Buttons */
    driverController.y().onTrue(new InstantCommand(() -> swerve.zeroTeleopGyro(), swerve));
    driverController.x().onTrue(gripper.toggleGripper());
    driverController.start().onTrue(lilArm.TurnLilArmToSetpoint(LilArmConstants.autoStartPoint));
    driverController.back().onTrue(lilArm.zeroLilArm());
    driverController.leftBumper().onTrue(new InstantCommand(() -> handler.switchCamera()));

    operatorController.rightBumper().onTrue(lilArm.closeLilArmSolenoid());
    operatorController.rightTrigger().onTrue(lilArm.openLilArmSolenoid());

    /* Operator Buttons */
       operatorController.y().onTrue(new PutItemInTheUpper(bigArm, lilArm, gripper));
       operatorController.a().onTrue(new PutItemInTheMiddle(lilArm, bigArm, gripper));
      operatorController.leftBumper().onTrue(gripper.changeWhatWeGrip());
      //operatorController.a().onTrue(lilArm.TurnLilArmToSetpoint(LilArmConstants.cubeMiddleSetPoint));
      // operatorController.x().onTrue(bigArm.TurnBigArmToSetpoint(BigArmConstants.cubeMiddleSetPoint));
      //operatorController.x().onTrue(new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.cubeMiddleSetPoint, lilArm, LilArmConstants.cubeMiddleSetPoint));
      // operatorController.x().onTrue(new MoveArmsToTheGround(gripper, lilArm, bigArm));
       operatorController.b().onTrue(intake);
       operatorController.x().onTrue(new GrabItemFromHighIntake(bigArm, lilArm));
      operatorController.start().onTrue(new EmergencyStop(lilArm,bigArm));
      operatorController.back().onTrue(bigArm.Hone());
    }

    /** 
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

      PathConstraints constraints = new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      return swerve.followTrajectory(PathPlanner.loadPath("One Meter Forward", constraints), true);
    }


    public void buildCameras() {
      UsbCamera forward = CameraServer.startAutomaticCapture("Forward", 0);

      // handler = new CameraHandler(forward, backward);
    }
  }  
