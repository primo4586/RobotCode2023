// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.commands.actions.Emargancy;
import frc.robot.commands.actions.GrabItemFromIntake;
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
  private Swerve swerve = new Swerve();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Gripper gripper, LilArm lilArm, BigArm bigArm) {
    boolean fieldRelative = true;
    boolean openLoop = true;

    VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(PoseStrategy.LOWEST_AMBIGUITY);
    swerve.setVisionPoseEstimator(visionPoseEstimator);
    bigArm.setDefaultCommand(bigArm.setMotorSpeed(() -> driverController.getRightY()));
    lilArm.setDefaultCommand(lilArm.setMotorSpeed(() -> driverController.getLeftY()));

    //swerve.setDefaultCommand(new TeleopSwerve(swerve, driverController, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop, () ->driverController.getRightTriggerAxis() > 0.5));

    // Configure the button bindings
    configureButtonBindings(gripper, lilArm, bigArm);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(Gripper gripper, LilArm lilArm, BigArm bigArm) {
    /* Driver Buttons */
    driverController.y().onTrue(new InstantCommand(() -> swerve.zeroTeleopGyro(), swerve));
    driverController.x().onTrue(gripper.toggleGripper());
    driverController.start().onTrue(swerve.LockWheelsChargeStation());
    driverController.leftTrigger().whileTrue(swerve.followTrajectoryToAligmentPose(gripper::getShouldGripCone));

    /* Operator Buttons */
    operatorController.y().onTrue(new PutItemInTheUpper(bigArm, lilArm, gripper));
    operatorController.a().onTrue(new PutItemInTheMiddle(lilArm, bigArm, gripper));
    operatorController.leftBumper().onTrue(gripper.changeWhatWeGrip());
    operatorController.x().onTrue(new MoveArmsToTheGround(gripper, lilArm, bigArm));
    operatorController.b().onTrue(new GrabItemFromIntake(lilArm, bigArm, gripper));
    operatorController.start().onTrue(new Emargancy(lilArm,bigArm));
    //TODO add upper intake
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
} 
