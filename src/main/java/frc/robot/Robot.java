// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.PrimoShuffleboard;
import frc.robot.commands.actions.AutoCollectCube;
import frc.robot.commands.actions.Ground;
import frc.robot.commands.actions.HighIntake;
import frc.robot.commands.actions.MiddleOfBot;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.vision.LimeLight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private AutoContainer autoContainer;

  private LilArm lilArm;
  private BigArm bigArm;
  private TelescopicArm telescopicArm;
  private boolean hasReset = false;
  private Gripper gripper = new Gripper();
  private Swerve swerve;
  private LimeLight limeLight;

  //private PowerDistribution PDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Timer.delay(1);
    swerve = new Swerve();
    Timer.delay(1);
    System.out.println("HERE1");
    PathPlannerServer.startServer(5813);
    bigArm = new BigArm();
    lilArm = new LilArm();
    telescopicArm = new TelescopicArm();
    limeLight = new LimeLight();
    System.out.println("HERE2");

    new RobotContainer(swerve, gripper ,lilArm, bigArm, telescopicArm);

    autoContainer = new AutoContainer(swerve, gripper, bigArm, lilArm, telescopicArm,limeLight);
    PrimoShuffleboard.getInstance().initDashboard(swerve, lilArm, bigArm, gripper);
    //PPSwerveControllerCommand.setLoggingCallbacks((v) -> {}, (v) -> {}, (v) -> {}, (v, v2) -> {});
    SmartDashboard.putBoolean("wheels aligned", true);
    //Leds leds= new Leds(0, 81);
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (RobotController.getUserButton()) {
      telescopicArm.zeroTeles();
    }

    //swerve.checkWheelsAlignment();

  }
  
  @Override
  public void autonomousInit() {
    swerve.setGyro(0);   
    bigArm.zeroEncoderForMiddleOfBot();
    telescopicArm.Home();
    m_autonomousCommand = autoContainer.getAutonomousCommand();//swerve.driveForTimeAtSpeed(new Translation2d(-1.25, 0), 3.6);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    PrimoShuffleboard.getInstance().initDashboard(swerve, lilArm, bigArm, gripper);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    if(!hasReset)  {
      swerve.zeroGyroForAutoEnd();
      hasReset = true;
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    System.out.println(swerve.getCurrentCommand());

    //objective.updateInputs();
    //checkNodeSelectorButtons();
  }
  @Override
  public void testInit() {
    
    swerve.setGyro(0);   
    //lilArm.zeroEncoderForIntake();
    LiveWindow.disableAllTelemetry();
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();.

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
  
  // public void checkNodeSelectorButtons() {
  //   if (objective.middleOfBotSubscriber.getAsBoolean()) {
  //     objective.middleOfBotPublisher.set(false);
  //     CommandScheduler.getInstance().schedule(new MiddleOfBot(lilArm, bigArm, telescopicArm, gripper));
  //   }
  //   if (objective.highIntakeSubscriber.getAsBoolean()) {
  //     objective.highIntakePublisher.set(false);
  //     CommandScheduler.getInstance().schedule(new HighIntake(bigArm, lilArm, gripper, telescopicArm));
  //   }
  //   if (objective.groundSubscriber.getAsBoolean()) {
  //     objective.groundPublisher.set(false);
  //     if (objective.groundDoubleClickSubscriber.getAsBoolean()) {
  //       objective.groundDoubleClickPublisher.set(false);
  //       CommandScheduler.getInstance().schedule(new AutoCollectCube(swerve, gripper, lilArm, bigArm, telescopicArm,limeLight));
  //     }
  //     else {
  //     CommandScheduler.getInstance().schedule(new Ground(gripper, lilArm, bigArm, telescopicArm));
  //     }
  //   }
  // }
}
