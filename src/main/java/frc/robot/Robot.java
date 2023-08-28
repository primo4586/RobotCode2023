// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.Objective;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private AutoContainer autoContainer;

  private LilArm lilArm;
  private BigArm bigArm;
  private TelescopicArm telescopicArm;
  private boolean hasReset = false;
  private Gripper gripper;
  private Swerve swerve;
  private PneumaticsControlModule pcm;
  public final Objective objective = new Objective();

  //private PowerDistribution PDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    PathPlannerServer.startServer(5813);
    pcm = new PneumaticsControlModule(44);
    pcm.enableCompressorDigital();
    //pcm.disableCompressor();
    ctreConfigs = new CTREConfigs();
    bigArm = new BigArm();
    gripper = new Gripper();
    lilArm = new LilArm();
    swerve = new Swerve();
    telescopicArm = new TelescopicArm();

    m_robotContainer = new RobotContainer(swerve, gripper ,lilArm, bigArm, objective, telescopicArm);

    autoContainer = new AutoContainer(swerve, gripper, bigArm, lilArm, telescopicArm);
    PrimoShuffleboard.getInstance().initDashboard(swerve, lilArm, bigArm, gripper, m_robotContainer.getDriverCamera());
    //PPSwerveControllerCommand.setLoggingCallbacks((v) -> {}, (v) -> {}, (v) -> {}, (v, v2) -> {});
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    objective.updateInputs();
    checkNodeSelectorButtons();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (RobotController.getUserButton()) {
      telescopicArm.zeroTeles();
    }

  }
  
  @Override
  public void autonomousInit() {
    swerve.zeroGyro();   
    lilArm.zeroEncoderForAuto();
    bigArm.zeroEncoderForMiddleOfBot();
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
    PrimoShuffleboard.getInstance().initDashboard(swerve, lilArm, bigArm, gripper, m_robotContainer.getDriverCamera());
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
  }

  @Override
  public void testInit() {
    //lilArm.zeroEncoderForIntake();
    LiveWindow.disableAllTelemetry();
    lilArm.zeroEncoderForAuto();
    bigArm.zeroEncoderForMiddleOfBot();
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();.

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
  
  public void checkNodeSelectorButtons() {
    if (objective.middleOfBotSubscriber.getAsBoolean()) {
      objective.middleOfBotPublisher.set(false);
      CommandScheduler.getInstance().schedule(new MiddleOfBot(lilArm, bigArm, telescopicArm, gripper));
    }
    if (objective.highIntakeSubscriber.getAsBoolean()) {
      objective.highIntakePublisher.set(false);
      CommandScheduler.getInstance().schedule(new HighIntake(bigArm, lilArm, gripper, telescopicArm));
    }
    if (objective.groundSubscriber.getAsBoolean()) {
      objective.groundPublisher.set(false);
      if (objective.groundDoubleClickSubscriber.getAsBoolean()) {
        objective.groundDoubleClickPublisher.set(false);
        CommandScheduler.getInstance().schedule(new AutoCollectCube(swerve, gripper, lilArm, bigArm, telescopicArm));
      }
      else {
      CommandScheduler.getInstance().schedule(new Ground(gripper, lilArm, bigArm, telescopicArm));
      }
    }
  }
}
