// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIOServer;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.PrimoShuffleboard;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.DaulArmSim;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

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
  private boolean hasReset = false;
  private Gripper gripper;
  private Swerve swerve;
  private PneumaticsControlModule pcm;
  private ObjectiveTracker objectiveTracker;
  public final Objective objective = new Objective();
  private DaulArmSim daulArmSim;

  // private DigitalInput input = new DigitalInput(3);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    pcm = new PneumaticsControlModule(44);
    pcm.enableCompressorDigital();
    ctreConfigs = new CTREConfigs();
    bigArm = new BigArm();
    gripper = new Gripper();
    gripper.turnOnLed();
    lilArm = new LilArm();
    swerve = new Swerve();
    daulArmSim = new DaulArmSim(bigArm,lilArm);

    m_robotContainer = new RobotContainer(swerve, gripper ,lilArm, bigArm, daulArmSim, objective);
    autoContainer = new AutoContainer(swerve, gripper, bigArm, lilArm);
    PrimoShuffleboard.getInstance().initDashboard(swerve, lilArm, bigArm, gripper, m_robotContainer.getDriverCamera());
    PPSwerveControllerCommand.setLoggingCallbacks((v) -> {}, (v) -> {}, (v) -> {}, (v, v2) -> {});
  
  objectiveTracker = new ObjectiveTracker(new NodeSelectorIOServer());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // gripper.turnOffLed();
    // SmartDashboard.putBoolean("Port View", input.get());
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();


    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    gripper.turnOffLed();
    lilArm.setPreference();
    bigArm.setPreference();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    swerve.zeroGyro();   
    lilArm.zeroEncoderForAuto();
    bigArm.zeroEncoderForIntake();
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
    gripper.turnOnLed();
    
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
    bigArm.zeroEncoderForIntake();
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();.

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
