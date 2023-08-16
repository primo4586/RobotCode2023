// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 
package frc.robot.commands.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.SwerveConstants;;

public class DriveUntilOtherSide extends CommandBase {
  /** Creates a new chargeAlign. */

  
  private final Swerve swerve;
  private boolean checkOtherSide;
  private double pitch;
  private Timer timer;
  private int direction;

  public DriveUntilOtherSide(Swerve swerve, boolean shouldDriveAwayFromDriver) {
      this.swerve = swerve;
      this.direction = shouldDriveAwayFromDriver? 1:-1;
      timer = new Timer();
    addRequirements(swerve);
  }

    @Override
    public void initialize() {
        checkOtherSide = false;
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(SwerveConstants.otherSideSpeed*direction, 0), 0, true, false);
        pitch = swerve.getRoll();
        if (Math.abs(pitch) <= 2) {
            if (checkOtherSide) {
                timer.start();
                checkOtherSide = false;
            }
        }
        else {
            checkOtherSide = true;
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0),0,true,false);
    }

}