// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 
package frc.robot.commands.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.SwerveConstants;;

public class DriveUntilOtherSide extends CommandBase {
  /** Creates a new chargeAlign. */

  
  private final Swerve swerve;

  private boolean startedClimbing;
  private boolean otherSide;

  public DriveUntilOtherSide(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

    @Override
    public void initialize() {
        startedClimbing = false;
        otherSide = false;
    }

    @Override
    public void execute() {
        double pitch = swerve.getRoll();
        SmartDashboard.putBoolean("Started climbing?", startedClimbing);
        if (!startedClimbing) {
            swerve.drive(new Translation2d(-SwerveConstants.preClimbSpeed, 0),0,true,false);
            startedClimbing = Math.abs(pitch) >SwerveConstants.preClimbTolerance;
        } else {
            if (Math.abs(pitch) <= SwerveConstants.afterClimbTolerance) {
                swerve.drive(new Translation2d(0, 0),0,true,false);
            } else {
                if(pitch<=-3){
                    otherSide = true;
                }
            if(!otherSide) {
              swerve.drive(new Translation2d(-Math.signum(pitch) * SwerveConstants.afterClimbSpeed, 0),0,true,false);
            }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return otherSide;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0),0,true,false);
    }

}