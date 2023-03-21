package frc.robot.commands.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BigArmConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.SwerveConstants;;

public class ChargeAlign extends CommandBase {
  /** Creates a new chargeAlign. */


  
  private final Swerve swerve;

  private boolean startedClimbing;
  private boolean otherSide;
  private boolean chargeOtherSide;
  private boolean startedGoingOtherSide;
  private boolean firstCharge;
  private Timer timer;
  private Timer secondTimer;
  
  public ChargeAlign(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
      startedClimbing = false;
      firstCharge = false;
      startedGoingOtherSide = false;
      otherSide = false;
      chargeOtherSide = false;

  }

  @Override
  public void execute() {
      double pitch = swerve.getRoll();
      SmartDashboard.putBoolean("Started climbing?", startedClimbing);
      SmartDashboard.putBoolean("firstCharge?", firstCharge);
      SmartDashboard.putBoolean("startedGoingOtherSide?", startedGoingOtherSide);
      SmartDashboard.putBoolean("otherSide?", otherSide);
          if (!startedClimbing) {
              swerve.drive(new Translation2d(-SwerveConstants.preClimbSpeed, 0),0,true,false);
              startedClimbing = Math.abs(pitch) >SwerveConstants.preClimbTolerance;
              secondTimer = new Timer();
              secondTimer.start();
          } 
          else {
              if (Math.abs(pitch) <= SwerveConstants.afterClimbTolerance) {
                  swerve.drive(new Translation2d(0, 0),0,true,false);
              } 
              else{
                if(secondTimer.hasElapsed(1)){
                    swerve.drive(new Translation2d(-Math.signum(pitch) * SwerveConstants.afterAfterClimbSpeed, 0),0,true,false);
                }
                else{
                    swerve.drive(new Translation2d(-Math.signum(pitch) * SwerveConstants.maybeAfterClimbSpeed, 0),0,true,false);
                }
              }
            }
          }

  @Override
  public boolean isFinished() {
      return false;
  }

  @Override
  public void end(boolean interrupted) {
      swerve.drive(new Translation2d(0, 0),0,true,false);
  }
}