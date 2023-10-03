package frc.robot.commands;

import frc.robot.Constants.Misc;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve swerve;
    private CommandXboxController controller;

    private int forwardAxis; 
    private int leftAxis;
    private int rotationAxis;

    // Should the swerve drive slower than before or not
    private BooleanSupplier slowMode; 
    double x = 0;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve swerve, CommandXboxController controller, int forwardAxis, int leftAxis, int rotationAxis, boolean fieldRelative, boolean openLoop, BooleanSupplier slowMode) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.controller = controller;
        this.forwardAxis = forwardAxis;
        this.leftAxis = leftAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.slowMode = slowMode ;
    }

    @Override
    public void initialize() {
        System.out.println("Teleop Swerve started (or also referred to as):");
        System.out.println("vroom vroom");
    }

    @Override
    public void execute() {
        // See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
        double xAxis = -controller.getRawAxis(forwardAxis); // Postive X value means forward / into the field
        double yAxis = -controller.getRawAxis(leftAxis); // Positive Y value means to the left side of the field 
        double rAxis = -controller.getRawAxis(rotationAxis); // Positive value means CCW rotation
        /* Deadbands */

        yAxis = (Math.abs(yAxis) < Misc.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Misc.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Misc.stickDeadband) ? 0 : rAxis;
        SmartDashboard.putNumber("xAxis", xAxis);
        SmartDashboard.putNumber("yAxis", yAxis);
        SmartDashboard.putNumber("rAxis", rAxis);
        x++;
        SmartDashboard.putNumber("drive check", x);
        
        
        if(!slowMode.getAsBoolean()){
            translation = new Translation2d(xAxis, yAxis).times(SwerveConstants.maxSpeed);
            rotation = rAxis * SwerveConstants.maxAngularVelocity;
            swerve.drive(translation, rotation, fieldRelative, openLoop);
        }
        else
        {
            translation = new Translation2d(xAxis, yAxis).times(SwerveConstants.slowModeSpeed);
            rotation = rAxis * SwerveConstants.slowModeAngularVelocity;
            swerve.drive(translation, rotation, fieldRelative, openLoop);
        } 
    }
}