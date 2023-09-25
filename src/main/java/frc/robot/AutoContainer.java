// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.PrimoShuffleboard;
import frc.robot.commands.autoCommands.Bump2Piece;
import frc.robot.commands.autoCommands.PieceCharge;
import frc.robot.commands.autoCommands.TwoAndHalfPiece;
import frc.robot.commands.autoCommands.TwoPiece;
import frc.robot.commands.utils.PIDBalance;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.util.CommandSelector;
import frc.robot.vision.LimeLight;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;

    public AutoContainer(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, TelescopicArm telescopicArm, LimeLight limeLight){
        this.autoPaths = new HashMap<String, Command>(); 

        // this.autoPaths.put("FF", swerve.stopModulescCommand().andThen(new FeedForwardCharacterization(
        //     swerve, true,
        //     new FeedForwardCharacterizationData("swerve"),
        //     swerve::runCharacterizationVolts, swerve::getCharacterizationVelocity)));

        this.autoPaths.put("blueUpperConeCube", new TwoPiece(true, false, true, bigArm, lilArm, gripper, swerve, telescopicArm));
        this.autoPaths.put("redUpperConeCube", new TwoPiece(true, false, false, bigArm, lilArm, gripper, swerve, telescopicArm));


        this.autoPaths.put("blueUpperConeCubeCube", new TwoAndHalfPiece(true, true, bigArm, lilArm, gripper, swerve, telescopicArm, limeLight));
        this.autoPaths.put("redUpperConeCubeCube", new TwoAndHalfPiece(true, false, bigArm, lilArm, gripper, swerve, telescopicArm, limeLight));
        

        this.autoPaths.put("cubeCharge",new PieceCharge(false, swerve, gripper, bigArm, lilArm, telescopicArm));
        this.autoPaths.put("coneCharge", new PieceCharge(true, swerve, gripper, bigArm, lilArm, telescopicArm));


        this.autoPaths.put("blueBumpConeCube", new Bump2Piece(true, true, bigArm, lilArm, gripper, swerve, telescopicArm, limeLight));
        this.autoPaths.put("redBumpConeCube", new Bump2Piece(true, false, bigArm, lilArm, gripper, swerve, telescopicArm, limeLight));


        //this.autoPaths.put("cubeAndDrive", gripper.setShouldGripConeCommand(false).andThen( new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm)).andThen(swerve.driveUntilMeters(1.5, 5, false)));
        //this.autoPaths.put("coneAndDrive", gripper.setShouldGripConeCommand(true).andThen( new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm)).andThen(swerve.driveUntilMeters(1.5, 5, false)));
        // this.autoPaths.put("cubeCharge", gripper.setShouldGripConeCommand(false).andThen(new DriveUntilOtherSide(swerve, true)).andThen(new FastCharge(false, swerve)).andThen(new PIDBalance(swerve)));
        // this.autoPaths.put("coneCharge", gripper.setShouldGripConeCommand(true).andThen(new DriveUntilOtherSide(swerve, true)).andThen(new FastCharge(false, swerve)).andThen(new PIDBalance(swerve)));
            
        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
    