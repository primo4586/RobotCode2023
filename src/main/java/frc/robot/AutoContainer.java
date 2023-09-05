// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.PrimoShuffleboard;
import frc.robot.autonomous.CommandSelector;
import frc.robot.commands.autoCommands.Bump2Piece;
import frc.robot.commands.autoCommands.PieceReloadCharge;
import frc.robot.commands.autoCommands.TwoAndHalfPiece;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.vision.LimeLight;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;

    public AutoContainer(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, TelescopicArm telescopicArm, LimeLight limeLight){
        this.autoPaths = new HashMap<String, Command>(); 

        this.autoPaths.put("blueUpperConeCubeCube", new TwoAndHalfPiece(true, true, bigArm, lilArm, gripper, swerve, telescopicArm, limeLight));
        this.autoPaths.put("redUpperConeCubeCube", new TwoAndHalfPiece(true, false, bigArm, lilArm, gripper, swerve, telescopicArm, limeLight));
        
        this.autoPaths.put("blueBumpConeCube", new Bump2Piece(true, true, bigArm, lilArm, gripper, swerve, telescopicArm, limeLight));
        this.autoPaths.put("redBumpConeCube", new Bump2Piece(true, false, bigArm, lilArm, gripper, swerve, telescopicArm, limeLight));
        
        this.autoPaths.put("blueConeReloadChargeUpperPiece", new PieceReloadCharge(true, true, true, swerve, gripper, bigArm, lilArm, telescopicArm));
        this.autoPaths.put("blueCubeReloadChargeUpperPiece", new PieceReloadCharge(false, true, true, swerve, gripper, bigArm, lilArm, telescopicArm));
        this.autoPaths.put("blueConeReloadChargeLowerPiece", new PieceReloadCharge(true, false, true, swerve, gripper, bigArm, lilArm, telescopicArm));
        this.autoPaths.put("blueCubeReloadChargeLowerPiece", new PieceReloadCharge(false, false, true, swerve, gripper, bigArm, lilArm, telescopicArm));
        
        this.autoPaths.put("redConeReloadChargeUpperPiece", new PieceReloadCharge(true, true, true, swerve, gripper, bigArm, lilArm, telescopicArm));
        this.autoPaths.put("redCubeReloadChargeUpperPiece", new PieceReloadCharge(false, true, true, swerve, gripper, bigArm, lilArm, telescopicArm));
        this.autoPaths.put("redConeReloadChargeLowerPiece", new PieceReloadCharge(true, false, true, swerve, gripper, bigArm, lilArm, telescopicArm));
        this.autoPaths.put("redCubeReloadChargeLowerPiece", new PieceReloadCharge(false, false, true, swerve, gripper, bigArm, lilArm, telescopicArm));
            
        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
    