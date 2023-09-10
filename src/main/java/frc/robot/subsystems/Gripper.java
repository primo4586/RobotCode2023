package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.GripperConstants;

public class Gripper extends SubsystemBase {

    private final WPI_TalonFX motor = GripperConstants.MOTOR;

    private GripperConstants.GripperState state = GripperConstants.GripperState.STOP;

    private boolean shouldGripCone;
    private final XboxController driverRumbleController = new XboxController(0);

    public Gripper() {
        setDefaultCommand(
                new StartEndCommand(
                        () -> setState(GripperConstants.GripperState.HOLD),
                        () -> setState(GripperConstants.GripperState.STOP),
                        this));
        shouldGripCone = true;
    }

    /**
     * @return a command that makes the gripper collect, and stops the gripper at
     *         the end of it
     */
    public CommandBase getCollectCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.COLLECT),
                () ->{
                    setState(GripperConstants.GripperState.HOLD);
                    driverRumbleController.setRumble(RumbleType.kBothRumble, 1);
                    new WaitCommand(0.2);
                    driverRumbleController.setRumble(RumbleType.kBothRumble, 0);},
                this).until(() -> motor.getSupplyCurrent() > GripperConstants.currentLimit));
    }

    /**
     * @return a command that makes the gripper collect slowly, and stops the
     *         gripper at the end of it
     */
    public CommandBase getSlowCollectCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.SLOW_COLLECT),
                () -> setState(GripperConstants.GripperState.HOLD),
                this).until(() -> motor.getSupplyCurrent() > GripperConstants.currentLimit));
    }

    /**
     * @return a command that makes the gripper eject, and stops the gripper at the
     *         end of it
     */
    public CommandBase getEjectCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.EJECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this));
    }

    /**
     * @return a command that makes the gripper eject slowly, and stops the gripper
     *         at the end of it
     */
    public CommandBase getSlowEjectCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.SLOW_EJECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this));
    }

    /**
     * @return a command that makes the gripper hold, and stops the gripper at the
     *         end of it
     */
    public CommandBase getHoldCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.HOLD),
                () -> setState(GripperConstants.GripperState.STOP),
                this));
    }

    /**
     * @return a command that stops the gripper
     */
    public InstantCommand getStopCommand() {
        return new InstantCommand(
                () -> setState(GripperConstants.GripperState.STOP),
                this);
    }

    public CommandBase getFullEjectCommand() {
        return new ProxyCommand(new StartEndCommand(
                () -> setState(GripperConstants.GripperState.FULL_EJECT),
                () -> setState(GripperConstants.GripperState.STOP),
                this));
    }

    public boolean isHolding() {
        return state == GripperConstants.GripperState.HOLD;
    }

    private void setState(GripperConstants.GripperState state) {
        this.state = state;
        motor.set(shouldGripCone?state.power:-state.power);
    }

    public Command changeWhatWeGrip() {
        shouldGripCone = !shouldGripCone;
        return Commands.none();
    }

    public boolean getShouldGripCone() {
        return shouldGripCone;
    }

    public InstantCommand setShouldGripConeCommand(boolean shouldGripCone){
        return new InstantCommand(()->{
            this.shouldGripCone = shouldGripCone;
        });
    }

    public void setShouldGripCone(boolean shouldGripCone) {
        this.shouldGripCone = shouldGripCone;
    }
}
