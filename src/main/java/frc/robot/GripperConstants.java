package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class GripperConstants {
    private static final int MOTOR_ID = 8;
    private static final boolean INVERTED = false;

    public static final double currentLimit = 32;

    public static final WPI_TalonFX MOTOR = new WPI_TalonFX(MOTOR_ID);

    static {
        MOTOR.configFactoryDefault();

        MOTOR.setInverted(INVERTED);
        MOTOR.setNeutralMode(NeutralMode.Brake);
        MOTOR.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                true, currentLimit, currentLimit, 0.001
        ));
    }
    public enum GripperState {
        STOP(0),
        COLLECT(-0.9),
        SLOW_COLLECT(-0.3),
        EJECT(0.43),
        FULL_EJECT(1),
        SLOW_EJECT(0.12),
        HOLD(-0.1);

        public final double power;

        GripperState(double power) {
            this.power = power;
        }
    }
}
