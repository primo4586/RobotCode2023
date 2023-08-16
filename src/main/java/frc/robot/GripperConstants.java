package frc.robot;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.util.CurrentWatcher;
import com.revrobotics.CANSparkMax;

public class GripperConstants {
    private static final int MOTOR_ID = 13;
    private static final boolean INVERTED = false;

    private static final double
            HOLD_TRIGGER_DURATION = 0.05,
            HOLD_TRIGGER_CURRENT = 40,
            CURRENT_LIMIT = 30;

    public static final CANSparkMax MOTOR = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);

    public static final CurrentWatcher.CurrentWatcherConfig HOLD_TRIGGER_CONFIG = new CurrentWatcher.CurrentWatcherConfig(
            MOTOR::getOutputCurrent,
            HOLD_TRIGGER_CURRENT,
            HOLD_TRIGGER_DURATION
    );
    static {
        MOTOR.restoreFactoryDefaults();

        MOTOR.setInverted(INVERTED);
        MOTOR.setIdleMode(CANSparkMax.IdleMode.kBrake);
        MOTOR.setSmartCurrentLimit((int)CURRENT_LIMIT);
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
