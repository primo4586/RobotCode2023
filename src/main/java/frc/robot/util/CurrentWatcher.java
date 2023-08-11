package frc.robot.util;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.io.Closeable;
import java.io.IOException;
import java.util.function.DoubleSupplier;

public class CurrentWatcher implements Closeable {
    private final DoubleSupplier currentSupplier;
    private final double currentThreshold;
    private final double timeThreshold;
    private final Runnable callback;
    private double lastTimeBelowThreshold;
    private final Notifier notifier;

    public CurrentWatcher(DoubleSupplier currentSupplier, double currentThreshold, double timeThreshold, Runnable callback) {
        this.currentSupplier = currentSupplier;
        this.currentThreshold = currentThreshold;
        this.timeThreshold = timeThreshold;
        this.callback = callback;
        this.notifier = new Notifier(this::checkCurrent);
        this.notifier.startPeriodic(0.02);
    }

    private void checkCurrent() {
        double current = currentSupplier.getAsDouble();

        if (current < currentThreshold) {
            lastTimeBelowThreshold = Timer.getFPGATimestamp();
            return;
        }

        double timeAboveThreshold = Timer.getFPGATimestamp() - lastTimeBelowThreshold;
        if (timeAboveThreshold >= timeThreshold) {
            if(callback != null)
                callback.run();
        }
    }

    @Override
    public void close() throws IOException {
        notifier.close();
    }

    public static class CurrentWatcherConfig {
        private final DoubleSupplier currentSupplier;
        private final double currentThreshold;
        private final double timeThreshold;

        public CurrentWatcherConfig(DoubleSupplier currentSupplier, double currentThreshold, double timeThreshold) {
            this.currentSupplier = currentSupplier;
            this.currentThreshold = currentThreshold;
            this.timeThreshold = timeThreshold;
        }

        public CurrentWatcher setup(Runnable callback) {
            return new CurrentWatcher(currentSupplier, currentThreshold, timeThreshold, callback);
        }
    }
}