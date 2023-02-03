// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This class implements an optical encoder.
 * 
 * NOTE: you will need to run the periodic() method at RECOMMENDED_RATE somehow.
 *       it is recommended that you will add it using the `addPeriodic()` 
 *       function that is supplied in the Robot class
 * 
 * @author Noam Goldfarb
 * @since 2023-02-03
 */
public class OpticEncoder {
    private DigitalInput sensorInput;
    private int pose;
    private boolean lastSensorInput;
    private BooleanSupplier isForward;
    public static final double RECOMMENDED_RATE = 0.01;

    public OpticEncoder(int port, BooleanSupplier isForward) {
        this.sensorInput = new DigitalInput(port);
        this.pose = 0;
        this.lastSensorInput = sensorInput.get();
        this.isForward = isForward;
        
    }

    /**
     * Updates the pose according to the direction
     */
    public void update() {
        if (this.sensorInput.get() && this.lastSensorInput == false) {
            if (this.isForward.getAsBoolean())
                this.pose++;
            else
                this.pose--;
        }

        this.lastSensorInput = this.sensorInput.get();
    }

    /**
     * Resets the position of the encoder to 0.
     */
    public void resetPose() {
        this.pose = 0;
    }

    /**
     * @return Current position of the encoder in ticks
     */
    public int getPose() {
        return this.pose;
    }

    public boolean getInput() {
        return this.sensorInput.get();
    }
}
