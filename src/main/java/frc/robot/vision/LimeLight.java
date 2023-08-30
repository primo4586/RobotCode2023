// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;

public class LimeLight {
    // limelight
    // x = target pose relative to cam on x axis
    // y = target pose relative to cam on y axis
    // targetExist = is there a target
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private DoubleSubscriber x = table.getDoubleTopic("tx").subscribe(0.0);
    private DoubleSubscriber y = table.getDoubleTopic("ty").subscribe(0.0);
    private DoubleSubscriber targetExist = table.getDoubleTopic("tv").subscribe(0.0);
    private StringSubscriber targetClass = table.getStringTopic("tclass").subscribe("nothing");
    private DoublePublisher camMode = table.getDoubleTopic("camMode").publish();
    UsbCamera limeLightCam;

    public LimeLight() {
        table.getEntry("ledMode").setNumber(1);
        camLimeLight();
    }
    
    public double getCubeX() {
        return x.getAsDouble();
    }

    public double getCubeY() {
        return y.getAsDouble();
    }

    public void cubeLimeLight() {
        camMode.set(0);
    }

    public void camLimeLight() {
        camMode.set(1);
    }

    public boolean getTargetExist() {
        return targetExist.getAsDouble() == 1;
    }

    public String getTargetClass() {
        return targetClass.get();
    }

}
