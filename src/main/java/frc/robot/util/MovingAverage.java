// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

/**
 * Helper class for storing and calculating a moving average
 */
public class MovingAverage {

    ArrayList<Double> numbers = new ArrayList<Double>();
    int maxSize;

    public MovingAverage(int maxSize) {
        this.maxSize = maxSize;
    }

    public void addNumber(double newNumber) {
        numbers.add(newNumber);
        if (numbers.size() > maxSize) {
            numbers.remove(0);
        }
    }

    public double getAverage() {
        double total = 0;

        for (double number : numbers) {
            total += number;
        }

        return total / numbers.size();
    }

    public int getSize() {
        return numbers.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        numbers.clear();
    }

}