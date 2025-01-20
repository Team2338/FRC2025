// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems.drivers;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserCANSensor extends SubsystemBase {

    public LaserCan sensor;
    /** Creates a new ExampleSubsystem. */
    public LaserCANSensor(int id) {
        sensor = new LaserCan(id);

        try {
            sensor.setRangingMode(LaserCan.RangingMode.SHORT);
            sensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException error) {
            System.out.println("Error setting up sensor");
        }

        CanBridge.runTCP();

    }

    public int getDistance() {
        LaserCan.Measurement measurement = sensor.getMeasurement();
        if(measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return sensor.getMeasurement().distance_mm;
        } else {
            return -1;
        }
    }

}
