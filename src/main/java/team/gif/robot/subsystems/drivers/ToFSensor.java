// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems.drivers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class ToFSensor extends SubsystemBase {
    public TimeOfFlight sensor;

    public ToFSensor(int id) {
        sensor = new TimeOfFlight(id);
        sensor.setRangingMode(RangingMode.Short, 25);
        sensor.setRangeOfInterest(6,0,10,15);
    }

    public double getSigma() {
        return sensor.getRangeSigma();
    }

    public double getDistance() {
        if (sensor.getRangeSigma()< 10) {
            return sensor.getRange();
        } else {
            return 2000.0;
       }
    }
}
