// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flapper extends SubsystemBase {
    private static Servo servo;

    public Flapper(int port) {
        servo = new Servo(port);
    }

    public void setDown() {
        servo.set(1.0);
    }
}
