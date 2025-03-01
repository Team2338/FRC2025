// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Flapper extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private static Servo servo;

    public Flapper(int port) {
        servo = new Servo(RobotMap.SERVO_PORT_ID);
    }

    public void setDown() {
        servo.set(0.5);
    }

    public double getServoPosition() {
        return servo.getPosition();
    }
}
