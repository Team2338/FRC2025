package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.ToFSensor;

public class WallDetector extends SubsystemBase {
    private static ToFSensor sensor;

    public WallDetector(){
        sensor = new ToFSensor(RobotMap.COLLECTOR_SIDE_SENSOR_ID);
    }

    /**
        returns distance to object (e.g. wall) in inches from bumper (3") edge
     */
    public double getDistance() {
        final double sensorOffset = 3.75; // distance from bumper edge to sensor

        return (sensor.getDistance() - (sensorOffset * 25.4)); // 1 inch = 25.4 mm
    }
}
