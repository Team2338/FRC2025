package team.gif.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class UiSmartDashboard {

    private NetworkTable networkTable;
    private NetworkTableEntry motorTempEntry;

    /**
     *  Widgets (e.g. gyro),
     *  buttons (e.g. SmartDashboard.putData("Reset", new ResetHeading()); ),
     *  and Chooser options (e.g. auto mode)
     *
     *  Placed in 2338-dashboard network table
     *  After SmartDashboard loads for the first time, move items from network table onto Dashboard tab
     *  and save file as "YYYY shuffleboard layout.json"
     */
    public UiSmartDashboard() {
        networkTable = NetworkTableInstance.getDefault().getTable("2338-dashboard");
        motorTempEntry = networkTable.getEntry("Motor Temp");
    }

    //adds autos to select

    /**
     * Values which are updated periodically should be placed here
     *
     * Convenient way to format a number is to use putString w/ format:
     *     elevatorPosEntry.setString(String.format("%11.2f", Elevator.getPosition());
     */
    public void updateUI() {
        motorTempEntry.setBoolean(Robot.diagnostics.getAnyMotorTempHot());
    }
}
