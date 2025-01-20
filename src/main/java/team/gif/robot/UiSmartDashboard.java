package team.gif.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import team.gif.lib.delay;

public class UiSmartDashboard {
    public SendableChooser<delay> delayChooser = new SendableChooser<>();
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
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("SmartDashboard");
        networkTable = NetworkTableInstance.getDefault().getTable("2338-dashboard");
        motorTempEntry = networkTable.getEntry("Motor Temp");

        delayChooser.setDefaultOption("0", delay.DELAY_0);
        delayChooser.addOption("1", delay.DELAY_1);
        delayChooser.addOption("2", delay.DELAY_2);
        delayChooser.addOption("3", delay.DELAY_3);
        delayChooser.addOption("4", delay.DELAY_4);
        delayChooser.addOption("5", delay.DELAY_5);
        delayChooser.addOption("6", delay.DELAY_6);
        delayChooser.addOption("7", delay.DELAY_7);
        delayChooser.addOption("8", delay.DELAY_8);
        delayChooser.addOption("9", delay.DELAY_9);
        delayChooser.addOption("10", delay.DELAY_10);
        delayChooser.addOption("11", delay.DELAY_11);
        delayChooser.addOption("12", delay.DELAY_12);
        delayChooser.addOption("13", delay.DELAY_13);
        delayChooser.addOption("14", delay.DELAY_14);
        delayChooser.addOption("15", delay.DELAY_15);


        shuffleboardTab.add("Delay", delayChooser)
                .withPosition(7, 0)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kTextView);
    }

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
