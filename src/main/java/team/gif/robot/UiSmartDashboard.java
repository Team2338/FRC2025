package team.gif.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team.gif.lib.RobotMode;
import team.gif.lib.delay;
import team.gif.robot.commands.drivetrain.Reset180;

public class UiSmartDashboard {
    public SendableChooser<delay> delayChooser = new SendableChooser<>();

    /**
     *  Widgets (e.g. gyro, text, True/False flags),
     *  buttons (e.g. SmartDashboard.putData("Reset", new ResetHeading()); ),
     *  and Chooser options (e.g. auto mode, auto delay)
     *
     *  Placed in SmartDashboard network table
     *  After dashboard loads for the first time, manually move items from network table onto respective dashboard tab
     *  and save file as "YYYY elastic-layout.json"
     */
    public UiSmartDashboard() {
        // add delay chooser to dashboard
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
        SmartDashboard.putData("Delay", delayChooser);

        SmartDashboard.putData("Reset180", new Reset180());
    }

    //adds autos to select

    /**
     * Widgets which are updated periodically should be placed here
     *
     * Convenient way to format a number is to use putString w/ format:
     *     elevatorPosEntry.setString(String.format("%11.2f", Elevator.getPosition());
     */
    public void updateUI() {
        // Update Main Dashboard
        SmartDashboard.putBoolean("Motor Temp", Robot.diagnostics.getAnyMotorTempHotFlash());

        // Update Developer Tab
        SmartDashboard.putString("Selected Shooter %", String.format("%11.2f", SmartDashboard.getNumber(RobotMap.UI.SHOOTER_PERC, 0)));
        SmartDashboard.putString("Elevator", String.format("%11.2f", Robot.elevator.getPosition()));

        SmartDashboard.putNumber("Left", Robot.shooter.getLeftD());
        SmartDashboard.putNumber("Right", Robot.shooter.getRightD());
        SmartDashboard.putBoolean("Mode(Std)", Robot.getRobotModeManual());

        // Update Diagnostics tab
        SmartDashboard.putNumber("Diagnostics/Swerve FL temp", Robot.swerveDrive.fLDriveTemp());
        SmartDashboard.putNumber("Diagnostics/Swerve FR temp", Robot.swerveDrive.fRDriveTemp());
        SmartDashboard.putNumber("Diagnostics/Swerve RL temp", Robot.swerveDrive.rLDriveTemp());
        SmartDashboard.putNumber("Diagnostics/Swerve RR temp", Robot.swerveDrive.rRDriveTemp());
    }
}
