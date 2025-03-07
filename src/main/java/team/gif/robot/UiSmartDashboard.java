package team.gif.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

        SmartDashboard.putBoolean("Elevator Stall", Robot.elevator.isStalled());

        SmartDashboard.putData("Commands", CommandScheduler.getInstance());
    }

    /**
     * Widgets which are updated periodically should be placed here
     *
     * Convenient way to format a number is to use putString w/ format:
     *     elevatorPosEntry.setString(String.format("%11.2f", Elevator.getPosition());
     */
    public void updateUI() {
        // Update Main Dashboard
        SmartDashboard.putBoolean("Motor Temp", Robot.diagnostics.getAnyMotorTempHotFlash());

        SmartDashboard.putString("Elevator", String.format("%11.2f", Robot.elevator.getPosition()));
        SmartDashboard.putString("Climber", String.format("%11.2f", Robot.climber.getPosition()));

        SmartDashboard.putNumber("Left", Robot.shooter.getLeftD());
        SmartDashboard.putNumber("Right", Robot.shooter.getRightD());
        SmartDashboard.putBoolean("Mode(Std)", Robot.getRobotMode() == RobotMode.STANDARD_OP);
        SmartDashboard.putString("Piston", Robot.climber.getPistonStateAsString());

        SmartDashboard.putBoolean("Unlocked", Robot.elevator.getElevatorUnlocked());

        // Update Diagnostics tab
        SmartDashboard.putNumber("Diagnostics/Swerve FL temp", Robot.swerveDrive.fLDriveTemp());
        SmartDashboard.putNumber("Diagnostics/Swerve FR temp", Robot.swerveDrive.fRDriveTemp());
        SmartDashboard.putNumber("Diagnostics/Swerve RL temp", Robot.swerveDrive.rLDriveTemp());
        SmartDashboard.putNumber("Diagnostics/Swerve RR temp", Robot.swerveDrive.rRDriveTemp());

        SmartDashboard.putBoolean("Diagnostics/Angle", Robot.diagnostics.getAtTargetAngle());

        SmartDashboard.putBoolean("Shooter/Shooter", Robot.shooter.getExitSensorState());
        SmartDashboard.putBoolean("Shooter/Index", Robot.shooter.getIndexerSensorState());
        SmartDashboard.putBoolean("Shooter/Reef L", Robot.shooter.sensorLeftActive());
        SmartDashboard.putBoolean("Shooter/Reef R", Robot.shooter.sensorRightActive());

        SmartDashboard.putBoolean("LimelightsEnabled", Robot.swerveDrive.getLimelightEnabled());
    }
}
