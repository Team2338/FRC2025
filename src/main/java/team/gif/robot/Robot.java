// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.lib.delay;
import team.gif.robot.commands.drivetrainPbot.DriveSwerve;
import team.gif.robot.subsystems.Diagnostics;
import team.gif.robot.subsystems.Shooter;
import team.gif.robot.subsystems.SwerveDrivetrainMk3;
import team.gif.robot.subsystems.drivers.Limelight;
import team.gif.robot.subsystems.drivers.Pigeon2_0;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private static RobotContainer robotContainer;
    public static Diagnostics diagnostics;
    public static OI oi;
    public static UiSmartDashboard uiSmartDashboard;
    private static delay chosenDelay;

    //Devices
    public static Pigeon2_0 pigeon;
    public static SwerveDrivetrainMk3 swerveDrive;
    //  public static SwerveDrivetrainMk4 swerveDrive;
    public static Limelight limelightCollector;
    public static Limelight limelightShooter;
    public static Shooter shooter;

    public static final boolean fullDashboard = true;
    private boolean runAutoScheduler;
    private Timer elapsedTime;
    private int counter =0;

    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    public Robot() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        pigeon = new Pigeon2_0(RobotMap.PIGEON_ID);
        limelightCollector = new Limelight("limelight-collect");
        limelightShooter = new Limelight("limelight-shooter");
        swerveDrive = new SwerveDrivetrainMk3();
        swerveDrive.setDefaultCommand(new DriveSwerve());
        //  swerveDrive = new SwerveDrivetrainMk4();
        shooter= new Shooter();
        robotContainer = new RobotContainer();
        diagnostics = new Diagnostics();
        oi = new OI();
        uiSmartDashboard = new UiSmartDashboard();
        pigeon.addToShuffleboard("Heading");

        // Add a second periodic function to remove non-essential updates from the main scheduler
        addPeriodic(this::secondPeriodic, 0.5, 0.05);

        elapsedTime = new Timer();
    }

    /**
    * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
    * that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        //Vision Localization
    //        limelightCollector.setRobotOrientation(pigeon.getCompassHeading(), 0, 0, 0, 0, 0);
        limelightCollector.setRobotOrientation(pigeon.getHeading(), 0, 0, 0, 0, 0);
        limelightShooter.setRobotOrientation(pigeon.getHeading(), 0, 0, 0, 0, 0);
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        chosenDelay = uiSmartDashboard.delayChooser.getSelected();

        // run scheduler immediately if no delay is selected
        if (chosenDelay.getValue() == 0) {
            if (autonomousCommand != null) {
                autonomousCommand.schedule();
            }
            runAutoScheduler = false;
        } else {
            // invoke delay
            elapsedTime.reset();
            elapsedTime.start();
            runAutoScheduler = true;
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        if (runAutoScheduler && (elapsedTime.get() > (chosenDelay.getValue()))) {
            if (autonomousCommand != null) {
                autonomousCommand.schedule();
            }
            runAutoScheduler = false;
            elapsedTime.stop();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        shooter.moveIndexerFromShuffleboard();

        double timeLeft = DriverStation.getMatchTime();
        oi.setRumble((timeLeft <= 15.0 && timeLeft >= 12.0) ||
                (timeLeft <= 5.0 && timeLeft >= 3.0));
    }

    public void secondPeriodic() {
        System.out.println(++counter);
        uiSmartDashboard.updateUI();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
