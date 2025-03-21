// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.lib.delay;
import team.gif.lib.RobotMode;
import team.gif.lib.drivePace;
import team.gif.robot.commands.climber.ClimberManualControl;
import team.gif.robot.commands.elevator.ElevatorManualControl;
import team.gif.robot.commands.elevator.ElevatorPIDControl;
import team.gif.robot.commands.shooter.StageCoral;
import team.gif.robot.commands.drivetrain.DriveSwerve;
//import team.gif.robot.commands.StageCoral;
//import team.gif.robot.commands.drivetrainPbot.DrivePracticeSwerve;
import team.gif.robot.subsystems.Climber;
import team.gif.robot.subsystems.Diagnostics;
import team.gif.robot.subsystems.Flapper;
import team.gif.robot.subsystems.Elevator;
import team.gif.robot.subsystems.Grabber;
import team.gif.robot.subsystems.Shooter;
import team.gif.robot.subsystems.SwerveDrivetrainMk4;
import team.gif.robot.subsystems.drivers.Limelight;
import team.gif.robot.subsystems.drivers.Pigeon2_0;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {


    // Framework objects
    private static RobotContainer robotContainer;
    public static Diagnostics diagnostics;
    public static OI oi;
    public static UiSmartDashboard uiSmartDashboard;
    private Command autonomousCommand;


    // Devices
    public static Pigeon2_0 pigeon;
    public static Compressor compressor;
    public static SwerveDrivetrainMk4 swerveDrive;
    public static Limelight limelightFront;
    public static Limelight limelightRight;
    public static Limelight limelightRear;
    public static Shooter shooter;
    public static Climber climber;
    public static Elevator elevator;
    public static Flapper flapper;
    public static Grabber grabber;

    // custom fields
    private boolean autoSchedulerOnHold;
    private static delay chosenDelay;
    public static final boolean fullDashboard = true;
    private final Timer elapsedTime;
    private static RobotMode robotMode;

    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    public Robot() {
        // Instantiate all the framework and device objects
        pigeon = new Pigeon2_0(RobotMap.PIGEON_ID);
        limelightFront = new Limelight("limelight-front");
        limelightRight = new Limelight("limelight-right");
        limelightRear = new Limelight("limelight-rear");
//        swerveDrive = new SwerveDrivetrainMk3();
        swerveDrive = new SwerveDrivetrainMk4();
        swerveDrive.setDefaultCommand(new DriveSwerve());
        shooter = new Shooter();
        climber = new Climber();
        elevator = new Elevator();
        elevator.setDefaultCommand(new ElevatorPIDControl());

        grabber = new Grabber();

        robotContainer = new RobotContainer();
        diagnostics = new Diagnostics();
        compressor = new Compressor(RobotMap.COMPRESSER, PneumaticsModuleType.CTREPCM);
        oi = new OI();
        uiSmartDashboard = new UiSmartDashboard();
        pigeon.addToShuffleboard("Heading");
        pigeon.resetPigeonPosition(180);
        flapper = new Flapper(RobotMap.SERVO_PORT_ID);
        shooter.setDefaultCommand(new StageCoral());

        // Add a second periodic function to remove non-essential updates from the main scheduler
        addPeriodic(this::secondPeriodic, 0.080, 0.05);

        climber.setPistonIn();

        elapsedTime = new Timer();

        robotMode = RobotMode.STANDARD_OP;

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
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        climber.setPistonIn();

        autonomousCommand = robotContainer.getAutonomousCommand();
        chosenDelay = uiSmartDashboard.delayChooser.getSelected();
        compressor.disable();

        // run scheduler immediately if no delay is selected
        if (chosenDelay.getValue() == 0) {
            if (autonomousCommand != null) {
                autonomousCommand.schedule();
            }
            autoSchedulerOnHold = false;
        } else {
            // invoke delay
            elapsedTime.reset();
            elapsedTime.start();
            autoSchedulerOnHold = true;
        }

        //drops servo at start of match
        flapper.setDown();

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // if delay was invoked, need to start autonomous after delay completes
        if (autoSchedulerOnHold && (elapsedTime.get() > (chosenDelay.getValue()))) {
            if (autonomousCommand != null) {
                autonomousCommand.schedule();
            }
            autoSchedulerOnHold = false;
            elapsedTime.stop();
        }

        Robot.shooter.runIndexerMotor();
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
        //-compressor.enableDigital();
        compressor.disable();
        climber.setPistonIn();

        flapper.setDown();

        swerveDrive.setDrivePace(drivePace.COAST_FR);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        // rumble the joysticks at various points during the match to notify the drive team
        double timeLeft = DriverStation.getMatchTime();
        oi.setRumble((timeLeft <= 15.0 && timeLeft >= 12.0) ||
                (timeLeft <= 5.0 && timeLeft >= 3.0));
    }

    public void secondPeriodic() {
//        System.out.println(++counter);
        uiSmartDashboard.updateUI();
        double heading = pigeon.get360Heading();
        double yawRate = pigeon.getYawRate();
        var alliance = DriverStation.getAlliance();
        if( alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ){
            heading = heading - 180;
        }
        limelightFront.setRobotOrientation(heading, yawRate, 0, 0, 0, 0);
        limelightRight.setRobotOrientation(heading, yawRate, 0, 0, 0, 0);
        limelightRear.setRobotOrientation(heading, yawRate, 0, 0, 0, 0);
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

    public static RobotMode getRobotMode() {
        return robotMode;
    }

    public static boolean isRobotInStandardOpMode() {
        return robotMode == RobotMode.STANDARD_OP;
    }

    static public void enableRobotModeManual() {
        robotMode = RobotMode.MANUAL;

        new ClimberManualControl().schedule();
        new ElevatorManualControl().schedule();
    }

    static public void enableRobotModeStandardOp() {
        robotMode = RobotMode.STANDARD_OP;

        elevator.enableElevator();
    }

    static public void runAuto() {
        robotContainer.getAutonomousCommand().schedule();
    }
}
