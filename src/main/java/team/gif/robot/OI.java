package team.gif.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team.gif.robot.commands.climber.ClimberDeploy;
import team.gif.robot.commands.climber.ClimberClimb;
import team.gif.robot.commands.climber.PistonToggleState;
import team.gif.robot.commands.driveModes.EnableRobotOrientedMode;
import team.gif.robot.commands.drivetrain.Reset180;
import team.gif.robot.commands.elevator.SetElevatorPosition;
import team.gif.robot.commands.shooter.Shoot;
import team.gif.robot.commands.driveModes.EnableBoost;
import team.gif.robot.commands.shooter.AutoDriveAndShoot;
import team.gif.robot.commands.drivetrain.Reset0;
import team.gif.robot.commands.toggleManualControl.ToggleManualControl;

public class OI {
    /*
     * Instantiate all joysticks/controllers and their buttons here
     *
     * Examples:
     * public final CommandXboxController driver = new CommandXboxController(0);
     *
     * public final Trigger dA = driver.a();
     */

    public final CommandXboxController driver = new CommandXboxController(RobotMap.DRIVER_CONTROLLER_ID);
    public final CommandXboxController aux = new CommandXboxController(RobotMap.AUX_CONTROLLER_ID);
    public final CommandXboxController test = new CommandXboxController(RobotMap.TEST_CONTROLLER_ID);

    public final Trigger dA = driver.a();
    public final Trigger dB = driver.b();
    public final Trigger dX = driver.x();
    public final Trigger dY = driver.y();
    public final Trigger dLBump = driver.leftBumper();
    public final Trigger dRBump = driver.rightBumper();
    public final Trigger dBack = driver.back();
    public final Trigger dStart = driver.start();
    public final Trigger dLStickBtn = driver.leftStick();
    public final Trigger dRStickBtn = driver.rightStick();
    public final Trigger dRTrigger = driver.rightTrigger();
    public final Trigger dLTrigger = driver.leftTrigger();
    public final Trigger dDPadUp = driver.povUp();
    public final Trigger dDPadRight = driver.povRight();
    public final Trigger dDPadDown = driver.povDown();
    public final Trigger dDPadLeft = driver.povLeft();
    public final Trigger dDPadDownLeft = driver.povDownLeft();

    public final Trigger aA = aux.a();
    public final Trigger aB = aux.b();
    public final Trigger aX = aux.x();
    public final Trigger aY = aux.y();
    public final Trigger aLBump = aux.leftBumper();
    public final Trigger aRBump = aux.rightBumper();
    public final Trigger aBack = aux.back();
    public final Trigger aStart = aux.start();
    public final Trigger aLStickBtn = aux.leftStick();
    public final Trigger aRStickBtn = aux.rightStick();
    public final Trigger aRTrigger = aux.rightTrigger();
    public final Trigger aLTrigger = aux.leftTrigger();
    public final Trigger aDPadUp = aux.povUp();
    public final Trigger aDPadRight = aux.povRight();
    public final Trigger aDPadDown = aux.povDown();
    public final Trigger aDPadLeft = aux.povLeft();
    public final Trigger aDPadDownLeft = aux.povDownLeft();

//    public final Trigger tA = test.a();
//    public final Trigger tB = test.b();
//    public final Trigger tX = test.x();
//    public final Trigger tY = test.y();
//    public final Trigger tLBump = test.leftBumper();
//    public final Trigger tRBump = test.rightBumper();
//    public final Trigger tBack = test.back();
//    public final Trigger tStart = test.start();
//    public final Trigger tLStickBtn = test.leftStick();
//    public final Trigger tRStickBtn = test.rightStick();
//    public final Trigger tRTrigger = test.rightTrigger();
//    public final Trigger tLTrigger = test.leftTrigger();
//    public final Trigger tDPadUp = test.povUp();
//    public final Trigger tDPadRight = test.povRight();
//    public final Trigger tDPadDown = test.povDown();
//    public final Trigger tDPadLeft = test.povLeft();


    public OI() {
        DriverStation.silenceJoystickConnectionWarning(true);
        /*
        *
        * Create controller actions here
        *
        * Usages:
        * dRTrigger.whileTrue(new CollectCommand());
        * dLTrigger.onTrue(new EjectCommand());
        * dA.whileTrue(new RepeatCommand(new RapidFire());
        * aStart.onTrue(new InstantCommand(Robot.elevator::zeroEncoder).ignoringDisable(true));
        *
        * onTrue (fka whenPressed)    Init->Execute repeats until IsFinished = true->End, will not start again at Init if still held down
        * whileTrue (fka whenHeld)    Init->Execute repeats until IsFinished = true or button released->End, will not start again at Init if still held down
        * whileTrue(new RepeatCommand()) (fka whileHeld)   Init->Execute repeats until IsFinished = true or button released->End, will start again at Init if still held down
        *
        * Simple Test:
        *   aX.onTrue(new PrintCommand("aX"));
        */

        /**
         * Driver Controller
         */

        // driver controls
        dStart.and(dDPadUp).onTrue(new Reset0());
        dStart.and(dDPadDown).onTrue(new Reset180());
        dStart.and(dDPadRight).onTrue(new InstantCommand(Robot.climber::zeroEncoder).ignoringDisable(true));
        dStart.and(dDPadLeft).onTrue(new InstantCommand(Robot.elevator::zeroEncoder).ignoringDisable(true));
        dRTrigger.whileTrue(new Shoot());
        dLBump.whileTrue(new EnableBoost());
        dX.whileTrue(new AutoDriveAndShoot(false));
        dB.whileTrue(new AutoDriveAndShoot(true));
        dRBump.whileTrue(new EnableRobotOrientedMode());

        // aux controls
        aStart.and(aDPadUp).onTrue(new Reset0());
        aStart.and(aDPadDown).onTrue(new Reset180());
        aStart.and(aDPadRight).onTrue(new InstantCommand(Robot.climber::zeroEncoder).ignoringDisable(true));
        aStart.and(aDPadLeft).onTrue(new InstantCommand(Robot.elevator::zeroEncoder).ignoringDisable(true));
        aY.whileTrue(new ClimberClimb());
        aA.whileTrue(new ClimberDeploy());
        aStart.and(aBack).toggleOnTrue(new ToggleManualControl());
        aRBump.onTrue(new PistonToggleState());
        aDPadUp.and(aStart.negate()).onTrue(new ConditionalCommand(new SetElevatorPosition(Constants.Elevator.LEVEL_4_POSITION), new InstantCommand(Robot::enableRobotModeManual), Robot::isRobotInStandardOpMode));
        aDPadLeft.and(aStart.negate()).onTrue(new ConditionalCommand(new SetElevatorPosition(Constants.Elevator.LEVEL_3_POSITION), new InstantCommand(Robot::enableRobotModeManual), Robot::isRobotInStandardOpMode));
        aDPadDown.and(aStart.negate()).onTrue(new ConditionalCommand(new SetElevatorPosition(Constants.Elevator.LEVEL_2_POSITION), new InstantCommand(Robot::enableRobotModeManual), Robot::isRobotInStandardOpMode));
        aLBump.onTrue(new ConditionalCommand(new SetElevatorPosition(Constants.Elevator.COLLECTOR_POSITION), new InstantCommand(Robot::enableRobotModeManual), Robot::isRobotInStandardOpMode));

        //test sys id for elevator, delete later
        //dLBump.whileTrue(Robot.elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        //dRBump.whileTrue(Robot.elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        //dX.whileTrue(Robot.elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        //dY.whileTrue(Robot.elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    }

    public void setRumble(boolean rumble) {
        driver.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, rumble ? 1.0 : 0.0);
        driver.getHID().setRumble(GenericHID.RumbleType.kRightRumble, rumble ? 1.0 : 0.0);
        aux.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, rumble ? 1.0 : 0.0);
        aux.getHID().setRumble(GenericHID.RumbleType.kRightRumble, rumble ? 1.0 : 0.0);
    }
}
