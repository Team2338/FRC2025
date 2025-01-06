package team.gif.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
//-    public final CommandXboxController test = new CommandXboxController(RobotMap.TEST_CONTROLLER_ID);

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
    }
}
