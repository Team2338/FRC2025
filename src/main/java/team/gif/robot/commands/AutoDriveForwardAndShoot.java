package team.gif.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AutoDriveForwardAndShoot extends Command {

    private boolean hasTarget;

    public AutoDriveForwardAndShoot() {
        super();
        addRequirements(Robot.shooter, Robot.swerveDrive);
// uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hasTarget = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        String state = "Not Shooting ***";

        if (Robot.shooter.isFireReady() && !hasTarget) {

            Robot.shooter.moveMotor(Constants.Shooter.SPEED_PERCENT);
            //System.out.println("Shooting");
            state = "Shooting";
            Robot.swerveDrive.drive(0.0, 0.0, 0.0);
            hasTarget = true;
        } else {
//            System.out.println("*** Not Shooting ***");
            if (!hasTarget) {
                Robot.swerveDrive.drive(0.15, 0.0, 0.0);
            }
        }
        System.out.println(Robot.shooter.sensorLeftActive()+" "+Robot.shooter.sensorRightActive() + " " + state);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.moveMotor(0);
    }
}
