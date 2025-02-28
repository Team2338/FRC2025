package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.drivePace;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AutonStrafeToTarget extends Command {

    private boolean hasTarget;
    private int counter;

    public AutonStrafeToTarget() {
        super();
        addRequirements(Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hasTarget = false;
        Robot.swerveDrive.setDrivePace(drivePace.COAST_RR);
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.elevator.getPosition() < Constants.Elevator.LEVEL_4_POSITION - 1 ) {
            System.out.println("counter " + counter++);
            return;
        }

        boolean leftSensor = Robot.shooter.sensorLeftActive();
        boolean rightSensor = Robot.shooter.sensorRightActive();

        // if neither sensor is active, default to moving left
        if (!leftSensor && !rightSensor) {
            Robot.swerveDrive.drive(0.0, 0.15, 0.0);
        }

        // if only the right sensor is active, move right
        if (!leftSensor && rightSensor) {
            Robot.swerveDrive.drive(0.0, -0.15, 0.0);
        }

        // if only the left sensor is active, move left
        if (leftSensor && !rightSensor) {
            Robot.swerveDrive.drive(0.0, 0.15, 0.0);
        }

        // if both sensors are active, the robot has the target
        if (leftSensor && rightSensor) {
            hasTarget = true;
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return hasTarget;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(0.0, 0.0, 0.0);
        Robot.swerveDrive.setDrivePace(drivePace.COAST_FR);
    }
}

