package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class Shoot extends Command {
    private int counter;

    /**
     * Runs the shooter motor and ends after a predefined set of seconds (self-ending). <br>
     * No prerequisites (i.e. regardless of game piece or reef branch sensors)
     */
    public Shoot() {
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.elevator.getPosition() > Constants.Elevator.ELEVATOR_POS_SHOOT_REQ) {
         return;
        }

        counter++;
        Robot.shooter.runShooterMotor();
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter > 12 || Robot.elevator.getPosition() < 10;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.stopShooterMotor();
    }
}
