package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.elevator.SetElevatorPosition;

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
        //Different speed for L1
        if (Math.abs(Robot.elevator.getPosition() - Constants.Elevator.LEVEL_1_POSITION) < 3) {
            Robot.shooter.runShooterMotorLevelOne();
        } else {
            Robot.shooter.runShooterMotor(Constants.Shooter.SHOOT_PERCENT);
        }

        counter++;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter > Constants.Shooter.SHOOT_CYCLES;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.stopShooterMotor();

        // if we are shooting LEVEL 1, we are using manual shoot so need to move elevator here
        if (Robot.elevator.getTargetPosition() == Constants.Elevator.LEVEL_1_POSITION ){
            new SetElevatorPosition(Constants.Elevator.COLLECTOR_POSITION).schedule();
        }
    }
}
