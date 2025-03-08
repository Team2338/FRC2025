package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.drivetrain.LongDriveAway;

public class SetElevatorPosition extends Command {

    private final double desiredPosition;
    private boolean grabberOut;

    public SetElevatorPosition(double targetPosition) {
        super();
        addRequirements(Robot.elevator);

        // do not allow code to set a point higher or lower than max/min
        if (targetPosition > Constants.Elevator.MAX_POS) { targetPosition = Constants.Elevator.MAX_POS; }
        if (targetPosition < Constants.Elevator.MIN_POS) { targetPosition = Constants.Elevator.MIN_POS; }

        desiredPosition = targetPosition;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        grabberOut = Robot.grabber.isOut();

        if (desiredPosition > Robot.elevator.getPosition()) {
            Robot.elevator.configMotionMagicUp();
            Robot.elevator.setMotionMagic(desiredPosition);
        } else {
            Robot.elevator.configMotionMagicDown();
            Robot.elevator.setMotionMagic(desiredPosition);
        }
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        //Only retract if elevator is on the way down (velocity < 0)
        if (grabberOut && Robot.elevator.getVelTPS() < 0) {
            if (desiredPosition == Constants.Elevator.LEVEL_2_POSITION && Robot.elevator.getPosition() < Constants.Elevator.LEVEL_2_POSITION + 5) {
                Robot.grabber.retract();
            }

            if (desiredPosition == Constants.Elevator.GRABBER_POSITION && Robot.elevator.getPosition() < Constants.Elevator.GRABBER_POSITION + 5) {
                Robot.grabber.retract();
            }
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // need to keep this command running until elevator is at target position
        // so it doesn't let PIDHold take over
        return Robot.elevator.isMotionMagicFinished();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (grabberOut && (desiredPosition == Constants.Elevator.LEVEL_2_POSITION || desiredPosition == Constants.Elevator.GRABBER_POSITION)) {
            new LongDriveAway().andThen(new SetElevatorPosition(0)).schedule();
        }
    }
}
