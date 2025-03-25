package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.drivetrain.LongDriveAway;

public class SetElevatorPosition extends Command {

    private final double desiredPosition;
    private boolean grabberMode;
    private int finishCounter;

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
        double position = Robot.elevator.getPosition();
        finishCounter = 0;

        grabberMode = Robot.grabber.isOut() && position > desiredPosition;

        if (desiredPosition > position) {
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
        if (grabberMode && Robot.elevator.getVelTPS() < 0) {
            if (desiredPosition == Constants.Elevator.LEVEL_2_POSITION && Robot.elevator.getPosition() < Constants.Elevator.LEVEL_2_POSITION + 5) {
                Robot.grabber.retract();
            }

            if (desiredPosition == Constants.Elevator.GRAB_ALGAE_LOW_POSITION && Robot.elevator.getPosition() < Constants.Elevator.GRAB_ALGAE_LOW_POSITION + 5) {
                Robot.grabber.retract();
            }
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // need to keep this command running until elevator is at target position
        // so it doesn't let PIDHold take over

        // using the finishCounter algorithm broke the algae grabber sequence
        // most likely due to the elevator no longer in the correct position
        // while leaning on the algae, so it never gets to 3 and then
        // doesn't finish. So when in grabber mode, just need one
        // good finished result
        if (grabberMode) {
            return Robot.elevator.isMotionMagicFinished();
        }

        if (Robot.elevator.isMotionMagicFinished()) {
            finishCounter++;
        } else {
            finishCounter = 0;
        }
        return finishCounter > 3;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (grabberMode && (desiredPosition == Constants.Elevator.LEVEL_2_POSITION || desiredPosition == Constants.Elevator.GRAB_ALGAE_LOW_POSITION)) {
            new LongDriveAway().schedule();
            //This time should match the isFinished time from LongDriveAway
            new WaitCommand(0.75).andThen(new SetElevatorPosition(0)).schedule();
        }
    }
}
