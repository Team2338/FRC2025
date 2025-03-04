package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ClimberClimb extends Command {

    public ClimberClimb() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //disable the elevator when climbing preventing the aux from accidentally raising the elevator
        Robot.elevator.disableElevator();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.climber.move(-Constants.Climber.CLIMB_PERCENT);

        // deploy the piston when the climber reaches a predetermined set point
        if (Robot.climber.getPosition() < Constants.Climber.PISTON_DEPLOY_POS && !Robot.climber.getPistonStateOut()) {
            Robot.climber.setPistonOut();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.climber.move(0);
    }
}
