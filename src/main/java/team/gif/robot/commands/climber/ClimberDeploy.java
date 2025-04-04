package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;


public class ClimberDeploy extends Command {

    public ClimberDeploy() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Disable the elevator when climbing preventing the aux from accidentally raising the elevator
        // Do not re-enable at the end of the command. Want to keep elevator disabled until
        // aux toggles manaul mode.
        //Robot.elevator.disableElevator();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.climber.move(Constants.Climber.DEPLOY_PERCENT);
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
