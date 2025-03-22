package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class StageCoral extends Command {

    public StageCoral() {
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.shooter.setShooterBrakeMode();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        boolean isIndexerSensorActive = Robot.shooter.getIndexerSensorState();
        boolean isExitSensorActive = Robot.shooter.getExitSensorState();

        //if both not active, don't move
        if (!isExitSensorActive && !isIndexerSensorActive) {
            Robot.shooter.runIndexerMotor();
//            Robot.shooter.stopShooterMotor();
            Robot.shooter.runTopIndexShooterMotor();
            return;
        }

        //if only indexer sensor active, move
        if (!isExitSensorActive && isIndexerSensorActive) {
            Robot.shooter.runIndexerMotor();
            Robot.shooter.runStageShooterMotor();
            return;
        }

        //if shooter sensor active, stop
        if (isExitSensorActive) {
            Robot.shooter.stopIndexerMotor();
            Robot.shooter.stopShooterMotor();
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
        Robot.shooter.stopIndexerMotor();
        Robot.shooter.stopShooterMotor();
        Robot.shooter.setShooterCoastMode();
    }
}
