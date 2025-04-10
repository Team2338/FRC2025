package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.drivePace;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ClimberClimb extends Command {

    public ClimberClimb() {
        super();

        addRequirements(Robot.climber, Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Disable the elevator when climbing preventing the aux from accidentally raising the elevator
        // Do not re-enable at the end of the command. Want to keep elevator disabled until
        // aux toggles manaul mode. Although Deploy called this, it is possible aux toggled manual
        // control after deploying so make sure the elevator continues to be disabled
        Robot.elevator.disableElevator();

        // while climbing, disable the shooter motor to reduce battery drain
        if (Robot.shooter.getDefaultCommand() != null) {
            Robot.shooter.getDefaultCommand().cancel();
            Robot.shooter.removeDefaultCommand();
        }
        Robot.shooter.stopShooterMotor();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.climber.move(-Constants.Climber.CLIMB_PERCENT);

        // deploy the piston when the climber reaches a predetermined set point
        // and only call it once
//        if (Robot.climber.getPosition() < Constants.Climber.PISTON_DEPLOY_POS && !Robot.climber.getPistonStateOut()) {
        Robot.climber.setPistonOut();

        // Drive forward at a slow speed to assist in the climb
        Robot.swerveDrive.setDrivePace(drivePace.COAST_ROT);
        Robot.swerveDrive.drive(0, Constants.Climber.DRIVE_SPEED_MPS, 0);
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
        Robot.swerveDrive.stopDrive();
        Robot.swerveDrive.setDrivePace(drivePace.COAST_FR);
    }
}
