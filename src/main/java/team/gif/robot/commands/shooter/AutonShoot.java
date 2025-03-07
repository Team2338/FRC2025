package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.commands.drivetrain.ShortDriveAway;
import team.gif.robot.commands.drivetrain.StopModules;

public class AutonShoot extends SequentialCommandGroup {
    public AutonShoot() {
        addCommands(
//                new MoveModulesIn(),
                new AutonStrafeToTarget(),
                new ParallelRaceGroup( // ends when any command ends // if robot does not have a target, don't shoot
                        new Shoot(),
                        new AutonHasNoTarget(), //Will abort shot if no target is detected
                        new StopModules()
                )
//                new SequentialCommandGroup(
//                        new ShortDriveAway(),
//                        new StopModules()
//                )
        );
    }
}
