package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Grabber extends SubsystemBase {
    private Solenoid grabber;

    public Grabber() {
        grabber = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.GRABBER_SOLENOID_PORT);
    }

    public void deploy() {
        grabber.set(true);
    }

    public void retract() {
        grabber.set(false);
    }

    /**
     *
     * @return true if grabber is out/deployed, false if in/retracted
     */
    public boolean isOut() {
        return grabber.get();
    }
}
