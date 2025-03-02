package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Grabber extends SubsystemBase {
    private DoubleSolenoid grabber;

    public Grabber() {
        grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.GRABBER_SOLENOID_IN_PORT, RobotMap.GRABBER_SOLENOID_OUT_PORT);
    }

    public void deploy() {
        grabber.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        grabber.set(DoubleSolenoid.Value.kReverse);
    }

}
