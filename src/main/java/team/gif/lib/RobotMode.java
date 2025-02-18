package team.gif.lib;

public enum RobotMode {
    ROBOT_CONTROL(0),
    MANUAL(1);

    private double value;

    RobotMode(double value) {
        this.value = value;
    }

    public double getValue() {
        return this.value;
    }
}