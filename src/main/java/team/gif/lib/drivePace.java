package team.gif.lib;

import team.gif.robot.Constants;

public enum drivePace {
    COAST_FR(Constants.Drivetrain.COAST_SPEED_METERS_PER_SECOND, true),
    COAST_RR(Constants.Drivetrain.COAST_SPEED_METERS_PER_SECOND, false),
    SLOW_FR(Constants.Drivetrain.SLOW_SPEED_METERS_PER_SECOND, true),
    SLOW_RR(Constants.Drivetrain.SLOW_SPEED_METERS_PER_SECOND, false),
    BOOST_FR(Constants.Drivetrain.BOOST_SPEED_METERS_PER_SECOND, true),
    BOOST_RR(Constants.Drivetrain.BOOST_SPEED_METERS_PER_SECOND, false);

    private double value;
    private boolean isFieldRelative;

    drivePace(double value, boolean isFieldRelative) {
        this.value = value;
        this.isFieldRelative = isFieldRelative;
    }

    public double getValue() {
        return this.value;
    }
    public boolean getIsFieldRelative() {
        return this.isFieldRelative;
    }
}
