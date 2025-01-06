package team.gif.robot.subsystems.drivers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {

    private final NetworkTable table;

    public static final int LED_PIPELINE = 0;
    public static final int LED_OFF      = 1;
    public static final int LED_BLINK    = 2;
    public static final int LED_ON       = 0;

    public static final int MODE_TRACKING  = 0;
    public static final int MODE_CAMERA    = 1;
    public static int test = 0;

    public double _mountingAngleLLDegrees = -1;
    public double _lensHeightInches = -1;
    public double _goalHeightInches = -1;
    public double _offsetInches = 0; // offset to be added to the calculated distance. Useful if, for example, shooter is in center of bot and limelight is on edge

    /**
     * Create a new limelight object.
     *
     * @param key NetworkTable key specified in limelight web config
     */
    public Limelight(String key) {
        table = NetworkTableInstance.getDefault().getTable(key);
        setLEDMode(Limelight.LED_ON);
    }

    /**
     * Create a new limelight object with default key.
     */
    public Limelight() {
        this("limelight");
    }

    /**
     * Sets the mode of the limelight's LED array.
     * mode 0 uses LED mode in current pipeline (see {@link this#setPipeline(int)}
     * mode 1 is 'force off'
     * mode 2 is 'force blink'
     * mode 3 is 'force on'
     *
     * @param mode desired LED mode
     */
    public void setLEDMode(int mode) {
        if (mode >= 0 && mode <= 3) {
            table.getEntry("ledMode").setNumber(mode);
//            System.out.println("LedMode set to mode " + mode);
        }
    }

    public void setLEDOn() {
        setLEDMode(3);
    }

    public void setLEDOff() {
        setLEDMode(1);
    }


    /**
     * Sets the limelight's mode of operation.
     * mode 0 activates vision processing (decreased exposure)
     * mode 1 activates driver vision (increased exposure, no processing)
     *
     * @param mode desired camera mode
     */
    public void setCamMode(int mode) {
        if (mode >= 0 && mode <= 1) {
            table.getEntry("camMode").setNumber(mode);
            System.out.println("cammode reset " + mode);
        }
    }

    /**
     * Sets the limelight's active vision pipeline.
     * The limelight stores up to 10 pipelines indexed 0-9 (These can be configured through the Web UI).
     *
     * @param pipeline desired vision pipeline
     */
    public void setPipeline(int pipeline) {
        if (pipeline >= 0 && pipeline <= 9) {
            table.getEntry("pipeline").setNumber(pipeline);

        }
    }

    /**
     * Sets the limelight's streaming mode.
     * mode 0 is "Standard - Side-by-side streams if a webcam is attached to Limelight"
     * mode 1 is "PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream"
     * mode 2 is "PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream"
     *
     * @param mode desired streaming mode
     */
    public void setStreamMode(int mode) {
        if (mode >= 0 && mode <= 2) {
            table.getEntry("stream").setNumber(mode);
        }
    }

    /**
     * Sets the limelight's snapshot mode. The limelight allows for taking pictures throughout a match.
     * mode 0 stops taking snapshots
     * mode 1 takes two snapshots per second.
     *
     * @param mode desired snapshot mode
     */
    public void setSnapshotMode(int mode) {
        if (mode >= 0 && mode <= 1) {
            table.getEntry("snapshot").setNumber(mode);
        }
    }

    /**
     * Returns whether the limelight has any valid targets.
     *
     * @return true if has target, false if not
     */
    public boolean hasTarget() {
        return (double)  (table.getEntry("tv").getNumber(0)) == 1;
    }

    /**
     *
     * @return true if does not have target, false if it does
     * Needed for autonomous to continue when a target is not found
     *     and can't use limelight::!hasTarget
     */
    //public boolean noTarget() {
    //return !hasTarget() || !Globals.shooterLimelightEnabled;
    //}

    /**
     * Returns the horizontal offset from crosshair to target.
     *
     * @return offset in degrees (-29.8 to +29.8)
     */
    public double getXOffset() {
        return table.getEntry("tx").getDouble(0.0);
    }

    /**
     * Returns the vertical offset from crosshair to target.
     *
     * @return offset in degrees (-24.85 to +24.85)
     */

    public double getYOffset() {return table.getEntry("ty").getDouble(0.0);}

    /**
     * Returns the percentage of screen area that the target fills.
     *
     * @return percentage of image (0 to 100)
     */
    public double getArea() {
        return table.getEntry("ta").getDouble(0.0);
    }

    /**
     * Returns the skew or rotation of the target.
     *
     * @return rotation in degrees (-90 to 0)
     */
    public double getSkew() {
        return table.getEntry("ts").getDouble(0.0);
    }

    /**
     * Returns the pipeline's latency contribution. Add at least 11 ms for image capture latency.
     *
     * @return latency in ms
     */
    public double getLatency() {
        return table.getEntry("tl").getDouble(0.0);
    }

    /**
     * Returns sidelength of the shortest side of the fitted bounding box.
     *
     * @return sidelength in pixels
     */
    public int getShortLength() {
        return table.getEntry("tshort").getNumber(0).intValue();
    }

    /**
     * Returns sidelength of the longest side of the fitted bounding box.
     *
     * @return sidelength in pixels
     */
    public int getLongLength() {
        return table.getEntry("tlong").getNumber(0).intValue();
    }

    /**
     * Returns horizontal sidelength of the rough bounding box.
     *
     * @return sidelength in pixels (0 to 320)
     */
    public int getHorizLength() {
        return table.getEntry("thoriz").getNumber(0).intValue();
    }

    /**
     * Returns vertical sidelength of the rough bounding box.
     *
     * @return sidelength in pixels (0 to 240)
     */
    public int getVertLength() {
        return table.getEntry("tvert").getNumber(0).intValue();
    }

    /**
     * Gets the pose of the camera relative to the target.
     *
     * @return x[0], y[1], z[2], pitch[3], yaw[4], roll[5]
     */
    public double[] getCamTran() {
        return table.getEntry("camtran").getDoubleArray(new double[]{0, 0, 0, 0, 0, 0});
    }

    /**
     * Returns a data value from 1 of 3 raw (ungrouped) contours given a valid key.
     * <p>
     * "tx[num]" : x position in normalized screenspace (-1 to +1)
     * "ty[num]" : y position in normalized screenspace (-1 to +1)
     * "ta[num]" : area (0 to 100)
     * "ts[num]" : skew (-90 to 0 degrees)
     * <p>
     * Can also be used to retrieve a data value from 1 of 2 crosshairs.
     * "cx[num]" : x position in normalized screenspace (-1 to +1)
     * "cy[num]" : y position in normalized screenspace (-1 to +1)
     *
     * @param key String key for desired data
     * @return raw data value (units vary)
     */
    public double getCustomValue(String key) {
        return table.getEntry(key).getDouble(0.0);
    }

    /**
     * Returns array of x values of each pixel coordinates*
     * the x valuse of the 4 corners in what order sadly i dont know this must be tested
     */
    public double[] getTcornx() {
        return table.getEntry("tcornx").getDoubleArray(new double[]{0, 0, 0, 0});
    }

    /**
     * Returns array of x values of each pixel coordinates*
     * the y valuse of the 4 corners in what order sadly i dont know this must be tested
     */
    public double[] getTcorny() {
        return table.getEntry("tcorny").getDoubleArray(new double[]{0, 0, 0, 0});
    }

    /**
     * Stores the distance parameters for the getDistance method to be called
     * @param mountingAngleLLDegrees degrees back the limelight is rotated from perfectly vertical
     * @param lensHeightInches distance in inches from the center of the Limelight lens to the floor
     * @param goalHeightInches  distance from the target/AprilTag to the floor
     */
    public void setDistanceEstimatorParams(double mountingAngleLLDegrees,
                                           double lensHeightInches,
                                           double goalHeightInches,
                                           double offsetInches) {
        _mountingAngleLLDegrees = mountingAngleLLDegrees;
        _lensHeightInches = lensHeightInches;
        _goalHeightInches = goalHeightInches;
        _offsetInches = offsetInches;
    }

    /**
     * Distance between the robot and the target/AprilTag in inches.
     * Distance parameters must be set using setDistanceEstimatorParams prior to calling this method <br>
     * @return distance in inches <br>
     *       returns -1 if the limelight does not have a target  <br>
     *       returns -2 if the parameters were not set <br>
     */
    public double getDistance() {
        if (_mountingAngleLLDegrees == -1 ||
            _lensHeightInches == -1 ||
            _goalHeightInches == -1 ){
            return -2;
        }
        return DistanceEstimator(_mountingAngleLLDegrees,_lensHeightInches,_goalHeightInches,_offsetInches);
    }

    /**
     * distance between the robot and the target/AprilTag in inches
     * @param mountingAngleLLDegrees degrees back the limelight is rotated from perfectly vertical
     * @param lensHeightInches distance in inches from the center of the Limelight lens to the floor
     * @param goalHeightInches  distance from the target/AprilTag to the floor
     * @return the distance in inches, returns -1 if the limelight does not have a target
     */
    public double DistanceEstimator(double mountingAngleLLDegrees, double lensHeightInches, double goalHeightInches, double offsetInches) {
        if (!hasTarget()) {
            return -1;
        }

        double targetOffsetAngle_Vertical = getYOffset();
        double angleToGoalDegrees = mountingAngleLLDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance (shooter is 4 inches behind limelight)
        return offsetInches + (goalHeightInches - lensHeightInches) / Math.tan(angleToGoalRadians);
    }
}
