package frc.robot;

public class Utils {
    /**
     * Clamps a value between a minimum and maximum value
     * @param value The value to clamp
     * @param min The minimum value in the clamp range
     * @param max The maximum value in the clamp range
     * @return The clamped value
     */
    public static double clamp(double value, double min, double max) {
        double target = value;
        if (value < min) {
            target = min;
        } else if (value > max) {
            target = max;
        }

        return target;
    }

    /**
     * Applies a deadband to a value. If a value is within the deadband, it will be set to 0
     * @param value The value to apply the deadband to
     * @param deadband The deadband
     * @return The deadbanded value
     */
    public static double deadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            value = 0;
        }

        return value;
    }

    /**
     * Applies a deadband of 0.1 to a value. If a value is within the deadband, it will be set to 0
     * @param value The value to apply the deadband to
     * @return The deadbanded value
     */
    public static double deadband(double value) {
        return deadband(value, 0.1);
    }
}
