package frc.robot.wrappers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    private final NetworkTable table;

    /**
     * Creates a new Limelight object
     * @param tableName NetworkTables table name for the limelight data
     */
    public Limelight(String tableName) {
        table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    /**
     * Sets the LED mode for the limelight
     * @param state Desired LED state
     */
    public void setLED(LEDState state) {
        table.getEntry("ledMode").setNumber(state.getID());
    }

    /**
     * Whether the limelight is tracking a target
     * @return Whether the limelight is tracking a target
     */
    public boolean isTrackingObject() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Gets the horizontal offset from the target
     * @return The horizontal offset from the target
     */
    public double getDeltaX() {
        return table.getEntry("tx").getDouble(0);
    }

    /**
     * Gets the vertical offset from the target
     * @return The vertical offset from the target
     */
    public double getDeltaY() {
        return table.getEntry("ty").getDouble(0);
    }

    /**
     * Gets the area of the target
     * @return The area of the target
     */
    public double getArea() {
        return table.getEntry("ta").getDouble(0);
    }

    /**
     * Sets the pipeline for the limelight to use to track targets
     * @param pipeline The target pipeline ID (0-9)
     */
    public void setPipeline(int pipeline) {
        if (pipeline < 0 || pipeline > 9) return;
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public enum Pipeline {
        MAIN(0);

        private final int state;

        Pipeline(int state) {
            this.state = state;
        }

        public int getID() {
            return state;
        }
    }

    public enum LEDState {
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);

        private final int id;

        LEDState(int id) {
            this.id = id;
        }

        public int getID() {
            return id;
        }
    }
}
