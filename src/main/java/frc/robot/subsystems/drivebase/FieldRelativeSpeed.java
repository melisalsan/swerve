package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;

public class FieldRelativeSpeed {
    private final double fieldX;
    private final double fieldY;

    /**
     * Creates a field relative speed object from robot relative speeds
     * @param robotX The robot's x speed
     * @param robotY The robot's y speed
     * @param robotAngle The robot's angle
     */
    public FieldRelativeSpeed(double robotX, double robotY, Rotation2d robotAngle) {
        // Gets the field relative X and Y components of the robot's speed in the X axis
        double robotXXComp = robotAngle.getCos() * robotX;
        double robotXYComp = robotAngle.getSin() * robotX;

        // Gets the field relative X and Y components of the robot's speed in the Y axis
        double robotYXComp = robotAngle.getSin() * robotY;
        double robotYYComp = robotAngle.getCos() * robotY;

        // Adds the field relative X and Y components of the robot's X and Y speeds to get the overall field relative X and Y speeds
        fieldX = robotXXComp + robotYXComp;
        fieldY = robotXYComp + robotYYComp;
    }

    /**
     * Gets the field relative X speed
     * @return The field relative X speed
     */
    public double getX() {
        return fieldX;
    }

    /**
     * Gets the field relative Y speed
     * @return The field relative Y speed
     */
    public double getY() {
        return fieldY;
    }
}
