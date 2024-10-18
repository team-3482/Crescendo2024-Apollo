// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.PhysicalConstants.ShooterConstants;

/** A class that represents a shot vector in m/s. */
public class ShotVector {
    private final double x;
    private final double y;
    private final double z;
    
    private final double norm;
    private final double yaw;
    private final double pitch;

    /**
     * Constructs a ShotVector.
     * @param x - The x component of the vector in m/s.
     * @param y - The y componnent of the vector in m/s.
     * @param z - The z component of the vector in m/s.
     */
    public ShotVector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        
        this.norm = Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2) + Math.pow(this.z, 2));
        this.yaw = Units.radiansToDegrees(Math.atan2(this.y, this.x));
        this.pitch = Units.radiansToDegrees(Math.asin(this.z / this.norm));
    }

    /**
     * Constructs an empty ShotVector.
     */
    public ShotVector() {
        this(0, 0, 0);
    }

    /**
     * Calculates the yaw of this ShotVector.
     * @return The yaw in degrees.
     */
    public double getYaw() {
        return this.yaw;
    }

    /**
     * Calculates the pitch of this ShotVector.
     * @return The pitch in degrees.
     */
    public double getPitch() {
        return this.pitch;
    }

    /**
     * Calculates the norm of this ShotVector in m/s.
     * @return The norm in m/s.
     */
    public double getNormMps() {
        return this.norm;
    }

    /**
     * Calculates the norm of this ShotVector in rot/s for the shooter rollers.
     * @return The norm in rot/s.
     */
    public double getNormRps() {
        return ShotVector.metersPerSecondToRotationsPerSecond(this.norm);
    }

    /**
     * Returns. the sum of two ShotVectors using the formula {@code x1 + x2, y1 + y2, z1 + z3}.
     * @param other - The ShotVector to add.
     * @return The sum of the ShotVectors.
     */
    public ShotVector plus(ShotVector other) {
        return new ShotVector(
            this.x + other.x,
            this.y + other.y,
            this.z + other.z
        );
    }

    /**
     * Returns the difference between two ShotVectors using the formula {@code x1 - x2, y1 - y2, z1 - z3}.
     * @param other - The ShotVector to substract.
     * @return The difference between the two ShotVectors.
     */
    public ShotVector minus(ShotVector other) {
        return new ShotVector(
            this.x - other.x,
            this.y - other.y,
            this.z - other.z
        );
    }

    /**
     * Calculates the distance between two ShotVectors.
     * @param other - The ShotVector to find the distance to.
     * @return The distance in m/s.
     */
    public double getDistance(ShotVector other) {
        return Math.sqrt(
            Math.pow(other.x - this.x, 2) +
            Math.pow(other.y - this.y, 2) +
            Math.pow(other.z - this.z, 2)
        );
    }

    /**
     * Creates a ShotVector from the given values.
     * @param yaw - The yaw of the robot in degrees.
     * @param pitch - The pitch of the pivot in degrees.
     * @param velocity - The velocity of the shooter in rot/s.
     * @return The ShotVector.
     */
    public static ShotVector fromYawPitchVelocity(double yaw, double pitch, double velocity) {
        Rotation2d yawRot = Rotation2d.fromDegrees(yaw);
        Rotation2d pitchRot = Rotation2d.fromDegrees(pitch);
        double velocityMpS = rotationsPerSecondToMetersPerSecond(velocity);

        return new ShotVector(
            velocityMpS * pitchRot.getCos() * yawRot.getSin(),
            velocityMpS * pitchRot.getCos() * yawRot.getSin(),
            velocityMpS * pitchRot.getSin()
        );
    }

    /**
     * Converts from rot/s to m/s for the shooter rollers.
     * @param speed - The speed in rot/s.
     * @return The speed in m/s.
     */
    public static double rotationsPerSecondToMetersPerSecond(double speed) {
        return speed * (2 * Math.PI * ShooterConstants.ROLLER_RADIUS);
    }

    /**
     * Converts from m/s to rot/s for the shooter rollers.
     * @param speed - The speed in m/s.
     * @return The speed in rot/s.
     */
    public static double metersPerSecondToRotationsPerSecond(double speed) {
        return speed / (2 * Math.PI * ShooterConstants.ROLLER_RADIUS);
    }

    @Override
    public String toString() {
        return String.format("ShotVector(X : %.2f, Y : %.2f, Z : %.2f)", this.x, this.y, this.z);
    }
}