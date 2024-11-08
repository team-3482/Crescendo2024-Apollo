// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.function.BiFunction;
import java.util.function.Function;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.PhysicalConstants.ShooterConstants;

/**
 * Constants used throughout the code that are not categorized in other constants files.
 */
public final class Constants {
    /** Constants used for maintaining a heading when using {@link SwerveRequest#FieldCentricFacingAngle} */
    public static final PhoenixPIDController HeadingControllerFacingAngle = new PhoenixPIDController(6, 0, 0);
    
    /**
     * Tab names in Shuffleboard.
     */
    public static final class ShuffleboardTabNames {
        public static final String DEFAULT = "Competition";
        public static final String UTILITIES = "Utilities";
    }

    /** Constants for the controller and any controller related assignments. */
    public static final class ControllerConstants {
        /** DriverStation ID of the driver controller. */
        public static final int DRIVER_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller. */
        public static final int OPERATOR_CONTROLLER_ID = 1;
        /** Removes input around the joystick's center (eliminates stick drift). */
        public static final double DEADBAND = 0.075;
        /** Whether or not to accept directional pad input for movement. */
        public static final boolean DPAD_DRIVE_INPUT = true;
        /** Speed multiplier when using fine-control movement. */
        public static final double FINE_CONTROL_MULT = 0.15;
    }

    /** Constants used for shooting commands. */
    public static final class ShootingConstants {
        /** The position of the pivot in degrees to shoot into the speaker from right in front of it. */
        public static final double PIVOT_POSITION_SPEAKER = 57.5;
        /** The speed of the shooter in rot/s to shoot into the SPEAKER from right in front of it. */
        public static final double SHOOTER_SPEED_SPEAKER = 150;
        
        /** The position of the pivot in degrees to shoot into the AMP from right in front of it. */
        public static final double PIVOT_POSITION_AMP = 100;
        /** The speed of the shooter in rot/s to shoot into the AMP from right in front of it. */
        public static final double SHOOTER_SPEED_AMP = 30;
        
        /** The position of the pivot in degrees to pass a note. */
        public static final double PIVOT_POSITION_PASS = 55;
        /** The tolerance for the heading when passing a note in degrees. */
        public static final double YAW_TOLERANCE_PASS = 2.5;
        /** The speed of the shooter in rot/s to pass a note. */
        public static final double SHOOTER_SPEED_PASS = 180;

        /** Maximum reasonable accurate distance for AutoShooting in meters. */
        public static final double MAX_SHOOTING_DISTANCE = 3.75;
        /** Maximum reasonable speed with which the robot can still shoot in m/s. */
        public static final double MAX_MOVEMENT_SPEED = 0.75;

        // Heuristic velocity function
        /** [ Minimum position in meters, minimum velocity in rot/s,  ]. */
        public static final double[] MIN_POSITION_VELOCITY = new double[]{ 1.5, 125 };
        /** [ Maximum position in meters, maximum velocity in rot/s ]. */
        public static final double[] MAX_POSITION_VELOCITY = new double[]{ 5.5, ShooterConstants.CRUISE_SPEED };
    }

    /**
     * Functions used for AutoShooting math.
     */
    public static final class ShootingFunctions {
        /**
         * Function that allows 10 cm of yaw error at 1 meter and calculates further
         * tolerances at larger distances.
         * @param distance - The distance to the SPEAKER in meters.
         * @return The yaw tolerance in degrees.
         */
        public static final Function<Double, Double> CALCULATE_YAW_TOLERANCE = (Double distance) -> {
            final double toleranceAt1Meter = 0.14; // 14 cm
            double tolerance = Math.atan(toleranceAt1Meter / distance);
            return Units.radiansToDegrees(tolerance);
        };

        /**
         * Function that allows 10 cm of pitch error at 1 meter and calculates further
         * tolerances at larger distances.
         * @param distance - The distance to the SPEAKER in meters.
         * @return The pitch tolerance in meters.
         */
        public static final Function<Double, Double> CALCULATE_PITCH_TOLERANCE = (Double distance) -> {
            final double toleranceAt1Meter = 0.05; // 5 cm
            double tolerance = Math.abs(
                Math.atan((Positions.SPEAKER_HEIGHT + (toleranceAt1Meter / 2)) / distance)
                - Math.atan((Positions.SPEAKER_HEIGHT - (toleranceAt1Meter / 2)) / distance)
            );
            return Units.radiansToDegrees(tolerance / 2);
        };

        /**
         * Function that allows 10 cm of shooter error at 1 meter and calculates further
         * tolerances at larger distances.
         * @param distance - The distance to the SPEAKER in meters.
         * @param pitch - The pitch in degrees.
         * @return The shooter error in m/s.
         */
        public static final BiFunction<Double, Double, Double> CALCULATE_SPEED_TOLERANCE = (Double distance, Double pitch) -> {
            final double toleranceAt1Meter = 0.025; // 2.5 cm
            final double GRAVITY = 9.81;
            pitch = Units.degreesToRadians(pitch);

            double v1 = Math.sqrt(
                (GRAVITY * Math.pow(distance, 2)) / (
                    2 * Math.pow(Math.cos(pitch), 2) * (
                        distance * Math.tan(pitch) - Positions.SPEAKER_HEIGHT - toleranceAt1Meter
                    )
                )
            );

            double v2 = Math.sqrt(
                (GRAVITY * Math.pow(distance, 2)) / (
                    2 * Math.pow(Math.cos(pitch), 2) * (
                        distance * Math.tan(pitch) - Positions.SPEAKER_HEIGHT + toleranceAt1Meter
                    )
                )
            );

            return Math.abs(Math.max(v1, v2) - Math.min(v1, v2));
        };
        
        /**
         * A function that linearly interpolates a distance for a velocity between 
         * {@link ShootingConstants#MIN_POSITION_VELOCITY} and {@link ShootingConstants#MAX_POSITION_VELOCITY}.
         * If below or above those distances, it will use the velocity at the defined points.
         * @param distance - The distance to the SPEAKER in meters.
         * @return The velocity in rot/s.
         */
        public static final Function<Double, Double> CALCULATE_SHOOTER_VELOCITY = (Double distance) -> {
            final double minD = ShootingConstants.MIN_POSITION_VELOCITY[0];
            final double minV = ShootingConstants.MIN_POSITION_VELOCITY[1];
            final double maxD = ShootingConstants.MAX_POSITION_VELOCITY[0];
            final double maxV = ShootingConstants.MAX_POSITION_VELOCITY[1];

            if (distance <= minD) {
                return minV;
            }
            else if (distance >= maxD) {
                return maxV;
            }
            else {
                return minV + (distance - minD) * (maxV - minV) / (maxD - minD);
            }
        };

        /**
         * A formula that calculates the pitch for the shooter based on the input velocity and the distance.
         * @param distance - The distance to the SPEAKER in meters.
         * @param velocity - The input velocity in m/s.
         * @return The pitch for the shot in degrees.
         */
        public static final BiFunction<Double, Double, Double> CALCULATE_SHOOTER_PITCH = (Double distance, Double velocity) -> {
            final double GRAVITY = 9.81;

            return Units.radiansToDegrees(Math.atan(
                (
                    Math.pow(velocity, 2)
                    - Math.sqrt(
                            Math.pow(velocity, 4) - GRAVITY * (
                                GRAVITY * Math.pow(distance, 2)
                                + 2 * Positions.SPEAKER_HEIGHT * Math.pow(velocity, 2)
                            )
                    )
                ) / (GRAVITY * distance)
            ));
        };
    }
}
