// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.function.BiFunction;
import java.util.function.Function;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.util.Units;

/**
 * Constants used throughout the code that are not categorized in other constants files.
 */
public final class Constants {
    /** Constants used for maintaining a heading when using {@link SwerveRequest#FieldCentricFacingAngle} */
    public static final PhoenixPIDController HeadingControllerFacingAngle = new PhoenixPIDController(5.5, 0, 0.1);
    
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
        public static final double PIVOT_POSITION_SPEAKER = 50;
        
        // TODO : Better tolerance function based on distance.
        /** How close to the target rotation to be before printing that it is ready to shoot .*/
        public static final double FACING_ANGLE_TOLERANCE = 2;

        // Heuristic velocity function // TODO 2 : Find
        /** [ Minimum position in meters, minimum velocity in rot/s ]. */
        public static final double[] MIN_POSITION_VELOCITY = new double[]{ 0, 0 };
        /** [ Maximum position in meters, maximum velocity in rot/s ]. */
        public static final double[] MAX_POSITION_VELOCITY = new double[]{ 0, 0 };
        
        /**
         * A function that linearly interpolates a distance for a velocity between 
         * {@link ShootingConstants#MIN_POSITION_VELOCITY} and {@link ShootingConstants#MAX_POSITION_VELOCITY}.
         * If below or above those distances, it will use the velocity at the defined points.
         * @param distance - The distance to the SPEAKER in meters.
         * @return The velocity in rot/s.
         */
        public static final Function<Double, Double> CALCULATE_SHOOTER_VELOCITY = (Double distance) -> {
            final double minD = MIN_POSITION_VELOCITY[0];
            final double minV = MIN_POSITION_VELOCITY[1];
            final double maxD = MAX_POSITION_VELOCITY[0];
            final double maxV = MAX_POSITION_VELOCITY[1];

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
                            Math.pow(velocity, 4)
                            - Math.pow(GRAVITY, 2) * Math.pow(distance, 2)
                            - 2 * GRAVITY * Math.pow(velocity, 2)
                    )
                ) / (GRAVITY * distance)
            ));
        };
    }
}
