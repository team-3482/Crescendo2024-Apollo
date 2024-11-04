// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Constants used throughout the code specifically related to subsystems or unchangeable aspects of the bot.
 * @implSpec BACK of the bot is 180 degrees / the battery, use that as a reference for directions.
 */
public final class PhysicalConstants {
    /**
     * Constants for physical attributes of the robot.
     */
    public static final class RobotConstants {
        /** Name of the CTRE CAN bus (configured on the CANivore). */
        public static final String CTRE_CAN_BUS = "ctre";
    }

    /** Constants for the IntakeSubsystem. */
    public static final class IntakeConstants {
        public static final int LEFT_INTAKE_MOTOR_ID = 20;
        public static final int RIGHT_INTAKE_MOTOR_ID = 21;
        public static final int FRONT_LASER_CHANNEL = 8;
        public static final int BACK_LASER_CHANNEL = 7;

        /** The velocity with which to intake in rot/s. */
        public static final double IDEAL_INTAKE_VELOCITY = 20;
        /** The velocity with which to intake past the back laser in rot/s. */
        public static final double SLOW_INTAKE_VELOCITY = 5;
        /** The velocity with which to eject in rot/s. */
        public static final double IDEAL_EJECT_VELOCITY = -5;

        /** The voltage with which to intake in volts. */
        public static final double IDEAL_INTAKE_VOLTAGE = 10;
        /** The voltage with which to intake past the back laser in volts. */
        public static final double SLOW_INTAKE_VOLTAGE = 5;
        /** The voltage with which eject in volts. */
        public static final double IDEAL_EJECT_VOLTAGE = -5;


        /** This is the gear ratio from the sensor to the rollers. */
        public static final double ROTOR_TO_MECHANISM_RATIO = 4; // 4:1 gearbox

        /** Tolerance for Commands using MotionMagic in rot/s. */
        public static final double VELOCITY_TOLERANCE = 0.5;

        /** Gains used for MotionMagic slot 0. */
        public static final class IntakeSlot0Gains {
            public static final double kG = 0;
            public static final double kS = 0.25;
            public static final double kV = 0.47;
            public static final double kA = 0;
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        /* More constants used with MotionMagic. */
        /**
         * Max mechanism rotations per second.
         * This value is not used for MotionMagicVelocity.
         * It is only here as a reminder.
         */
        public static final double CRUISE_SPEED = 23;
        /** Max mechanism rotations per second^2. */
        public static final double ACCELERATION = 100;
    }

    /** Constants for the ShooterSubsystem. */
    public static final class ShooterConstants {
        public static final int TOP_SHOOTER_MOTOR_ID = 31;
        public static final int BOTTOM_SHOOTER_MOTOR_ID = 30;

        /** This is the gear ratio from the sensor to the rollers. */
        public static final double ROTOR_TO_MECHANISM_RATIO = (double) 18 / 50; // 50 tooth -> 18 tooth

        /**  The radius of the shooter rollers in meters. */
        public static final double ROLLER_RADIUS_METERS = Units.inchesToMeters(1.75) / 2;

        /** Tolerance for Commands using MotionMagic in rot/s. */
        public static final double VELOCITY_TOLERANCE = 2;

        /** Gains used for MotionMagic slot 0. */
        public static final class ShooterSlot0Gains {
            public static final double kG = 0;
            public static final double kS = 0.3;
            public static final double kV = 0.045;
            public static final double kA = 0;
            public static final double kP = 0.3;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        /* More constants used with MotionMagic. */
        /**
         * Max mechanism rotations per second.
         * This value is not used for MotionMagicVelocity.
         * It is only here as a reminder.
         */
        public static final double CRUISE_SPEED = 240; 
        /** Max mechanism rotations per second^2 */
        public static final double ACCELERATION = 800;
    }

    /** Constants for the PivotSubsystem. */
    public static final class PivotConstants {
        public static final int LEFT_PIVOT_MOTOR_ID = 40;
        public static final int RIGHT_PIVOT_MOTOR_ID = 41;

        /** Upper soft stop angle in degrees. */
        public static final double UPPER_ANGLE_LIMIT = 100;
        /** Any angle below this would shoot a note into the back Limelight or the protections. */
        public static final double ABOVE_LIMELIGHT_ANGLE = 16;
        /** Lower soft stop angle in degrees. */
        public static final double LOWER_HARD_STOP = 2.796678; // Hard stop
        
        /** This is the gear ratio from the sensor to the pivot. */
        public static final double ROTOR_TO_MECHANISM_RATIO = 100; // 5:1 -> 2:1 -> 10 : 1 = 100:1
        
        /** Tolerance for Commands using MotionMagic in degrees. */
        public static final double POSITION_TOLERANCE = 0.5;

        /** Gains used for MotionMagic slot 0. */
        public static final class PivotSlot0Gains {
            public static final double kG = 0.24;
            public static final double kS = 0.1;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 512;
            public static final double kI = 0;
            public static final double kD = 8;
        }

        /* More constants used with MotionMagic. */
        /** Max mechanism rotations per second. */
        public static final double CRUISE_SPEED = 0.5;
        /** Max mechanism rotations per second^2 (any higher causes the bot to move). */
        public static final double ACCELERATION = 1.25;
    }
}
