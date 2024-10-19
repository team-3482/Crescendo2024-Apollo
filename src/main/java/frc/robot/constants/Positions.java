// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Optional;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.limelights.VisionSubsystem;

/**
 * A class that stores positions used for initialization and calculations.
 */
public final class Positions {
    /**
     * An enum used to decide which position to initialize to.
     */
    public enum PositionInitialization {
        TOP,
        MIDDLE,
        BOTTOM,
        /** This position will be initialized to whatever the {@link VisionSubsystem} sees */
        LIMELIGHT
        ;
    }

    /** SPEAKER positions to target */
    private static final Map<DriverStation.Alliance, Translation3d> SPEAKER_TARGET = Map.ofEntries(
        Map.entry(DriverStation.Alliance.Blue, new Translation3d(-0.45, 5.55, Units.inchesToMeters(95))),
        Map.entry(DriverStation.Alliance.Red, new Translation3d(16.95, 5.55, Units.inchesToMeters(95)))
    );

    /** AMP positions to line up to */
    private static final Map<DriverStation.Alliance, Pose2d> AMP_TARGET = Map.ofEntries(
        Map.entry(DriverStation.Alliance.Blue,
            new Pose2d(new Translation2d(1.8, 7.66), Rotation2d.fromDegrees(90))
        ),
        Map.entry(DriverStation.Alliance.Red,
            new Pose2d(new Translation2d(14.7, 7.66), Rotation2d.fromDegrees(-90))
        )
    );

    /** Initial bot positions used for initializing odometry, blue-alliance relative. */
    private static final Map<DriverStation.Alliance, Map<PositionInitialization, Pose2d>> STARTING_POSITIONS = Map.ofEntries(
        Map.entry(DriverStation.Alliance.Blue, Map.ofEntries(
            Map.entry(PositionInitialization.TOP, new Pose2d(new Translation2d(0.75, 6.66), Rotation2d.fromDegrees(60))),
            Map.entry(PositionInitialization.MIDDLE, new Pose2d(new Translation2d(1.34, 5.55), Rotation2d.fromDegrees(0))),
            Map.entry(PositionInitialization.BOTTOM, new Pose2d(new Translation2d(0.75, 4.45), Rotation2d.fromDegrees(300))))),
        Map.entry(DriverStation.Alliance.Red, Map.ofEntries(
            Map.entry(PositionInitialization.TOP, new Pose2d(new Translation2d(15.8, 6.66), Rotation2d.fromDegrees(120))),
            Map.entry(PositionInitialization.MIDDLE, new Pose2d(new Translation2d(15.2, 5.55), Rotation2d.fromDegrees(180))),
            Map.entry(PositionInitialization.BOTTOM, new Pose2d(new Translation2d(15.8, 4.50), Rotation2d.fromDegrees(240)))))
    );

    /**
     * Get the starting position of the robot.
     * @param position to get.
     * @return a Pose2d.
     */
    public static Pose2d getStartingPose(PositionInitialization position) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            System.err.println("Starting Position : Alliance is empty ; relying on Limelight data.");
        }

        if (alliance.isEmpty() || position == PositionInitialization.LIMELIGHT) {
            System.out.println("getStartingPose() | Using LIMELIGHT data");
            return VisionSubsystem.getInstance().getEstimatedPose();
        }

        return Positions.STARTING_POSITIONS.get(alliance.get()).get(position);
    }

    /**
     * Gets the SPEAKER target position for the current alliance.
     * @throws RuntimeException if the alliance is empty.
     * @return a Translation3d to aim for.
     */
    public static Translation3d getSpeakerTarget() throws RuntimeException {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            throw new RuntimeException("SPEAKER Target : Alliance is empty ; cannot target SPEAKER.");
        }
        return Positions.SPEAKER_TARGET.get(alliance.get());
    }
    
    /**
     * Gets the AMP target position for the current alliance.
     * @throws RuntimeException if the alliance is empty.
     * @return a Pose2d to drive to.
     */
    public static Pose2d getAmpTarget() throws RuntimeException {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            throw new RuntimeException("AMP Target : Alliance is empty ; cannot target AMP.");
        }
        return Positions.AMP_TARGET.get(alliance.get());
    }
}
