// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelights;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.LimelightConstants.VisionConstants;
import frc.robot.swerve.CommandSwerveDrivetrain;

/** 
 * A class that manages AprilTag Limelights for vision.
 * <p>Optimizes detection for better performance and pushes
 * position updates to the internal odometer.
 */
public class VisionSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile VisionSubsystem instance;
    private static Object mutex = new Object();

    public static VisionSubsystem getInstance() {
        VisionSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new VisionSubsystem();
                }
            }
        }
        return instance;
    }

    /** Used to run vision processing on a separate thread. */
    private final Notifier notifier;

    /** Latest Limelight data. May contain faulty data unsuitable for odometry. */
    private volatile VisionData[] limelightDatas = new VisionData[2];
    /** Last heartbeat of the front LL (updated every frame) */
    private volatile long lastHeartbeatFrontLL = 0;
    /** Last heartbeat of the back LL (updated every frame) */
    private volatile long lastHeartbeatBackLL = 0;
    private volatile Timer lastDataTimer;

    // Lists used for tag filtering. Final to reduce wasted processing power.
    private final List<Integer> BLUE_SOURCE = Arrays.asList(1, 2, 3, 4);
    private final List<Integer> RED_SPEAKER = Arrays.asList(1, 2, 3, 4, 5);
    private final List<Integer> RED_AMP = Arrays.asList(5, 4, 3);
    private final List<Integer> BLUE_AMP = Arrays.asList(6, 7, 8);
    private final List<Integer> BLUE_SPEAKER = Arrays.asList(6, 7, 8, 9, 10);
    private final List<Integer> RED_SOURCE = Arrays.asList(7, 8, 9, 10);

    /** Creates a new VisionSubsystem. */
    private VisionSubsystem() {
        super("VisionSubsystem");

        if (LimelightConstants.PUBLISH_CAMERA_FEEDS) {
            // Shuffleboard camera feeds.
            HttpCamera frontLLCamera = new HttpCamera(
                LimelightConstants.FRONT_APRIL_TAG_LL,
                "http://" + LimelightConstants.FRONT_APRIL_TAG_LL + ".local:5800/stream.mjpg"
            );
            HttpCamera backLLCamera = new HttpCamera(
                LimelightConstants.BACK_APRIL_TAG_LL,
                "http://" + LimelightConstants.BACK_APRIL_TAG_LL + ".local:5800/stream.mjpg"
            );

            Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
                .add(LimelightConstants.FRONT_APRIL_TAG_LL, frontLLCamera)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withProperties(Map.of("Show Crosshair", false, "Show Controls", false))
                .withPosition(0, 0);
            Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
                .add(LimelightConstants.BACK_APRIL_TAG_LL, backLLCamera)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withProperties(Map.of("Show Crosshair", false, "Show Controls", false))
                .withPosition(6, 0);
        }

        this.lastDataTimer = new Timer();
        this.lastDataTimer.start();

        this.notifier = new Notifier(() -> notifierLoop());
        this.notifier.setName("Vision Notifier");
        // Assuming ~40 fps / 25 ms cycle.
        this.notifier.startPeriodic(0.025);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Uses a Notifier for separate-thread Vision processing
    }

    /**
     * This method is used in conjunction with a Notifier to run vision processing on a separate thread.
     */
    private synchronized void notifierLoop() {
        // This method gets data in about 4 to 8 ms.
        VisionData[] filteredLimelightDatas = getFilteredLimelightData(false);
        
        // This loop generally updates data in about 6 ms, but may double or triple for no apparent reason.
        // This causes loop overrun warnings, however, it doesn't seem to be due to inefficient code and thus can be ignored.
        for (VisionData data : filteredLimelightDatas) {
            if (data.canTrustRotation) {
                // Only trust rotational data when adding this pose.
                CommandSwerveDrivetrain.getInstance().setVisionMeasurementStdDevs(VecBuilder.fill(
                    9999999,
                    9999999,
                    recentVisionData() ? 1 : 0.5
                ));
                CommandSwerveDrivetrain.getInstance().addVisionMeasurement(
                    data.MegaTag.pose,
                    data.MegaTag.timestampSeconds
                );
            }

            if (data.canTrustPosition) {
                if (CommandSwerveDrivetrain.getInstance().getState().Pose.getTranslation()
                        .getDistance(data.MegaTag2.pose.getTranslation())
                        <= 2
                ) {
                    this.lastDataTimer.restart();
                }

                // Only trust positional data when adding this pose.
                CommandSwerveDrivetrain.getInstance().setVisionMeasurementStdDevs(VecBuilder.fill(
                    recentVisionData() ? 0.7 : 0.1,
                    recentVisionData() ? 0.7 : 0.1,
                    9999999
                ));
                CommandSwerveDrivetrain.getInstance().addVisionMeasurement(
                    data.MegaTag2.pose,
                    data.MegaTag2.timestampSeconds
                );
            }
        }

        // This method is suprprisingly efficient, generally below 1 ms.
        optimizeLimelights();
    }

    /**
     * Gets the most trustworthy data from each Limelight. Also updates {@link VisionSubsystem#limelightDatas} and heartbeats.
     * @param useStored - When true, no data will be retrieved from the Limelights, stored data will be filtered instead.
     * @return LimelightData for each trustworthy Limelight data.
     * @apiNote Will theoretically stop updating data if the heartbeat resets.
     * However, this happens at 2e9 frames, which would take consecutive 96 days at a consistent 240 fps.
     */
    private VisionData[] getFilteredLimelightData(boolean useStored) {
        LimelightHelpers.PoseEstimate frontLLDataMT2 = null;
        LimelightHelpers.PoseEstimate backLLDataMT2 = null;
        long heartbeatFrontLL = -1;
        long heartbeatBackLL = -1;

        // Periodic logic
        if (!useStored) {
            double rotationDegrees = CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees();
            LimelightHelpers.SetRobotOrientation(LimelightConstants.FRONT_APRIL_TAG_LL,
                rotationDegrees, 0, 0, 0, 0, 0
            );
            LimelightHelpers.SetRobotOrientation(LimelightConstants.BACK_APRIL_TAG_LL,
                rotationDegrees, 0, 0, 0, 0, 0
            );
            
            heartbeatFrontLL = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.FRONT_APRIL_TAG_LL, "hb").getInteger(-1);
            heartbeatBackLL = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.BACK_APRIL_TAG_LL, "hb").getInteger(-1);

            if (heartbeatFrontLL == -1 || this.lastHeartbeatFrontLL < heartbeatFrontLL) {
                frontLLDataMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.FRONT_APRIL_TAG_LL);
                LimelightHelpers.PoseEstimate frontLLDataMT = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.FRONT_APRIL_TAG_LL);
                this.limelightDatas[0] = new VisionData(LimelightConstants.FRONT_APRIL_TAG_LL, frontLLDataMT, frontLLDataMT2);
                this.lastHeartbeatFrontLL = heartbeatFrontLL == -1 ? this.lastHeartbeatFrontLL : heartbeatFrontLL;
            }
            
            if (heartbeatBackLL == -1 || this.lastHeartbeatBackLL < heartbeatBackLL) {
                backLLDataMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.BACK_APRIL_TAG_LL);
                LimelightHelpers.PoseEstimate backLLDataMT = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.BACK_APRIL_TAG_LL);
                this.limelightDatas[1] = new VisionData(LimelightConstants.BACK_APRIL_TAG_LL, backLLDataMT, backLLDataMT2);
                this.lastHeartbeatBackLL = heartbeatBackLL == -1 ? this.lastHeartbeatBackLL : heartbeatBackLL;
            }

            ChassisSpeeds robotChassisSpeeds = CommandSwerveDrivetrain.getInstance().getCurrentRobotChassisSpeeds();
            double velocity = Math.sqrt(Math.pow(robotChassisSpeeds.vxMetersPerSecond, 2) + Math.pow(robotChassisSpeeds.vyMetersPerSecond, 2));
            // If the bot's angular velocity is greater than 270 deg/s, translational velocity is over 2 m/s,
            // or for both LLs the data is outdated or has no data, ignore vision updates.
            if (Math.abs(Units.radiansToDegrees(robotChassisSpeeds.omegaRadiansPerSecond)) > 270
                || Math.abs(velocity) > 2 // m/s
                || (this.lastHeartbeatBackLL != heartbeatBackLL && this.lastHeartbeatFrontLL != heartbeatFrontLL)
                || ((frontLLDataMT2 == null || frontLLDataMT2.tagCount == 0) && (backLLDataMT2 == null || backLLDataMT2.tagCount == 0))
                || ((frontLLDataMT2 == null || frontLLDataMT2.avgTagDist > VisionConstants.TRUST_TAG_DISTANCE)
                    && (backLLDataMT2 == null || backLLDataMT2.avgTagDist > VisionConstants.TRUST_TAG_DISTANCE))
            ) {
                return new VisionData[0];
            }
        }

        // Allows LLs to compare data even if they have unsynced FPS / heartbeats.
        // Upon startup, these may still be null, so it is important to check for them or robot code could crash.
        // Also, ignore data that is older than 1 second.
        double timestampNow = Timer.getFPGATimestamp();
        if (frontLLDataMT2 == null && this.limelightDatas[0] != null) {
            frontLLDataMT2 = this.limelightDatas[0].MegaTag2;
            
            if (frontLLDataMT2 != null && Math.abs(frontLLDataMT2.timestampSeconds - timestampNow) > 1) {
                frontLLDataMT2 = null;
            }
        }
        if (backLLDataMT2 == null && this.limelightDatas[1] != null) {
            backLLDataMT2 = this.limelightDatas[1].MegaTag2;
            
            if (backLLDataMT2 != null && Math.abs(backLLDataMT2.timestampSeconds - timestampNow) > 1) {
                backLLDataMT2 = null;
            }
        }
        if (frontLLDataMT2 == null && backLLDataMT2 == null) {
            return new VisionData[0];
        }

        // Returns the data with the greater tag count.
        // Will only return the data if it has the same heartbeat as just captured (if it doesn't,
        // this means the data was retrieved from this.limelightDatas and not during this loop).
        if ((frontLLDataMT2 != null && (useStored || this.lastHeartbeatFrontLL == heartbeatFrontLL))
            && (backLLDataMT2 == null
                || backLLDataMT2.avgTagDist > VisionConstants.TRUST_TAG_DISTANCE
                || frontLLDataMT2.tagCount > backLLDataMT2.tagCount)) {
                return new VisionData[]{ this.limelightDatas[0] };
        }
        else if ((backLLDataMT2 != null && (useStored || this.lastHeartbeatBackLL == heartbeatBackLL))
            && (frontLLDataMT2 == null
                || frontLLDataMT2.avgTagDist > VisionConstants.TRUST_TAG_DISTANCE
                || backLLDataMT2.tagCount > frontLLDataMT2.tagCount)) {
                return new VisionData[]{ this.limelightDatas[1] };
        }

        // Returns the data that's closer to its respective camera than 90% of the other's distance.
        // 90% is a heuristic.
        if ((!useStored && this.lastHeartbeatFrontLL == heartbeatFrontLL)
            && frontLLDataMT2.avgTagDist < backLLDataMT2.avgTagDist * 0.9) {
            return new VisionData[]{ this.limelightDatas[0] };
        }
        else if ((!useStored && this.lastHeartbeatBackLL == heartbeatBackLL)
            && backLLDataMT2.avgTagDist < frontLLDataMT2.avgTagDist * 0.9) {
            return new VisionData[]{ this.limelightDatas[1] };
        }

        // This return statement assumes that both LLs have the same amount of tags and
        // are near equadistant from one another.
        return this.limelightDatas;
    }

    /**
     * A helper method used to optimize Limelight FPS.
     */
    private void optimizeLimelights() {
        byte index = 0; // Used only for setting the optimized flag, so that this can be a for-each loop.
        for (VisionData limelightData : this.limelightDatas) {
            if (limelightData == null || limelightData.optimized) {
                return;
            }
            else {
                this.limelightDatas[index++].optimized = true;
            }
            
            // Avoid unnecessary optimization for a LL with no tags and
            // reset any optimization that might have been done previously.
            if (limelightData.MegaTag2 == null || limelightData.MegaTag2.tagCount == 0) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 1.5f);
                LimelightHelpers.SetFiducialIDFiltersOverride(limelightData.name, VisionConstants.ALL_TAG_IDS);
                LimelightHelpers.setCropWindow(
                    limelightData.name,
                    -VisionConstants.DEFAULT_CROP_SIZE,
                    VisionConstants.DEFAULT_CROP_SIZE,
                    -VisionConstants.DEFAULT_CROP_SIZE,
                    VisionConstants.DEFAULT_CROP_SIZE
                );
                continue;
            }

            // Downscaling closer to tags.
            if (limelightData.MegaTag2.avgTagDist < 1.5) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 3);
            }
            else if (limelightData.MegaTag2.avgTagDist < 2) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 2);
            }
            else {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightData.name, 1.5f);
            }

            // Tag filtering for nearby tags.
            Set<Integer> nearbyTagsSet = new HashSet<Integer>();
            for (LimelightHelpers.RawFiducial fiducial : limelightData.MegaTag2.rawFiducials) {
                switch (fiducial.id) {
                    case 1:
                    case 2:
                        nearbyTagsSet.addAll(this.BLUE_SOURCE);
                    case 3:
                    case 4:
                        nearbyTagsSet.addAll(this.RED_SPEAKER);
                    case 5:
                        nearbyTagsSet.addAll(this.RED_AMP);
                    case 6:
                        nearbyTagsSet.addAll(this.BLUE_AMP);
                    case 7:
                    case 8:
                        nearbyTagsSet.addAll(this.BLUE_SPEAKER);
                    case 9:
                    case 10:
                        nearbyTagsSet.addAll(this.RED_SOURCE);
                    default: // 11, 12, 13, 14, 15, and 16 have no relevant tags near them.
                        nearbyTagsSet.add(fiducial.id);
                }
            }
            int[] nearbyTagsArray = nearbyTagsSet.stream().mapToInt(i -> i).toArray();
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightData.name, nearbyTagsArray);
            
            // Smart cropping around on-screen AprilTags and potential nearby ones.
            // For explanations of variables such as tx vs txnc, see :
            // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#basic-targeting-data.
            if (limelightData.MegaTag2.rawFiducials.length == 0) {
                LimelightHelpers.setCropWindow(
                    limelightData.name,
                    -VisionConstants.DEFAULT_CROP_SIZE,
                    VisionConstants.DEFAULT_CROP_SIZE,
                    -VisionConstants.DEFAULT_CROP_SIZE,
                    VisionConstants.DEFAULT_CROP_SIZE
                );
            }
            else {
                LimelightHelpers.RawFiducial txncBig = null;
                LimelightHelpers.RawFiducial txncSmall = null;
                LimelightHelpers.RawFiducial tyncBig = null;
                LimelightHelpers.RawFiducial tyncSmall = null;
                double sideLength = 0;
                
                // Finds the txnc and tync that are furthest from the crosshair
                // (for largest bounding box that will include all targets on screen).
                for (LimelightHelpers.RawFiducial fiducial: limelightData.MegaTag2.rawFiducials) {
                    // This formula is explained below.
                    sideLength = Math.sqrt(fiducial.ta * VisionConstants.FOV_AREA) / 2;
                    
                    if (txncBig == null || fiducial.txnc + sideLength > txncBig.txnc) {
                        txncBig = fiducial;
                    }
                    if (txncSmall == null || fiducial.txnc - sideLength < txncSmall.txnc) {
                        txncSmall = fiducial;
                    }
                    
                    if (tyncBig == null || fiducial.tync + sideLength > tyncBig.tync) {
                        tyncBig = fiducial;
                    }
                    if (tyncSmall == null || fiducial.tync - sideLength < tyncSmall.tync) {
                        tyncSmall = fiducial;
                    }
                }
                
                // The formulas used below create the bounding boxes around targets and work in this way :
                //
                // The position of the target (x or y) that was found to be the furthest from the principal pixel for that direction
                // (largest/smallest x and largest/smallest y).
                //     MINUS for the smallest positions (left/bottom of the box) or PLUS for the largest positions (right/top of the box).
                //         The length of the side of the targets — This is found in the following way :
                //           We know the FOV area (AprilTagLLConstants.FOV_AREA) -> We know percentage of screen target occupies (ta) ->
                //           Targets are roughly squares at most angles so sqrt(target area in pixels) = side lengths.
                //         Which is MULTIPLIED by a function that scales with distance (further away needs larger box due
                //         to bot movements having more impact on target position from the camera's POV) in the following way :
                //           ` 2 (heuristic, this determines general box size) * ln(distance to target + 1) `
                //           The +1 is added to the natural log to avoid a negative value for distances of less than 1 meter,
                //           even if those are very rare. Natural log is probably not the best function for this, but it works well enough.
                //
                // Together this comes out as (be careful to follow order of operations) :
                // ` Target Position +/- Target Length * (2 * ln(Distance + 1)) `
                //
                // In the end this is DIVIDED by HALF of the rough width or height of the FOV,
                // because Limelights expect cropping to be [-1.0, 1.0].

                double xSmall = (txncSmall.txnc - Math.sqrt(txncSmall.ta * VisionConstants.FOV_AREA) * (2 * Math.log(txncSmall.distToCamera + 1)))
                    / (VisionConstants.FOV_X / 2);
                double xBig = (txncBig.txnc + Math.sqrt(txncBig.ta * VisionConstants.FOV_AREA) * (2 * Math.log(txncBig.distToCamera + 1)))
                    / (VisionConstants.FOV_X / 2);
                
                LimelightHelpers.setCropWindow(
                    limelightData.name,
                    // In the x directions, 2.5x the size of the box if there are expected tags there.
                    // This allows the LL to lose the second tag for a few frame without cropping solely to
                    // the remaining one and no longer seeing the other (since crop only resets when both tags are lost).
                    //                           leftmost coordinate - 1.5 * (horizontal size of box) = a box 2.5x its original size
                    getNearbyTagDirection(txncSmall.id) < 0 ? xSmall - 1.5 * Math.abs(xBig - xSmall) : xSmall,
                    getNearbyTagDirection(txncBig.id) > 0 ? xBig + 1.5 * Math.abs(xBig - xSmall) : xBig,
                    (tyncSmall.tync - Math.sqrt(tyncSmall.ta * VisionConstants.FOV_AREA) * (2 * Math.log(tyncBig.distToCamera + 1)))
                        / (VisionConstants.FOV_Y / 2),
                    (tyncBig.tync + Math.sqrt(tyncBig.ta * VisionConstants.FOV_AREA) * (2 * Math.log(tyncSmall.distToCamera + 1)))
                        / (VisionConstants.FOV_Y / 2)
                );
            }
        }
    }
    
    /**
     * This is a helper for {@link VisionSubsystem#optimizeLimelights(VisionData[])} smart cropping.
     * @param id - The target ID to consider.
     * @return Whether to expect another tag on the left, right, or neither.
     * @apiNote Left : -1 ; Right : +1 ; Neither : 0.
     */
    private int getNearbyTagDirection(int id) {
        switch (id) {
            case 1:
            case 3:
            case 7:
            case 9:
                return -1;
            case 2:
            case 4:
            case 8:
            case 10:
                return 1;
            default:
                return 0;
        }
    }

    /**
     * Gets the most trustworthy data from each Limelight and returns a {@link Pose2d} object.
     * @return The most accurate {@link Pose2d}, or an empty one if there is none.
     * @apiNote If MegaTag rotation cannot be trusted, it will use the odometry's current rotation.
     */
    public Pose2d getEstimatedPose() {
        VisionData[] filteredLimelightDatas = getFilteredLimelightData(true);

        if (filteredLimelightDatas.length == 0) {
            System.err.println("getEstimatedPose() | NO LIMELIGHT DATA, DEFAULTING TO EMTPY POSE2D");
            return new Pose2d();
        }
        else if (filteredLimelightDatas.length == 1) {
            if (filteredLimelightDatas[0].MegaTag2.tagCount == 0) {
                return new Pose2d();
            }

            return new Pose2d(
                filteredLimelightDatas[0].MegaTag2.pose.getTranslation(),
                filteredLimelightDatas[0].canTrustRotation ?
                    filteredLimelightDatas[0].MegaTag.pose.getRotation() : CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation()
            );
        }
        else {
            if (filteredLimelightDatas[0].MegaTag2.tagCount == 0 || filteredLimelightDatas[1].MegaTag2.tagCount == 0) {
                return new Pose2d();
            }

            // Average them for best accuracy
            return new Pose2d(
                // (First translation + Second translation) / 2
                filteredLimelightDatas[0].MegaTag2.pose.getTranslation().plus(filteredLimelightDatas[1].MegaTag2.pose.getTranslation()).div(2),
                filteredLimelightDatas[0].canTrustRotation ?
                    // First rotation / 2 + Second rotation / 2
                    //
                    // This is done to avoid problems due to Rotation2d being [0, 360) 
                    // Ex : 180+180=0 followed by 0/2=0 when it should be 180+180=360 and 360/2=180.
                    filteredLimelightDatas[0].MegaTag.pose.getRotation().div(2)
                        .plus(filteredLimelightDatas[1].MegaTag.pose.getRotation().div(2)) :
                    CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation()
            );
        }
    }

    /**
     * Returns if vision data has been processed in the last {@link VisionConstants#RECENT_DATA_CUTOFF}.
     * @return If there is recent vision data.
     */
    public boolean recentVisionData() {
        return !this.lastDataTimer.hasElapsed(VisionConstants.RECENT_DATA_CUTOFF);
    }
}
