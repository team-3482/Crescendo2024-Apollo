package frc.robot.limelights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.LimelightConstants.VisionConstants;
import frc.robot.swerve.CommandSwerveDrivetrain;

/**
 * <p>A helper class used for storing MegaTag and MegaTag2 data from a Limelight
 * to avoid unnecessary calls to the NetworkTables API.
 * @apiNote MT rotation should not be combined with MT2 pose, because their timestamps may differ.
 */
public class VisionData {
    public final String name;
    public final LimelightHelpers.PoseEstimate MegaTag;
    public final LimelightHelpers.PoseEstimate MegaTag2;
    public final boolean canTrustRotation;
    public final boolean canTrustPosition;
    /** Flag set after optimization to avoid re-optimizing data twice in a row on low FPS. */
    public boolean optimized;

    /**
     * Creates a new LimelightData object.
     * @param name - The name of this Limelight.
     * @param MegaTag data.
     * @param MegaTag2 data.
     */
    public VisionData(String name, LimelightHelpers.PoseEstimate MegaTag, LimelightHelpers.PoseEstimate MegaTag2) {
        this.name = name;
        this.MegaTag = MegaTag;
        this.MegaTag2 = MegaTag2;
        this.optimized = false;

        // These are calculated when the measurement is created
        this.canTrustRotation = canTrustRotation();
        this.canTrustPosition = canTrustPosition();
    }

    /**
     * Checks if the average tag distance and bot's rotational and translational velocities
     * are reasonable for trusting rotation data, as well as MegaTag having >= 2 targets.
     * @return Whether rotation data can be trusted.
     * @apiNote Dist <= 3 meters ; Angular <= 160 deg/s ; Translational <= 2 m/s.
     */
    private boolean canTrustRotation() {
        ChassisSpeeds robotChassisSpeeds = CommandSwerveDrivetrain.getInstance().getCurrentRobotChassisSpeeds();
        double velocity = Math.sqrt(Math.pow(robotChassisSpeeds.vxMetersPerSecond, 2) + Math.pow(robotChassisSpeeds.vyMetersPerSecond, 2));
        return this.MegaTag2 != null
            // && this.MegaTag2.tagCount > 0
            && this.MegaTag2.avgTagDist <= 3 // 3 Meters
            && this.MegaTag != null
            && this.MegaTag.tagCount >= 2
            && Units.radiansToDegrees(robotChassisSpeeds.omegaRadiansPerSecond) <= 160
            && velocity <= 2;
    }

    /**
     * Checks if the MegaTag2 Pose2d is within an acceptable distance of the bot's position.
     * @return Whether position data can be trusted.
     */
    private boolean canTrustPosition() {
        ChassisSpeeds robotChassisSpeeds = CommandSwerveDrivetrain.getInstance().getCurrentRobotChassisSpeeds();
        double velocity = Math.sqrt(Math.pow(robotChassisSpeeds.vxMetersPerSecond, 2) + Math.pow(robotChassisSpeeds.vyMetersPerSecond, 2));
        return this.MegaTag2 != null
            && this.MegaTag2.tagCount > 0
            && this.MegaTag2.avgTagDist < VisionConstants.TRUST_TAG_DISTANCE
            // && CommandSwerveDrivetrain.getInstance().getState().Pose.getTranslation()
                // .getDistance(this.MegaTag2.pose.getTranslation()) <= 1.5
            && Units.radiansToDegrees(robotChassisSpeeds.omegaRadiansPerSecond) <= 160
            && velocity <= 2;
    }
}