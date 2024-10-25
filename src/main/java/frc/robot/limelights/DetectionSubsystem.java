// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelights;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.LimelightConstants.DetectionConstants;
import frc.robot.swerve.CommandSwerveDrivetrain;
import frc.robot.utilities.FilteredTranslation;

/** A class that manages note detection and associated calculations. */
public class DetectionSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile DetectionSubsystem instance;
    private static Object mutex = new Object();

    public static DetectionSubsystem getInstance() {
        DetectionSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new DetectionSubsystem();
                }
            }
        }
        return instance;
    }

    /** Used to run detection processing on a separate thread. */
    private final Notifier notifier;

    private volatile ArrayList<DetectionData> recentDetectionDatas = new ArrayList<DetectionData>();
    private volatile ArrayList<FilteredTranslation> recentFilteredTranslations = new ArrayList<FilteredTranslation>();

    /** Last heartbeat of the LL (updated every frame) */
    private volatile long lastHeartbeatLL = 0;

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Note Pose");
    private final DoubleArrayPublisher note1 = table.getDoubleArrayTopic("note1").publish();
    private final DoubleArrayPublisher note2 = table.getDoubleArrayTopic("note2").publish();
    private final DoubleArrayPublisher note3 = table.getDoubleArrayTopic("note3").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("DetectionSubsystem", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "TOP"))
        .withSize(4, 3)
        .withPosition(12, 4);
    private final GenericEntry widthDistanceEntry = shuffleboardLayout
        .add("Closest Width-D (m)", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0, "Max", DetectionConstants.MAX_NOTE_DISTANCE_TRUST, "Num tick marks", 5))
        .withSize(2, 1)
        .withPosition(0, 0)
        .getEntry();
    private final GenericEntry pitchDistanceEntry = shuffleboardLayout
        .add("Closest Pitch-D (m)", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0, "Max", DetectionConstants.MAX_NOTE_DISTANCE_TRUST, "Num tick marks", 5))
        .withSize(2, 1)
        .withPosition(0, 1)
        .getEntry();

    /** Creates a new DetectionSubsystem. */
    private DetectionSubsystem() {
        super("DetectionSubsystem");

        // Shuffleboard camera feed.
        if (LimelightConstants.PUBLISH_CAMERA_FEEDS) {
            HttpCamera frontLLCamera = new HttpCamera(
                LimelightConstants.NOTE_DETECTION_LL,
                "http://" + LimelightConstants.NOTE_DETECTION_LL + ".local:5800/stream.mjpg"
            );

            Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
                .add(LimelightConstants.NOTE_DETECTION_LL, frontLLCamera)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withProperties(Map.of("Show Crosshair", false, "Show Controls", false))
                .withPosition(12, 0);
        }

        this.notifier = new Notifier(() -> notifierLoop());
        this.notifier.setName("Detection Notifier");
        // Assuming ~30 fps / ~33 ms cycle.
        this.notifier.startPeriodic(1.0 / 30);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Uses a Notifier for separate-thread Vision processing
    }

    /**
     * This method is used in conjunction with a Notifier to run detection processing on a separate thread.
     * @see _ Trust : {@link DetectionData#canTrustData()}
     * <li> Processing : {@link DetectionSubsystem#processRecentDetectionDatas()}
     * @apiNote Will add trusted DetectionData and always process current DetectionData.
     */
    private synchronized void notifierLoop() {
        DetectionData[] detectionDatasArray = getDetectionDatas();
        ArrayList<DetectionData> detectionDatasList = new ArrayList<DetectionData>();

        for (DetectionData data : detectionDatasArray) {
            if (data.canTrustData) {
                detectionDatasList.add(data);
            }
        }

        if (detectionDatasList.size() != 0) {
            // Don't replace stale data, it will get cleared in
            // {@link DetectionSubsystem#processRecentDetectionDatas} if too stale.
            double recentSize = this.recentDetectionDatas.size();
            for (int i = 0; i < detectionDatasList.size(); i++) {
                if (i < recentSize) {
                    this.recentDetectionDatas.set(i, detectionDatasList.get(i));
                }
                else {
                    this.recentDetectionDatas.add(i, detectionDatasList.get(i));
                }
            }

            // Process the updated data (for the FilteredTranslations)
            processRecentDetectionDatas(false);
        }
        // Just clear stale data if there is any
        else {
            processRecentDetectionDatas(true);
        }

        if (DetectionConstants.PUBLISH_NOTE_POSES) {
            Pose2d[] recentNotePoses = getRecentNotePoses();
            
            switch (recentNotePoses.length) {
                case 0:
                    this.note3.set(new double[0]);
                    this.note2.set(new double[0]);
                    this.note1.set(new double[0]);
                    break;
                case 1:
                    this.note3.set(new double[0]);
                    this.note2.set(new double[0]);
                    this.note1.set(pose2dToDoubleArray(recentNotePoses[0]));
                    break;
                case 2:
                    this.note3.set(new double[0]);
                    this.note2.set(pose2dToDoubleArray(recentNotePoses[1]));
                    this.note1.set(pose2dToDoubleArray(recentNotePoses[0]));
                    break;
                default:
                    this.note3.set(pose2dToDoubleArray(recentNotePoses[2]));
                    this.note2.set(pose2dToDoubleArray(recentNotePoses[1]));
                    this.note1.set(pose2dToDoubleArray(recentNotePoses[0]));
                    break;    
            }
        }

        this.fieldTypePub.set("Field2d");
    }

    /**
     * Gets the recently saved note positions.
     * @return The recently calculated note positions.
     * @apiNote Uses data up to {@link DetectionConstants#STALE_DATA_CUTOFF} seconds old.
     */
    public Pose2d[] getRecentNotePoses() {
        Pose2d[] recentNotePoses = new Pose2d[this.recentFilteredTranslations.size()];
        
        for (int i = 0; i < recentNotePoses.length; i++) {
            recentNotePoses[i] = new Pose2d(
                this.recentFilteredTranslations.get(i).getLastTranslation(),
                new Rotation2d()
            );
        }
        
        return recentNotePoses;
    }

    /**
     * A helper that updates {@link DetectionSubsystem#recentFilteredTranslations}
     * based on {@link DetectionSubsystem#recentDetectionDatas}.
     * @param onlyClearStale - Whether or not to only clear stale data and not calculate new translations.
     */
    private synchronized void processRecentDetectionDatas(boolean onlyClearStale) {
        // Remove any stale DetectionData
        double timeCutoff = Timer.getFPGATimestamp() - DetectionConstants.STALE_DATA_CUTOFF;
        for (int i = 0; i < this.recentDetectionDatas.size(); i++) {
            if (this.recentDetectionDatas.get(i).timestamp <= timeCutoff) {
                this.recentDetectionDatas.remove(i);
                if (i < this.recentFilteredTranslations.size()) {
                    this.recentFilteredTranslations.remove(i);
                }
                i--;
            }
        }

        // Clear any extra Notes from storage
        while (this.recentFilteredTranslations.size() > this.recentDetectionDatas.size()) {
            this.recentFilteredTranslations.remove(this.recentFilteredTranslations.size() - 1);
        }

        // If there was no new DetectionData, don't reprocess current data or
        // it will mislead the linear filters with repetitive data.
        if (onlyClearStale) return;

        boolean updatedShuffleboard = false;

        for (int i = 0; i < this.recentDetectionDatas.size(); i++) {
            DetectionData data = this.recentDetectionDatas.get(i);
            
            double widthDistance = getDistanceFromWidth(data.width, data.tx);
            double pitchDistance = getDistanceFromPitch(data.ty);

            // Only update Shuffleboard data for the closest trustworthy Note
            if (!updatedShuffleboard && data.canTrustData) {
                this.widthDistanceEntry.setDouble(widthDistance);
                this.pitchDistanceEntry.setDouble(pitchDistance);
                updatedShuffleboard = true;
            }

            double distance;

            if (data.canTrustWidth) {
                distance = widthDistance;
            }
            else if (data.canTrustPitch) {
                distance = pitchDistance;
            }
            else {
                continue;
            }

            if (distance <= DetectionConstants.MAX_NOTE_DISTANCE_TRUST) {
                Translation2d noteTranslation = getNoteTranslation(distance, data.tx);
                
                if (i >= this.recentFilteredTranslations.size()) {
                    this.recentFilteredTranslations.add(new FilteredTranslation(noteTranslation));
                }
                else {
                    Translation2d calculatedTranslation = this.recentFilteredTranslations.get(i).getNextTranslation(noteTranslation);
                    
                    // If over 1 meter of error, then the noteTranslation is probably a new Note.
                    // Thus, the filter is reinitialized to the new translation.
                    if (noteTranslation.getDistance(calculatedTranslation) >= 1) {
                        this.recentFilteredTranslations.set(i, new FilteredTranslation(noteTranslation));
                    }
                }
            }
        }

        if (!updatedShuffleboard) {
            this.widthDistanceEntry.setDouble(0);
            this.pitchDistanceEntry.setDouble(0);
            return;
        }
    }

    /**
     * Helper that turns a Pose2d into a NetworkTables-readable double array.
     * @param pose - The pose to convert.
     * @return The double array representation of the pose.
     * @apiNote The rotation is in radians.
     */
    private double[] pose2dToDoubleArray(Pose2d pose) {
        return new double[]{
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
    }

    /**
     * Helper that sorts detection data and returns an array of detected Notes.
     * @return Detection data.
     * @see {@link LimelightHelpers#getBotPoseEstimate(String limelightName, String entryName)}
     * for getting the data and timestamps.
     */
    private synchronized DetectionData[] getDetectionDatas() {
        long recentHeartbeatLL = LimelightHelpers.getLimelightNTTableEntry(LimelightConstants.NOTE_DETECTION_LL, "hb").getInteger(-1);

        // If not on a new frame, do not re-process a previous frame.
        if (recentHeartbeatLL == -1 || this.lastHeartbeatLL < recentHeartbeatLL) {
            this.lastHeartbeatLL = recentHeartbeatLL == -1 ? this.lastHeartbeatLL : recentHeartbeatLL;
        }
        else {
            return new DetectionData[0];
        }

        DoubleArrayEntry rawDetectionsEntry = LimelightHelpers.getLimelightDoubleArrayEntry(LimelightConstants.NOTE_DETECTION_LL, "rawdetections");
        double tl = LimelightHelpers.getLimelightNTDouble(LimelightConstants.NOTE_DETECTION_LL, "tl");
        double cl = LimelightHelpers.getLimelightNTDouble(LimelightConstants.NOTE_DETECTION_LL, "cl");

        TimestampedDoubleArray tsValue = rawDetectionsEntry.getAtomic();
        double[] rawDetections = tsValue.value;
        ArrayList<DetectionData> detectionDatas = new ArrayList<DetectionData>();

        double adjustedTimestamp = (tsValue.timestamp / 1000000.0) - ((tl + cl) / 1000.0);

        for (int i = 0; i < rawDetections.length; i += 12) {
            double tx = rawDetections[i + 1];
            double ty = rawDetections[i + 2];

            double[] xCorners = new double[]{
                rawDetections[i + 4], // Top left
                rawDetections[i + 6], // Top right
            };

            double[] yCorners = new double[]{
                rawDetections[i + 5], // Top left
                rawDetections[i + 11], // Bottom left
            };

            detectionDatas.add(new DetectionData(tx, ty, xCorners, yCorners, adjustedTimestamp));
        }

        return detectionDatas.toArray(new DetectionData[detectionDatas.size()]);
    }

    /**
     * Calculates the distance of a note using its width.
     * This is more trustworthy than the height, because the width varies more.
     * @param noteWidth - The width of the note in pixels.
     * @param tx - Raw tx from the camera in degrees.
     * @return The distance to the note in meters.
     * @see {@link DetectionConstants#PIXEL_TO_RAD}.
     * @apiNote This function is mostly heuristic and has about 7 cm error above 0.5 meters.
     */
    private double getDistanceFromWidth(double noteWidth, double tx) {
        double theta = noteWidth / DetectionConstants.PIXEL_TO_RAD;
        
        double distance = DetectionConstants.NOTE_DIAMETER / Math.tan(theta);
        distance += DetectionConstants.NOTE_DIAMETER / 2;
        
        if (distance >= 0.5) {
            distance += DetectionConstants.WIDTH_DIST_ERROR_CORRECTION.apply(distance);
        }
        
        distance = Math.abs(distance / Math.cos(Units.degreesToRadians(1.75 * tx)));
        
        return distance;
    }

    /**
     * Calculates the distance of a note using the vertical angle.
     * This is less accurate and should be used when the full width of the note is unavailable.
     * @param ty - Raw ty from the camera in degrees.
     * @return The distance to the note in meters.
     */
    private double getDistanceFromPitch(double ty) {
        double pitch = DetectionConstants.LIMELIGHT_POSITION.getRotation().getY();
        double theta = Math.PI / 2 - Math.abs(pitch + Units.degreesToRadians(ty));
        double height = DetectionConstants.LIMELIGHT_POSITION.getZ(); // - DetectionConstants.NOTE_Height
        
        double distance = height * Math.tan(theta);
        distance += DetectionConstants.PITCH_DIST_ERROR_CORRECTION.apply(distance);
        
        return distance;
    }

    /**
     * Calculates the Translation2d of a note in field space.
     * @param distance - The distance to the note from the camera.
     * @param tx - Raw tx from the camera in degrees.
     * @return The Translation2d of the note.
     */
    private Translation2d getNoteTranslation(double distance, double tx) {
        double yaw = Math.PI / 2 + DetectionConstants.LIMELIGHT_POSITION.getRotation().getZ();
        double theta = yaw + Units.degreesToRadians(tx);
        boolean over90Deg = false;
        
        if (theta > Math.PI / 2) {
            theta = Math.PI - theta;
            over90Deg = true;
        }

        Pose2d botPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
        
        Translation2d cameraToNote = new Translation2d(
            distance * Math.sin(theta),
            distance * Math.cos(theta)
        );

        Translation2d botToNote = new Translation2d(
            DetectionConstants.LIMELIGHT_POSITION.getX() + cameraToNote.getX(),
            -DetectionConstants.LIMELIGHT_POSITION.getY()
                + cameraToNote.getY() * (over90Deg ? -1 : 1)
                
        );

        double theta2 = Math.atan(botToNote.getY() / botToNote.getX()) + botPose.getRotation().getRadians();
        double dBotToNote = Math.sqrt(Math.pow(botToNote.getY(), 2) + Math.pow(botToNote.getX(), 2));

        Translation2d botToNoteFieldSpace = new Translation2d(
            dBotToNote * Math.cos(theta2),
            dBotToNote * Math.sin(theta2)
        );

        Translation2d noteTranslation = new Translation2d(
            botPose.getX() + botToNoteFieldSpace.getX(), 
            botPose.getY() + botToNoteFieldSpace.getY() 
        );

        return noteTranslation;
    }
}
