// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import java.util.Map;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.PhysicalConstants.PivotConstants.PivotSlot0Gains;

public class PivotSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile PivotSubsystem instance;
    private static Object mutex = new Object();

    public static PivotSubsystem getInstance() {
        PivotSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new PivotSubsystem();
                }
            }
        }
        return instance;
    }

    /** Runs Shuffleboard updates on a separate thread. */
    private final Notifier notifier;

    private TalonFX leftPivotMotor = new TalonFX(PivotConstants.LEFT_PIVOT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private TalonFX rightPivotMotor = new TalonFX(PivotConstants.RIGHT_PIVOT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("PivotSubsystem", BuiltInLayouts.kGrid)
        .withSize(2, 4)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "TOP"));
    private GenericEntry shuffleboardPositionDial = shuffleboardLayout
        .add("Pivot Position Dial", 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", 0, "Max", 180, "Show Value", true))
        .withSize(2, 2)
        .withPosition(0, 0)
        .getEntry();
    private GenericEntry shuffleboardPositionWidget = shuffleboardLayout
        .add("Pivot Position Float", "0")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(2, 1)
        .withPosition(0, 1)
        .getEntry();

    /** Creates a new PivotSubsystem. */
    private PivotSubsystem() {
        super("PivotSubsystem");

        configureMotionMagic();
        setPositionHardStop();

        // 20 ms update frequency (1 robot cycle)
        this.rightPivotMotor.getPosition().setUpdateFrequency(50);
        this.rightPivotMotor.getVelocity().setUpdateFrequency(50);

        // Right motor used for all PivotSubsystem control (get/set position)
        this.leftPivotMotor.setControl(new Follower(PivotConstants.RIGHT_PIVOT_MOTOR_ID, true));

        this.notifier = new Notifier(() -> notifierLoop());
        this.notifier.setName("Pivot Notifier");
        // 100 ms cycle, or 5 robot cycles.
        this.notifier.startPeriodic(0.1);
    }
    
    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Uses a Notifier for separate-thread Shuffleboard publishing.
    }

    /**
     * This method is used in conjunction with a Notifier to publish
     * Shuffleboard data on a separate thread.
     */
    private synchronized void notifierLoop() {
        double position = getPosition();
        this.shuffleboardPositionDial.setInteger((int) (position + 0.5));
        this.shuffleboardPositionWidget.setString(String.format("%.2f", position));
    }

    /**
     * A helper method that configures MotionMagic on both motors.
     */
    private void configureMotionMagic() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ration from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = PivotConstants.ROTOR_TO_MECHANISM_RATIO; 

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Configs.kG = PivotSlot0Gains.kG;
        slot0Configs.kS = PivotSlot0Gains.kS;
        slot0Configs.kV = PivotSlot0Gains.kV;
        slot0Configs.kA = PivotSlot0Gains.kA;
        slot0Configs.kP = PivotSlot0Gains.kP;
        slot0Configs.kI = PivotSlot0Gains.kI;
        slot0Configs.kD = PivotSlot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = PivotConstants.ACCELERATION;
        // motionMagicConfigs.MotionMagicJerk = PivotConstants.MOTION_MAGIC_JERK;

        // Motor-specific configurations.
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Right motor not inverted.
        this.rightPivotMotor.getConfigurator().apply(configuration);

        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Left motor inverted.
        this.leftPivotMotor.getConfigurator().apply(configuration);
    }

    /**
     * Goes to a position using Motion Magic slot 0.
     * @param position - The position for the pivot in degrees.
     * @param clamp - Whether or not to clamp with the soft limits.
     * @apiNote The soft limits in {@link PivotConstants}.
     */
    public void motionMagicPosition(double position, boolean clamp) {
        if (clamp) {
            position = MathUtil.clamp(position, PivotConstants.LOWER_HARD_STOP, PivotConstants.UPPER_ANGLE_LIMIT);
        }

        MotionMagicVoltage control = motionMagicVoltage
            .withSlot(0)
            .withPosition(Units.degreesToRotations(position));
        
        this.rightPivotMotor.setControl(control);
    }
    /**
     * Goes to a position using Motion Magic slot 0.
     * @param position - The position for the pivot in degrees.
     * @apiNote The position is clamped by the soft limits in {@link PivotConstants}.
     */
    public void motionMagicPosition(double position) {
        motionMagicPosition(position, true);
    }

    /**
     * Sets the speed for the pivot motors.
     * @param speed - Speed from -1.0 to 1.0 for the motors.
     * @param safe - Whether or not to respect soft limits. See {@link PivotConstants} for limits.
     */
    public void setPivotSpeed(double speed, boolean safe) {
        if (safe && speed != 0) {
            double position = getPosition();
            if ((speed > 0 && position >= PivotConstants.UPPER_ANGLE_LIMIT)
                || (speed < 0 && position <= PivotConstants.LOWER_HARD_STOP)) {
                    speed = 0;
                }
        }

        this.rightPivotMotor.set(speed);
    }

    /**
     * Sets the speed for the pivot motors.
     * @param speed - Speed from -1.0 to 1.0 for the motors.
     * @apiNote Will respect soft limits marked in {@link PivotConstants}.
     */
    public void setPivotSpeed(double speed) {
        setPivotSpeed(speed, true);
    }

    /**
     * Sets the mechanism position of both motors.
     * @param position - The position in degrees.
     */
    public void setPosition(double position) {
        position = Units.degreesToRotations(position);
        this.rightPivotMotor.setPosition(position);
        this.leftPivotMotor.setPosition(position);
    }

    /**
     * Sets the mechanism position of both motors to the lower hard stop.
     * @apiNote See hard stop at {@link PivotConstants#LOWER_HARD_STOP}
     */
    public void setPositionHardStop() {
        setPosition(PivotConstants.LOWER_HARD_STOP);
    }

    /**
     * Gets the mechanism position of the right motor.
     * @return The angle in degrees.
     */
    public double getPosition() {
        return Units.rotationsToDegrees(this.rightPivotMotor.getPosition().getValueAsDouble());
    }

    /**
     * Checks if the current position is within
     * {@link PivotConstants#POSITION_TOLERANCE} of an input position.
     * @param position - The position to compare to in rot.
     */
    public boolean withinTolerance(double position) {
        return Math.abs(getPosition() - position) <= PivotConstants.POSITION_TOLERANCE;
    }

    /**
     * Gets the mechanism velocity of the right motor.
     * @return The velocity in degrees/s.
     */
    public double getVelocity() {
        return Units.rotationsToDegrees(this.rightPivotMotor.getVelocity().getValueAsDouble());
    }
}
