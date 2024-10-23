// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A class that filter Translation2d linearly over {@link FilteredTranslation#TAPS} inputs
 * based on distance between x components and y components.
 */
public class FilteredTranslation {
    /**
     * Number of samples to average over.
     * @see {@link LinearFilter#movingAverage(int)}.
     */
    public static final int TAPS = 15;
    private final LinearFilter xFilter;
    private final LinearFilter yFilter;
    private double lastX;
    private double lastY;

    /**
     * Constructs a FilteredTranslation with the X and Y components equal to the provided values.
     * @param x The x component of the translation.
     * @param y The y component of the translation.
     */
    public FilteredTranslation(double x, double y) {
        this.xFilter = LinearFilter.movingAverage(FilteredTranslation.TAPS);
        this.yFilter = LinearFilter.movingAverage(FilteredTranslation.TAPS);
        
        // These calls are expensive but I am unsure if there is a better way to
        // set a brand new LinearFilter to a value ?
        
        // Limiter because filter won't return exactly x to every decimal and
        // a while loop would run infinitely.
        for (int limiter = 0; limiter < FilteredTranslation.TAPS && this.lastX != x; limiter++) {
            this.lastX = this.xFilter.calculate(x);
        }
        for (int limiter = 0; limiter < FilteredTranslation.TAPS && this.lastY != y; limiter++) {
            this.lastY = this.yFilter.calculate(y);
        }
    }

    /**
     * Constructs a FilteredTranslation with a translation.
     * @param translation - The translation to use.
     */
    public FilteredTranslation(Translation2d translation) {
        this(translation.getX(), translation.getY());
    }

    /**
     * Constructs a FilteredTranslation with X and Y components equal to zero.
     */
    public FilteredTranslation() {
        this(0, 0);
    }

    /**
     * Calculates the distance between two translations in 2D space.
     *
     * <p>The distance between translations is defined as √((x₂−x₁)²+(y₂−y₁)²).
     *
     * @param other The translation to compute the distance to.
     * @return The distance between the two translations.
     */
    public double getDistance(Translation2d other) {
        return Math.hypot(other.getX() - this.lastX, other.getY() - this.lastY);
    }

    /**
     * Returns the X component of the translation.
     * @return The X component of the translation.
     */
    public double getLastX() {
        return this.lastX;
    }
    
    /**
     * Returns the Y component of the translation.
     * @return The Y component of the translation.
     */
    public double getLastY() {
        return this.lastY;
    }

    /**
     * Calculates the next X component of the translation.
     * @param x - The input X component.
     * @return The next X component of the translation.
     */
    public double getNextX(double x) {
        this.lastX = this.xFilter.calculate(x);
        return this.lastX;
    }

    /**
     * Calculates the next Y component of the translation.
     * @param y - The input Y component.
     * @return The next Y component of the translation.
     */
    public double getNextY(double y) {
        this.lastY = this.yFilter.calculate(y);
        return this.lastY;
    }

    /**
     * Gets the Translation2d of the last X and last Y components.
     * @return The translation.
     */
    public Translation2d getLastTranslation() {
        return new Translation2d(this.lastX, this.lastY);
    }

    /**
     * Calculates the next translation.
     * @param translation - The input translation.
     * @return The next translation.
     */
    public Translation2d getNextTranslation(Translation2d translation) {
        return new Translation2d(
            getNextX(translation.getX()),
            getNextY(translation.getY())
        );
    }

    @Override
    public String toString() {
        return String.format("FilteredTranslation(LastX : %.2f, LastY : %.2f)", this.lastX, this.lastY);
    }
}