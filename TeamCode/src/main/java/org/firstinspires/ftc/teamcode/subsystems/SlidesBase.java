package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.control.FeedforwardController;
import com.aimrobotics.aimlib.control.LowPassFilter;
import com.aimrobotics.aimlib.control.PIDController;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.aimrobotics.aimlib.control.SimpleControlSystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class SlidesBase extends Mechanism {

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    private DcMotorEx activeEncoderMotor;
    private double lastActiveEncoderExtension;

    private final String leftSlideName;
    private final String rightSlideName;

    private final DcMotorSimple.Direction leftMotorDirection;
    private final DcMotorSimple.Direction rightMotorDirection;

    protected double activeTargetExtension = 0;
    private double manualPower = 0;

    private final SimpleControlSystem controlSystem;

    private static final double PROXIMITY_THRESHOLD = 0.25;
    private static final double CURRENT_THRESHOLD = 5000;
    private static final double MINIMUM_POWER = 0.03;

    private static final double TICKS_PER_INCH = 3; // TODO CALCULATE THIS


    public enum SlidesControlState {
        AUTONOMOUS, MANUAL
    }
    protected SlidesControlState activeControlState = SlidesControlState.AUTONOMOUS;

    /**
     * Constructor for the slides base
     * @param leftSlideName the name of the left slide motor
     * @param rightSlideName the name of the right slide motor
     * @param leftMotorDirection the direction of the left motor
     * @param rightMotorDirection the direction of the right motor
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param derivativeLowPassGain the derivative low pass gain
     * @param integralSumMax the maximum integral sum
     * @param kV the velocity feedforward gain
     * @param kA  the acceleration feedforward gain
     * @param kStatic the static feedforward gain
     * @param kCos the cosine feedforward gain (Only this or kG should be used)
     * @param kG the gravity feedforward gain (Only this or kCos should be used)
     * @param lowPassGain the low pass gain
     */
    public SlidesBase(String leftSlideName, String rightSlideName, DcMotorSimple.Direction leftMotorDirection, DcMotorSimple.Direction rightMotorDirection, double kP, double kI, double kD, double derivativeLowPassGain, double integralSumMax, double kV, double kA, double kStatic, double kCos, double kG, double lowPassGain) {
        this.leftSlideName = leftSlideName;
        this.rightSlideName = rightSlideName;
        this.leftMotorDirection = leftMotorDirection;
        this.rightMotorDirection = rightMotorDirection;
        PIDController pidController = new PIDController(kP, kI, kD, derivativeLowPassGain, integralSumMax);
        FeedforwardController feedforwardController = new FeedforwardController(kV, kA, kStatic, kCos, kG);
        LowPassFilter lowPassFilter = new LowPassFilter(lowPassGain);
        controlSystem = new SimpleControlSystem(pidController, feedforwardController, lowPassFilter);
    }

    /**
     * Initialize the slides base
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(DcMotorEx.class, leftSlideName);
        rightSlide = hwMap.get(DcMotorEx.class, rightSlideName);
        setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftSlide.setDirection(leftMotorDirection);
        rightSlide.setDirection(rightMotorDirection);
        activeEncoderMotor = leftSlide;
        lastActiveEncoderExtension = 0;
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        switch (activeControlState){
            case AUTONOMOUS:
                update();
                break;
            case MANUAL:
                applyManualPower();
                break;
        }
    }

    /**
     * Set the mode of the slides
     * @param mode the mode to set the slides to
     */
    public void setMode(DcMotorEx.RunMode mode) {
        leftSlide.setMode(mode);
        rightSlide.setMode(mode);
    }

    /**
     * Set the zero power behavior of the slides
     * @param behavior the zero power behavior to set the slides to
     */
    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        leftSlide.setZeroPowerBehavior(behavior);
        rightSlide.setZeroPowerBehavior(behavior);
    }

    /**
     * Set the active control state of the slides
     * @param activeControlState the active control state of the slides
     */
    public void setActiveControlState(SlidesControlState activeControlState) {
        this.activeControlState = activeControlState;
    }


    /**
     * Apply a preset manual power to the slides. If the power is below the minimum power threshold, hold the position.
     * Use with updateManualPower() to set the manual power
     */
    private void applyManualPower() {
        if (Math.abs(manualPower) > MINIMUM_POWER && !currentSpikeDetected()) {
            setPower(manualPower);
        } else {
            holdPosition();
        }
    }

    /**
     * Update the manual power of the slides
     * @param power the power to set the slides to
     */
    public void updateManualPower(double power) {
        manualPower = power;
    }

    /**
     * Set the power of the slides and update the last position
     * Only updates the last position when the slides are running vs. every loop
     * Slides will likely be running every loop
     * @param power the power to set the slides to
     */
    private void setPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        updateLastExtension();
    }

    /**
     * Get the output power of the slides based on the system update for the active target extension
     * @return the output power for the slide motors
     */
    private double getTargetOutputPower() {
        return controlSystem.update(getCurrentExtension());
    }

    /**
     * Update the power of the slides based on the control system's output
     */
    private void update() {
        double power = getTargetOutputPower();
        setPower(power);
    }

    /**
     * Hold the position of the slides
     */
    private void holdPosition() {
        setTargetExtension(getLastExtension());
        update();
    }

    /**
     * Set the target extension for the slides in inches
     * @param targetExtension the target position for the slides in inches
     */
    public void setTargetExtension(double targetExtension) {
        activeTargetExtension = targetExtension;
        controlSystem.setTarget(activeTargetExtension);
    }

    /**
     * Check if the slides are at the target position
     * @return true if the slides are within the proximity threshold of the target position
     */
    public boolean isAtTargetExtension() {
        return Math.abs(getCurrentExtension() - activeTargetExtension) < PROXIMITY_THRESHOLD;
    }

    /**
     * Check if the slides are over the current threshold
     * @return true if the slides are over the current threshold
     */
    public boolean currentSpikeDetected() {
        return activeEncoderMotor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD;
    }

    /**
     * Get the current position of the slides
     * @return the current position of the slides
     */
    public double getCurrentPosition() {
        return activeEncoderMotor.getCurrentPosition();
    }

    public double getCurrentExtension() {
        return ticksToInches(getCurrentPosition());
    }

    /**
     * Set the last active encoder extension to the encoder motor's current extension in inches
     */
    private void updateLastExtension() {
        lastActiveEncoderExtension = ticksToInches(getCurrentPosition());
    }

    /**
     * Get the last active encoder extension in inches
     * @return the last active encoder extension in inches
     */
    private double getLastExtension() {
        return lastActiveEncoderExtension;
    }



    private double inchesToTicks(double degrees) {
        return degrees * TICKS_PER_INCH;
    }

    private double ticksToInches(double ticks) {
        return ticks * (1/TICKS_PER_INCH);
    }
}