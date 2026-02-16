package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    // Hardware Declarations
    DcMotorEx flywheelMotorRight;
    DcMotorEx flywheelMotorLeft;
    Servo pitchServo;
    TelemetryDebug telemetryDebug;
    Servo light;
    Limelight3A limelight;

    // Calculation Variables
    double distanceFromTarget;
    double compensatedDistance;
    private double lastPosition = 0;
    private double lastTime = 0;
    private double lastDistance = 0.0;
    private long lastTimeV = 0;
    private double filteredVelocity = 0.0;
    private double filteredRPM = 0;
    private double filteredDistance = 0;
    private double compensationCoefficient = 0.3; // TODO: Tune this!!
    final PIDFController flywheelPIDF = new PIDFController(0.002, 0.00005, 0.000003, 0.00022);
    private double[] errorBuffer = new double[20]; // Stores last 20 errors
    private int bufferIndex = 0; // Tracks where we are in the array
    private double rollingErrorAverage = 0;
    boolean isRed;
    private boolean hasAccelerated = false;

    /**
     *
     * @param hardwareMap Used to retrieve hardware from configuration file in driver hub
     * @param isRed       Set per alliance color
     */
    public Shooter(HardwareMap hardwareMap, boolean isRed, TelemetryDebug debug) {
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "rightFlywheelMotor");
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "leftFlywheelMotor");
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        light = hardwareMap.get(Servo.class, "led");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        this.isRed = isRed;

        this.telemetryDebug = debug;

        lastTime = System.nanoTime() / 1E9;
        light.setPosition(0);
    }

    public void start() {
        light.setPosition(0.29);
    }

    /**
     * Called every loop to calculate distance from target
     */
    public void update() {
        light.setPosition(isAccelerated() ? 0.57 : 0.29);
    }

    /**
     * Accelerates flywheel using Velocity PID based on distance from target
     */
    public void accelerate() {
        if (!limelight.isRunning()) { limelight.start(); }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double rawDistance = 87.78147 * (Math.pow(result.getTa(), -0.41445));
            filteredDistance = filteredDistance == 0 ? rawDistance : (0.2 * rawDistance) + (0.8 * filteredDistance);
            if (filteredDistance != 0 && lastDistance == 0) {
                lastDistance = filteredDistance;
            }
            compensatedDistance = filteredDistance + (getVelocityToTarget(filteredDistance) * compensationCoefficient);
            distanceFromTarget = filteredDistance;
        } else {
            compensatedDistance = filteredDistance;
            lastTimeV = 0;
        }

        telemetryDebug.createWatcher("Compensated Distance", compensatedDistance);


        if (!hasAccelerated) {
            hasAccelerated = true;
        }

        // Calculate power based on velocity error
        double currentRPM = getFlywheelRPM();
        double targetRPM = 9.8 * compensatedDistance + 1795;
        double power = flywheelPIDF.calculate(currentRPM, targetRPM);

        flywheelMotorRight.setPower(power);
        flywheelMotorLeft.setPower(power);

        telemetryDebug.createWatcher("Flywheel Power: ", power);
        telemetryDebug.createWatcher("Error", flywheelPIDF.getPositionError());

        updateErrorAverage(flywheelPIDF.getPositionError());
        double targetPitch = Math.max(0.0, Math.min(0.86, -0.00468917 * compensatedDistance + 0.87));
        pitchServo.setPosition(targetPitch); // 0.86 is the bottom max
    }

    public void updateErrorAverage(double currentError) {
        // 1. Overwrite the oldest value with the newest one
        errorBuffer[bufferIndex] = Math.abs(currentError);

        // 2. Move the index forward, wrapping around to 0 if we hit 20
        bufferIndex = (bufferIndex + 1) % errorBuffer.length;

        // 3. Calculate the new average of the array
        double sum = 0;
        for (double error : errorBuffer) {
            sum += error;
        }
        rollingErrorAverage = sum / errorBuffer.length;
        telemetryDebug.createWatcher("Average Error", rollingErrorAverage);
    }

    public boolean isAccelerated() {
        return flywheelPIDF.getSetPoint() != 0 && Math.abs(flywheelPIDF.getPositionError()) < 100;
    }

    /**
     * Stops flywheel
     */
    public void idle() {
        double currentRPM = getFlywheelRPM(false);

        if (currentRPM < 50) {
            flywheelMotorRight.setPower(0);
            flywheelMotorLeft.setPower(0);
        } else {
            double power = flywheelPIDF.calculate(currentRPM, 0);
            flywheelMotorRight.setPower(hasAccelerated ? power : 0);
            flywheelMotorLeft.setPower(hasAccelerated ? power : 0);
        }

        light.setPosition(0.29);
        limelight.pause();
    }

    public void stop() {
        limelight.pause();
        flywheelMotorRight.setPower(0);
        flywheelMotorLeft.setPower(0);
        light.setPosition(0.29);
    }

    public void backSpin(double power) {
        flywheelMotorRight.setPower(-Math.abs(power));
        flywheelMotorLeft.setPower(-Math.abs(power));
    }

    public double getFlywheelRPM() {
        // 1. Refresh current values EVERY loop
        double currentPosition = flywheelMotorRight.getCurrentPosition();
        double currentTime = System.nanoTime() / 1E9;

        // 2. Calculate change in time
        double dt = currentTime - lastTime;

        // Safety check: if the loop is too fast, don't divide by zero
        if (dt < 0.0001) return filteredRPM;

        // 3. Calculate velocity
        double deltaTicks = currentPosition - lastPosition;
        double ticksPerSecond = deltaTicks / dt;
        double rawRPM = (ticksPerSecond / 8192.0) * 60.0;

        // 4. Low Pass Filter (Smooths the jitter)
        filteredRPM = (0.2 * rawRPM) + (0.8 * filteredRPM);

        // 5. Save current values as "last" values for the NEXT loop
        lastPosition = currentPosition;
        lastTime = currentTime;

        return Math.abs(filteredRPM);
    }

    public double getFlywheelRPM(boolean absVal) {
        // 1. Refresh current values EVERY loop
        double currentPosition = flywheelMotorRight.getCurrentPosition();
        double currentTime = System.nanoTime() / 1E9;

        // 2. Calculate change in time
        double dt = currentTime - lastTime;

        // Safety check: if the loop is too fast, don't divide by zero
        if (dt < 0.0001) return filteredRPM;

        // 3. Calculate velocity
        double deltaTicks = currentPosition - lastPosition;
        double ticksPerSecond = deltaTicks / dt;
        double rawRPM = (ticksPerSecond / 8192.0) * 60.0;

        // 4. Low Pass Filter (Smooths the jitter)
        filteredRPM = (0.2 * rawRPM) + (0.8 * filteredRPM);

        // 5. Save current values as "last" values for the NEXT loop
        lastPosition = currentPosition;
        lastTime = currentTime;

        if (absVal) {
            return Math.abs(filteredRPM);
        } else {
            return filteredRPM;
        }
    }


    public double getVelocityToTarget(double distanceFromTarget) {
        double alpha = 0.2; // lower = smoother, more lag
        long now = System.nanoTime();

        if (lastTimeV == 0) {
            lastTimeV = now;
            lastDistance = distanceFromTarget;
            return 0.0;
        }

        double dt = (now - lastTimeV) * 1e-9; // seconds
        if (dt <= 0) return filteredVelocity;

        double rawVelocity = (distanceFromTarget - lastDistance) / dt;

        // low-pass filter
        filteredVelocity = alpha * rawVelocity + (1 - alpha) * filteredVelocity;

        lastDistance = distanceFromTarget;
        lastTimeV = now;

        return filteredVelocity;
    }

    public double getDistanceFromTarget() {
        return distanceFromTarget;
    }

    public double getFlywheelPower() {
        return flywheelMotorRight.getPower();
    }

    public double getPitch() {
        return pitchServo.getPosition();
    }

    public double getPosition() {
        return flywheelMotorRight.getCurrentPosition();
    }
}