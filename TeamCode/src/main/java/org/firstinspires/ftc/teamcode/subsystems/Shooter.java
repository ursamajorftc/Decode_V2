package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class Shooter {
    double distanceFromTarget;
    double compensatedDistance;
    DcMotorEx flywheelMotorRight;
    DcMotorEx flywheelMotorLeft;
    Servo pitchServo;
    Follower follower;
    Ballistics ballistics;
    TelemetryDebug telemetryDebug;
    private double lastPosition = 0;
    private double lastTime = 0;
    private double filteredRPM = 0;
    Servo light;
    Limelight3A limelight;

    LynxModule hub;

    final Pose REDTARGET = new Pose(143.0, 137.0);
    final Pose BLUETARGET = new Pose (143.0, 6.0);

    // TODO: Tune this!
    final PIDFController flywheelPIDF  = new PIDFController(0.002, 0.00005, 0.000003, 0.00022);
    private double[] errorBuffer = new double[20]; // Stores last 20 errors
    private int bufferIndex = 0; // Tracks where we are in the array
    private double rollingErrorAverage = 0;

    boolean isRed;
    private boolean hasAccelerated = false;

    /**
     *
     * @param hardwareMap Used to retrieve hardware from configuration file in driver hub
     * @param follower Used to determine distance to target using odometry
     * @param isRed Set per alliance color
     */
    public Shooter (HardwareMap hardwareMap, Follower follower, boolean isRed, TelemetryDebug debug) {
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
        this.follower = follower;
        this.isRed = isRed;

        // Initialize Ballistics (fills the LUTs)
        this.ballistics = new Ballistics();
        this.telemetryDebug = debug;

        hub = hardwareMap.getAll(LynxModule.class).get(0);

        lastTime = System.nanoTime() / 1E9;
        light.setPosition(0);
    }
    public void start () {
        light.setPosition(0.29);
    }

    /**
     * Called every loop to calculate distance from target
     */
    public void update () {
//        double targetX = (isRed ? REDTARGET.getX() : BLUETARGET.getX());
//        double targetY = (isRed ? REDTARGET.getY() : BLUETARGET.getY());
//
//        double robotX = follower.getPose().getX();
//        double robotY = follower.getPose().getY();
//
//        telemetryDebug.createWatcher("Robot X", robotX);
//        telemetryDebug.createWatcher("Robot Y", robotY);
//
//        double dx = targetX - robotX;
//        double dy = targetY - robotY;
//
//        if (distanceFromTarget >= 0.0) {
//            Vector robotVelocityVector = follower.getVelocity();
//            double vx = robotVelocityVector.getXComponent(); // Forward velocity
//            double vy = robotVelocityVector.getYComponent(); // Strafe velocity
//
//            // 2. Rotate to Field-Centric Velocity
//            // Rotation matrix: x' = x cos θ - y sin θ, y' = x sin θ + y cos θ
//            double robotHeading = follower.getHeading();
//            double vFieldX = vx * Math.cos(robotHeading) - vy * Math.sin(robotHeading);
//            double vFieldY = vx * Math.sin(robotHeading) + vy * Math.cos(robotHeading);
//            Vector fieldVelocityVector = new Vector(Math.hypot(vFieldX, vFieldY), Math.atan2(vFieldY, vFieldX));
//
//            Vector vectorToTarget = new Vector(new Pose(
//                    dx / distanceFromTarget,
//                    dy / distanceFromTarget));
//
//            double velocityToTarget = fieldVelocityVector.dot(vectorToTarget);
//
//            telemetryDebug.createWatcher("Velocity to Target", velocityToTarget);
//
//            // TODO: Tune the coefficient
//            double compensationCoefficient = 0.0;
//            compensatedDistance = distanceFromTarget + (compensationCoefficient * velocityToTarget);
//        } else {
//            compensatedDistance = distanceFromTarget;
//        }

        if((flywheelPIDF.getSetPoint() != 0 && Math.abs(flywheelPIDF.getPositionError()) < 100)) {
            light.setPosition(0.555);
        } else {
            light.setPosition(0.29);
        }

    }

    /**
     * Accelerates flywheel using Velocity PID based on distance from target
     */
    public void accelerateFlywheel () {
        if (!limelight.isRunning()) {
            limelight.start();
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            distanceFromTarget = 87.78147 * (Math.pow(result.getTa(), -0.41445));
        }

        double targetX = (isRed ? REDTARGET.getX() : BLUETARGET.getX());
        double targetY = (isRed ? REDTARGET.getY() : BLUETARGET.getY());

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        telemetryDebug.createWatcher("Robot X", robotX);
        telemetryDebug.createWatcher("Robot Y", robotY);

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        if (distanceFromTarget >= 0.0) {
            Vector robotVelocityVector = follower.getVelocity();
            double vx = robotVelocityVector.getXComponent(); // Forward velocity
            double vy = robotVelocityVector.getYComponent(); // Strafe velocity

            // 2. Rotate to Field-Centric Velocity
            // Rotation matrix: x' = x cos θ - y sin θ, y' = x sin θ + y cos θ
            double robotHeading = follower.getHeading();
            double vFieldX = vx * Math.cos(robotHeading) - vy * Math.sin(robotHeading);
            double vFieldY = vx * Math.sin(robotHeading) + vy * Math.cos(robotHeading);
            Vector fieldVelocityVector = new Vector(Math.hypot(vFieldX, vFieldY), Math.atan2(vFieldY, vFieldX));

            Vector vectorToTarget = new Vector(new Pose(
                    dx / distanceFromTarget,
                    dy / distanceFromTarget));

            double velocityToTarget = fieldVelocityVector.dot(vectorToTarget);

            telemetryDebug.createWatcher("Velocity to Target", velocityToTarget);

            // TODO: Tune the coefficient
            double compensationCoefficient = 0.5;
            compensatedDistance = distanceFromTarget + (compensationCoefficient * velocityToTarget);
        } else {
            compensatedDistance = distanceFromTarget;
        }

        telemetryDebug.createWatcher("Compensated Distance", compensatedDistance);



        if (!hasAccelerated) {hasAccelerated = true;}



        // Calculate power based on velocity error
        // TODO: Revert back to interpolated values once tuned
        double currentRPM = getFlywheelRPM();
        // y=-2083.85427+1072.08529 * ln(x)
//        double targetRPM = -2278 + (1125 * Math.log(compensatedDistance));
        double targetRPM = 9.8 * compensatedDistance+1795;
        double power = flywheelPIDF.calculate(currentRPM, targetRPM);

        flywheelMotorRight.setPower(power);
        flywheelMotorLeft.setPower(power);
//        flywheelMotorRight.setPower(1);
//        flywheelMotorLeft.setPower(1);
        telemetryDebug.createWatcher("Flywheel Power: ", power);
        telemetryDebug.createWatcher("Error", flywheelPIDF.getPositionError());
        updateErrorAverage(flywheelPIDF.getPositionError());
//        pitchServo.setPosition(ballistics.calculatePitch(compensatedDistance)); //0 highest, 1 lowest
        double targetPitch = Math.max(0.0, Math.min(0.86, -0.00468917*compensatedDistance+0.87));
        pitchServo.setPosition(targetPitch);
        // 0.86 is the bottom max

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

    /**
     * Stops flywheel
     */
    public void idle () {
//        flywheelMotorRight.setPower(0);
//        flywheelMotorLeft.setPower(0);
        double currentRPM = getFlywheelRPM(false);
//        if (currentRPM >=0 ) {
//            double power = flywheelPIDF.calculate(currentRPM, 0);
//            flywheelMotorRight.setPower(hasAccelerated ? power : 0);
//            flywheelMotorLeft.setPower(hasAccelerated ? power : 0);
//        } else {
//            flywheelMotorRight.setPower(0);
//            flywheelMotorLeft.setPower(0);
//        }

        if (Math.abs(currentRPM) < 20) {
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
    public void stop () {
        limelight.pause();
        flywheelMotorRight.setPower(0);
        flywheelMotorLeft.setPower(0);
        light.setPosition(0.29);
    }

    public void backSpin (double power) {
        flywheelMotorRight.setPower(-Math.abs(power));
        flywheelMotorLeft.setPower(-Math.abs(power));
    }

    /**
     * Interpolated Look-up Table Class
     */
    public static class Ballistics {
        InterpLUT flywheelLut = new InterpLUT();
        InterpLUT pitchLut = new InterpLUT();

        // Tunable offsets
        public double flywheelSpeedCorrection = 0;
        public double pitchCorrection = 0;

        public Ballistics () {
            // Any request from LUT requires key to be in range
            flywheelLut.add(0, 2035);   // Safety defaults
            pitchLut.add(0, 0.86);

            // TODO: Add real data points here

            flywheelLut.add(65.7, 2410);
            flywheelLut.add(70.6, 2500);
            flywheelLut.add(85.6, 2600);
            flywheelLut.add(106.8, 3000);
            //flywheelLut.add(142.54, 3380);
            flywheelLut.add(147.2, 3270);
            flywheelLut.add(164.8, 3370);
           // flywheelLut.add(159.2, 3400);
            flywheelLut.add(300, 5000);

            pitchLut.add(65.7, 0.7);
            pitchLut.add(70.6,0.7);
            pitchLut.add(85.6, 0.65);
           // pitchLut.add(142.54, 0.5);
            pitchLut.add(147.2, 0.34);
          //  pitchLut.add(159.2, 0.5);
            pitchLut.add(164.8, 0.25);
            pitchLut.add(300, 0);



            flywheelLut.createLUT();
            pitchLut.createLUT();
        }

        /**
         *
         * @param distance Distance from target
         * @return Flywheel Speed in Ticks per second
         */
        public double calculateFlywheelSpeed(double distance) {
            return flywheelLut.get(distance) + flywheelSpeedCorrection;
        }

        /**
         *
         * @param distance Distance from target
         * @return Pitch servo position
         */
        public double calculatePitch (double distance) {
            return pitchLut.get(distance) + pitchCorrection;
        }

    }
    public void increaseHeight() { ballistics.pitchCorrection += 0.02; }
    public void decreaseHeight() { ballistics.pitchCorrection -= 0.02; }
    public void adjustHeight(double pitchCorrection) { ballistics.pitchCorrection += pitchCorrection; }
    public void increaseFlywheelSpeed() { ballistics.flywheelSpeedCorrection += 0.1; }
    public void decreaseFlywheelSpeed() { ballistics.flywheelSpeedCorrection -= 0.1; }
    public void adjustFlywheelSpeed(double flywheelSpeedCorrection) { ballistics.flywheelSpeedCorrection += flywheelSpeedCorrection; }
    public void resetLuts() {
        ballistics.pitchCorrection = 0;
        ballistics.flywheelSpeedCorrection = 0;
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

    public double getDistanceFromTarget () { return distanceFromTarget; }
    public double getFlywheelPower() { return flywheelMotorRight.getPower(); }
    public double getPitch () { return pitchServo.getPosition(); }
    public double getPosition () {return flywheelMotorRight.getCurrentPosition();}
    public void zeroPitchServo() { pitchServo.setPosition(0.0); }
    public void maxPitchServo() {pitchServo.setPosition(1);}
    public double getVoltage () {return hub.getInputVoltage(VoltageUnit.VOLTS);}
    public double toRpm (double ticksPerSecond) {
        final double TICKS_PER_REVOLUTION = 8192.0;
        final double SECONDS_PER_MINUTE = 60.0;

        return (ticksPerSecond / TICKS_PER_REVOLUTION) * SECONDS_PER_MINUTE;
    }

    public boolean isPitchServoThere(){
        if (pitchServo != null){
            return true;
        }else {
            return false;
        }
    }
}