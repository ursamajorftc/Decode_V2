package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    double distanceFromTarget;
    DcMotorEx flywheelMotorRight;
    DcMotorEx flywheelMotorLeft;
    Servo pitchServo;
    Follower follower;
    Ballistics ballistics;

    final Pose REDTARGET = new Pose(137.0, 143.0);
    final Pose BLUETARGET = new Pose (6.0, 143.0);

    // TODO: Tune this!
    final PIDFController flywheelPIDF  = new PIDFController(0.01, 0.0, 0.0, 0.05);

    boolean isRed;

    /**
     *
     * @param hardwareMap Used to retrieve hardware from configuration file in driver hub
     * @param follower Used to determine distance to target using odometry
     * @param isRed Set per alliance color
     */
    public Shooter (HardwareMap hardwareMap, Follower follower, boolean isRed) {
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "rightFlywheelMotor");
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "leftFlywheelMotor");
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        flywheelMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        this.follower = follower;
        this.isRed = isRed;

        // Initialize Ballistics (fills the LUTs)
        this.ballistics = new Ballistics();
    }

    /**
     * Called every loop to calculate distance from target
     */
    public void update () {
        double targetX = (isRed ? REDTARGET.getX() : BLUETARGET.getX());
        double targetY = (isRed ? REDTARGET.getY() : BLUETARGET.getY());

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        // Standard Euclidean distance calculation
        distanceFromTarget = Math.hypot(targetX-robotX, targetY-robotY);
    }

    /**
     * Accelerates flywheel using Velocity PID based on distance from target
     */
    public void accelerateFlywheel () {
        // Retrieves Targets from Ballistic Class
//        double targetVel = ballistics.calculateFlywheelSpeed(distanceFromTarget);
//        double targetPitch = ballistics.calculatePitch(distanceFromTarget);

        // Calculate power based on velocity error
        // TODO: Revert back to interpolated values once tuned
        double power = flywheelPIDF.calculate(flywheelMotorRight.getVelocity(), 14.7); // 16.666 is max
        flywheelMotorRight.setPower(power);
        flywheelMotorLeft.setPower(power);

        pitchServo.setPosition(0.025); //0 highest, 1 lowest
    }

    /**
     * Stops flywheel
     */
    public void idle () {
        flywheelMotorRight.setPower(0);
        flywheelMotorLeft.setPower(0);
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
            flywheelLut.add(0, 1000);   // Safety defaults
            pitchLut.add(0, 0.5);

            // TODO: Add real data points here
            flywheelLut.add(112.190418486, 13);


            pitchLut.add(112.190418486, 0.1);


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
    public double getDistanceFromTarget () { return distanceFromTarget; }
    public double getFlywheelPower() { return flywheelMotorRight.getPower(); }
    public double getFlywheelSpeed() { return flywheelMotorRight.getVelocity(); }
    public double getPitch () { return pitchServo.getPosition(); }
    public void zeroPitchServo() { pitchServo.setPosition(0.0); }
    public void maxPitchServo() {pitchServo.setPosition(1);}

    public boolean isPitchServoThere(){
        if (pitchServo != null){
            return true;
        }else {
            return false;
        }
    }
}