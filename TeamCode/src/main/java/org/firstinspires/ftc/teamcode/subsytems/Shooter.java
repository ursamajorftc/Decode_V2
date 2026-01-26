package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    double distanceFromTarget;
    DcMotorEx flywheelMotor;
    Servo pitchServo;
    Follower follower;
    Ballistics ballistics;

    final Pose REDTARGET = new Pose(137.0, 143.0);
    final Pose BLUETARGET = new Pose (6.0, 143.0);

    // TODO: Tune this!
    final PIDFController flywheelPIDF  = new PIDFController(0.0, 0.0, 0.0, 1.0/2800.0);

    boolean isRed;

    /**
     *
     * @param hardwareMap Used to retrieve hardware from configuration file in driver hub
     * @param follower Used to determine distance to target using odometry
     * @param isRed Set per alliance color
     */
    public Shooter (HardwareMap hardwareMap, Follower follower, boolean isRed) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        double targetVel = ballistics.calculateFlywheelSpeed(distanceFromTarget);
        double targetPitch = ballistics.calculatePitch(distanceFromTarget);

        // Calculate power based on velocity error
        double power = flywheelPIDF.calculate(flywheelMotor.getVelocity(), targetVel);
        flywheelMotor.setPower(power);

        pitchServo.setPosition(targetPitch);
    }

    /**
     * Stops flywheel
     */
    public void idle () {
        flywheelMotor.setPower(0);
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
            flywheelLut.add(55, 1500);
            pitchLut.add(55, 0.45);

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
}