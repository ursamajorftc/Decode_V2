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
    Ballistics ballistics; // Instance variable, not static class usage

    final Pose REDTARGET = new Pose(137.0, 143.0);
    final Pose BLUETARGET = new Pose (6.0, 143.0);

    // F is usually critical for flywheels. Set F to 1.0 / MaxVelocity initially
    final PIDFController flywheelPIDF  = new PIDFController(0.0, 0.0, 0.0, 1.0/2800.0);

    boolean isRed;

    public Shooter (HardwareMap hardwareMap, Follower follower, boolean isRed) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Required for custom PID

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        this.follower = follower;
        this.isRed = isRed;

        // Initialize Ballistics (fills the LUTs)
        this.ballistics = new Ballistics();
    }

    public void update () {
        Pose currentPosition = follower.getPose();
        Pose target = isRed ? REDTARGET : BLUETARGET;
        // Standard Euclidean distance calculation
        distanceFromTarget = Math.hypot(target.getX() - currentPosition.getX(), target.getY() - currentPosition.getY());
    }

    public void accelerateFlywheel () {
        double targetVel = ballistics.calculateFlywheelSpeed(distanceFromTarget);
        double targetPitch = ballistics.calculatePitch(distanceFromTarget);

        // Calculate power based on velocity error
        double power = flywheelPIDF.calculate(flywheelMotor.getVelocity(), targetVel);

        flywheelMotor.setPower(power);
        pitchServo.setPosition(targetPitch);
    }

    public void idle () {
        flywheelMotor.setPower(0);
    }
    public static class Ballistics {
        InterpLUT flywheelLut = new InterpLUT();
        InterpLUT pitchLut = new InterpLUT();

        // Tunable offsets
        public double flywheelSpeedCorrection = 0;
        public double pitchCorrection = 0;

        public Ballistics () {
            // FORMAT: .add(Distance, Value)
            flywheelLut.add(0, 1000);   // Safety defaults
            pitchLut.add(0, 0.5);

            // TODO: Add real data points here
            flywheelLut.add(55, 1500);
            pitchLut.add(55, 0.45);

            flywheelLut.createLUT();
            pitchLut.createLUT();
        }

        public double calculateFlywheelSpeed(double distance) {
            return flywheelLut.get(distance) + flywheelSpeedCorrection;
        }

        public double calculatePitch (double distance) {
            return pitchLut.get(distance) + pitchCorrection;
        }
    }
}