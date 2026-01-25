package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret {
    DcMotorEx turretMotor;
    Follower follower;
    Limelight3A limelight;

    double turretPosition; // in Radians

    // TODO: TUNE THIS VALUE!
    final double RADIANSPERTICK = 1.0;

    final Pose REDTARGET = new Pose(137.0, 143.0);
    final Pose BLUETARGET = new Pose (6.0, 143.0);

    // Note: Limelight PID input is Degrees, Odometry PID input is Radians.
    // You will need very different P values for these.
    final PIDFController limelightPIDF = new PIDFController(0.03, 0.0, 0.001, 0.0);
    final PIDFController odometryPIDF = new PIDFController(0.8, 0.0, 0.05, 0.0);

    double relativeTargetHeading;
    boolean isRed;

    public Turret (HardwareMap hardwareMap, Follower follower, boolean isRed) {
        this.follower = follower;
        this.isRed = isRed;

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isRed ? 0 : 1);
        limelight.start();
    }

    public void update () {
        // Calculate turret angle relative to the robot chassis
        turretPosition = turretMotor.getCurrentPosition() * RADIANSPERTICK;

        // Get target coordinates
        double targetX = (isRed ? REDTARGET.getX() : BLUETARGET.getX());
        double targetY = (isRed ? REDTARGET.getY() : BLUETARGET.getY());

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getHeading();

        // Calculate the field-centric angle to the target
        double fieldAngleToTarget = Math.atan2(targetY - robotY, targetX - robotX);

        // Convert to robot-centric angle (where the turret needs to point)
        // Angle Wrapping: ensures we calculate the shortest distance (-PI to PI)
        relativeTargetHeading = AngleUnit.normalizeRadians(fieldAngleToTarget - robotHeading);
    }

    public void aim () {
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // Limelight tx is in degrees. Target is 0.
            double output = limelightPIDF.calculate(llResult.getTx(), 0);
            turretMotor.setPower(output);
        } else {
            // Fallback to Odometry
            // We want turretPosition to match relativeTargetHeading
            double output = odometryPIDF.calculate(turretPosition, relativeTargetHeading);
            turretMotor.setPower(output);
        }
    }

    public void idle () {
        // Keep turret centered forward (position 0 relative to robot)
        turretMotor.setPower(odometryPIDF.calculate(turretPosition, 0));
    }

    public void stop () {
        turretMotor.setPower(0);
    }
}