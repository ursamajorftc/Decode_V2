package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Everything is in RADIANS
public class Turret {
    CRServo leftTurretServo;
    CRServo rightTurretServo;
    DcMotorEx flywheelMotor;
    Follower follower;
    Limelight3A limelight;

    double turretPosition;

    // TODO: TUNE THIS VALUE!
    final double RADIANSPERTICK = 1.0;

    final Pose REDTARGET = new Pose(137.0, 143.0);
    final Pose BLUETARGET = new Pose (6.0, 143.0);

    // TODO: Tune these. Expect very different P values!
    final PIDFController limelightPIDF = new PIDFController(0.03, 0.0, 0.001, 0.0);
    final PIDFController odometryPIDF = new PIDFController(0.8, 0.0, 0.05, 0.0);

    double relativeTargetHeading;
    boolean isRed;

    /**
     *
     * @param hardwareMap Used to retrieve hardware from configuration file in driver hub
     * @param follower Used as fallback to determine distance to target using odometry
     * @param isRed Set per alliance color
     */
    public Turret (HardwareMap hardwareMap, Follower follower, boolean isRed) {
        // Stores follower and alliance color
        this.follower = follower;
        this.isRed = isRed;

        // Declares and sets up turret servos
        leftTurretServo = hardwareMap.get(CRServo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(CRServo.class, "rightTurretServo");

        // We're using flywheel motor's position as positioning for the turret
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotorRight");

        // Declares and sets up limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isRed ? 0 : 1);
        limelight.start();
    }

    /**
     * Calculates relative position of the target using odometry
     */
    public void update () {
        // Calculate turret angle relative to the robot chassis
        turretPosition = flywheelMotor.getCurrentPosition() * RADIANSPERTICK;

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

    /**
     * Aim using limelight or odometry as fallback
     */
    public void aim () {
        // Retrieve limelight data
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // Limelight tx is in degrees. Target is 0.
            double power = limelightPIDF.calculate(llResult.getTx(), 0);
            rightTurretServo.setPower(power);
            leftTurretServo.setPower(power);
        } else {
            // Fallback to Odometry
            // We want turretPosition to match relativeTargetHeading
            double power = odometryPIDF.calculate(turretPosition, relativeTargetHeading);
            rightTurretServo.setPower(power);
            leftTurretServo.setPower(power);
        }
    }

    /**
     * Stops turret and limelight when not aiming
     */
    public void idle () {
        // Keep turret centered forward (position 0 relative to robot)
        rightTurretServo.setPower(odometryPIDF.calculate(turretPosition, 0));
        leftTurretServo.setPower(odometryPIDF.calculate(turretPosition, 0));
        limelight.pause();
    }

    /**
     * Avoid using this, use idle() method instead
     */
    public void stop () {
        rightTurretServo.setPower(0);
        leftTurretServo.setPower(0);
        limelight.stop();
    }
}