package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

// Servo turret left is control hub port 5
// Everything is in RADIANS
@Configurable
public class Turret {
    CRServo leftTurretServo;
    CRServo rightTurretServo;
    DcMotorEx turretEncoder;
    Follower follower;
    Limelight3A limelight;
    TelemetryDebug telemetryDebug;


    double turretPosition;
    double compensation;
    double isLLgetting;

    // TODO: TUNE THIS VALUE!
    public static double RADIANSPERTICK = -0.000227259253725;

    final Pose REDTARGET = new Pose(137.0, 143.0);
    final Pose BLUETARGET = new Pose(6.0, 143.0);

    // TODO: Tune these. Expect very different P values!
    final PIDFController limelightPIDF = new PIDFController(0.015, 0.014, 0.00015, 0.005);
    final PIDFController odometryPIDF = new PIDFController(0.6, 0.2, 0.0, 0.005);

    double relativeTargetHeading;
    boolean isRed;

    /**
     * @param hardwareMap Used to retrieve hardware from configuration file in driver hub
     * @param follower    Used as fallback to determine distance to target using odometry
     * @param isRed       Set per alliance color
     */
    public Turret(HardwareMap hardwareMap, Follower follower, boolean isRed, TelemetryDebug telemetryDebug) {
        // Stores follower and alliance color
        this.follower = follower;
        this.isRed = isRed;

        // Declares and sets up turret servos
        leftTurretServo = hardwareMap.get(CRServo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(CRServo.class, "rightTurretServo");

        // We're using flywheel motor's position as positioning for the turret
        turretEncoder = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Declares and sets up limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isRed ? 0 : 1);
        limelight.start();

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetryDebug = telemetryDebug;
    }

    /**
     * Calculates relative position of the target using odometry
     */
    public void update() {
        // Calculate turret angle relative to the robot chassis
        turretPosition = AngleUnit.normalizeRadians(turretEncoder.getCurrentPosition() * RADIANSPERTICK);

        // Get target coordinates
        double targetX = (isRed ? REDTARGET.getX() : BLUETARGET.getX());
        double targetY = (isRed ? REDTARGET.getY() : BLUETARGET.getY());

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double robotHeading = follower.getHeading();

        // Calculate the field-centric angle to the target
        double fieldAngleToTarget = Math.atan2(dy, dx);

        // Convert to robot-centric angle (where the turret needs to point)
        // Angle Wrapping: ensures we calculate the shortest distance (-PI to PI)
        relativeTargetHeading = AngleUnit.normalizeRadians(fieldAngleToTarget - robotHeading);

        double vx = follower.getVelocity().getXComponent();
        double vy = follower.getVelocity().getYComponent();

        double vFieldX = vx * Math.cos(robotHeading) - vy * Math.sin(robotHeading);
        double vFieldY = vx * Math.sin(robotHeading) + vy * Math.cos(robotHeading);

        double sin = Math.sin(fieldAngleToTarget);
        double cos = Math.cos(fieldAngleToTarget);

        // Perpendicular (sideways) velocity relative to target
        // TODO: Add linearVelocity and angularVelocity to telemetry
        // TODO: Strafe left or right perpendicular to target should have linear velocity change consistently
        // TODO: Spinning in place should mean angular velocity is nonzero, while linear is close to 0
        double linearVelocity = (-sin * vFieldX) + (cos * vFieldY);
        telemetryDebug.createWatcher("Tangential Velocity", linearVelocity);

        // Convert angular velocity to linear velocity aurafully
        double turretRadius = 4.205;
        double angularVelocity = -follower.getAngularVelocity() * turretRadius;
        telemetryDebug.createWatcher("Angular Velocity", angularVelocity);

        // TODO: Tune this
        compensation = (linearVelocity + angularVelocity);
        telemetryDebug.createWatcher("Total Compensation", compensation);

    }

    /**
     * Aim using limelight or odometry as fallback
     */
    public void aim() {
        limelight.start();
        // Retrieve limelight data
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // TODO: Tune this
            double compensationCoefficient = -1.0; //Coefficient is -1.1 left, Coefficient is () Right
            double power = limelightPIDF.calculate(llResult.getTx(), 0 + compensation * compensationCoefficient);
            power = normalizePower(power);
            isLLgetting = power;
            rightTurretServo.setPower(power);
            leftTurretServo.setPower(power);
        } else {
            // Fallback to Odometry
            // We want turretPosition to match relativeTargetHeading
            double targetPosition = AngleUnit.normalizeRadians(relativeTargetHeading);
            double power = odometryPIDF.calculate(turretPosition, targetPosition);
            power = normalizePower(-power);

            rightTurretServo.setPower(power);
            leftTurretServo.setPower(power);
        }
    }

    /**
     * Stops turret and limelight when not aiming
     */
    public void idle() {
        // Keep turret centered forward (position 0 relative to robot)
//        double power = normalizePower(odometryPIDF.calculate(turretPosition, 0));
//        rightTurretServo.setPower(power);
//        leftTurretServo.setPower(power);
        rightTurretServo.setPower(0);
        leftTurretServo.setPower(0);
        limelight.pause();
    }

    public void adjustTurret(double power) {
        rightTurretServo.setPower(power);
        leftTurretServo.setPower(power);
    }

    /**
     * Avoid using this, use idle() method instead
     */
    public void stop() {
        rightTurretServo.setPower(0);
        leftTurretServo.setPower(0);
        limelight.stop();
    }

    private double normalizePower(double power) {
        if (turretPosition >= Math.PI / 2 && power < 0) {
            return 0;
        } else if (turretPosition <= -Math.PI / 4 && power > 0) {
            return 0;
        } else {
            return Math.max(-1, Math.min(1, power));
        }
    }

    public double getTurretPosition() {
        return AngleUnit.normalizeRadians(turretEncoder.getCurrentPosition() * RADIANSPERTICK);
    }

    public void setTurretPosition(double turretPosition) {
        this.turretPosition = turretPosition;
    }

    public double getRelativeTargetHeading() {
        return relativeTargetHeading;
    }
    public void resetEncoder() {
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}