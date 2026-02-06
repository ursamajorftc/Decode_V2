package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// Servo turret left is control hub port 5
// Everything is in RADIANS
@Configurable
public class Turret {
    CRServo leftTurretServo;
    CRServo rightTurretServo;
    DcMotorEx intakeMotor;
    Follower follower;
    Limelight3A limelight;

    double turretPosition;

    double isLLgetting;

    // TODO: TUNE THIS VALUE!
    public static double RADIANSPERTICK = 1.0;

    final Pose REDTARGET = new Pose(137.0, 143.0);
    final Pose BLUETARGET = new Pose (6.0, 143.0);

    // TODO: Tune these. Expect very different P values!
    final PIDFController limelightPIDF = new PIDFController(0.01, 0.0, 0.001, 0.01);
    final PIDFController odometryPIDF = new PIDFController(0.3, 0.0, 0.003, 0.0);

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
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

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
        turretPosition = AngleUnit.normalizeRadians(intakeMotor.getCurrentPosition() * RADIANSPERTICK);

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
        limelight.start();
        // Retrieve limelight data
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // Limelight tx is in degrees. Target is 0.
            double power = limelightPIDF.calculate(llResult.getTx(), 0);
            //power = normalizePower(power);
//            power = 0;
            isLLgetting = power;
            rightTurretServo.setPower(power);
            leftTurretServo.setPower(power);
        } else {
            // Fallback to Odometry
            // We want turretPosition to match relativeTargetHeading
//            double power = odometryPIDF.calculate(turretPosition, relativeTargetHeading);
//            power = normalizePower(power);

            rightTurretServo.setPower(0);
            leftTurretServo.setPower(0);
        }
    }

    /**
     * Stops turret and limelight when not aiming
     */
    public void idle () {
        // Keep turret centered forward (position 0 relative to robot)
//        double power = normalizePower(odometryPIDF.calculate(turretPosition, 0));
//        rightTurretServo.setPower(power);
//        leftTurretServo.setPower(power);
        rightTurretServo.setPower(0);
        leftTurretServo.setPower(0);
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

    private double normalizePower (double power) {
        if (turretPosition >= Math.PI/2 || turretPosition <= -Math.PI/2) {
            return 0;
        } else {
            return Math.max(-1, Math.min(1, power));
        }
    }
    public double getTurretPosition() { return intakeMotor.getCurrentPosition(); }
    public double getRelativeTargetHeading() { return relativeTargetHeading; }

    public double checkLL(){ return isLLgetting; }

}