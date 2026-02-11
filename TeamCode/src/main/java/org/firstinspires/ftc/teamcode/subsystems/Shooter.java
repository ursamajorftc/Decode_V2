package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
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

    LynxModule hub;

    final Pose REDTARGET = new Pose(137.0, 143.0);
    final Pose BLUETARGET = new Pose (6.0, 143.0);

    // TODO: Tune this!
    final PIDFController flywheelPIDF  = new PIDFController(0.07, 0.0, 0.0, 0.05);

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


        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        this.follower = follower;
        this.isRed = isRed;

        // Initialize Ballistics (fills the LUTs)
        this.ballistics = new Ballistics();

        hub = hardwareMap.getAll(LynxModule.class).get(0);
    }

    /**
     * Called every loop to calculate distance from target
     */
    public void update () {
        double targetX = (isRed ? REDTARGET.getX() : BLUETARGET.getX());
        double targetY = (isRed ? REDTARGET.getY() : BLUETARGET.getY());

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        // Standard Euclidean distance calculation
        distanceFromTarget = Math.hypot(dx, dy);

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

            // TODO: Tune the coefficient
            double compensationCoefficient = 0.0;
            compensatedDistance = distanceFromTarget + (compensationCoefficient * velocityToTarget);
        } else {
            compensatedDistance = distanceFromTarget;
        }

    }

    /**
     * Accelerates flywheel using Velocity PID based on distance from target
     */
    public void accelerateFlywheel () {
        // Retrieves Targets from Ballistic Class
//        double targetVel = ballistics.calculateFlywheelSpeed(compensatedDistance);
//        double targetPitch = ballistics.calculatePitch(compensatedDistance);

        // Calculate power based on velocity error
        // TODO: Revert back to interpolated values once tuned
        double power = flywheelPIDF.calculate(flywheelMotorRight.getVelocity(), ((0.00294117647059*(compensatedDistance)) + 5.078529412)  *  (13/ hub.getInputVoltage(VoltageUnit.VOLTS))); // 16.666 is max
        flywheelMotorRight.setPower(power);
        flywheelMotorLeft.setPower(power);

//        pitchServo.setPosition(ballistics.calculatePitch(compensatedDistance)); //0 highest, 1 lowest
        pitchServo.setPosition(0.043);
    }

    /**
     * Stops flywheel
     */
    public void idle () {
        flywheelMotorRight.setPower(0);
        flywheelMotorLeft.setPower(0);
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
            flywheelLut.add(0, 4.7);   // Safety defaults
            pitchLut.add(0, 0.042);

            // TODO: Add real data points here


            flywheelLut.add(1, 4.8);
            flywheelLut.add(60.2189384516,5);
            flywheelLut.add(71.2380348421,5);
            flywheelLut.add(91.8430034076,5);
            flywheelLut.add(100.883309198,5.1);
            flywheelLut.add(102.592238687,5.3);
            flywheelLut.add(116.89333451,5.3);
            flywheelLut.add(126.3, 5.45); //good
            flywheelLut.add(141.704101355,5.34);
            flywheelLut.add(143.3, 5.5); // good
            flywheelLut.add(200, 5.6);
            flywheelLut.add(300, 5.8);

            pitchLut.add(1, 0.042);
            pitchLut.add(60.2189384516,0.045);
            pitchLut.add(71.2380348421,0.045);
            pitchLut.add(91.8430034076,0.04);
            pitchLut.add(100.883309198,0.043);
            pitchLut.add(102.592238687,0.044);
            pitchLut.add(116.89333451,0.043);
            pitchLut.add(126.3, 0.044); // good
            pitchLut.add(127.1, 0.043);
            pitchLut.add(141.704101355,0.044);
            pitchLut.add(143.3, 0.044);//good
            pitchLut.add(200, 0.045);
            pitchLut.add(300, 0.047);



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
    public double getVoltage () {return hub.getInputVoltage(VoltageUnit.VOLTS);}

    public boolean isPitchServoThere(){
        if (pitchServo != null){
            return true;
        }else {
            return false;
        }
    }
}