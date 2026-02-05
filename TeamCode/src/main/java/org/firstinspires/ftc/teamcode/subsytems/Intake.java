package org.firstinspires.ftc.teamcode.subsytems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Intake {
    DcMotorEx intakeMotor;
    DcMotorEx transferMotor;
    Timer transferTimer = new Timer();

    Servo rightGateServo;
    Servo leftGateServo;

    public static double rightGateServoOpen = 0.5;
    public static double rightGateServoClosed = 0.4;
    public static double leftGateServoOpen = 0.5;
    public static double leftGateServoClosed = 0.4;

    /**
     * @param hardwareMap Used to retrieve hardware from configuration file in driver hub
     */
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
//        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);
        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightGateServo = hardwareMap.get(Servo.class, "rightGateServo");
        leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");
    }

    public void intake () {
        intakeMotor.setPower(1.0);
    }

    /**
     * Transfers ball to shooter
     */
    public void transfer () {
        transferTimer.resetTimer();
    }

    public void stop () { intakeMotor.setPower(0); }

    public void openGates(){
        rightGateServo.setPosition(rightGateServoOpen);
        leftGateServo.setPosition(leftGateServoOpen);
    }

    public void idle (){
        rightGateServo.setPosition(rightGateServoClosed);
        leftGateServo.setPosition(leftGateServoClosed);
    }

    /**
     * Will constantly check to see if transfer timer is under a threshold to send ball to shooter.
     * Moves balls across queue after ball is transferred
     */
    public void update () {
        double time = transferTimer.getElapsedTimeSeconds();
        // TODO: Tune the amount of time the transfer motor should run
        if (time < 1.0) {
            transferMotor.setPower(0.75);
        } else {
            transferMotor.setPower(0);
        }
    }
}
