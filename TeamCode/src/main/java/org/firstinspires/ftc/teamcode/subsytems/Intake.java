package org.firstinspires.ftc.teamcode.subsytems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotorEx intakeMotor;
    DcMotorEx transferMotor;
    Timer transferTimer = new Timer();

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

    /**
     * Will constantly check to see if transfer timer is under a threshold to send ball to shooter.
     * Moves balls across queue after ball is transferred
     */
    public void update () {
        double time = transferTimer.getElapsedTimeSeconds();
        if (time < 1.0) {
            transferMotor.setPower(0.75);
        } else {
            transferMotor.setPower(0);
        }

        if (time > 0.5 && time < 1.0) {
            intakeMotor.setPower(1.0);
        }
    }
}
