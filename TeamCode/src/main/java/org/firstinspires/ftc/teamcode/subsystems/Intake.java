package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Intake {
    DcMotorEx intakeMotor;
    /**
     * @param hardwareMap Used to retrieve hardware from configuration file in driver hub
     */
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    public void intake () { intakeMotor.setPower(1.0); }
    public void intake (double power) {intakeMotor.setPower(power);}
    public void stop () { intakeMotor.setPower(0); }
    public void backSpin (double power) { intakeMotor.setPower(-Math.abs(power)); }

}
