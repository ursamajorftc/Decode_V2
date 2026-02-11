package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
public class ShooterE {

    //Objects
    double distanceFromTarget;
    DcMotorEx flywheelMotorRight;
    DcMotorEx flywheelMotorLeft;
    DcMotorEx encoder;
    VoltageSensor batteryVoltage;





    //Constants
    private static final double TICKS_PER_REV = 8192.0;
    private static final double SECONDS_PER_MINUTE = 60.0;
    //PIDF constants
    public double F = 0;  // Volts per RPM error
    public double P = 0;  // Volts per RPM (feedforward)

    //Variables
    public double highRPM = 4000;      // Desired flywheel speed
    public double lowRPM = 900;
    public double targetRPM = highRPM;
    public double currentRPM;        // Measured speed
    public double errorRPM;

    public double motorPower;
    public PIDFController PIDF_MotorInput  = new PIDFController(P, 0.0, 0.0, F);



    // Connects each hardware variable to a port on the Control hub
    public ShooterE (HardwareMap hardwareMap) {
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "rightFlywheelMotor");
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "leftFlywheelMotor");
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
    }


    public void updateShooter() {
        currentRPM = getFlywheelRPM();
        errorRPM = targetRPM - currentRPM;

        PIDF_MotorInput.setP(P);
        PIDF_MotorInput.setF(F);


        // Used to calculate motor power feeding in CurrentRPM and Target RPM, then compensating the voltage for each value
        motorPower = PIDF_MotorInput.calculate(currentRPM, targetRPM) * (12/ batteryVoltage.getVoltage());


        // Apply to motors
        flywheelMotorRight.setPower(motorPower);
        flywheelMotorLeft.setPower(motorPower);
    }



    // Gets the Current Flywheel RPM
    public double getFlywheelRPM() {
        double ticksPerSecond = flywheelMotorRight.getVelocity();
        return (ticksPerSecond / TICKS_PER_REV) * SECONDS_PER_MINUTE;
    }


}
