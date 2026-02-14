package org.firstinspires.ftc.teamcode.teleOps;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "poo", group = "Competition TeleOps")
public class testTeleOp extends OpMode {
    private DcMotorEx flywheelRight;
    private DcMotorEx flywheelLeft;
    private Servo pitch;

    // Memory for the filter/velocity calculation
    private double lastPosition = 0;
    private double lastTime = 0;
    private double filteredRPM = 0;

    private PIDFController pid = new PIDFController(0.0001, 0.0, 0.00001, 0.000225225225);

    @Override
    public void init() {
        flywheelRight = hardwareMap.get(DcMotorEx.class, "rightFlywheelMotor");
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "leftFlywheelMotor");
        pitch = hardwareMap.get(Servo.class, "pitchServo");

        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize lastTime so the first 'dt' isn't huge
        lastTime = System.nanoTime() / 1E9;
    }

    @Override
    public void loop() {
        flywheelLeft.setPower(1.0);
        flywheelRight.setPower(1.0);

        // This method now handles updating its own "current" values
        double manualRPM = getFlywheelSpeed();

        telemetry.addData("Position", flywheelRight.getCurrentPosition());
        telemetry.addData("Calculated RPM", manualRPM);
        telemetry.addData("Encoder RPM", toRpm(flywheelRight.getVelocity()));
        telemetry.addData("PID Error", pid.getPositionError());
        telemetry.update();
    }

    public double getFlywheelSpeed() {
        // 1. Refresh current values EVERY loop
        double currentPosition = flywheelRight.getCurrentPosition();
        double currentTime = System.nanoTime() / 1E9;

        // 2. Calculate change in time
        double dt = currentTime - lastTime;

        // Safety check: if the loop is too fast, don't divide by zero
        if (dt < 0.0001) return filteredRPM;

        // 3. Calculate velocity
        double deltaTicks = currentPosition - lastPosition;
        double ticksPerSecond = deltaTicks / dt;
        double rawRPM = (ticksPerSecond / 8192.0) * 60.0;

        // 4. Low Pass Filter (Smooths the jitter)
        filteredRPM = (0.2 * rawRPM) + (0.8 * filteredRPM);

        // 5. Save current values as "last" values for the NEXT loop
        lastPosition = currentPosition;
        lastTime = currentTime;

        return Math.abs(filteredRPM);
    }
    public double toRpm (double ticksPerSecond) {
        final double TICKS_PER_REVOLUTION = 8192.0;
        final double SECONDS_PER_MINUTE = 60.0;

        return (ticksPerSecond / TICKS_PER_REVOLUTION) * SECONDS_PER_MINUTE;
    }
}
