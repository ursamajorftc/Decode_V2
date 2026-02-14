package org.firstinspires.ftc.teamcode.teleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterE;

@TeleOp (name = "Tuning Velocity PIDF", group = "Competition TeleOps")
public class Tuning extends OpMode {
    private GamepadEx controller2;
    private ShooterE shooterE;

    private TelemetryManager telemetryManager;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;


    @Override
    public void init() {
        shooterE = new ShooterE(hardwareMap);


    }


    @Override
    public void loop() {

        // Change the target velocity with Y button
        if (gamepad1.yWasPressed()){
            if(shooterE.targetRPM == shooterE.highRPM){
                shooterE.targetRPM = shooterE.lowRPM;
            } else {
                shooterE.targetRPM = shooterE.highRPM;
            }

        }

        //change the index we use in setpSize list, going back to start if stepIndex goes above 3
        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }


        //Increase or decrease F or P value for PIDF tuning
        if(gamepad1.dpadLeftWasPressed()){
            shooterE.F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            shooterE.F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            shooterE.P -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()){
            shooterE.P += stepSizes[stepIndex];
        }




        shooterE.updateShooter();

        telemetry.addData("Target RPM", shooterE.targetRPM);
        telemetry.addData("Current RPM", shooterE.getFlywheelRPM());
        telemetry.addData("Error", shooterE.errorRPM);
        telemetry.addData("F Value (dPad Left- and Right+)", shooterE.F);
        telemetry.addData("P Value (dPad Up+ and Down-)", shooterE.P);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addData("Motor Input", shooterE.motorPower);
        telemetry.update();

    }

}