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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryDebug;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@TeleOp (name = "Blue TeleOp", group = "Competition TeleOps")
public class BlueTeleOp extends OpMode {
    private Intake intake;
    private Turret turret;
    private Shooter shooter;
    private Follower follower;
    private GamepadEx controller2;
    private TelemetryManager telemetryManager;
    private TelemetryDebug telemetryDebug;

    private boolean previousTriggerState = false;


    @Override
    public void init() {
        // Set Bulk Reading to Auto
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(53.40327868852459, 57.3377049180328, Math.PI));

        for (int i = 0; i < 3; i++) {
            follower.update();
        }
        telemetryDebug = new TelemetryDebug();
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap, follower, false);
        shooter = new Shooter(hardwareMap, follower, false, telemetryDebug);


        controller2 = new GamepadEx(gamepad2);

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();


    }

    public void start() {
        follower.startTeleOpDrive();
        shooter.start();
    }

    @Override
    public void loop() {
        follower.update();
        intake.update();
        turret.update();
        shooter.update();
        controller2.readButtons();
        telemetryManager.update();

        if (follower.getPose() != null) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y, // Forward/Back
                    -gamepad1.left_stick_x, // Strafe
                    -gamepad1.right_stick_x * 0.5, // Turn
                    true // TRUE = Robot Centric
            );
        }

        // --- INTAKE CONTROL ---
        if (gamepad1.left_trigger > 0.1) {
            intake.intake();
        } else if (gamepad1.dpad_down && !gamepad1.right_bumper) {
            // Backspin only happens if we aren't trying to shoot
            intake.backSpin(0.7);
        } else {
            intake.stop();
        }

// --- SHOOTER & TURRET CONTROL ---
        if (gamepad1.right_bumper) {
            // Shooting takes top priority
            turret.aim();
            shooter.accelerateFlywheel();
        } else if (gamepad1.dpad_down) {
            // Backspin happens if we aren't shooting
            shooter.backSpin(1);
            turret.idle();
        } else {
            // SAFETY: If neither button is held, the shooter MUST idle
            shooter.stop();
            turret.idle();
        }

        telemetry.addData("Turret Position", turret.getTurretPosition());
        telemetry.addData("Distance From Target", shooter.getDistanceFromTarget());
        telemetry.addData("Relative Target Angle", turret.getRelativeTargetHeading());
        telemetry.addData("Pitch Servo Position", shooter.getPitch());
        telemetry.addData("Voltage", shooter.getVoltage());
        telemetry.addData("Current RPM", shooter.getFlywheelRPM() );
        telemetry.addData("Current Position", shooter.getPosition());
        for (TelemetryDebug.watcher w : telemetryDebug.watchers){
            telemetry.addData(w.getName(), w.getValue());
        }
    }

    @Override
    public void stop () {
        turret.stop();
    }
}
