package org.firstinspires.ftc.teamcode.teleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.Turret;

import java.util.List;

@TeleOp (name = "Blue TeleOp", group = "Competition TeleOps")
public class BlueTeleOp extends OpMode {
    private Intake intake;
    private Turret turret;
    private Shooter shooter;
    private Follower follower;
    private GamepadEx controller2;
    private TelemetryManager telemetry;

    private boolean previousTriggerState = false;

    @Override
    public void init() {
        // Set Bulk Reading to Auto
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap, follower, false);
        shooter = new Shooter(hardwareMap, follower, false);

        controller2 = new GamepadEx(gamepad2);

        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override
    public void loop() {
        follower.update();
        intake.update();
        turret.update();
        shooter.update();
        controller2.readButtons();
        telemetry.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_x, // Forward/Back
                -gamepad1.left_stick_y, // Strafe
                -gamepad1.right_stick_x, // Turn
                true // TRUE = Robot Centric
        );

        if (gamepad2.a) {
            intake.intake();
        } else {
            intake.stop();
        }

        boolean currentTriggerState = gamepad2.right_trigger > 0.1;
        if (currentTriggerState && !previousTriggerState) {
            intake.transfer();
        }
        previousTriggerState = currentTriggerState;

        if (gamepad2.right_bumper) {
            turret.aim();
            shooter.accelerateFlywheel();
            intake.openGates();
        } else {
            turret.idle();
            shooter.idle();
            intake.idle();
        }

        if (gamepad1.left_bumper) { shooter.zeroPitchServo(); }

        // Uses FTCLib gamepad methods for edge detection
        if (controller2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            shooter.decreaseFlywheelSpeed();
        }
        if (controller2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            shooter.increaseFlywheelSpeed();
        }
        if (controller2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            shooter.increaseHeight();
        }
        if (controller2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            shooter.decreaseHeight();
        }
        if (controller2.wasJustPressed(GamepadKeys.Button.B)) {
            shooter.resetLuts();
        }

        telemetry.debug("Turret Position", turret.getTurretPosition());
        telemetry.debug("Distance From Target", shooter.getDistanceFromTarget());
        telemetry.debug("Relative Target Angle", turret.getRelativeTargetHeading());
        telemetry.debug("Flywheel Power", shooter.getFlywheelPower());
        telemetry.debug("Flywheel Speed", shooter.getFlywheelSpeed());
        telemetry.debug("Pitch Servo Position", shooter.getPitch());
    }

    @Override
    public void stop () {
        turret.stop();
    }
}
