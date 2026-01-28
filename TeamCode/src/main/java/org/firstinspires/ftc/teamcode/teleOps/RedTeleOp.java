package org.firstinspires.ftc.teamcode.teleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.Turret;

import java.util.List;

@TeleOp (name = "Red TeleOp", group = "Competition TeleOps")
public class RedTeleOp extends OpMode {
    private Intake intake;
    private Turret turret;
    private Shooter shooter;
    private Follower follower;
    private GamepadEx driver2Op;

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
        turret = new Turret(hardwareMap, follower, true);
        shooter = new Shooter(hardwareMap, follower, true);

        driver2Op = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
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
        } else {
            turret.idle();
            shooter.idle();
        }

        // Uses FTCLib gamepad methods for edge detection
        if (driver2Op.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            shooter.decreaseFlywheelSpeed();
        }
        if (driver2Op.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            shooter.increaseFlywheelSpeed();
        }
        if (driver2Op.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            shooter.increaseHeight();
        }
        if (driver2Op.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            shooter.decreaseHeight();
        }
        if (driver2Op.wasJustPressed(GamepadKeys.Button.B)) {
            shooter.resetLuts();
        }


        follower.update();
        intake.update();
        turret.update();
        shooter.update();
        driver2Op.readButtons();
    }

    @Override
    public void stop () {
        turret.stop();
    }
}
