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
        follower.setStartingPose(new Pose(66.85901639344263, 80.47213114754099, Math.PI));

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
                    -gamepad1.right_stick_x, // Turn
                    true // TRUE = Robot Centric
            );
        }

        if (gamepad2.a) {
            intake.intake();
        } else {
            intake.stop();
            if (gamepad2.right_stick_y > 0) {
                shooter.backSpin(gamepad2.right_stick_y);
                intake.backSpin(gamepad2.right_stick_y);
            }
        }
        if (gamepad2.b) {
            intake.transfer();
        }

//        if (gamepad1.dpad_right){
//            shooter.maxPitchServo();
//        }else if (gamepad1.dpad_left){
//            shooter.zeroPitchServo();
//        }

        boolean currentTriggerState = gamepad2.right_trigger > 0.1;
        if (currentTriggerState && !previousTriggerState) {
            intake.transfer();
        }
        previousTriggerState = currentTriggerState;

        if (gamepad2.right_bumper) {
            turret.aim();
            shooter.accelerateFlywheel();
//            intake.openGates();
        } else {
            turret.idle();
            shooter.idle();
//            intake.idle();
            turret.adjustTurret(gamepad2.left_stick_x);
        }

//        if (gamepad1.left_bumper) { shooter.zeroPitchServo(); }

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
        if (controller2.wasJustPressed(GamepadKeys.Button.X)) {
            shooter.resetLuts();
        }

//        if(gamepad1.y){
//            telemetryDebug.createWatcher("Test Watcher: ", "Nirav is a poopy idiot");
//        }

        telemetry.addData("Turret Position", turret.getTurretPosition());
        telemetry.addData("Distance From Target", shooter.getDistanceFromTarget());
        telemetry.addData("Relative Target Angle", turret.getRelativeTargetHeading());
        telemetry.addData("Pitch Servo Position", shooter.getPitch());
        telemetry.addData("Voltage", shooter.getVoltage());
        telemetry.addData("Currnet RPM", shooter.getFlywheelRPM() );
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
