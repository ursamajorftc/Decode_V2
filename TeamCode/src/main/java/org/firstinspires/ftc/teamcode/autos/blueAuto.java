package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryDebug;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Blue Auto", group = "Competition Autos")
@Configurable
public class blueAuto extends OpMode {
    boolean hasShot = false;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opmodeTimer;
    private int pathState = 1; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private LynxModule hub;
    private TelemetryDebug telemetryDebug;
    private double intakePower = 1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(53.351, 8.498, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetryDebug = new TelemetryDebug(telemetry);
        turret = new Turret(hardwareMap, follower, false);
        shooter = new Shooter(hardwareMap, false, telemetryDebug);
        intake = new Intake(hardwareMap);

        hub = hardwareMap.getAll(LynxModule.class).get(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        shooter.update();
        turret.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        double voltage = hub.getInputVoltage(VoltageUnit.VOLTS);
        if (voltage > 12.4) {
            intakePower = 12.4/voltage;
        } else {
            intakePower = 1;
        }

        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
    }

    @Configurable
    private static class Paths {

        public static PathChain Path1;
        public static PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.351, 8.498),
                                    new Pose(53.351,25)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
            case 1:
//                pathTimer.resetTimer();
                shootThreeBalls(pathTimer);
               if (hasShot) {
                    follower.followPath(paths.Path1);
                    setPathState(pathState + 1);
               }
                break;
            case 2:
                if (!follower.isBusy()) {
                    pathState = 0;
                }
                break;
        }

        return pathState;
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
        hasShot = false;
    }
public void shootThreeBalls(Timer pathTimer) {
//    pathTimer.resetTimer();
    double time = pathTimer.getElapsedTimeSeconds();


    // 1. Keep the flywheel and turret active for the WHOLE sequence
    if (time < 6.6) {
        turret.aim();
        shooter.accelerate();
    } else {
        intake.stop();
        shooter.idle();
    }

    // 2. Pulse the intake for the three individual shots
    // Shot 1
    if (time > 3.9 && time < 4.5) {
        intake.intake(intakePower);
    }
    // Gap 1
    else if (time >= 4.5 && time < 5) {
        intake.stop();
    }
    // Shot 2
    else if (time >= 5 && time < 5.5) {
        intake.intake(intakePower);
    }
//    // Gap 2
//    else if (time >= 5.2 && time <= 6.0) {
//        intake.stop();
//
//    }
////     Shot 3
//    else if (time >= 6.0 && time < 6.5) {
//        intake.intake(intakePower);
//    }
    if (time >= 5.6){
        intake.stop();
        shooter.idle();
        turret.idle();
        hasShot = true;
    } else {
        hasShot = false;
    }
}
}
