package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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

@Autonomous(name = "Blue Twelve Ball Far", group = "Competition Autos")
public class blueTwelveBallFar extends OpMode {
    boolean hasShot = false;
    boolean hasBackspun = false;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opmodeTimer;
    private int pathState = 1; // Current autonomous path state (state machine)
    private blueTwelveBallFar.Paths paths; // Paths defined in the Paths class
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
        follower.setStartingPose(new Pose(21.738, 122.952, Math.toRadians(144)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetryDebug = new TelemetryDebug(telemetry);
        turret = new Turret(hardwareMap, follower, false, telemetryDebug);
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
            intakePower = 12.4 / voltage;
        } else {
            intakePower = 1;
        }

        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
    }

    @Configurable
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path13;
        public PathChain Path12;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.738, 122.952),

                                    new Pose(54.759, 98.814)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.759, 98.814),

                                    new Pose(42.538, 83.517)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.538, 83.517),

                                    new Pose(16.641, 84.234)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.641, 84.234),

                                    new Pose(30.807, 84.179)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(30.807, 84.179),

                                    new Pose(14.152, 75.724)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.152, 75.724),

                                    new Pose(61.090, 81.490)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(145))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(61.090, 81.490),

                                    new Pose(45.938, 59.683)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.938, 59.683),

                                    new Pose(9.745, 60.110)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.745, 60.110),
                                    new Pose(39.652, 56.893),
                                    new Pose(60.083, 80.462)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.083, 80.462),

                                    new Pose(48.166, 35.255)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.166, 35.255),

                                    new Pose(9.034, 36.428)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.034, 36.428),

                                    new Pose(60.324, 81.297)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.324, 81.297),

                                    new Pose(18.276, 69.593)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90))

                    .build();
        }
    }


    //    public int autonomousPathUpdate() {
//        // Add your state machine Here
//        // Access paths with paths.pathName
//        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
//        switch (pathState) {
//            case 0:
//                if (!follower.isBusy()) {
//                    shootThreeBalls(pathTimer, true);
//                    if (hasShot) {
//                        follower.followPath(paths.Path1);
//                        setPathState(pathState + 1);
//                    }
//                }
//                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path2);
//                    setPathState(pathState + 1);
//                }
//                break;
//            case 2:
//                if (!follower.isBusy()) {
//                    intake.intake(0.4);
//                    follower.followPath(paths.Path3);
//                    setPathState(pathState + 1);
//                }
//                break;
//            case 3:
//                if (!follower.isBusy()) {
//                    intake.stop();
//                    follower.followPath(paths.Path4);
//                    setPathState(pathState + 1);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path5);
//                    setPathState(pathState + 1);
//                }
//                break;
//            case 5:
//                shooter.accelerate();
//                turret.aim();
//                if (!follower.isBusy()) {
//                    shootThreeBalls(pathTimer, false);
//                    if (hasShot) {
//                        follower.followPath(paths.Path6);
//                        setPathState(-1);
//                    }
//                }
//        }
//
//        return pathState;
//    }
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1: // 1. DO PATH 1
                follower.followPath(paths.Path1);
                setPathState(2);
                break;

            case 2: // WAIT FOR PATH 1
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3: // 2. SHOOT THREE BALLS
                shootThreeBalls(pathTimer, true); // true = long shoot (3.5s)
                if (hasShot) {
                    setPathState(4);
                }
                break;

            case 4: // 3. DO PATH 2
                follower.followPath(paths.Path2);
                setPathState(5);
                break;

            case 5: // WAIT FOR PATH 2
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6: // 4. INTAKE WHILE DOING PATH 3
                intake.intake(0.4);
                follower.followPath(paths.Path3);
                setPathState(7);
                break;

            case 7: // WAIT FOR PATH 3
                if (!follower.isBusy()) {
                    intake.stop();
                    setPathState(8);
                }
                break;

            case 8: // 5. DO PATH 4
                follower.followPath(paths.Path4);
                setPathState(9);
                break;

            case 9: // WAIT FOR PATH 4
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10: // 6. DO PATH 5
                follower.followPath(paths.Path5);
                setPathState(11);
                break;

            case 11: // WAIT FOR PATH 5
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;

            case 12: // 7. ACCELERATE AND AIM WHILE DOING PATH 6
                shooter.accelerate();
                turret.aim();
                follower.followPath(paths.Path6);
                setPathState(13);
                break;

            case 13: // WAIT FOR PATH 6 (Keep Aiming!)
                shooter.accelerate();
                turret.aim();
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;

            case 14: // 8. SHOOT THREE BALLS
                shootThreeBalls(pathTimer, false); // false = quick shoot (1.0s)
                if (hasShot) {
                    setPathState(-1); // End of Auto
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

    public void shootThreeBalls(Timer pathTimer, boolean accelerateBeforeShot) {
        double time = pathTimer.getElapsedTimeSeconds();

        if (accelerateBeforeShot) {
            // 1. Keep the flywheel and turret active for the WHOLE sequence
            if (time < 3.5) {
                turret.aim();
                shooter.accelerate();
            }

            if (time >= 2.5 && time < 3.5) {
                intake.intake();
            } else {
                intake.stop();
                shooter.stop();
                hasShot = true;
            }
        } else {
            if (time < 1.0) {
                shooter.accelerate();
                turret.aim();
                intake.intake();
            } else {
                intake.stop();
                shooter.stop();
                turret.idle();
                hasShot = true;
            }
        }
    }
//    public void backSpin () {
//        intake.backSpin(0.7);
//        shooter.backSpin(1.0);
//    }
//    public void backSpin (double duration) {
//        Timer sequenceTimer = new Timer();
//        double time = sequenceTimer.getElapsedTimeSeconds();
//
//        if (time < duration) {
//            intake.backSpin(0.7);
//            shooter.backSpin(1.0);
//        } else {
//            intake.stop();
//            shooter.stop();
//        }
//    }
}

