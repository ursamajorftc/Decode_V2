package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utilities.Datavault;
import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Red Far", group = "Competition Autos")
public class redNineBallFar extends OpMode {
    public Follower follower; // Pedro Pathing follower instance
    boolean hasShot = false;
    boolean hasBackspun = false;
    private Timer pathTimer, opmodeTimer;
    private int pathState = 1; // Current autonomous path state (state machine)
    private redNineBallFar.Paths paths; // Paths defined in the Paths class
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private TelemetryDebug telemetryDebug;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(95.659, 8.000, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetryDebug = new TelemetryDebug(telemetry);
        turret = new Turret(hardwareMap, follower, true, telemetryDebug);
        shooter = new Shooter(hardwareMap, true, telemetryDebug);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        shooter.update();
        turret.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        telemetry.addData("Turret Position", turret.getTurretPosition());
        telemetry.addData("Distance From Target", shooter.getDistanceFromTarget());
        telemetry.addData("Relative Target Angle", turret.getRelativeTargetHeading());
        telemetry.addData("Pitch Servo Position", shooter.getPitch());
        telemetry.addData("Current RPM", shooter.getFlywheelRPM());
        telemetry.addData("Current Position", shooter.getPosition());
        for (TelemetryDebug.watcher w : telemetryDebug.watchers) {
            telemetry.addData(w.getName(), w.getValue());
        }
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
        turret.resetEncoder();
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    pulseThreeBalls(pathTimer, true, 149);
                } else {
                    turret.aimWithoutOdometry(-1);
                    shooter.accelerate(149);
                }
                if (hasShot) {
                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(paths.Path1);
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy()) {
                    intake.intake();
                    setPathState(4);
                }
                break;
            case 4:
                follower.followPath(paths.Path2);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(55);
                    intake.intake();
                }
                break;
            case 55:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(575);
                    intake.stop();
                } else {
                    intake.intake(0.8);
                }
                break;
            case 575:
                backSpin(pathTimer, 0.2);
                if (hasBackspun) {
                    setPathState(6);
                }
                break;
            case 6:
                shooter.accelerate(151.5);
                follower.followPath(paths.Path3);
                setPathState(7);
                break;
            case 7:
                shooter.accelerate(151.5);
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                shooter.accelerate(151.5);

                // Always aim while in this state to be ready
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    turret.aimWithoutOdometry(-1);
                }

                // Start the intake sequence after 2.5s
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    pulseThreeBalls(pathTimer, false, 151.5);
                }

                if (hasShot) {
                    setPathState(9);
                }
                break;
            case 9:
                follower.followPath(paths.Path4);
                setPathState(10);
                break;
            case 10:
                if (!follower.isBusy()){
                    intake.intake();
                    follower.followPath(paths.Path5);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 2.5 && follower.atPose(new Pose(141.334, 9.443), 1, 2)) {
                    setPathState(115);
                    intake.stop();
                } else {
                    intake.intake();
                }

                break;
            case 115:
                backSpin(pathTimer, 0.2);
                if (hasBackspun) {
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    shooter.accelerate(151.5);
                    follower.followPath(paths.Path6);
                    setPathState(13);
                }
                break;
            case 13:
                shooter.accelerate(151.5);
                turret.stop();
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                shooter.accelerate(151.5);

                // Always aim while in this state to be ready
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    turret.aimWithoutOdometry(-1);
                }

                // Start the intake sequence after 2.5s
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    pulseThreeBalls(pathTimer, false, 151.5);
                }

                if (hasShot) {
                    setPathState(15);
                }
                break;
            case 15:
                follower.followPath(paths.Path7);
                setPathState(16);
                break;
            case 16:
                if (!follower.isBusy()){
                    saveData();
                    setPathState(-1);
                    requestOpModeStop();
                }
        }

        return pathState;
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
        hasShot = false;
        hasBackspun = false;
    }

    public void shootThreeBalls(Timer pathTimer, boolean longShoot) {
        double time = pathTimer.getElapsedTimeSeconds();
        double duration = longShoot ? 4.0 : 2.5;
        double intakeStartTime = longShoot ? 3.0 : 0.75;

        if (time < duration) {
            // Keep these running for the entire duration
            turret.aimWithoutOdometry(-1);
            shooter.accelerate();

            // Only intake when the flywheel is (presumably) ready
            if (time >= intakeStartTime && shooter.isReady()) {
                intake.intake();
            } else {
                intake.stop();
            }
        } else {
            // Action is FINALLY done
            intake.stop();
            shooter.stop();
            hasShot = true;
        }

    }

    public void shootThreeBalls(Timer pathTimer, boolean longShoot, double distance) {
        double sequenceTime = pathTimer.getElapsedTimeSeconds();
        double duration = longShoot ? 5.75 : 4;
        double intakeStartTime = longShoot ? 3.0 : 0.75;

        if (sequenceTime < duration) {
            // Keep these running for the entire duration
            turret.aimWithoutOdometry(-1);
            shooter.accelerate(distance);

            // Only intake when the flywheel is (presumably) ready
            if (sequenceTime >= intakeStartTime) {
                intake.intake();
            } else {
                intake.stop();
            }
        } else {
            // Action is FINALLY done
            intake.stop();
            shooter.stop();
            hasShot = true;
        }

    }

    public void pulseThreeBalls(Timer pathTimer, boolean longShoot, double distance) {
        double sequenceTime = pathTimer.getElapsedTimeSeconds();
        double duration = longShoot ? 5.5 : 3.25;
        if (sequenceTime < duration) {
            turret.aimWithoutOdometry();
            shooter.accelerate(distance);
        } else {
            turret.stop();
            shooter.stop();
            hasShot = true;
        }
        double accelerationTime = (longShoot ? 3.0 : 0.75);

        if (sequenceTime >= accelerationTime && sequenceTime < accelerationTime+0.5) {
            intake.intake();
        } else if (sequenceTime >= accelerationTime+0.8 && sequenceTime < accelerationTime+1.3) {
            intake.intake();
        } else if (sequenceTime >= accelerationTime + 1.6 && sequenceTime < accelerationTime + 2.4) {
            intake.intake();
        } else {
            intake.stop();
        }
    }

    public void backSpin(Timer pathTimer, double duration) {
        double sequenceTime = pathTimer.getElapsedTimeSeconds();
        if (sequenceTime <= duration) {
            shooter.backSpin(1);
            intake.backSpin(0.7);
        } else {
            intake.stop();
            shooter.stop();
            hasBackspun = true;
        }
    }

    @Configurable


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.659, 8.000),

                                    new Pose(101.089, 37)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(101.089, 37),

                                    new Pose(129.875, 37)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.875, 37),

                                    new Pose(95.472, 8.131)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.472, 8.131),

                                    new Pose(141.593, 35.285)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(141.593, 35.285),

                                    new Pose(141.334, 9.443)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(270))
                    .setBrakingStrength(2)
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(141.334, 9.443),
                                    new Pose(117.452, 31.497),
                                    new Pose(95.738, 8.200)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))

                    .build();
            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.738, 8.200),

                                    new Pose(95.800, 20.262)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();

        }
    }


    public void saveData() {
        Datavault.finalPose = follower.getPose();
        Datavault.turretPosition = turret.getTurretPosition();
    }

    @Override
    public void stop() {
        saveData();
    }

}

