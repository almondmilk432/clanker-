package org.firstinspires.ftc.teamcode.auto;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.brakeL;
import org.firstinspires.ftc.teamcode.subsystems.brakeR;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake;
import org.firstinspires.ftc.teamcode.subsystems.shootadj;
import org.firstinspires.ftc.teamcode.subsystems.stopper;

import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "redClose12C", group = "robot")
public class redClose12C extends NextFTCOpMode {

    public redClose12C () {
        addComponents(
                new SubsystemComponent(outtake.INSTANCE, intake.INSTANCE, brakeL.INSTANCE, brakeR.INSTANCE,
                        shootadj.INSTANCE, stopper.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean pathStarted = false;
    public static Pose redEndC = new Pose();

    private final Pose startPose   = new Pose(122, 122, Math.toRadians(36));
    private final Pose scorePose   = new Pose(96, 95, Math.toRadians(40));
    private final Pose goPPG       = new Pose(96, 88, Math.toRadians(0));
    private final Pose PPGc        = new Pose(80, 100);
    private final Pose gPPG        = new Pose(122, 88, Math.toRadians(0));
    private final Pose openG = new Pose(122,76, Math.toRadians(0));
    private final Pose openGc = new Pose(130, 45);
    private final Pose goPGP       = new Pose(90, 65, Math.toRadians(0));
    private final Pose gPGP        = new Pose(129, 65, Math.toRadians(0));
    private final Pose returnPGP   = new Pose(95, 60);
    private final Pose goGPP       = new Pose(90, 42, Math.toRadians(0));
    private final Pose gGPP        = new Pose(129, 42, Math.toRadians(0));
    private final Pose leave       = new Pose(95, 80, Math.toRadians(40));

    private PathChain scorePreload, gotoPPG, grabPPG, scorePPG;
    private PathChain gotoPGP, grabPGP, scorePGP;
    private PathChain gotoGPP, grabGPP, scoreGPP, leaveBase, openGate, reverse, returnOpen, reverse2;

    public enum Pathstate {
        scorepreload, wait, closeGate, gotoPPG, grabPPG, scorePPG, wait1, closeGate1,
        gotoPGP, grabPGP, scorePGP, wait2, closeGate2,
        gotoGPP, grabGPP, scoreGPP, wait3, closeGate3,
        leaveBase, stop, openGate, reverse, returnOpen, scoreG, waitG, reverse2
    }

    Pathstate pathState;

    @Override
    public void onInit() {

        brakeL.INSTANCE.up.schedule();
        brakeR.INSTANCE.up.schedule();
        stopper.INSTANCE.stop.schedule();
        shootadj.INSTANCE.lowL().schedule();

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathState = Pathstate.scorepreload;
        intake.INSTANCE.In().schedule();
    }

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        gotoPPG = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, PPGc, goPPG))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goPPG.getHeading())
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(goPPG, gPPG))
                .setLinearHeadingInterpolation(goPPG.getHeading(), gPPG.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(goPGP, openG))
                .setLinearHeadingInterpolation(goPGP.getHeading(), openG.getHeading())
                .build();

        reverse = follower.pathBuilder()
                .addPath(new BezierLine(gPGP, goPGP))
                .setLinearHeadingInterpolation(gPGP.getHeading(), goPGP.getHeading())
                .build();

        reverse2 = follower.pathBuilder()
                .addPath(new BezierLine(openG, goPGP))
                .setLinearHeadingInterpolation(openG.getHeading(), goPGP.getHeading())
                .build();

        returnOpen = follower.pathBuilder()
                .addPath(new BezierLine(goPGP, scorePose))
                .setLinearHeadingInterpolation(goPGP.getHeading(), scorePose.getHeading())
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(gPPG, scorePose))
                .setLinearHeadingInterpolation(gPPG.getHeading(), scorePose.getHeading())
                .build();


        gotoPGP = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goPGP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goPGP.getHeading())
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(goPGP, gPGP))
                .setLinearHeadingInterpolation(goPGP.getHeading(), gPGP.getHeading())
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierCurve(openG, returnPGP, scorePose))
                .setLinearHeadingInterpolation(openG.getHeading(), scorePose.getHeading())
                .build();

        gotoGPP = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goGPP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goGPP.getHeading())
                .build();

        grabGPP = follower.pathBuilder()
                .addPath(new BezierLine(goGPP, gGPP))
                .setLinearHeadingInterpolation(goGPP.getHeading(), gGPP.getHeading())
                .build();

        scoreGPP = follower.pathBuilder()
                .addPath(new BezierLine(gGPP, scorePose))
                .setLinearHeadingInterpolation(gGPP.getHeading(), scorePose.getHeading())
                .build();

        leaveBase = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leave))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leave.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        if (pathState == null) return;

        switch (pathState) {

            case scorepreload:
                if (!pathStarted) {
                    outtake.INSTANCE.supers().schedule();
                    actionTimer.resetTimer();

                    follower.followPath(scorePreload);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.wait;
                }
                break;

            case wait:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    stopper.INSTANCE.go.schedule();
                    actionTimer.resetTimer();
                    pathState = Pathstate.closeGate;
                }
                break;

            case closeGate:
                if (actionTimer.getElapsedTimeSeconds() >= 1.25) {
                    stopper.INSTANCE.stop.schedule();
                    outtake.INSTANCE.Stop().schedule();
                    pathStarted = false;
                    pathState = Pathstate.gotoPGP;
                }
                break;

            case gotoPGP:
                if (!pathStarted) {
                    follower.followPath(gotoPGP);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.grabPGP;
                }
                break;

            case grabPGP:
                if (!pathStarted) {
                    follower.followPath(grabPGP, 1, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.reverse;
                }
                break;

            case reverse:
                if (!pathStarted) {
                    follower.followPath(reverse);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.openGate;
                }
                break;

            case openGate:
                if (!pathStarted) {
                    follower.followPath(openGate);
                    actionTimer.resetTimer();
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.waitG;
                }
                break;

            case waitG:
                if (actionTimer.getElapsedTimeSeconds() >= 2) {
                    actionTimer.resetTimer();
                    pathState = Pathstate.reverse2;
                }
                break;

            case reverse2:
                if (!pathStarted) {
                    follower.followPath(reverse2);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.returnOpen;
                }

            case returnOpen:
                if (!pathStarted) {
                    outtake.INSTANCE.supers().schedule();
                    actionTimer.resetTimer();

                    follower.followPath(returnOpen);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.wait1;
                }
                break;


            case wait1:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    stopper.INSTANCE.go.schedule();
                    actionTimer.resetTimer();
                    pathState = Pathstate.closeGate1;
                }
                break;

            case closeGate1:
                if (actionTimer.getElapsedTimeSeconds() >= 1.25) {
                    stopper.INSTANCE.stop.schedule();
                    outtake.INSTANCE.Stop().schedule();
                    pathStarted = false;
                    pathState = Pathstate.gotoPPG;
                }
                break;


            case gotoPPG:
                if (!pathStarted) {
                    follower.followPath(gotoPPG);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.grabPPG;
                }
                break;


            case grabPPG:
                if (!pathStarted) {
                    follower.followPath(grabPPG, 1, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.scorePPG;
                }
                break;

            case scorePPG:
                if (!pathStarted) {
                    outtake.INSTANCE.supers().schedule();
                    actionTimer.resetTimer();

                    follower.followPath(scorePPG);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.wait2;
                }
                break;

            case wait2:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    stopper.INSTANCE.go.schedule();
                    actionTimer.resetTimer();
                    pathState = Pathstate.closeGate2;
                }
                break;

            case closeGate2:
                if (actionTimer.getElapsedTimeSeconds() >= 1.25) {
                    stopper.INSTANCE.stop.schedule();
                    outtake.INSTANCE.Stop().schedule();
                    pathStarted = false;
                    pathState = Pathstate.gotoGPP;
                }
                break;

            case gotoGPP:
                if (!pathStarted) {
                    follower.followPath(gotoGPP);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.grabGPP;
                }
                break;

            case grabGPP:
                if (!pathStarted) {
                    follower.followPath(grabGPP, 1, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.scoreGPP;
                }
                break;

            case scoreGPP:
                if (!pathStarted) {
                    outtake.INSTANCE.supers().schedule();
                    actionTimer.resetTimer();

                    follower.followPath(scoreGPP);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.wait3;
                }
                break;

            case wait3:
                if (actionTimer.getElapsedTimeSeconds() >= 0.75) {
                    stopper.INSTANCE.go.schedule();
                    actionTimer.resetTimer();
                    pathState = Pathstate.closeGate3;
                }
                break;

            case closeGate3:
                if (actionTimer.getElapsedTimeSeconds() >= 1.25) {
                    stopper.INSTANCE.stop.schedule();
                    outtake.INSTANCE.Stop().schedule();
                    pathStarted = false;
                    pathState = Pathstate.leaveBase;
                }
                break;

            case leaveBase:
                if (!pathStarted) {
                    follower.followPath(leaveBase);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.stop;
                }
                break;

            case stop:
                break;
        }
    }

    @Override
    public void onUpdate() {
        super.onUpdate();
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void onStop(){

        follower.update();
        redClose12C.redEndC = follower.getPose();

        intake.INSTANCE.Stop().schedule();
        outtake.INSTANCE.Stop().schedule();
    }
}
