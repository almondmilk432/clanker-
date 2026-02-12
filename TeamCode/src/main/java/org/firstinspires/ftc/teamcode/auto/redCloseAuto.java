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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.brakeL;
import org.firstinspires.ftc.teamcode.subsystems.brakeR;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake;
import org.firstinspires.ftc.teamcode.subsystems.shootadj;
import org.firstinspires.ftc.teamcode.subsystems.stopper;

import dev.nextftc.ftc.components.BulkReadComponent;

@Disabled
@Autonomous(name = "redCloseAuto", group = "robot")
public class redCloseAuto extends NextFTCOpMode {

    public redCloseAuto () {
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

    private final Pose startPose   = new Pose(122, 122, Math.toRadians(36));
    private final Pose scorePose   = new Pose(95, 95, Math.toRadians(40));
    private final Pose goPPG       = new Pose(95, 88, Math.toRadians(0));
    private final Pose PPGc        = new Pose(80, 100);
    private final Pose gPPG        = new Pose(120, 88, Math.toRadians(0));
    private final Pose goPGP       = new Pose(90, 65, Math.toRadians(0));
    private final Pose gPGP        = new Pose(126, 65, Math.toRadians(0));
    private final Pose returnPGP   = new Pose(95, 60);
    private final Pose goGPP       = new Pose(90, 39, Math.toRadians(0));
    private final Pose gGPP        = new Pose(126, 39, Math.toRadians(0));
    private final Pose leave       = new Pose(125, 67, Math.toRadians(180));

    private PathChain scorePreload, gotoPPG, grabPPG, scorePPG;
    private PathChain gotoPGP, grabPGP, scorePGP;
    private PathChain gotoGPP, grabGPP, scoreGPP, leaveBase;

    public enum Pathstate {
        scorepreload, wait, closeGate, gotoPPG, grabPPG, scorePPG, wait1, closeGate1,
        gotoPGP, grabPGP, scorePGP, wait2, closeGate2,
        gotoGPP, grabGPP, scoreGPP, wait3, closeGate3,
        leaveBase, stop
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
                .addPath(new BezierCurve(gPGP, returnPGP, scorePose))
                .setLinearHeadingInterpolation(gPGP.getHeading(), scorePose.getHeading())
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
                    outtake.INSTANCE.Outs().schedule();
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
                if (actionTimer.getElapsedTimeSeconds() >= 2) {
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
                    follower.followPath(grabPPG, 0.6, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.scorePPG;
                }
                break;

            case scorePPG:
                if (!pathStarted) {
                    outtake.INSTANCE.Outs().schedule();
                    actionTimer.resetTimer();

                    follower.followPath(scorePPG);
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
                if (actionTimer.getElapsedTimeSeconds() >= 2) {
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
                    follower.followPath(grabPGP, 0.6, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.scorePGP;
                }
                break;

            case scorePGP:
                if (!pathStarted) {
                    outtake.INSTANCE.Outs().schedule();
                    actionTimer.resetTimer();

                    follower.followPath(scorePGP);
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
                if (actionTimer.getElapsedTimeSeconds() >= 2) {
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
                    follower.followPath(grabGPP, 0.5, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.scoreGPP;
                }
                break;

            case scoreGPP:
                if (!pathStarted) {
                    outtake.INSTANCE.Outs().schedule();
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
                if (actionTimer.getElapsedTimeSeconds() >= 2) {
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
        intake.INSTANCE.Stop().schedule();
        outtake.INSTANCE.Stop().schedule();
    }
}
