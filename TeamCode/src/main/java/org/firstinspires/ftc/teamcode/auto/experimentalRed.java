package org.firstinspires.ftc.teamcode.auto;

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

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Autonomous(name = "experimentalRed", group = "robot")
public class experimentalRed extends NextFTCOpMode {

    public experimentalRed () {
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
    private final Pose goPGP = new Pose(100, 57, Math.toRadians(0));
    private final Pose PGPc        = new Pose(68, 70);
    private final Pose gPGP       = new Pose(126, 88, Math.toRadians(0));
    private final Pose openGate = new Pose(133, 60, Math.toRadians(30));
    private final Pose openGatec = new Pose(115, 60);
    private final Pose returnGatec   = new Pose(95, 55);
    private final Pose goPPG       = new Pose(133, 60, Math.toRadians(0));
    private final Pose PPGc = new Pose(115, 60);
    private final Pose gPPG        = new Pose(127, 83, Math.toRadians(0));
    private final Pose goGPP       = new Pose(100, 35, Math.toRadians(0));
    private final Pose gGPP        = new Pose(133, 35, Math.toRadians(0));
    private final Pose leave       = new Pose(85, 105, Math.toRadians(40));

    private PathChain scorePreload, go1, grab1, score1;
    private PathChain OpenGate, scoreGate, go2, grab2, score2;
    private PathChain go3, grab3, score3, leaveBase;

    public enum Pathstate {
        scorepreload, wait, closeGate, go1, grab1, score1, wait1, closeGate1,
        OpenGate, scoregate, waitGate, closeGateGate, go2, grab2, score2, wait2, closeGate2,
        go3, grab3, score3, wait3, closeGate3,
        leaveBase, stop
    }

    Pathstate pathState;

    @Override
    public void onInit() {

        brakeL.INSTANCE.up.schedule();
        brakeR.INSTANCE.up.schedule();
        stopper.INSTANCE.stop.schedule();
        shootadj.INSTANCE.lowL().schedule();
        shootadj.INSTANCE.lowR().schedule();

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("good luck ;)");
        telemetry.addLine("₍^. .^₎⟆");
        telemetry.update();
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

        go1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, PGPc, goPGP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goPGP.getHeading())
                .build();

        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(goPGP, gPGP))
                .setLinearHeadingInterpolation(goPGP.getHeading(), gPGP.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(gPGP, scorePose))
                .setLinearHeadingInterpolation(gPGP.getHeading(), scorePose.getHeading())
                .build();

        OpenGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, openGatec, openGate))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGate.getHeading())
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(openGate, returnGatec, scorePose))
                .setLinearHeadingInterpolation(openGate.getHeading(), scorePose.getHeading())
                .build();

        go2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, PPGc, goPPG))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goPPG.getHeading())
                .build();

        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(goPPG, gPPG))
                .setLinearHeadingInterpolation(goPPG.getHeading(), gPPG.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(gPPG, scorePose))
                .setLinearHeadingInterpolation(gPGP.getHeading(), scorePose.getHeading())
                .build();

        go3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goGPP))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goGPP.getHeading())
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(goGPP, gGPP))
                .setLinearHeadingInterpolation(goGPP.getHeading(), gGPP.getHeading())
                .build();

        score3 = follower.pathBuilder()
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
                if (actionTimer.getElapsedTimeSeconds() >= 0) {
                    stopper.INSTANCE.go.schedule();
                    actionTimer.resetTimer();
                    pathState = Pathstate.closeGate;
                }
                break;

            case closeGate:
                if (actionTimer.getElapsedTimeSeconds() >= 1) {
                    stopper.INSTANCE.stop.schedule();
                    outtake.INSTANCE.Stop().schedule();
                    pathStarted = false;
                    pathState = Pathstate.go1;
                }
                break;

            case go1:
                if (!pathStarted) {
                    follower.followPath(go1);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.grab1;
                }
                break;

            case grab1:
                if (!pathStarted) {
                    follower.followPath(grab1, 1, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.score1;
                }
                break;

            case score1:
                if (!pathStarted) {
                    outtake.INSTANCE.Outs().schedule();
                    actionTimer.resetTimer();

                    follower.followPath(score1);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.wait1;
                }
                break;

            case wait1:
                if (actionTimer.getElapsedTimeSeconds() >= 0) {
                    stopper.INSTANCE.go.schedule();
                    actionTimer.resetTimer();
                    pathState = Pathstate.closeGate1;
                }
                break;

            case closeGate1:
                if (actionTimer.getElapsedTimeSeconds() >= 1) {
                    stopper.INSTANCE.stop.schedule();
                    outtake.INSTANCE.Stop().schedule();
                    pathStarted = false;
                    pathState = Pathstate.OpenGate;
                }
                break;

            case OpenGate:
                if (!pathStarted){
                    follower.followPath(OpenGate);
                    pathStarted = true;
                } else if (!follower.isBusy()){
                    pathStarted = false;
                    pathState = Pathstate.scoregate;
                }
                break;

            case scoregate:
                if (!pathStarted){
                    outtake.INSTANCE.Outs().schedule();
                    actionTimer.resetTimer();

                    follower.followPath(scoreGate);
                    pathStarted = true;
                } else if (!follower.isBusy()){
                    pathStarted = false;
                    pathState = Pathstate.waitGate;
                }
                break;

            case waitGate:
                if (actionTimer.getElapsedTimeSeconds() >= 0) {
                stopper.INSTANCE.go.schedule();
                actionTimer.resetTimer();
                pathState = Pathstate.closeGateGate;
                }
                break;

            case closeGateGate:
                if (actionTimer.getElapsedTimeSeconds() >= 1) {
                    stopper.INSTANCE.stop.schedule();
                    outtake.INSTANCE.Stop().schedule();
                    pathStarted = false;
                    pathState = Pathstate.go2;
                }
                break;

            case go2:
                if (!pathStarted) {
                    follower.followPath(go2);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.grab2;
                }
                break;

            case grab2:
                if (!pathStarted) {
                    follower.followPath(grab2, 1, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.score2;
                }
                break;

            case score2:
                if (!pathStarted) {
                    outtake.INSTANCE.Outs().schedule();
                    actionTimer.resetTimer();

                    follower.followPath(score2);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.wait2;
                }
                break;

            case wait2:
                if (actionTimer.getElapsedTimeSeconds() >= 0) {
                    stopper.INSTANCE.go.schedule();
                    actionTimer.resetTimer();
                    pathState = Pathstate.closeGate2;
                }
                break;

            case closeGate2:
                if (actionTimer.getElapsedTimeSeconds() >= 1) {
                    stopper.INSTANCE.stop.schedule();
                    outtake.INSTANCE.Stop().schedule();
                    pathStarted = false;
                    pathState = Pathstate.go3;
                }
                break;

            case go3:
                if (!pathStarted) {
                    follower.followPath(go3);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.grab3;
                }
                break;

            case grab3:
                if (!pathStarted) {
                    follower.followPath(grab3, 1, true);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.score3;
                }
                break;

            case score3:
                if (!pathStarted) {
                    outtake.INSTANCE.Outs().schedule();
                    actionTimer.resetTimer();

                    follower.followPath(score3);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = Pathstate.wait3;
                }
                break;

            case wait3:
                if (actionTimer.getElapsedTimeSeconds() >= 0) {
                    stopper.INSTANCE.go.schedule();
                    actionTimer.resetTimer();
                    pathState = Pathstate.closeGate3;
                }
                break;

            case closeGate3:
                if (actionTimer.getElapsedTimeSeconds() >= 1) {
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

