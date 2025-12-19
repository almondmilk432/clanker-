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

@Autonomous(name = "blueFarAuto", group = "robot")
public class blueFarAuto extends NextFTCOpMode {


    public blueFarAuto () {
        addComponents(
                new SubsystemComponent(outtake.INSTANCE, intake.INSTANCE, brakeL.INSTANCE, brakeR.INSTANCE,
                        shootadj.INSTANCE, stopper.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private void runPath(PathChain path, Pathstate nextState) {
        if (!pathStarted) {
            follower.followPath(path);
            pathStarted = true;
        } else if (!follower.isBusy()) {
            pathStarted = false;
            pathState = nextState;
        }
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean pathStarted = false;



    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(60, 19, Math.toRadians(111));
    private final Pose scorePose2 = new Pose(60, 19, Math.toRadians(118));
    private final Pose scorePose3 = new Pose(60, 19, Math.toRadians(110));
    private final Pose goPPG = new Pose(60, 38, Math.toRadians(180));
    private final Pose PPGc = new Pose(62, 36);
    private final Pose gPPG = new Pose(20, 38, Math.toRadians(180));
    private final Pose goPGP = new Pose(60, 60, Math.toRadians(180));
    private final Pose PGPc = new Pose(67, 59);
    private final Pose gPGP = new Pose(20, 60, Math.toRadians(180));
    private final Pose goGPP = new Pose(55, 87, Math.toRadians(180));
    private final Pose GPPc = new Pose(67, 83);
    private final Pose gGPP = new Pose(25, 87, Math.toRadians(180));
    private final Pose returnGPP = new Pose(53, 63);
    private final Pose leave = new Pose(38, 32, Math.toRadians(0));

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
        shootadj.INSTANCE.up().schedule();

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
                .setVelocityConstraint(0.005)
                .build();

        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(goPPG, gPPG))
                .setLinearHeadingInterpolation(goPPG.getHeading(), gPPG.getHeading())
                .setVelocityConstraint(0.005)
                .build();

        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(gPPG, scorePose2))
                .setLinearHeadingInterpolation(gPPG.getHeading(), scorePose2.getHeading())
                .build();

        gotoPGP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2, PGPc, goPGP))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), goPGP.getHeading())
                .setVelocityConstraint(0.1)
                .build();

        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(goPGP, gPGP))
                .setLinearHeadingInterpolation(goPGP.getHeading(), gPGP.getHeading())
                .setVelocityConstraint(0.05)
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(gPGP, scorePose2))
                .setLinearHeadingInterpolation(gPGP.getHeading(), scorePose2.getHeading())
                .build();

        gotoGPP = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2, GPPc, goGPP))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), goGPP.getHeading())
                .setVelocityConstraint(0.1)
                .build();

        grabGPP = follower.pathBuilder()
                .addPath(new BezierLine(goGPP, gGPP))
                .setLinearHeadingInterpolation(goGPP.getHeading(), gGPP.getHeading())
                .setVelocityConstraint(0.05)
                .build();

        scoreGPP = follower.pathBuilder()
                .addPath(new BezierCurve(gGPP, returnGPP, scorePose2))
                .setLinearHeadingInterpolation(gGPP.getHeading(), scorePose2.getHeading())
                .build();

        leaveBase = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, leave))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), leave.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        if (pathState == null) return;

        switch (pathState) {

            case scorepreload:
                if (!pathStarted) {
                    outtake.INSTANCE.Outc().schedule();
                    actionTimer.resetTimer();
                }
                runPath(scorePreload, Pathstate.wait);
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
                runPath(gotoPPG, Pathstate.grabPPG);
                break;

            case grabPPG:
                runPath(grabPPG, Pathstate.scorePPG);
                break;

            case scorePPG:
                if (!pathStarted) {
                    outtake.INSTANCE.Outc().schedule();
                    actionTimer.resetTimer();
                }
                runPath(scorePPG, Pathstate.wait1);
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
                runPath(gotoPGP, Pathstate.grabPGP);
                break;

            case grabPGP:
                runPath(grabPGP, Pathstate.scorePGP);
                break;

            case scorePGP:
                if (!pathStarted) {
                    outtake.INSTANCE.Outc().schedule();
                    actionTimer.resetTimer();
                }
                runPath(scorePGP, Pathstate.wait2);
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
                runPath(gotoGPP, Pathstate.grabGPP);
                break;

            case grabGPP:
                runPath(grabGPP, Pathstate.scoreGPP);
                break;

            case scoreGPP:
                if (!pathStarted) {
                    outtake.INSTANCE.Outc().schedule();
                    actionTimer.resetTimer();
                }
                runPath(scoreGPP, Pathstate.wait3);
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
                runPath(leaveBase, Pathstate.stop);
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
