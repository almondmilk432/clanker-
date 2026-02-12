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

@Autonomous(name = "blueCollab", group = "robot")
public class blueCollab extends NextFTCOpMode {


    public blueCollab () {
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
    private final Pose grabGate = new Pose(10, 24, Math.toRadians(90));
    private final Pose PPGc = new Pose(62, 36);
    private final Pose corner = new Pose(20, 17, Math.toRadians(180));
    private final Pose goPGP = new Pose(60, 60, Math.toRadians(180));
    private final Pose PGPc = new Pose(67, 59);
    private final Pose gPGP = new Pose(20, 60, Math.toRadians(180));
    private final Pose goGPP = new Pose(55, 87, Math.toRadians(180));
    private final Pose GPPc = new Pose(67, 83);
    private final Pose gGPP = new Pose(25, 87, Math.toRadians(180));
    private final Pose returnGPP = new Pose(53, 63);
    private final Pose leave = new Pose(38, 32, Math.toRadians(0));

    private PathChain scorePreload, grabgate, scoreGate, grabCorner, scoreCorner;
    private PathChain gotoPGP, grabPGP, scorePGP;
    private PathChain gotoGPP, grabGPP, scoreGPP, leaveBase;

    public enum Pathstate {
        scorepreload, wait, closeGate, gotoGate, waitGrab, scoreGate, wait1, closeGate1,
        GotoCorner, GrabCorner, scoreCorner, wait2, closeGate2,
        gotoGPP, grabGPP, scoreGPP, wait3, closeGate3,
        leaveBase, stop
    }

    Pathstate pathState;

    @Override
    public void onInit() {



        brakeL.INSTANCE.up.schedule();
        brakeR.INSTANCE.up.schedule();
        stopper.INSTANCE.stop.schedule();
        shootadj.INSTANCE.upL().schedule();

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

        grabgate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, grabGate))
                .setLinearHeadingInterpolation(scorePose.getHeading(), grabGate.getHeading())
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierLine(grabGate, scorePose))
                .setLinearHeadingInterpolation(grabGate.getHeading(), scorePose.getHeading())
                .setVelocityConstraint(0.005)
                .build();

        grabCorner = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, corner))
                .setLinearHeadingInterpolation(scorePose.getHeading(), corner.getHeading())
                .build();

        scoreCorner = follower.pathBuilder()
                .addPath(new BezierLine(corner, scorePose))
                .setLinearHeadingInterpolation(corner.getHeading(), scorePose.getHeading())
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
                if (actionTimer.getElapsedTimeSeconds() >= 1.25) {
                    stopper.INSTANCE.stop.schedule();
                    outtake.INSTANCE.Stop().schedule();
                    pathStarted = false;
                    pathState = Pathstate.gotoGate;
                }
                break;

            case gotoGate:
                runPath(grabgate, Pathstate.waitGrab);
                break;

            case waitGrab:
                if (actionTimer.getElapsedTimeSeconds() >= .5) {
                    pathState = Pathstate.wait3;
                }
                break;

            case wait3:
                if (actionTimer.getElapsedTimeSeconds() >= 3) {
                    pathState = Pathstate.scoreGate;
                }
                break;

            case scoreGate:
                if (!pathStarted) {
                    outtake.INSTANCE.Outc().schedule();
                    actionTimer.resetTimer();
                }
                runPath(scoreGate, Pathstate.wait1);
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
                    pathState = Pathstate.GotoCorner;
                }
                break;

            case GotoCorner:
                runPath(grabCorner, Pathstate.scoreCorner);
                break;

            case scoreCorner:
                runPath(scoreCorner, Pathstate.wait2);
                break;


            case wait2:
                if (actionTimer.getElapsedTimeSeconds() >= 0) {
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
