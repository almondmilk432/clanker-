package org.firstinspires.ftc.teamcode.teleop;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.blueClose12C;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.brakeL;
import org.firstinspires.ftc.teamcode.subsystems.brakeR;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake;
import org.firstinspires.ftc.teamcode.subsystems.shootadj;
import org.firstinspires.ftc.teamcode.subsystems.stopper;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.vision.LL3a;
import org.firstinspires.ftc.teamcode.vision.interpolation_table;

import java.util.function.Supplier;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="blueTele", group = "Robot")
public class blueTele extends NextFTCOpMode {

    public blueTele() {

    }

    private Follower follower;
    // public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> ShootC, ShootF;
    private TelemetryManager telemetryM;
    private static final double LLCorrection = 0.02;





    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(blueClose12C.blueEndC);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        LL3a.init(hardwareMap, 2);

        addComponents(
                new SubsystemComponent(outtake.INSTANCE, intake.INSTANCE, brakeL.INSTANCE, brakeR.INSTANCE,
                        shootadj.INSTANCE, stopper.INSTANCE, LL3a.INSTANCE, turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );


        ShootC = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(85, 95).mirror())))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(80), 0.8))
                .build();

        ShootF = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(60, 19))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(111), 0.8))
                .build();

    }

    @Override
    public void onStartButtonPressed() {

        follower.startTeleopDrive();

        LL3a.INSTANCE.onStart();


        intake.INSTANCE.In().schedule();
        outtake.INSTANCE.idle().schedule();
        brakeL.INSTANCE.up.schedule();
        brakeR.INSTANCE.up.schedule();
        shootadj.INSTANCE.midL().schedule();
        shootadj.INSTANCE.midR().schedule();
        stopper.INSTANCE.stop.schedule();





        button(() -> gamepad2.y)
                .whenBecomesTrue(intake.INSTANCE.Out())
                .whenBecomesTrue(outtake.INSTANCE.reverse())
                .whenBecomesTrue(stopper.INSTANCE.go)
                .whenBecomesFalse(outtake.INSTANCE.idle())
                .whenBecomesFalse(stopper.INSTANCE.stop)
                .whenBecomesFalse(intake.INSTANCE.In());

        button(() -> gamepad1.b)
                .whenBecomesTrue(brakeL.INSTANCE.down)
                .whenBecomesTrue(brakeR.INSTANCE.down)
                .whenBecomesTrue(() -> gamepad1.rumble(1000))
                .whenBecomesFalse(() -> gamepad1.stopRumble())
                .whenBecomesFalse(brakeL.INSTANCE.up)
                .whenBecomesFalse(brakeR.INSTANCE.up);

        button(()-> gamepad2.a)
                .whenBecomesTrue(stopper.INSTANCE.go)
                //.whenBecomesTrue(() -> gamepad2.rumble(250))
                // .whenBecomesFalse(() -> gamepad2.stopRumble())
                .whenBecomesFalse(stopper.INSTANCE.stop);

        button(()-> gamepad2.dpad_up)
                .whenBecomesTrue(shootadj.INSTANCE.upL())
                .whenBecomesTrue(shootadj.INSTANCE.upR());
        button(()-> gamepad2.dpad_left)
                .whenBecomesTrue(shootadj.INSTANCE.midL())
                .whenBecomesTrue(shootadj.INSTANCE.midR());
        button(()-> gamepad2.dpad_down)
                .whenBecomesTrue(shootadj.INSTANCE.lowL())
                .whenBecomesTrue(shootadj.INSTANCE.lowR());


        button(() -> gamepad2.x)
                .whenBecomesTrue(intake.INSTANCE.Stop())
                .whenBecomesFalse(intake.INSTANCE.In());

        button(()-> gamepad2.dpad_right)
                .whenBecomesTrue(outtake.INSTANCE.Outs())
                .whenBecomesFalse(outtake.INSTANCE.idle());

        button(() -> outtake.getVelocity() >= 800)
                .whenBecomesTrue(() -> gamepad2.rumble(150))
                .whenBecomesFalse(() -> gamepad2.stopRumble());

        button(()-> gamepad2.dpad_left)
                .whenBecomesTrue(outtake.INSTANCE.Outc())
                .whenBecomesFalse(outtake.INSTANCE.idle());

        button(() -> gamepad2.right_bumper)
                .whenBecomesFalse(() -> outtake.INSTANCE.idle().schedule());




    }

    public void onUpdate() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        BindingManager.update();

        telemetry.addData("outtake velocity", outtake.getVelocity());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("ll sees something:" , LL3a.INSTANCE.hasValidTarget());
        telemetry.addData("TX", LL3a.INSTANCE.Tx());
        telemetry.addData("Distance (m)", LL3a.INSTANCE.getDistance());




        telemetry.update();

        if (gamepad1.y)
            follower.setPose(follower.getPose().withHeading(0));

        double rotInput = -gamepad1.right_stick_x;

        if (gamepad2.touchpadWasPressed() &&
                LL3a.INSTANCE != null &&
                LL3a.INSTANCE.hasValidTarget()) {

            double tx = LL3a.INSTANCE.Tx(); // horizontal error in degrees
            rotInput = -tx * LLCorrection;
            if (Math.abs(tx) < 0.5) { //saftety to prevent oscolation. Remove if unneeded
                rotInput = 0;
            }
        }

        if (gamepad2.right_bumper &&
                LL3a.INSTANCE != null &&
                LL3a.INSTANCE.hasValidTarget()) {

            double distance = LL3a.INSTANCE.getDistance();
            double rpm = interpolation_table.rpmForDistance(distance);

            outtake.INSTANCE.targetVel(rpm).schedule();
        }


        //path following
        if (gamepad1.leftBumperWasPressed()) {
            follower.followPath(ShootC.get());
            automatedDrive = true;
        }


        if (gamepad1.rightBumperWasPressed()) {
            follower.followPath(ShootF.get());
            automatedDrive = true;
        }
        if (gamepad1.rightBumperWasReleased()) {
            automatedDrive = false;
        }




        if (!automatedDrive) {
            //Make the last parameter false for field-centric

            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    rotInput,
                    false // F= Feild Centric   T= Robot Centric

            );
        }

    }

    public void onStop() {
        BindingManager.reset();
        intake.INSTANCE.Stop().schedule();
        outtake.INSTANCE.Stop().schedule();
        turret.INSTANCE.stop().schedule();
    }
}