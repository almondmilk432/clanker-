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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.brakeL;
import org.firstinspires.ftc.teamcode.subsystems.brakeR;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake;
import org.firstinspires.ftc.teamcode.subsystems.shootadj;
import org.firstinspires.ftc.teamcode.subsystems.stopper;
import org.firstinspires.ftc.teamcode.vision.LL3a;
import org.firstinspires.ftc.teamcode.vision.interpolation_table;

import java.util.function.Supplier;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="redTele", group = "Robot")
public class redTele extends NextFTCOpMode {

    public redTele() {

    }
    //public static final Pose ShootP = new Pose(85, 95, Math.toRadians(50)); //put your desired position and heading here

    private Follower follower;
    // public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> Shoot;
    private TelemetryManager telemetryM;
    private static final double LLCorrection = 0.02;




    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(blueFarAuto.autoEndPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        LL3a.init(hardwareMap, 1);

        addComponents(
                new SubsystemComponent(outtake.INSTANCE, intake.INSTANCE, brakeL.INSTANCE, brakeR.INSTANCE,
                        shootadj.INSTANCE, stopper.INSTANCE, LL3a.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );


        Shoot = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(85, 95))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(50), 0.8))
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

        button(() -> outtake.getVelocity() >= 900)
                .whenBecomesTrue(() -> gamepad2.rumble(150))
                .whenBecomesFalse(() -> gamepad2.stopRumble());

        button(()-> gamepad2.left_bumper)
                .whenBecomesTrue(outtake.INSTANCE.Outc())
                .whenBecomesFalse(outtake.INSTANCE.idle());

        button(() -> gamepad2.right_bumper)
                .whenBecomesTrue(() -> {
                    if (LL3a.INSTANCE != null && LL3a.INSTANCE.hasValidTarget()) {

                        double d = LL3a.INSTANCE.getDistanceToTag();
                        double rpm = interpolation_table.rpmForDistance(d);
                        double hood = interpolation_table.hoodForDistance(d);

                        outtake.INSTANCE.targetVel(rpm).schedule();
                        shootadj.INSTANCE.getAngleL();
                        shootadj.INSTANCE.getAngleR();
                    }
                })
                .whenBecomesFalse(() -> {
                    outtake.INSTANCE.idle().schedule();
                    shootadj.INSTANCE.midL().schedule();
                    shootadj.INSTANCE.midR().schedule();
                });




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
        if (LL3a.INSTANCE != null) {
            telemetry.addData("LL has target", LL3a.INSTANCE.hasValidTarget());
            telemetry.addData("LL distance", LL3a.INSTANCE.getDistanceToTag());
        } else {
            telemetry.addLine("LL3a INSTANCE NULL");
        }



        telemetry.update();

        if (gamepad1.square)
            follower.setPose(follower.getPose().withHeading(0));

        double rotInput = -gamepad1.right_stick_x;

        if (gamepad2.touchpadWasPressed() &&
                LL3a.INSTANCE != null &&
                LL3a.INSTANCE.hasValidTarget()) {

            double tx = LL3a.INSTANCE.getDistanceToTag(); // horizontal error in degrees
            rotInput = -tx * LLCorrection;
            if (Math.abs(tx) < 0.5) { //saftety to prevent oscolation. Remove if unneeded
                rotInput = 0;
            }
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
    }
}
