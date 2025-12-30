package org.firstinspires.ftc.teamcode.teleop;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
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

@TeleOp(name="tele", group = "Robot")
public class tele extends NextFTCOpMode {

    public tele() {
        addComponents(
                new SubsystemComponent(outtake.INSTANCE, intake.INSTANCE, brakeL.INSTANCE, brakeR.INSTANCE,
                        shootadj.INSTANCE, stopper.INSTANCE, LL3a.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    //public static final Pose ShootP = new Pose(85, 95, Math.toRadians(50)); //put your desired position and heading here

    private Follower follower;
    // public static Pose startingPose; //See MoveTestAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> Shoot;
    private LL3a limelight;
    private TelemetryManager telemetryM;
    private boolean visionPipelineActive = false;




    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(blueFarAuto.autoEndPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        LL3a.init(hardwareMap, 1);

        Shoot = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(85, 95))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(50), 0.8))
                .build();


    }

    @Override
    public void onStartButtonPressed() {

        follower.startTeleopDrive();

        intake.INSTANCE.In().schedule();
        outtake.INSTANCE.Stop().schedule();
        brakeL.INSTANCE.up.schedule();
        brakeR.INSTANCE.up.schedule();
        shootadj.INSTANCE.midL().schedule();
        shootadj.INSTANCE.midR().schedule();
        stopper.INSTANCE.stop.schedule();





        button(() -> gamepad2.y)
                .whenBecomesTrue(intake.INSTANCE.Out())
                .whenBecomesFalse(intake.INSTANCE.In());

        button(() -> gamepad1.b)
                .whenBecomesTrue(brakeL.INSTANCE.down)
                        .whenBecomesTrue(brakeR.INSTANCE.down)
                                .whenBecomesFalse(brakeL.INSTANCE.up)
                                        .whenBecomesFalse(brakeR.INSTANCE.up);

        button(()-> gamepad2.a)
                .whenBecomesTrue(stopper.INSTANCE.go)
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
                        .whenBecomesFalse(outtake.INSTANCE.Stop());

        button(()-> gamepad2.left_bumper)
                .whenBecomesTrue(outtake.INSTANCE.Outc())
                .whenBecomesFalse(outtake.INSTANCE.Stop());






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
        telemetry.addData("LL has target", LL3a.INSTANCE.hasValidTarget());
        telemetry.addData("LL has MT2", LL3a.INSTANCE.hasMT2());
        telemetry.addData("LL distance", LL3a.INSTANCE.getDistanceToTag());
        telemetry.addData("LL result null", LL3a.INSTANCE.latest == null);


        telemetry.update();

        if (gamepad1.square)
            follower.setPose(follower.getPose().withHeading(0));

        if (gamepad2.right_bumper && limelight.hasValidTarget()) {

            double distance = limelight.getDistanceToTag();



            double rpm = interpolation_table.rpmForDistance(distance);
            double hood = interpolation_table.hoodForDistance(distance);


            outtake.INSTANCE.targetVel(rpm).schedule();
            shootadj.INSTANCE.targetPosL(hood).schedule();
            shootadj.INSTANCE.targetPosR(hood).schedule();
        }


        if (!automatedDrive) {
            //Make the last parameter false for field-centric

            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // F= Feild Centric   T= Robot Centric

            );
        }

        //Automated PathFollowing
        if (gamepad1.touchpadWasPressed()) {
            follower.followPath(Shoot.get());
            automatedDrive = true;
        }
        if (gamepad1.touchpadWasReleased()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

    }

    public void onStop() {
        BindingManager.reset();
        intake.INSTANCE.Stop().schedule();
        outtake.INSTANCE.Stop().schedule();
    }
}
