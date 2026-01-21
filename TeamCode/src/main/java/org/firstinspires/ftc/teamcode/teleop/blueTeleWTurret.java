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
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.vision.LL3a;
import org.firstinspires.ftc.teamcode.vision.interpolation_table;

import java.util.function.Supplier;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="blueTeleWTurret", group = "Robot")
public class blueTeleWTurret extends NextFTCOpMode {

    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> Shoot;
    private TelemetryManager telemetryM;

    /* =====================
       TURRET VISION TUNING
       ===================== */
    private static final double TURRET_KP = 0.08;
    private static final double MAX_STEP = 1.5;
    private static final double TURRET_DEADBAND = 0.5;

    public blueTeleWTurret() {}

    public void onInit() {

        follower = Constants.createFollower(hardwareMap);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Init Limelight
        LL3a.init(hardwareMap, 2);

        // Register subsystems
        addComponents(
                new SubsystemComponent(
                        outtake.INSTANCE,
                        intake.INSTANCE,
                        brakeL.INSTANCE,
                        brakeR.INSTANCE,
                        shootadj.INSTANCE,
                        stopper.INSTANCE,
                        turret.INSTANCE,
                        LL3a.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

        Shoot = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(85, 95))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(50),
                                0.8
                        )
                )
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

        /* =====================
           BINDINGS
           ===================== */

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

        button(() -> gamepad2.a)
                .whenBecomesTrue(stopper.INSTANCE.go)
                .whenBecomesFalse(stopper.INSTANCE.stop);

        button(() -> gamepad2.dpad_up)
                .whenBecomesTrue(shootadj.INSTANCE.upL())
                .whenBecomesTrue(shootadj.INSTANCE.upR());

        button(() -> gamepad2.dpad_left)
                .whenBecomesTrue(shootadj.INSTANCE.midL())
                .whenBecomesTrue(shootadj.INSTANCE.midR());

        button(() -> gamepad2.dpad_down)
                .whenBecomesTrue(shootadj.INSTANCE.lowL())
                .whenBecomesTrue(shootadj.INSTANCE.lowR());

        button(() -> gamepad2.x)
                .whenBecomesTrue(intake.INSTANCE.Stop())
                .whenBecomesFalse(intake.INSTANCE.In());

        button(() -> gamepad2.dpad_right)
                .whenBecomesTrue(outtake.INSTANCE.Outs())
                .whenBecomesFalse(outtake.INSTANCE.idle());

        button(() -> outtake.getVelocity() >= 900)
                .whenBecomesTrue(() -> gamepad2.rumble(150))
                .whenBecomesFalse(() -> gamepad2.stopRumble());

        button(() -> gamepad2.left_bumper)
                .whenBecomesTrue(outtake.INSTANCE.Outc())
                .whenBecomesFalse(outtake.INSTANCE.idle());

        // Vision-assisted shooting
        button(() -> gamepad2.right_bumper)
                .whenBecomesTrue(() -> {
                    if (LL3a.INSTANCE.hasValidTarget()) {

                        double d = LL3a.INSTANCE.Tx();
                        double rpm = interpolation_table.rpmForDistance(d);
                        double hood = interpolation_table.hoodForDistance(d);

                        outtake.INSTANCE.targetVel(rpm).schedule();
                        shootadj.INSTANCE.targetPosL(hood).schedule();
                        shootadj.INSTANCE.targetPosR(hood).schedule();
                    }
                })
                .whenBecomesFalse(() -> {
                    outtake.INSTANCE.idle().schedule();
                    shootadj.INSTANCE.midL().schedule();
                    shootadj.INSTANCE.midR().schedule();
                });
    }

    public void onUpdate() {

        follower.update();
        telemetryM.update();
        BindingManager.update();

        telemetry.addData("outtake velocity", outtake.getVelocity());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        if (LL3a.INSTANCE != null) {
            telemetry.addData("LL has target", LL3a.INSTANCE.hasValidTarget());
            telemetry.addData("LL distance", LL3a.INSTANCE.Tx());
            telemetry.addData("turret degrees", turret.INSTANCE.getAngleDeg());
        }

        telemetry.update();

        if (gamepad1.square)
            follower.setPose(follower.getPose().withHeading(0));

        /* =====================
           TURRET AUTO TRACKING
           ===================== */

        if (LL3a.INSTANCE.hasValidTarget()) {

            double tx = LL3a.INSTANCE.Tx();

            if (Math.abs(tx) > TURRET_DEADBAND) {

                double correction = -tx * TURRET_KP;

                // Rate limit
                correction = Math.max(-MAX_STEP, Math.min(MAX_STEP, correction));

                double newTarget = turret.INSTANCE.getAngleDeg() + correction;
                turret.INSTANCE.setGoalAngleDeg(newTarget);
            }
        }

        /* =====================
           DRIVER CONTROL
           ===================== */

        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
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
