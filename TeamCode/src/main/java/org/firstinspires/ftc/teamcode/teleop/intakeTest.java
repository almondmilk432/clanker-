package org.firstinspires.ftc.teamcode.teleop;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.brakeL;
import org.firstinspires.ftc.teamcode.subsystems.brakeR;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake;
import org.firstinspires.ftc.teamcode.subsystems.shootadj;
import org.firstinspires.ftc.teamcode.subsystems.stopper;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.vision.LL3a;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="intaketest", group = "Robot")
public class intakeTest extends NextFTCOpMode {

    public intakeTest() {}

    public void onInit() {
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
    }

    public void onStartButtonPressed() {
        intake.INSTANCE.In().schedule();

        button(() -> gamepad2.x)
                .whenBecomesTrue(intake.INSTANCE.Stop())
                .whenBecomesFalse(intake.INSTANCE.In());
    }

    public void onUpdate(){
        BindingManager.update();

    }
    public void onStop(){
        BindingManager.update();
        intake.INSTANCE.Stop().schedule();
    }
}

