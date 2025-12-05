package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class shootadj implements Subsystem {
    public static final shootadj INSTANCE = new shootadj();

    private shootadj() {
    }

    private ServoEx servo = new ServoEx("shootadj");

    public Command up = new SetPosition(servo, .15).requires(this);
    public Command mid = new SetPosition(servo, .25).requires(this);
    public Command low = new SetPosition(servo, 0.4).requires(this);
}