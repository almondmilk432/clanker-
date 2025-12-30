package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class stopper implements Subsystem {
    public static final stopper INSTANCE = new stopper();

    private stopper() {
    }

    private ServoEx servo = new ServoEx("stopper");

    public Command stop = new SetPosition(servo, 1).requires(this);
    public Command go = new SetPosition(servo, 0.6).requires(this);
}
