package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class brakeR implements Subsystem {
    public static final brakeR INSTANCE = new brakeR();

    private brakeR() {
    }

    private ServoEx servo = new ServoEx("brakeR");

    public Command up = new SetPosition(servo, .425).requires(this);
    public Command down = new SetPosition(servo, .25).requires(this);
}
