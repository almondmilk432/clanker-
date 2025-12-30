package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class shootadj implements Subsystem {
    public static final shootadj INSTANCE = new shootadj();

    private shootadj() {
    }

    private ServoEx servoL = new ServoEx("adjL");
    private ServoEx servoR = new ServoEx("adjR");

    public Command upL() {
        return new SetPosition(servoL, .05).requires(this);
    }
    public Command upR() {
        return new SetPosition(servoR, .4).requires(this);
    }



    public Command midL() {
        return new SetPosition(servoL, .15).requires(this);
    }
    public Command midR(){
        return new SetPosition(servoR, .35).requires(this);
    }



    public Command lowL() {
        return new SetPosition(servoL, .4).requires(this);
    }
    public Command lowR() {
        return new SetPosition(servoR, .1).requires(this);
    }



    public Command targetPosR(double pos) {
        pos = Math.max(0.4, Math.min(0.1, pos));
        return new SetPosition(servoR, pos).requires(this);
    }
    public Command targetPosL(double pos) {
        pos = Math.max(0.4, Math.min(0.5, pos));
        return new SetPosition(servoL, pos).requires(this);
     }

    public double getAngleL() {
        return servoL.getPosition();
    }
    public double getAngleR() {
        return servoR.getPosition();
    }



}