package org.firstinspires.ftc.teamcode.subsystems;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class intake implements Subsystem {
    public static final intake INSTANCE = new intake();
    private intake() { }

    private MotorEx motor = new MotorEx("intake").reversed();
    private CRServoEx servoR = new CRServoEx("intakeR");
    private CRServoEx servoL = new CRServoEx("intakeL");



    private ControlSystem controlSystem = ControlSystem.builder()
            .basicFF(11, 0,0)
            .build();

    public Command In (){
        new SetPower(servoR, 1).requires(this);
        new SetPower(servoL, -1).requires(this);
        return new RunToVelocity(controlSystem, 5000).requires(this);

    }


    public Command Out (){
        new SetPower(servoR, -1).requires(this);
        new SetPower(servoL, 1).requires(this);
        return new RunToVelocity(controlSystem, -5000).requires(this);
    }


    public Command Stop (){
        new SetPower(servoR, 0).requires(this);
        new SetPower(servoL, 0).requires(this);
        return new RunToVelocity(controlSystem, 0).requires(this);
    }

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));

    }
}

