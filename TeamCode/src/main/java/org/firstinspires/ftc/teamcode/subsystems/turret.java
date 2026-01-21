package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.control.KineticState;


public class turret implements Subsystem {

    public static final turret INSTANCE = new turret();
    private turret() {}

    private double goalTicks = 0;

    private MotorEx motor = new MotorEx("turret");


    private static final double ticsPerRev = 537.7;   // goBILDA TPR ratio
    private static final double gearRatio = 3.243;        // update to current gear ratio if needed
    private static final double degreesPerTic =
            360.0 / (ticsPerRev * gearRatio);

    private static final double minDegree = -90;
    private static final double MaxDegree = 90;


    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0.0008)
            .build();





    public void setGoalAngleDeg(double degrees) {
        degrees = clamp(degrees, minDegree, MaxDegree);
        goalTicks = degrees / degreesPerTic;
        controlSystem.setGoal(new KineticState(goalTicks));
    }

    public void trackTx(double txDeg) {
        // If tx is positive, target is to the right.
        // Typical correction is subtract tx from turret angle
        setGoalAngleDeg(getAngleDeg() - txDeg);
    }

    public Command targetAngle(double degrees) {
        degrees = clamp(degrees, minDegree, MaxDegree);

        double targetTicks = degrees / degreesPerTic;
        return new RunToPosition(controlSystem, targetTicks).requires(this);
    }




    public Command trackTarget(double tx) {
        double newTarget = getAngleDeg() - tx;
        return targetAngle(newTarget);
    }

    public Command stop() {
        return new RunToPosition(controlSystem,
                motor.getState().getPosition())
                .requires(this);
    }

    public Command seekByDegrees(double deltaDeg) {
        return targetAngle(getAngleDeg() + deltaDeg);
    }


    public double getAngleDeg() {
        return motor.getState().getPosition() * degreesPerTic;
    }


    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
