package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import dev.nextftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LL3a implements Subsystem {

    /* =======================
       SINGLETON INSTANCE
       ======================= */
    public static LL3a INSTANCE;

    private Limelight3A limelight;
    private IMU imu;

    public LLResult latest;
    private Pose3D mt2Pose;

    private final int startupPipeline;

    /* =======================
       PRIVATE CONSTRUCTOR
       ======================= */
    private LL3a(int pipeline) {
        this.startupPipeline = pipeline;
    }

    /* =======================
       SAFE INITIALIZATION
       ======================= */
    public static void init(HardwareMap hardwareMap, int pipeline) {
        INSTANCE = new LL3a(pipeline);
        INSTANCE.initHardware(hardwareMap);
    }

    private void initHardware(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                );

        imu.initialize(new IMU.Parameters(orientation));
    }

    /* =======================
       NEXTFTC LIFECYCLE
       ======================= */
    public void onStart() {
        limelight.start();
        limelight.pipelineSwitch(startupPipeline);
    }

    @Override
    public void periodic() {
        YawPitchRollAngles yaw = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(yaw.getYaw());

        latest = limelight.getLatestResult();

        if (latest != null && latest.isValid()) {
            mt2Pose = latest.getBotpose_MT2();
        } else {
            mt2Pose = null;
        }
    }

    /* =======================
       PUBLIC API (SAFE)
       ======================= */
    public boolean hasMT2() {
        return mt2Pose != null;
    }

    public Pose3D getPose() {
        return mt2Pose;
    }

    public double getDistanceToTag() {
        if (!hasMT2()) return 0;
        double x = mt2Pose.getPosition().x;
        double y = mt2Pose.getPosition().y;
        return Math.hypot(x, y);
    }

    public boolean hasValidTarget() {
        return latest != null && latest.isValid();
    }

    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }
}
