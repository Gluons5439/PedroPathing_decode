package org.firstinspires.ftc.teamcode.vision.Limelight;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="AprilTag Test Limelight", group="Vision")
public class Limelight extends OpMode {

    private Limelight3A limelight;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(2);
    }

    @Override
    public void start() {
        limelight.start();

    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}