/*package org.firstinspires.ftc.teamcode.pedroPathing.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Camera")
public class AutoInitCameraOnly extends OpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("visionPortal status", visionPortal.getCameraState());
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            telemetry.addData("Detected an AprilTag", String.format("ID %s", detection.id));
        }
    }

    @Override
    public void loop() {
    }
}

 */


package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Camera")
public class AutoInitCameraOnly extends OpMode {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private CRServo axonMotor;  // CRServo controlling the Axon

    // Constants for tracking
    private static final int TARGET_TAG_ID = 20;
    private static final double IMAGE_WIDTH = 640.0;  // Assuming 640x480 resolution
    private static final double CENTER_X = IMAGE_WIDTH / 2.0;
    private static final double DEAD_ZONE = 15.0; // Pixels within which we stop moving
    private static final double kP = 0.0025; // Proportional constant (tune this)
    private static final double MAX_POWER = 0.4; // Max speed for servo movement

    @Override
    public void init() {
        // Initialize webcam and AprilTag detection
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .build();

        // Initialize CRServo connected to Servo Port 3
        axonMotor = hardwareMap.get(CRServo.class, "axonMotor");

        // Debugging: Show all devices
        for (CRServo servo : hardwareMap.getAll(CRServo.class)) {
            telemetry.addLine("Found CRServo: " + hardwareMap.getNamesOf(servo));
        }
        for (WebcamName cam : hardwareMap.getAll(WebcamName.class)) {
            telemetry.addLine("Found Webcam: " + hardwareMap.getNamesOf(cam));
        }
        telemetry.update();

        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("VisionPortal Status", visionPortal.getCameraState());

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            telemetry.addData("Detected AprilTag", "ID %d", detection.id);
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagDetection targetTag = null;

        // Find AprilTag ID 20
        for (AprilTagDetection detection : detections) {
            if (detection.id == TARGET_TAG_ID) {
                targetTag = detection;
                break;
            }
        }

        if (targetTag != null) {
            double tagX = targetTag.center.x;
            double error = tagX - CENTER_X;

            telemetry.addData("Tracking Tag ID", TARGET_TAG_ID);
            telemetry.addData("Tag X", tagX);
            telemetry.addData("Error from center", error);

            if (Math.abs(error) > DEAD_ZONE) {
                // Proportional control
                double power = -error * kP;

                // Clamp power to max range
                power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

                axonMotor.setPower(power);
                telemetry.addData("Servo Power", power);
            } else {
                axonMotor.setPower(0);
                telemetry.addLine("Tag centered — servo stopped.");
            }
        } else {
            // Tag not found
            axonMotor.setPower(0);
            telemetry.addLine("Tag ID 20 not found — holding position.");
        }

        telemetry.update();
    }
}


