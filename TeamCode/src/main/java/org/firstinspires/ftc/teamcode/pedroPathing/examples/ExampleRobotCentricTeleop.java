/**

package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer; // <-- IMPORT THIS
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer; // <-- AND IMPORT THIS
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;



@TeleOp(name = "Example Robot-Centric Teleop", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void init() {
        Localizer myLocalizer = new PinpointLocalizer(hardwareMap, startPose);

        follower = new Follower(hardwareMap, myLocalizer, FConstants.class, LConstants.class);

        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {



        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();

    }

    @Override
    public void stop() {
    }
}
 **/

/**
package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Example Robot-Centric Teleop", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    // --- NEW TURRET VARIABLES ---
    private Servo turretServo1;
    private Servo turretServo2;


    private double turretTargetPosition = 0.5;





    @Override
    public void init() {

        Localizer myLocalizer = new PinpointLocalizer(hardwareMap, startPose);


        follower = new Follower(hardwareMap, myLocalizer, FConstants.class, LConstants.class);

        follower.setStartingPose(startPose);



        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");




            turretServo1.setPosition(turretTargetPosition);
            turretServo2.setPosition(turretTargetPosition);

        } catch (Exception e) {
            telemetry.addData("Error", "Could not find turret servos. Check config.");
        }

    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        follower.startTeleopDrive();
    }


    @Override
    public void loop() {


        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();






        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));


        telemetry.addData("Turret Target Pos", "%.3f", turretTargetPosition);
        telemetry.addData("Gamepad2 Left Stick X", "%.3f", gamepad2.left_stick_x);



        telemetry.update();

    }



    private void handleTurretControl() {

        double turretMoveInput = gamepad2.left_stick_x;


        turretTargetPosition = (turretMoveInput + 1.0) / 2.0;




        turretTargetPosition = Range.clip(turretTargetPosition, 0.0, 1.0);


        turretServo1.setPosition(turretTargetPosition);
        turretServo2.setPosition(turretTargetPosition);
    }




    @Override
    public void stop() {
    }
}
 **/
package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// --- VISION IMPORTS ---
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
// --- END VISION IMPORTS ---

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.List;

@TeleOp(name = "Example TeleOp with Turret Auto-Aim", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {

    // === Pedro Pathing (Drivetrain) ===
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    // === Turret Servos ===
    private Servo turretServo1;
    private Servo turretServo2;

    private double turretTargetPosition = 0.5;


    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private static final int TARGET_TAG_ID = 20;

    // *** PIXEL-BASED CONSTANTS ADAPTED TO REUSE EXISTING NAMES ***
    // AIM_KP is now the conversion from PIXEL error to SERVO position delta. (TUNE THIS!)
    private static final double AIM_KP = 0.0003;
    // TARGET_YAW_DEADZONE is now the PIXEL DEADZONE (e.g., 15.0 pixels)
    private static final double TARGET_YAW_DEADZONE = 15.0;
    // MAX_CORRECTION is now the MAX SERVO POSITION CHANGE per loop.
    private static final double MAX_CORRECTION = 0.01;

    // *** NEW: Required for pixel center calculation ***
    private static final double IMAGE_WIDTH = 640.0;
    private static final double CENTER_X = IMAGE_WIDTH / 2.0;


    @Override
    public void init() {
        Localizer myLocalizer = new PinpointLocalizer(hardwareMap, startPose);
        follower = new Follower(hardwareMap, myLocalizer, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
            turretServo1.setPosition(turretTargetPosition);
            turretServo2.setPosition(turretTargetPosition);
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find turret servos. Check config.");
        }

        try {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcamName)
                    .addProcessor(aprilTagProcessor)
                    .build();
            telemetry.addData("VisionPortal", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find Webcam 1. Check config.");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // --- 1. DRIVETRAIN CONTROL ---
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();


        // --- 2. TURRET CONTROL LOGIC ---
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (gamepad2.a) {
            handleAutoAim(detections); // Uses PIXEL error to set turretTargetPosition
        } else {
            handleTurretControl(); // Uses manual joystick to set turretTargetPosition
        }

        // --- 3. APPLY TURRET POSITION ---
        turretTargetPosition = Range.clip(turretTargetPosition, 0.0, 1.0);
        turretServo1.setPosition(turretTargetPosition);
        turretServo2.setPosition(turretTargetPosition);


        // --- 4. TELEMETRY ---
        telemetry.addData("Turret Control Mode", gamepad2.a ? "**AUTO-AIM** (Pixel-Based)" : "**MANUAL** (Direct Map)");
        telemetry.addData("Turret Target Pos", "%.3f", turretTargetPosition);
        telemetry.addData("Gamepad2 Left Stick X", "%.3f", gamepad2.left_stick_x);
        telemetry.update();

    }



    private void handleTurretControl() {
        double turretMoveInput = gamepad2.left_stick_x;

        // Map the joystick input [-1.0 to 1.0] to the servo position [0.0 to 1.0].
        turretTargetPosition = (turretMoveInput + 1.0) / 2.0;
    }


    /**
     * Handles pixel-based auto-aiming logic, adapted from your CRServo sample.
     * The correction is applied continuously to the Servo's position.
     */
    private void handleAutoAim(List<AprilTagDetection> detections) {
        AprilTagDetection targetTag = null;

        for (AprilTagDetection detection : detections) {
            if (detection.id == TARGET_TAG_ID) {
                targetTag = detection;
                break;
            }
        }

        if (targetTag != null) {
            // Use tag's center.x for robust targeting
            double tagX = targetTag.center.x;
            double pixelError = tagX - CENTER_X; // Positive error means tag is to the right

            telemetry.addData("Tag Center X", "%.2f", tagX);

            // TARGET_YAW_DEADZONE is now the PIXEL DEADZONE (e.g., 15.0 pixels)
            if (Math.abs(pixelError) > TARGET_YAW_DEADZONE) {

                // AIM_KP is now the conversion from PIXEL error to SERVO position delta.
                double correction = pixelError * AIM_KP;

                // MAX_CORRECTION is the max SERVO POSITION CHANGE per loop.
                correction = Range.clip(correction, -MAX_CORRECTION, MAX_CORRECTION);

                // Apply correction. Positive pixelError (tag right) requires a NEGATIVE correction
                // to decrease the servo position (move turret left).
                turretTargetPosition -= correction;

                telemetry.addLine("AUTO-AIM: **TRACKING** (Pixel Error)");
            } else {
                telemetry.addLine("AUTO-AIM: **LOCKED ON!** (Pixel Deadzone)");
            }
            telemetry.addData("Pixel Error", "%.2f px", pixelError);

        } else {
            telemetry.addLine("AUTO-AIM: **Searching for Tag ID 20...**");
        }
    }


    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
            visionPortal.close();
        }
    }
}