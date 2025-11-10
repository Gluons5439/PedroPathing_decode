/*

package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

// --- LIMELIGHT IMPORTS ---
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Example TeleOp with Turret Auto-Aim", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    private Servo turretServo1;
    private Servo turretServo2;

    private double turretTargetPosition = 0.5;

    private DcMotor shooterMotor;
    private static final double SHOOTER_MAX_POWER = 1.0;

    // --- LIMELIGHT DECLARATION ---
    private Limelight3A limelight;

    private static final int TARGET_TAG_ID = 20;

    // --- UPDATED CONSTANTS FOR STABLE AIMING ---
    // Proportional constant (Kp) significantly reduced to stop oscillation
    private static final double AIM_KP = 0.0008;
    // Deadzone in degrees (tx)
    private static final double AIM_DEADZONE = 1.5;
    // Max rate of change for the servo position, reduced for smoother moves
    private static final double MAX_CORRECTION = 0.003;


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
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Shooter", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find shooter.");
        }

        // --- LIMELIGHT INITIALIZATION ---
        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            // Set to Pipeline 0
            limelight.pipelineSwitch(0);
            telemetry.addData("Limelight", "Initialized on Pipeline 0");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find Limelight 3A. Check config.");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        if (limelight != null) {
            limelight.start();
        }
    }

    @Override
    public void loop() {

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        // Control mode selection
        if (gamepad2.a) {
            handleAutoAim();
        } else {
            handleTurretControl();
        }

        // Apply turret position
        turretTargetPosition = Range.clip(turretTargetPosition, 0.0, 1.0);
        if (turretServo1 != null && turretServo2 != null) {
            turretServo1.setPosition(turretTargetPosition);
            turretServo2.setPosition(turretTargetPosition);
        }

        handleShooterControl();

        telemetry.addData("Turret Control Mode", gamepad2.a ? "**AUTO-AIM** (Limelight Degrees)" : "**MANUAL** (Direct Map)");
        telemetry.addData("Turret Target Pos", "%.3f", turretTargetPosition);
        telemetry.addData("Shooter Power", "%.1f (R. Bumper)", (shooterMotor != null) ? shooterMotor.getPower() : 0.0);
        telemetry.update();
    }

    private void handleShooterControl() {
        if (shooterMotor == null) return;

        if (gamepad2.right_bumper) {
            shooterMotor.setPower(SHOOTER_MAX_POWER);
        } else {
            shooterMotor.setPower(0.0);
        }
    }

    private void handleTurretControl() {
        double turretMoveInput = gamepad2.left_stick_x;
        turretTargetPosition = (turretMoveInput + 1.0) / 2.0;
    }

    // --- LIMELIGHT AUTO-AIM LOGIC ---
    private void handleAutoAim() {
        if (limelight == null) {
            telemetry.addLine("AUTO-AIM: **Limelight not initialized!**");
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            telemetry.addData("Target X (Degrees)", "%.2f", tx);

            if (Math.abs(tx) > AIM_DEADZONE) {

                double correction = tx * AIM_KP;

                correction = Range.clip(correction, -MAX_CORRECTION, MAX_CORRECTION);

                // Subtract correction to move the turret toward 0.0 (left) if tx is positive (target is right)
                turretTargetPosition -= correction;

                telemetry.addLine("AUTO-AIM: **TRACKING** (Degrees Error)");
                telemetry.addData("Correction", "%.5f", correction);
            } else {
                telemetry.addLine("AUTO-AIM: **LOCKED IN!** (Degrees Deadzone)");
            }

        } else {
            telemetry.addLine("AUTO-AIM: **Limelight No Target**");
        }
    }


    @Override
    public void stop() {
        if (shooterMotor != null) {
            shooterMotor.setPower(0);
        }
    }
}

 */

package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Example TeleOp with Turret Auto-Aim", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    private Servo turretServo1;
    private Servo turretServo2;

    private double turretTargetPosition = 0.5;

    private DcMotor shooterMotor;
    private static final double SHOOTER_MAX_POWER = 1.0;

    private Limelight3A limelight;

    private static final int TARGET_TAG_ID = 20;

    private static final double AIM_KP = 0.0008;
    private static final double AIM_DEADZONE = 1.5;
    private static final double MAX_CORRECTION = 0.001;

    private boolean isAutoAimActive = false;
    private boolean a_was_pressed = false;


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
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Shooter", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find shooter.");
        }

        // --- LIMELIGHT INITIALIZATION ---
        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            // Set to Pipeline 0
            limelight.pipelineSwitch(0);
            telemetry.addData("Limelight", "Initialized on Pipeline 0");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find Limelight 3A. Check config.");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        if (limelight != null) {
            limelight.start();
        }
    }

    @Override
    public void loop() {

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        boolean a_is_pressed = gamepad2.a;

        if (a_is_pressed && !a_was_pressed) {
            isAutoAimActive = !isAutoAimActive;
        }
        a_was_pressed = a_is_pressed;


        if (isAutoAimActive) {
            handleAutoAim();
        } else {
            handleTurretControl();
        }

        turretTargetPosition = Range.clip(turretTargetPosition, 0.0, 1.0);
        if (turretServo1 != null && turretServo2 != null) {
            turretServo1.setPosition(turretTargetPosition);
            turretServo2.setPosition(turretTargetPosition);
        }

        handleShooterControl();

        telemetry.addData("Turret Control Mode", isAutoAimActive ? "🤖 **AUTO-AIM** (Toggle ON)" : "🎮 **MANUAL** (Toggle OFF)");
        telemetry.addData("Turret Target Pos", "%.3f", turretTargetPosition);
        telemetry.addData("Shooter Power", "%.1f (R. Bumper)", (shooterMotor != null) ? shooterMotor.getPower() : 0.0);
        telemetry.update();
    }

    private void handleShooterControl() {
        if (shooterMotor == null) return;

        if (gamepad2.right_bumper) {
            shooterMotor.setPower(SHOOTER_MAX_POWER);
        } else {
            shooterMotor.setPower(0.0);
        }
    }

    private void handleTurretControl() {
        double turretMoveInput = gamepad2.left_stick_x;
        turretTargetPosition = (turretMoveInput + 1.0) / 2.0;
    }

    private void handleAutoAim() {
        if (limelight == null) {
            telemetry.addLine("AUTO-AIM: **Limelight not initialized!**");
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            telemetry.addData("Target X (Degrees)", "%.2f", tx);

            if (Math.abs(tx) > AIM_DEADZONE) {

                double correction = tx * AIM_KP;

                correction = Range.clip(correction, -MAX_CORRECTION, MAX_CORRECTION);

                turretTargetPosition -= correction;

                telemetry.addLine("AUTO-AIM: **TRACKING** (Degrees Error)");
                telemetry.addData("Correction", "%.5f", correction);
            } else {
                telemetry.addLine("AUTO-AIM: **LOCKED IN!** (Degrees Deadzone)");
            }

        } else {
            telemetry.addLine("AUTO-AIM: **Limelight No Target**");
        }
    }


    @Override
    public void stop() {
        if (shooterMotor != null) {
            shooterMotor.setPower(0);
        }
    }
}