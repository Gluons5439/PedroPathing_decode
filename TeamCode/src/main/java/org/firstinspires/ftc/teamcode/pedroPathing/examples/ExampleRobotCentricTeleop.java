/**
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

@TeleOp(name = "RobotCentricDrive(TT)", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    private Servo turretServo1;
    private Servo turretServo2;
    private double turretTargetPosition = 0.5;

    private DcMotor shooterMotor;
    private static final double SHOOTER_MAX_POWER = -1.0;

    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = -1.0;

    private Limelight3A limelight;

    private static final int TARGET_TAG_ID = 20;

    private static final double AIM_KP = 0.0008;
    private static final double AIM_DEADZONE = 1.5;
    private static final double MAX_CORRECTION = 0.001;

    private boolean isAutoAimActive = false;
    private boolean a_was_pressed = false;

    // NEW shooter toggle variables
    private boolean shooterActive = false;
    private boolean b_was_pressed = false;

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
            telemetry.addData("Error", "Could not find turret servos.");
        }

        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Shooter", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find shooter.");
        }

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Intake", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find Intake motor.");
        }

        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.pipelineSwitch(0);
            telemetry.addData("Limelight", "Initialized on Pipeline 0");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find Limelight 3A.");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        if (limelight != null) limelight.start();
    }

    @Override
    public void loop() {

        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
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
        handleIntakeControl();

        telemetry.addData("Turret Mode", isAutoAimActive ? "AUTO-AIM" : "MANUAL");
        telemetry.addData("Turret Pos", "%.3f", turretTargetPosition);
        telemetry.addData("Shooter Active", shooterActive);
        telemetry.update();
    }


    private void handleShooterControl() {
        if (shooterMotor == null) return;

        boolean b_is_pressed = gamepad2.b;

        if (b_is_pressed && !b_was_pressed) {
            shooterActive = !shooterActive;
        }
        b_was_pressed = b_is_pressed;

        shooterMotor.setPower(shooterActive ? SHOOTER_MAX_POWER : 0.0);
    }


    private void handleIntakeControl() {
        if (intakeMotor == null) return;

        if (gamepad1.right_bumper) {
            intakeMotor.setPower(INTAKE_POWER);  // Intake in (-1)
        }
        else if (gamepad1.left_bumper) {
            intakeMotor.setPower(0.25);          // Intake out
        }
        else {
            intakeMotor.setPower(0);             // Stop
        }
    }

    private void handleTurretControl() {
        double turretMoveInput = gamepad2.left_stick_x;
        turretTargetPosition = (turretMoveInput + 1.0) / 2.0;
    }

    private void handleAutoAim() {
        if (limelight == null) {
            telemetry.addLine("AUTO-AIM: Limelight not initialized!");
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            telemetry.addData("Target X", "%.2f°", tx);

            if (Math.abs(tx) > AIM_DEADZONE) {
                double correction = Range.clip(tx * AIM_KP, -MAX_CORRECTION, MAX_CORRECTION);
                turretTargetPosition -= correction;

                telemetry.addLine("Auto-Aim: Tracking");
            } else {
                telemetry.addLine("Auto-Aim: Locked");
            }

        } else {
            telemetry.addLine("Auto-Aim: No Target");
        }
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
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

@TeleOp(name = "RobotCentricDrive(TT)", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    // -------------------------------------
    // TURRET
    // -------------------------------------
    private Servo turretServo1;
    private Servo turretServo2;
    private double turretTargetPosition = 0.5;

    // -------------------------------------
    // SHOOTER
    // -------------------------------------
    private DcMotor shooterMotor;
    private static final double SHOOTER_MAX_POWER = -1.0;
    private boolean shooterActive = false;
    private boolean b_was_pressed = false;

    // -------------------------------------
    // INTAKE
    // -------------------------------------
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = -1.0;

    // -------------------------------------
    // TRANSFER POSITION SERVOS
    // -------------------------------------
    private Servo transferServo1;
    private Servo transferServo2;

    // UP = into turret, DOWN = reset
    private static final double TRANSFER_UP_POS = 1.0;
    private static final double TRANSFER_DOWN_POS = 0.0;

    // -------------------------------------
    // LIMELIGHT AUTO-AIM
    // -------------------------------------
    private Limelight3A limelight;
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

        // ----------------------------
        // TURRET
        // ----------------------------
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
            turretServo1.setPosition(turretTargetPosition);
            turretServo2.setPosition(turretTargetPosition);
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find turret servos.");
        }

        // ----------------------------
        // SHOOTER
        // ----------------------------
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find shooter motor.");
        }

        // ----------------------------
        // INTAKE
        // ----------------------------
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find intake motor.");
        }

        // ----------------------------
        // TRANSFER POSITION SERVOS
        // ----------------------------
        try {
            transferServo1 = hardwareMap.get(Servo.class, "Transfer1");
            transferServo2 = hardwareMap.get(Servo.class, "Transfer2");

            transferServo1.setPosition(TRANSFER_DOWN_POS);
            transferServo2.setPosition(TRANSFER_DOWN_POS);

            telemetry.addData("Transfer", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find transfer servos.");
        }

        // ----------------------------
        // LIMELIGHT
        // ----------------------------
        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.pipelineSwitch(0);
            telemetry.addData("Limelight", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find Limelight.");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        if (limelight != null) limelight.start();
    }

    @Override
    public void loop() {

        // ----------------------------
        // DRIVE
        // ----------------------------
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

        // ----------------------------
        // A → toggle auto aim
        // ----------------------------
        boolean a_is_pressed = gamepad2.a;
        if (a_is_pressed && !a_was_pressed)
            isAutoAimActive = !isAutoAimActive;
        a_was_pressed = a_is_pressed;

        if (isAutoAimActive) handleAutoAim();
        else handleTurretControl();

        turretTargetPosition = Range.clip(turretTargetPosition, 0, 1);
        if (turretServo1 != null && turretServo2 != null) {
            turretServo1.setPosition(turretTargetPosition);
            turretServo2.setPosition(turretTargetPosition);
        }

        // ----------------------------
        // SUBSYSTEMS
        // ----------------------------
        handleShooterControl();     // B toggle
        handleTransferControl();    // NEW

        telemetry.addData("Turret Mode", isAutoAimActive ? "AUTO-AIM" : "MANUAL");
        telemetry.addData("Turret Pos", turretTargetPosition);
        telemetry.addData("Shooter Active", shooterActive);
        telemetry.update();
    }

    // ----------------------------
    // SHOOTER (B TOGGLE)
    // ----------------------------
    private void handleShooterControl() {
        if (shooterMotor == null) return;

        boolean b_is_pressed = gamepad2.b;

        if (b_is_pressed && !b_was_pressed)
            shooterActive = !shooterActive;

        b_was_pressed = b_is_pressed;

        shooterMotor.setPower(shooterActive ? SHOOTER_MAX_POWER : 0);
    }


    private void handleTransferControl() {
        if (transferServo1 == null || transferServo2 == null) return;

        boolean LB = gamepad2.left_bumper;
        boolean RB = gamepad2.right_bumper;

        // BOTH bumpers raise transfer to UP
        if (LB || RB) {
            transferServo1.setPosition(TRANSFER_UP_POS);
            transferServo2.setPosition(TRANSFER_DOWN_POS);

            // Only RB turns intake ON
            if (RB && intakeMotor != null) {
                intakeMotor.setPower(INTAKE_POWER);
            } else if (intakeMotor != null) {
                intakeMotor.setPower(0);
            }

            return;

        }

        // Neither bumper → go DOWN + stop intake
        transferServo1.setPosition(TRANSFER_DOWN_POS);
        transferServo2.setPosition(TRANSFER_DOWN_POS);

        if (intakeMotor != null)
            intakeMotor.setPower(0);
    }


    private void handleTurretControl() {
        double input = gamepad2.left_stick_x;
        turretTargetPosition = (input + 1) / 2.0;
    }


    private void handleAutoAim() {
        if (limelight == null) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addLine("Auto-Aim: No Target");
            return;
        }

        double tx = result.getTx();
        telemetry.addData("Target X", tx);

        if (Math.abs(tx) > AIM_DEADZONE) {
            double correction = Range.clip(tx * AIM_KP, -MAX_CORRECTION, MAX_CORRECTION);
            turretTargetPosition -= correction;
            telemetry.addLine("Auto-Aim: Tracking");
        } else {
            telemetry.addLine("Auto-Aim: Locked");
        }
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
    }
}

