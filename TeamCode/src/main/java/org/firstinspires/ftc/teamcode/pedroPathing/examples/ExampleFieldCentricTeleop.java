/**
 *

package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and field-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024


@TeleOp(name = "Example Field-Centric Teleop", group = "Examples")
public class ExampleFieldCentricTeleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
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



        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
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

@TeleOp(name = "Example Field-Centric TeleOp with Turret Auto-Aim", group = "Examples")
public class ExampleFieldCentricTeleop extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    private Servo turretServo1;
    private Servo turretServo2;

    private double turretTargetPosition = 0.5;

    private DcMotor shooterMotor;
    private static final double SHOOTER_MAX_POWER = 1.0;

    private Limelight3A limelight;

    private static final double AIM_KP = 0.0008;
    private static final double AIM_DEADZONE = 1.5;
    private static final double MAX_CORRECTION = 0.001;

    // --- NEW: Toggle variables ---
    private boolean turretEnabled = false;  // turret starts OFF
    private boolean b_was_pressed = false;

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
            telemetry.addData("Error", "Could not find turret servos.");
        }

        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Shooter", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Shooter not found.");
        }

        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.pipelineSwitch(0);
            telemetry.addData("Limelight", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Limelight not found.");
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
                false  // FIELD CENTRIC
        );
        follower.update();



        boolean b_is_pressed = gamepad2.b;
        if (b_is_pressed && !b_was_pressed) {
            turretEnabled = !turretEnabled;

            // Auto-Aim turns off when turret disabled
            if (!turretEnabled) {
                isAutoAimActive = false;
            }
        }
        b_was_pressed = b_is_pressed;




        boolean a_is_pressed = gamepad2.a;
        if (turretEnabled && a_is_pressed && !a_was_pressed) {
            isAutoAimActive = !isAutoAimActive;
        }
        a_was_pressed = a_is_pressed;



        if (turretEnabled) {
            if (isAutoAimActive) {
                handleAutoAim();
            } else {
                handleTurretControl();
            }
        }

        turretTargetPosition = Range.clip(turretTargetPosition, 0.0, 1.0);
        if (turretServo1 != null && turretServo2 != null) {
            turretServo1.setPosition(turretTargetPosition);
            turretServo2.setPosition(turretTargetPosition);
        }



        handleShooterControl();



        telemetry.addData("Turret Enabled (B)", turretEnabled);
        telemetry.addData("Auto-Aim (A)", isAutoAimActive);
        telemetry.addData("Turret Position", turretTargetPosition);

        telemetry.addData("Shooter Power", shooterMotor != null ? shooterMotor.getPower() : 0);

        telemetry.update();
    }


    //
    // ---------- SHOOTER CONTROL ----------
    //
    private void handleShooterControl() {
        if (shooterMotor == null) return;

        if (gamepad2.right_bumper) {
            shooterMotor.setPower(SHOOTER_MAX_POWER);
        } else {
            shooterMotor.setPower(0.0);
        }
    }

    //
    // ---------- MANUAL TURRET CONTROL ----------
    //
    private void handleTurretControl() {
        double turretMoveInput = gamepad2.left_stick_x;
        turretTargetPosition = (turretMoveInput + 1.0) / 2.0;
    }

    //
    // ---------- AUTO-AIM USING LIMELIGHT ----------
    //
    private void handleAutoAim() {
        if (limelight == null) {
            telemetry.addLine("AUTO-AIM: Limelight not initialized!");
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            telemetry.addData("Target X (°)", tx);

            if (Math.abs(tx) > AIM_DEADZONE) {

                double correction = Range.clip(tx * AIM_KP, -MAX_CORRECTION, MAX_CORRECTION);
                turretTargetPosition -= correction;

                telemetry.addLine("AUTO-AIM: Tracking...");
                telemetry.addData("Correction", correction);

            } else {
                telemetry.addLine("AUTO-AIM: Locked on!");
            }

        } else {
            telemetry.addLine("AUTO-AIM: No target");
        }
    }


    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0);
    }
}
