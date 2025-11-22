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
    // TRANSFER POSITION SERVOS (opposite to each other)
    // -------------------------------------
    private Servo transferServo1;
    private Servo transferServo2;
    private static final double TRANSFER_UP_POS = 0.6;    // up position
    private static final double TRANSFER_DOWN_POS = 0.2;  // down position

    // ----------------------------
    // TRANSFER SEQUENCE STATE
    // ----------------------------
    private boolean lbSequenceActive = false;
    private boolean rbSequenceActive = false;
    private boolean lb_was_pressed = false;
    private boolean rb_was_pressed = false;
    private int rbStep = 0;
    private long rbTimer = 0;
    private long lbTimer = 0;
    private static final long INITIAL_WAIT_TIME = 900;        // 0.9 seconds wait before moving
    private static final long BALL_MOVE_TIME = 900;           // ms per ball
    private static final long INTERVAL_BETWEEN_BALLS = 900;   // ms between balls

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
        // TRANSFER SERVOS
        // ----------------------------
        try {
            transferServo1 = hardwareMap.get(Servo.class, "Transfer1");
            transferServo2 = hardwareMap.get(Servo.class, "Transfer2");

            // Initialize servos in opposite positions (down)
            transferServo1.setPosition(TRANSFER_DOWN_POS);
            transferServo2.setPosition(1.0 - TRANSFER_DOWN_POS);

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
        handleTransferControl();    // LB/RB transfer sequence

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

    // ----------------------------
    // TRANSFER + INTAKE SEQUENCE
    // ----------------------------
    private void handleTransferControl() {
        if (transferServo1 == null || transferServo2 == null) return;

        boolean LB = gamepad2.left_bumper;
        boolean RB = gamepad2.right_bumper;
        long currentTime = System.currentTimeMillis();

        // ------------------------
        // LB → move ball to top of turret (one ball)
        // ------------------------
        if (LB && !lb_was_pressed && !lbSequenceActive && !rbSequenceActive) {
            lbSequenceActive = true;
            lbTimer = currentTime;
            // Servos opposite: servo1 up, servo2 opposite (1.0 - position) to push ball upwards
            transferServo1.setPosition(TRANSFER_UP_POS);
            transferServo2.setPosition(1.0 - TRANSFER_UP_POS);
            if (intakeMotor != null) intakeMotor.setPower(0); // intake OFF
        }
        lb_was_pressed = LB;

        if (lbSequenceActive) {
            if (currentTime - lbTimer >= BALL_MOVE_TIME) {
                // Reset servos to down position (opposite: servo1 down, servo2 opposite)
                transferServo1.setPosition(TRANSFER_DOWN_POS);
                transferServo2.setPosition(1.0 - TRANSFER_DOWN_POS);
                lbSequenceActive = false;
            }
            return;
        }

        // ------------------------
        // RB → wait 0.9s, then move last 2 balls with intake ON
        // ------------------------
        if (RB && !rb_was_pressed && !rbSequenceActive && !lbSequenceActive) {
            rbSequenceActive = true;
            rbStep = 0;
            rbTimer = currentTime;
            // Turn intake ON immediately
            if (intakeMotor != null) intakeMotor.setPower(INTAKE_POWER);
            // Servos start in down position (waiting) - opposite positions
            transferServo1.setPosition(TRANSFER_DOWN_POS);
            transferServo2.setPosition(1.0 - TRANSFER_DOWN_POS);
        }
        rb_was_pressed = RB;

        if (rbSequenceActive) {
            long elapsed = currentTime - rbTimer;

            // Wait 0.9 seconds before starting
            if (elapsed < INITIAL_WAIT_TIME) {
                // Still waiting, keep servos in down position (opposite)
                transferServo1.setPosition(TRANSFER_DOWN_POS);
                transferServo2.setPosition(1.0 - TRANSFER_DOWN_POS);
                return;
            }

            // After wait, move 2 balls
            long timeSinceStart = elapsed - INITIAL_WAIT_TIME;

            if (rbStep < 2) {
                // Calculate which ball we're on
                int ballNumber = rbStep;
                long ballStartTime = ballNumber * INTERVAL_BETWEEN_BALLS;
                long ballEndTime = ballStartTime + BALL_MOVE_TIME;

                if (timeSinceStart >= ballStartTime && timeSinceStart < ballEndTime) {
                    // Move this ball: servos opposite to push upwards
                    transferServo1.setPosition(TRANSFER_UP_POS);
                    transferServo2.setPosition(1.0 - TRANSFER_UP_POS);
                } else if (timeSinceStart >= ballEndTime) {
                    // This ball is done, prepare for next - reset to down (opposite)
                    transferServo1.setPosition(TRANSFER_DOWN_POS);
                    transferServo2.setPosition(1.0 - TRANSFER_DOWN_POS);
                    if (timeSinceStart >= ballEndTime + 100) { // Small delay between balls
                        rbStep++;
                    }
                }
            } else {
                // Done with both balls - reset to down (opposite)
                transferServo1.setPosition(TRANSFER_DOWN_POS);
                transferServo2.setPosition(1.0 - TRANSFER_DOWN_POS);
                if (intakeMotor != null) intakeMotor.setPower(0);
                rbSequenceActive = false;
            }
            return;
        }

        // ------------------------
        // default idle - only set if not in a sequence
        // ------------------------
        if (!lbSequenceActive && !rbSequenceActive) {
            // Servos in opposite down positions
            transferServo1.setPosition(TRANSFER_DOWN_POS);
            transferServo2.setPosition(1.0 - TRANSFER_DOWN_POS);
            if (intakeMotor != null) intakeMotor.setPower(0);
        }
    }

    // ----------------------------
    // MANUAL TURRET CONTROL
    // ----------------------------
    private void handleTurretControl() {
        double input = gamepad2.left_stick_x;
        turretTargetPosition = (input + 1) / 2.0;
    }

    // ----------------------------
    // AUTO AIM
    // ----------------------------
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
        if (transferServo1 != null) transferServo1.setPosition(TRANSFER_DOWN_POS);
        if (transferServo2 != null) transferServo2.setPosition(1.0 - TRANSFER_DOWN_POS);
    }
}
