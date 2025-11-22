package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "BlueBackShots")
public class BlueBack extends OpMode {

    private Follower follower;

    // Paths corresponding to requested sequence
    private Path pathA;  // p0→p1→p2→p3
    private Path pathB;  // p3→p4→p5→p6
    private Path pathC;  // p6→p7→p8
    private Path pathD;  // p8→p9

    private DcMotor shooterMotor;
    private final Timer shootTimer = new Timer();// GET RID OF FINAL!!!!!

    /*
     * State machine:
     * 0: Shoot #1 (start)
     * 1: Drive pathA
     * 2: Shoot #2
     * 3: Drive pathB
     * 4: Shoot #3
     * 5: Drive pathC
     * 6: Shoot #4
     * 7: Drive pathD
     * 8: finished
     */
    private int state = 0;
    private boolean shooting = false;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(0));
    private final Pose endPose   = new Pose(50, 63.5, Math.toRadians(0));

    @Override
    public void init() {

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Points
        Point p0 = new Point(startPose);
        Point p1 = new Point(39.5, 35);
        Point p2 = new Point(43, 35.2);
        Point p3 = new Point(70.5, 75.5);
        Point p4 = new Point(37, 59.9);
        Point p5 = new Point(12, 59.5);
        Point p6 = new Point(64, 83.5);
        Point p7 = new Point(16.18, 83);
        Point p8 = new Point(38.3, 115);
        Point p9 = new Point(endPose);

        // Build path A (0→3)
        pathA = new Path(new BezierCurve(p0, p1, p2, p3));
        pathA.setConstantHeadingInterpolation(startPose.getHeading());

        // Build path B (3→6)
        pathB = new Path(new BezierCurve(p3, p4, p5, p6));
        pathB.setConstantHeadingInterpolation(startPose.getHeading());

        // Build path C (6→8)
        pathC = new Path(new BezierCurve(p6, p7, p8));
        pathC.setConstantHeadingInterpolation(startPose.getHeading());

        // Build path D (8→9)
        pathD = new Path(new BezierCurve(p8, p9));
        pathD.setConstantHeadingInterpolation(startPose.getHeading());

        telemetry.addLine("Initialized.");
        telemetry.update();
    }

    @Override
    public void start() {
        state = 0;
        startShooting(); // Shoot #1 immediately
    }

    @Override
    public void loop() {

        follower.update();

        switch (state) {

            case 0: // Shooting #1
                if (shootTimer.getElapsedTimeSeconds() >= 4.0) {
                    stopShooting();
                    follower.followPath(pathA);
                    state = 1;
                }
                break;

            case 1: // Drive pathA
                if (!follower.isBusy()) {
                    startShooting();  // Shoot #2
                    state = 2;
                }
                break;

            case 2: // Shooting #2
                if (shootTimer.getElapsedTimeSeconds() >= 4.0) {
                    stopShooting();
                    follower.followPath(pathB);
                    state = 3;
                }
                break;

            case 3: // Drive pathB
                if (!follower.isBusy()) {
                    startShooting(); // Shoot #3
                    state = 4;
                }
                break;

            case 4: // Shooting #3
                if (shootTimer.getElapsedTimeSeconds() >= 4.0) {
                    stopShooting();
                    follower.followPath(pathC);
                    state = 5;
                }
                break;

            case 5: // Drive pathC
                if (!follower.isBusy()) {
                    startShooting(); // Shoot #4
                    state = 6;
                }
                break;

            case 6: // Shooting #4
                if (shootTimer.getElapsedTimeSeconds() >= 4.0) {
                    stopShooting();
                    follower.followPath(pathD);
                    state = 7;
                }
                break;

            case 7: // Drive pathD
                if (!follower.isBusy()) {
                    state = 8; // done
                }
                break;

            case 8:
                // Finished — idle
                break;
        }

        // Telemetry
        Pose pose = follower.getPose();
        telemetry.addData("State", state);
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", pose.getHeading());
        telemetry.addData("Shooting", shooting);
        telemetry.update();
    }

    private void startShooting() {
        shooterMotor.setPower(1.0);
        shootTimer.resetTimer();
        shooting = true;
    }

    private void stopShooting() {
        shooterMotor.setPower(0.0);
        shooting = false;
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0.0);
    }
}
