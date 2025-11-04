package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer; // <-- IMPORT THIS
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer; // <-- THIS IS THE CORRECT IMPORT PATH
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@Autonomous(name = "AutoSpec")
public class SpecimenAuto extends OpMode {
    private Follower follower;
    private int pathState;

    // --- NEW POSES ---
    // We'll keep the original start
    private final Pose startingPose = new Pose(9, 45, Math.toRadians(0));

    // 2 meters = 78.74 inches.
    // Move 78.74 inches forward (in the +X direction)
    private final Pose pose1 = new Pose(9 + 78.74, 45, Math.toRadians(0));

    // From pose1, turn 90 degrees CCW (to face +Y) and move 78.74 inches forward (in the +Y direction)
    private final Pose pose2 = new Pose(87.74, 45 + 78.74, Math.toRadians(90));
    // --- END NEW POSES ---

    // --- NEW PATHS ---
    private Path moveForward;
    private Path turnAndMoveRight;
    // --- END NEW PATHS ---

    Timer pathTimer, opmodeTimer;
    public void buildPaths()
    {
        // Path 1: Move forward 2 meters
        moveForward = new Path(new BezierLine(new Point(startingPose), new Point(pose1)));
        // Keep the heading at 0 degrees
        moveForward.setConstantHeadingInterpolation(startingPose.getHeading());

        // Path 2: Turn to 90 degrees and move 2 meters "right" (which is now "forward" for the robot)
        turnAndMoveRight = new Path(new BezierLine(new Point(pose1), new Point(pose2)));
        // Set the heading to 90 degrees for this entire path
        turnAndMoveRight.setConstantHeadingInterpolation(Math.toRadians(90));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start the first path
                follower.followPath(moveForward);
                setPathState(1);
                break;
            case 1:
                // When the first path is done, start the second path
                if(!follower.isBusy()) {
                    follower.followPath(turnAndMoveRight,true);
                    setPathState(2);
                }
                break;
            case 2:
                // When the second path is done, stop.
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }




    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // --- MODIFIED INITIALIZATION ---

        // 1. Create the PinpointLocalizer, passing in the starting pose.
        Localizer myLocalizer = new PinpointLocalizer(hardwareMap, startingPose);

        // 2. Pass this custom localizer into the Follower constructor.
        // This overrides the default internal IMU localizer.
        follower = new Follower(hardwareMap, myLocalizer, FConstants.class, LConstants.class);

        // --- END OF MODIFICATION ---

        follower.setStartingPose(startingPose);
        buildPaths();
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/

    public void stop() {
    }

}

