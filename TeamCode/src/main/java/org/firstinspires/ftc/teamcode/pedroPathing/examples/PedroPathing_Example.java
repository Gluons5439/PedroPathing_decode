package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Example Auto Blue", group = "Examples")
public class PedroPathing_Example extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        // Create the pose for the forward movement (2 blocks forward)
        Pose forwardPose = new Pose(11, 111, Math.toRadians(270));  // Move 2 blocks forward
        // Create the pose for the left turn (90 degrees left)
        Pose leftTurnPose = new Pose(11, 109, Math.toRadians(180));  // Turn left by 90 degrees

        // Define the paths for moving forward and turning left
        Path forwardPath = new Path(new BezierLine(new Point(startPose), new Point(forwardPose)));
        forwardPath.setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading());

        Path leftTurnPath = new Path(new BezierLine(new Point(forwardPose), new Point(leftTurnPose)));
        leftTurnPath.setLinearHeadingInterpolation(forwardPose.getHeading(), leftTurnPose.getHeading());

        // Define PathChains with the same variable names as in your original code
        grabPickup1 = follower.pathBuilder()
                .addPath(forwardPath)  // Add forward path
                .addPath(leftTurnPath)  // Add left turn path
                .build();

        // The rest of the paths remain the same (grabPickup2, grabPickup3, scorePickup1, etc.)
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        // The park path is unchanged from your original code
        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    // Move forward and then turn left by 2 blocks (grabPickup1 includes both forward and left turn)
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    // Grab sample at pickup1 and proceed to score it
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    // Grab second sample
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    // Score second sample
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // Grab third sample
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    // Score third sample
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    // Park robot after scoring all samples
                    follower.followPath(park, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    // Stop running any paths once everything is completed
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is called when we start the auto. */
    @Override
    public void init() {
        // Initialize the follower, path builder, and timers
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        // Build paths when the program initializes
        buildPaths();

        // Set the initial path state
        setPathState(0);
    }

    @Override
    public void loop() {
        autonomousPathUpdate();
    }
}
