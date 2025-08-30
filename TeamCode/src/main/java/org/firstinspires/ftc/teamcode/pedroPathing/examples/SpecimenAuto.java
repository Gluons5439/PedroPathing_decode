package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

public class SpecimenAuto {
    private Follower follower;
    private int pathState;

    private final Pose startingPose = new Pose(9, 40, Math.toRadians(0));
    //specOne control points
    private PathChain sidespecimen1,backspecimen1,forwardspecimen2,sidespecimen2,backspecimen2,forwardspecimen3,sidespecimen3,park;
    private Path forwardspecimen1;
    Timer pathTimer, opmodeTimer;
    public void buildPaths()
    {
        forwardspecimen1= follower.pathBuilder()
                        .addPath(new BezierLine(new Point(startingPose), new Point(specimen1Pos1))
                                        .build());

        forwardspecimen1.setConstantHeadingInterpolation(startingPose.getHeading())

        pushSpecimen=follower.pathBuilder()
                .addPath(new BezierLine(new Point(specOne), new Point(pushSpecOne)))
                .build();
        goToEndPose=new Path(new BezierLine(new Point(pushSpecOne),new Point(endPose)));
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(frontSpecOne);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushSpecimen,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(goToEndPose,true);
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

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
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
