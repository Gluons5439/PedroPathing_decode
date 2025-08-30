package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@Autonomous(name = "AutoBlue")
public class SpecimenAuto extends OpMode {
    private Follower follower;
    private int pathState;

    private final Pose startingPose = new Pose(9, 40, Math.toRadians(0));
    private final Pose specimen1Pos1 = new Pose(60, 40, startingPose.getHeading());
    private final Pose specimen1Pos2 = new Pose(60, 21, startingPose.getHeading());
    private final Pose specimen1Pos3 = new Pose(20, 21, startingPose.getHeading());
    private final Pose specimen2Pos1 = new Pose(60, 10, startingPose.getHeading());
    private final Pose specimen2Pos2 = new Pose(20, 10, startingPose.getHeading());
    private final Pose specimen3Pos1 = new Pose(60, 5, startingPose.getHeading());
    private final Pose parkSpecimen = new Pose(15, 5, startingPose.getHeading());
    //specOne control points
    private PathChain sidespecimen1,backspecimen1,forwardspecimen2,sidespecimen2,backspecimen2,forwardspecimen3,sidespecimen3;
    private Path forwardspecimen1, park;
    Timer pathTimer, opmodeTimer;
    public void buildPaths()
    {
        forwardspecimen1= new Path(new BezierLine(new Point(startingPose), new Point(specimen1Pos1)));
        sidespecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pos1), new Point(specimen1Pos2)))
                .build();
        backspecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pos2), new Point(specimen1Pos3)))
                .build();
        forwardspecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pos3), new Point(specimen1Pos2)))
                .build();
        sidespecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pos2),new Point(specimen2Pos1)))
                .build();
        backspecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2Pos1),new Point(specimen2Pos2)))
                .build();
        forwardspecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2Pos2), new Point(specimen2Pos1)))
                .build();
        sidespecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2Pos1), new Point(specimen3Pos1)))
                .build();
        park =new Path(new BezierLine(new Point(specimen3Pos1), new Point(parkSpecimen)));


        forwardspecimen1.setConstantHeadingInterpolation(startingPose.getHeading());

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(forwardspecimen1);
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
                    follower.followPath(sidespecimen1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(backspecimen1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(forwardspecimen2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(sidespecimen2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(backspecimen2,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(forwardspecimen3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(sidespecimen3,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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

