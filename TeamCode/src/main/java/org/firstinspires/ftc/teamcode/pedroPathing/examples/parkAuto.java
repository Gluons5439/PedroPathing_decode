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
@Autonomous(name = "AutoPark")
public class parkAuto extends OpMode {
    private Follower follower;
    private int pathState;

    private final Pose startingPose = new Pose(9, 110, Math.toRadians(0));
    private final Pose midPose = new Pose(60, 110, startingPose.getHeading());
    private final Pose park = new Pose(60, 100, startingPose.getHeading());
    private Path forwardPark, sidePark;
    Timer pathTimer, opmodeTimer;
    public void buildPaths()
    {
        forwardPark= new Path(new BezierLine(new Point(startingPose), new Point(midPose)));

        sidePark =new Path(new BezierLine(new Point(midPose), new Point(park)));


        forwardPark.setConstantHeadingInterpolation(startingPose.getHeading());
        sidePark.setConstantHeadingInterpolation(startingPose.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(forwardPark);
                setPathState(1);
                break;
            case 1:


                if(!follower.isBusy()) {
                    follower.followPath(sidePark,true);
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

