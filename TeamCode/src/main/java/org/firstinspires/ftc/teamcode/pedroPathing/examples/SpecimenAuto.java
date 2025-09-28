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
@Autonomous(name = "AutoSpec")
public class SpecimenAuto extends OpMode {
    private Follower follower;
    private int pathState;

    private final Pose startingPose = new Pose(9, 45, Math.toRadians(0));
    private final Pose standPose = new Pose (30,34, startingPose.getHeading());
    private final Pose specimen1Pos1 = new Pose(58, 34, startingPose.getHeading());
    private final Pose specimen1Pos2 = new Pose(58, 23, startingPose.getHeading());
    private final Pose specimen1Pos3 = new Pose(18, 23, startingPose.getHeading());
    private final Pose specimen2Pos1 = new Pose(58, 13, startingPose.getHeading());
    private final Pose specimen2Pos2 = new Pose(18, 13, startingPose.getHeading());
    private final Pose specimen3Pos1 = new Pose(58, 9, startingPose.getHeading());
    private final Pose parkSpecimen = new Pose(15, 9, startingPose.getHeading());
    //specOne control points
    private PathChain forwardspecimen1, sidespecimen1,backspecimen1,forwardspecimen2,sidespecimen2,backspecimen2,forwardspecimen3,sidespecimen3;
    private Path  diagonalspecimen , park;
    Timer pathTimer, opmodeTimer;
    public void buildPaths()
    {
        diagonalspecimen= new Path(new BezierLine(new Point(startingPose), new Point(standPose)));
        forwardspecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(standPose), new Point(specimen1Pos1)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        sidespecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pos1), new Point(specimen1Pos2)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        backspecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pos2), new Point(specimen1Pos3)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        forwardspecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pos3), new Point(specimen1Pos2)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        sidespecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pos2),new Point(specimen2Pos1)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        backspecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2Pos1),new Point(specimen2Pos2)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        forwardspecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2Pos2), new Point(specimen2Pos1)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        sidespecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2Pos1), new Point(specimen3Pos1)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        park =new Path(new BezierLine(new Point(specimen3Pos1), new Point(parkSpecimen)));


        diagonalspecimen.setConstantHeadingInterpolation(startingPose.getHeading());
        park.setConstantHeadingInterpolation(startingPose.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(diagonalspecimen);
                setPathState(1);
                break;
            case 1:


                 if(!follower.isBusy()) {
                       follower.followPath(forwardspecimen1,true);
                    setPathState(2);
                }
                break;
            case 2:
                 if(!follower.isBusy()) {
                     follower.followPath(sidespecimen1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                      follower.followPath(backspecimen1,true);
                    setPathState(4);
                }
                break;
            case 4:
                  if(!follower.isBusy()) {
                      follower.followPath(forwardspecimen2,true);
                    setPathState(5);
                }
                break;
            case 5:
                      if(!follower.isBusy()) {
                            follower.followPath(sidespecimen2,true);
                    setPathState(6);
                }
                break;
            case 6:
                     if(!follower.isBusy()) {
                          follower.followPath(backspecimen2, true);
                    setPathState(7);
                }
                break;
            case 7:
                 if(!follower.isBusy()) {
                        follower.followPath(forwardspecimen3,true);
                    setPathState(8);
                }
                break;
            case 8:
                  if(!follower.isBusy()) {
                        follower.followPath(sidespecimen3,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(park,true);
                    setPathState(10);
                }
                break;
            case 10:
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

