/**
 *

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
@Autonomous(name = "AutoSample")
public class SampleAuto extends OpMode {
    private Follower follower;
    private int pathState;

    private final Pose startingPose = new Pose(9, 105, Math.toRadians(0));
    private final Pose sample1Pos1 = new Pose(60, 110, startingPose.getHeading());
    private final Pose sample1Pos2 = new Pose(60, 120, startingPose.getHeading());
    private final Pose sample1Pos3 = new Pose(10, 120, startingPose.getHeading());
    private final Pose sample2Pos1 = new Pose(70, 130, startingPose.getHeading());
    private final Pose sample2Pos2 = new Pose(15, 130, startingPose.getHeading());
    private final Pose sample3Pos1 = new Pose(70, 135, startingPose.getHeading());
    private final Pose parkSample = new Pose(25, 135, startingPose.getHeading());
    //specOne control points
    private PathChain sidesample1,backsample1,forwardsample2,sidesample2,backsample2,forwardsample3,sidesample3;
    private Path forwardsample1, backsample3;
    Timer pathTimer, opmodeTimer;
    public void buildPaths()
    {
        forwardsample1= new Path(new BezierLine(new Point(startingPose), new Point(sample1Pos1)));
        sidesample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pos1), new Point(sample1Pos2)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        backsample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pos2), new Point(sample1Pos3)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        forwardsample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pos3), new Point(sample1Pos2)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        sidesample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pos2),new Point(sample2Pos1)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        backsample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2Pos1),new Point(sample2Pos2)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        forwardsample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2Pos2), new Point(sample2Pos1)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        sidesample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2Pos1), new Point(sample3Pos1)))
                .setConstantHeadingInterpolation(startingPose.getHeading())
                .build();
        backsample3 =new Path(new BezierLine(new Point(sample3Pos1), new Point(parkSample)));


        forwardsample1.setConstantHeadingInterpolation(startingPose.getHeading());
        backsample3.setConstantHeadingInterpolation(startingPose.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(forwardsample1);
                setPathState(1);
                break;
            case 1:


                if(!follower.isBusy()) {
                    follower.followPath(sidesample1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(backsample1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(forwardsample2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(sidesample2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(backsample2,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(forwardsample3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(sidesample3,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(backsample3,true);
                    setPathState(9);
                }
                break;
            case 9:
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


    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    public void stop() {
    }

}
*/

/**
package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "AutoSample")
public class SampleAuto extends OpMode {

    private Follower follower;
    private Path bezierPath;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose endPose = new Pose(50, 20, Math.toRadians(0));

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        bezierPath = new Path(new BezierCurve(new Point(startPose), new Point(endPose)));
        bezierPath.setConstantHeadingInterpolation(startPose.getHeading());
    }
    @Override
    public void start() {
        follower.followPath(bezierPath);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
*/

    package org.firstinspires.ftc.teamcode.pedroPathing.examples;

    import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
    import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

    import com.pedropathing.follower.Follower;
    import com.pedropathing.localization.Pose;
    import com.pedropathing.pathgen.BezierCurve;
    import com.pedropathing.pathgen.BezierLine;
    import com.pedropathing.pathgen.Path;
    import com.pedropathing.pathgen.Point;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;

    import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
    import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

 @Autonomous(name = "AutoSample")
 public class SampleAuto extends OpMode {

 private Follower follower;
 private Path bezierPath;

 private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
 private final Pose endPose = new Pose(60, 30, Math.toRadians(0));

 @Override
 public void init() {
 follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
 follower.setStartingPose(startPose);

 // Define control points for an actual Bezier curve
 Point p0 = new Point(startPose);                        // Start
 Point p1 = new Point(30, 60);                           // Control Point 1
 Point p2 = new Point(45, 0);                            // Control Point 2
 Point p3 = new Point(endPose);                          // End

 bezierPath = new Path(new BezierCurve(p0, p1, p2, p3));  // Cubic Bezier curve
 bezierPath.setConstantHeadingInterpolation(startPose.getHeading());
 }

 @Override
 public void start() {
 follower.followPath(bezierPath);
 }

 @Override
 public void loop() {
 follower.update();

 telemetry.addData("x", follower.getPose().getX());
 telemetry.addData("y", follower.getPose().getY());
 telemetry.addData("heading", follower.getPose().getHeading());
 telemetry.update();
 }
 }


