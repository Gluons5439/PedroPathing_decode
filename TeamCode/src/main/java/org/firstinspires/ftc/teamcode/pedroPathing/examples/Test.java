/**
 *

package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


public class Test {
    public static Pose startingPose = new Pose(15.5, 61.75, Math.toRadians(270));
    public static PathChain pushBotAuto;

    public static void main(String[] args) {


        PedroPathingBotEntity robot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 17.5)
                .setConstraints(70, -50, -25, -60, 3)
                .build();

        pushBotAuto = robot.getDrive().pathBuilder()
                // 1. Bezier curve to (56, 0)
                .addPath(new Path(new BezierCurve(
                        new Point(15.5, 61.75, Point.CARTESIAN), // Start
                        new Point(30, 45, Point.CARTESIAN),      // Control point 1
                        new Point(45, 20, Point.CARTESIAN),      // Control point 2
                        new Point(56, 0, Point.CARTESIAN)        // End at bottom right
                )))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))

                // 2. Back up from (56, 0) slightly (e.g., to y = 5)
                .addPath(new Path(new BezierLine(
                        new Point(56, 0, Point.CARTESIAN),
                        new Point(56, 61.75, Point.CARTESIAN)
                )))
                .setConstantHeadingInterpolation(Math.toRadians(90)) // Still facing up

                // 3. Push block left across the field
                .addPath(new Path(new BezierLine(
                        new Point(56, 61.75, Point.CARTESIAN),
                        new Point(-50, 61.75, Point.CARTESIAN)
                )))
                .setConstantHeadingInterpolation(Math.toRadians(90)) // Face up while pushing

                .build();

        robot.followPath(pushBotAuto);

        // Load field background image
        Image img = null;
        try {
            img = ImageIO.read(new File("C:/Users/mhsgl/RoboticsGluons25-26/field-2024-juice-dark.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }








        // Start simulation
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}
 */
package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "PushBotAuto", group = "PedroPathing")
public class Test extends OpMode {

    private Follower follower;
    private Timer timer;

    public static Pose startingPose = new Pose(15.5, 61.75, Math.toRadians(270));
    public static PathChain pushBotAuto;

    @Override
    public void init() {
        // Initialize PedroPathing Follower with your drivetrain settings
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setPose(startingPose);

        // Define the path chain
        pushBotAuto = follower.pathBuilder()
                // 1. Curve to (56, 0)
                .addPath(new Path(new BezierCurve(
                        new Point(15.5, 61.75, Point.CARTESIAN),
                        new Point(30, 45, Point.CARTESIAN),
                        new Point(45, 20, Point.CARTESIAN),
                        new Point(56, 0, Point.CARTESIAN)
                )))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))

                // 2. Back up (simulate pulling block back)
                .addPath(new Path(new BezierLine(
                        new Point(56, 0, Point.CARTESIAN),
                        new Point(56, 61.75, Point.CARTESIAN)
                )))
                .setConstantHeadingInterpolation(Math.toRadians(90))

                // 3. Push left
                .addPath(new Path(new BezierLine(
                        new Point(56, 61.75, Point.CARTESIAN),
                        new Point(-50, 61.75, Point.CARTESIAN)
                )))
                .setConstantHeadingInterpolation(Math.toRadians(90))

                .build();

        follower.followPath(pushBotAuto, true);

        timer = new Timer();
        timer.resetTimer();
    }

    @Override
    public void loop() {
        // Update follower based on time
        // dt = timer.update();
        follower.update();

        // Send telemetry
        Pose pose = follower.getPose();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
        //telemetry.addData("Finished", follower.isFinished());
        telemetry.update();
    }
}

