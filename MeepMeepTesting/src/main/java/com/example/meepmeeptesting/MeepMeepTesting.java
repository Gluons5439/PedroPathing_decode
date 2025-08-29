/*
package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.pedropathing.DefaultBotBuilder;
import com.noahbres.meepmeep.pedropathing.entity.PedroPathingBotEntity;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.*;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    public static PathChain middleSpikeMark;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        PedroPathingBotEntity robot = new DefaultBotBuilder(meepMeep)
                // Set this to the length and width of your robot
                .setDimensions(15, 17.5)
                // Set this based on your follower constants for PedroPathing
                // (xMovement, yMovement, forwardZeroPowerAcceleration, lateralZeroPowerAcceleration, zeroPowerAccelerationMultiplier)
                .setConstraints(70, -50, -25, -60, 3)
                .build();

        middleSpikeMark = robot.getDrive().pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startingPose), new Point(12, 37.5, Point.CARTESIAN))))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(220))
                .addPath(new Path(new BezierLine(new Point(12, 37.5, Point.CARTESIAN), new Point(52.5, 27, Point.CARTESIAN))))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.PI, 0.7)
                .addPath(new BezierLine(
                        new Point(52.5, 27, Point.CARTESIAN),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .addParametricCallback(0.5, () -> {})
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-65, 11.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierCurve(
                        new Point(-65, 11.5, Point.CARTESIAN),
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(52.2, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(
                        new Point(52.2, 28, Point.CARTESIAN),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-65, 11.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierCurve(
                        new Point(-65, 11.5, Point.CARTESIAN),
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(52.2, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(
                        new Point(52.2, 28, Point.CARTESIAN),
                        new Point(30, 10.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(
                        new Point(30, 10.5, Point.CARTESIAN),
                        new Point(-65, 11.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addParametricCallback(0.8, () -> {
                })
                .addPath(new BezierCurve(
                        new Point(-65, 11.5, Point.CARTESIAN),
                        new Point(-40, 10.5, Point.CARTESIAN),
                        new Point(37, 10.5, Point.CARTESIAN),
                        new Point(52.2, 28, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new Path(new BezierLine(new Point(52.2, 28, Point.CARTESIAN), new Point(47, 35, Point.CARTESIAN))))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        robot.followPath(middleSpikeMark);
        Image img = null;
        try { img = ImageIO.read(new File("C:/Users/mhsgl/RoboticsGluons25-26/field-2024-juice-dark.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}

 ////ASASDSASERTSD
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.pedropathing.DefaultBotBuilder;
import com.noahbres.meepmeep.pedropathing.entity.PedroPathingBotEntity;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.*;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static Pose2d startingPose = new Pose2d(0, 56, Math.toRadians(270)); // Starting position at (0, 56)
    public static PathChain robotPath;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        PedroPathingBotEntity robot = new DefaultBotBuilder(meepMeep)
                // Set robot dimensions (width, length)
                .setDimensions(15, 17.5)
                // Set constraints (max speed, acceleration, etc.)
                .setConstraints(70, -50, -25, -60, 3)
                .build();

        // Define the path for the robot to move from (0, 56) to (56, 0), return, and then move right
        robotPath = robot.getDrive().pathBuilder()
                // First segment: move from (0, 56) to (56, 0) with a Bezier curve
                .addPath(new Path(new BezierCurve(
                        new Point(startingPose.getX(), startingPose.getY(), Point.CARTESIAN), // Start point (0, 56)
                        new Point(28, 28, Point.CARTESIAN), // Control points for a smooth curve
                        new Point(42, 14, Point.CARTESIAN), // Another control point
                        new Point(56, 0, Point.CARTESIAN) // End point (56, 0)
                )))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(90)) // Heading to follow the curve

                // Second segment: return the same way from (56, 0) back to (0, 56)
                .addPath(new BezierCurve(
                        new Point(56, 0, Point.CARTESIAN), // Starting point (56, 0)
                        new Point(42, 14, Point.CARTESIAN), // Control points for smooth return
                        new Point(28, 28, Point.CARTESIAN),
                        new Point(startingPose.getX(), startingPose.getY(), Point.CARTESIAN) // End point (0, 56)
                ))
                .setConstantHeadingInterpolation(startingPose.getHeading())

                // Third segment: move to the right side of the field (e.g., (100, 10) on the right)
                .addPath(new BezierLine(
                        new Point(startingPose.getX(), startingPose.getY(), Point.CARTESIAN), // Start point (0, 56)
                        new Point(100, 10, Point.CARTESIAN) // End point on the far right (100, 10)
                ))
                .setConstantHeadingInterpolation(Math.PI) // Move right horizontally

                // Build the entire path chain
                .build();

        // Follow the defined path
        robot.followPath(robotPath);

        // Load the field background image
        Image img = null;
        try {
            img = ImageIO.read(new File("C:/Users/mhsgl/RoboticsGluons25-26/field-2024-juice-dark.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        // Set up and start the MeepMeep simulation
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot) // Add the robot to the simulation
                .start();
    }
}
*/
/**

package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.pedropathing.DefaultBotBuilder;
import com.noahbres.meepmeep.pedropathing.entity.PedroPathingBotEntity;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.*;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    public static PathChain middleSpikeMark;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        PedroPathingBotEntity robot = new DefaultBotBuilder(meepMeep)
                // Set this to the length and width of your robot
                .setDimensions(15, 17.5)
                // Set this based on your follower constants for PedroPathing
                // (xMovement, yMovement, forwardZeroPowerAcceleration, lateralZeroPowerAcceleration, zeroPowerAccelerationMultiplier)
                .setConstraints(70, -50, -25, -60, 3)
                .build();

        // Define the PushBot-style autonomous routine
        middleSpikeMark = robot.getDrive().pathBuilder()
                // 1. Push forward toward the block
                .addPath(new Path(new BezierLine(
                        new Point(startingPose.getX(), startingPose.getY(), Point.CARTESIAN),
                        new Point(15.5, 37.5, Point.CARTESIAN)
                )))
                .setConstantHeadingInterpolation(Math.toRadians(270)) // Face forward

                // 2. Push backward, pulling the block back
                .addPath(new Path(new BezierLine(
                        new Point(15.5, 37.5, Point.CARTESIAN),
                        new Point(15.5, 61.75, Point.CARTESIAN)
                )))
                .setConstantHeadingInterpolation(Math.toRadians(270)) // Still face forward

                // 3. Push the block all the way to the left side of the field
                .addPath(new Path(new BezierLine(
                        new Point(15.5, 61.75, Point.CARTESIAN),
                        new Point(-60, 61.75, Point.CARTESIAN)
                )))
                .setConstantHeadingInterpolation(Math.toRadians(270)) // Still face forward

                .build();

        // Tell the robot to follow the defined path
        robot.followPath(middleSpikeMark);

        // Load the field background image
        Image img = null;
        try {
            img = ImageIO.read(new File("C:/Users/mhsgl/RoboticsGluons25-26/field-2024-juice-dark.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        // Start MeepMeep simulation
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}
*/


package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.pedropathing.DefaultBotBuilder;
import com.noahbres.meepmeep.pedropathing.entity.PedroPathingBotEntity;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.*;
import com.pedropathing.follower.Follower;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    public static PathChain pushBotAuto;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        PedroPathingBotEntity robot = new Follower(hardwareMap, FConstants.class, LConstants.class)
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
