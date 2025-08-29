

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.pedropathing.DefaultBotBuilder;
import com.noahbres.meepmeep.pedropathing.entity.PedroPathingBotEntity;
import com.noahbres.meepmeep.pedropathing.lib.pathgeneration.*;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class AutoTest {
    public static Pose2d startingPose = new Pose2d(15.5, 61.75, Math.toRadians(270));
    public static PathChain pushBotAuto;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

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