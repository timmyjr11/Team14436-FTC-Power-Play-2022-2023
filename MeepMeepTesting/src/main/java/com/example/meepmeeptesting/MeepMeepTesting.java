package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        // Red bot
        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 30, Math.toRadians(275), Math.toRadians(60), 16.965)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37.5, -61.3, Math.toRadians(90)))
                                .lineToConstantHeading(new Vector2d(-12, -60))
                                .lineToConstantHeading(new Vector2d(-12, -34))
                                .turn(Math.toRadians(-45))
                                .waitSeconds(1)
                                .turn(Math.toRadians(45))
                                .lineToConstantHeading(new Vector2d(-12, -12))
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(-55, -12))
                                .lineToConstantHeading(new Vector2d(-35, -12))
                                .turn(Math.toRadians(-135))
                                .waitSeconds(1)
                                .turn(Math.toRadians(135))
                                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-35, -12))
                                .turn(Math.toRadians(-135))
                                .waitSeconds(1)
                                .turn(Math.toRadians(-135))
                                .lineToConstantHeading(new Vector2d(-34, -35))
                                .build()
                );

        // Blue bot
        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 30, Math.toRadians(275), Math.toRadians(60), 16.965)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(37.5, -61.3, Math.toRadians(90)))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(12, -55), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(11, -33, Math.toRadians(135)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(12, -24, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(12, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(55, -12, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rightBot)
                .addEntity(leftBot)
                .start();

    }
}