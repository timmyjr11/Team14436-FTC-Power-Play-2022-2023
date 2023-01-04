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
        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 30, Math.toRadians(275), Math.toRadians(60), 16.965)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37.5, -61.3, Math.toRadians(90)))
                                .lineToConstantHeading(new Vector2d(-12, -60))
                                .lineToConstantHeading(new Vector2d(-12, -34))
                                .turn(Math.toRadians(-45))
                                .waitSeconds(0.5)
                                .turn(Math.toRadians(45))
                                .lineToConstantHeading(new Vector2d(-12, -12))
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(-55, -12))
                                .lineToConstantHeading(new Vector2d(-35, -12))
                                .turn(Math.toRadians(-135))
                                .waitSeconds(0.5)
                                .turn(Math.toRadians(135))
                                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-35, -12))
                                .turn(Math.toRadians(-135))
                                .waitSeconds(0.5)
                                .turn(Math.toRadians(-135))
                                .lineToConstantHeading(new Vector2d(-35, -35))
                                .build()
                );

        // Blue bot
        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 30, Math.toRadians(275), Math.toRadians(60), 16.965)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(37.5, -61.3, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(25, -57), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(12, -55), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(12, -30, Math.toRadians(135)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(12, -24, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(12, -17, Math.toRadians(0)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(32, -12), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(55, -12, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(55, -12, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(35, -35, Math.toRadians(270)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot)
                .addEntity(blueBot)
                .start();

    }
}