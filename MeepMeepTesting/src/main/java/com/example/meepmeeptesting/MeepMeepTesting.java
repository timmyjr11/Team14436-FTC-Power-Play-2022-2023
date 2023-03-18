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
        MeepMeep meepMeep = new MeepMeep(720);

        RoadRunnerBotEntity redBotCool = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(56, 30, Math.toRadians(270), Math.toRadians(180), 13.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37.5, -61.3, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-34, -50, Math.toRadians(180)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-34, -20, Math.toRadians(180)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(225)), Math.toRadians(45))
                                .waitSeconds(0.25)
                                .splineToSplineHeading(new Pose2d(-47, -11.75, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-56.5, -11.75), Math.toRadians(180))
                                .waitSeconds(0.25)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-47, -11.75, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(225)), Math.toRadians(45))
                                .setReversed(false)
                                .waitSeconds(0.25)
                                .splineToSplineHeading(new Pose2d(-47, -11.75, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-56.5, -11.75), Math.toRadians(180))
                                .waitSeconds(0.25)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-47, -11.75, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(225)), Math.toRadians(45))
                                .setReversed(false)
                                .waitSeconds(0.25)
                                .splineToSplineHeading(new Pose2d(-47, -11.75, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-56.5, -11.75), Math.toRadians(180))
                                .waitSeconds(0.25)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-47, -11.75, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(225)), Math.toRadians(45))
                                .setReversed(false)
                                .waitSeconds(0.25)
                                .splineToSplineHeading(new Pose2d(-47, -11.75, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-56.5, -11.75), Math.toRadians(180))
                                .waitSeconds(0.25)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-47, -11.75, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(225)), Math.toRadians(45))
                                .setReversed(false)
                                .waitSeconds(0.25)
                                .splineToSplineHeading(new Pose2d(-47, -11.75, Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-56.5, -11.75), Math.toRadians(180))
                                .waitSeconds(0.25)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-47, -11.75, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(225)), Math.toRadians(45))
                                .setReversed(false)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBotCool)
                .start();

    }
}