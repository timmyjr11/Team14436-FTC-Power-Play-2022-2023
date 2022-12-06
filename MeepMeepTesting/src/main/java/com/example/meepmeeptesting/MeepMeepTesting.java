package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 30, Math.toRadians(275), Math.toRadians(60), 16.965)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.25, -62.5, Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                                })
                                .waitSeconds(0.3)
                                .strafeTo(new Vector2d(-4, -59))
                                .lineToLinearHeading(new Pose2d(-4, -10, Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                                })
                                .turn(Math.toRadians(47))
                                .forward(7)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .waitSeconds(1)
                                .back(7)
                                .turn(Math.toRadians(-47))
                                .lineToLinearHeading(new Pose2d(-4, -60, Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                                })
                                .turn(Math.toRadians(-90))
                                .build()
                );

        //RoadRunnerBotEntity forwardBot = new DefaultBotBuilder(meepMeep)

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myFirstBot)

                .start();

    }
}