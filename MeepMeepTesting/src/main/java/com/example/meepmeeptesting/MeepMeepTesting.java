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

        /*RoadRunnerBotEntity redBotV1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(56, 30, Math.toRadians(270), Math.toRadians(180), 13.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37.5, -58, Math.toRadians(90)))
                                .waitSeconds(0.3)
                                .strafeTo(new Vector2d(-4, -59))
                                .lineToLinearHeading(new Pose2d(-4, -30, Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                                })
                                .turn(Math.toRadians(-47))
                                .forward(7)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .waitSeconds(1)
                                .back(7)
                                .turn(Math.toRadians(47))
                                .lineToLinearHeading(new Pose2d(-4, -60, Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                                })
                                .turn(Math.toRadians(-90))
                                .build()
                );
*/

        RoadRunnerBotEntity redBotCool = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(56, 30, Math.toRadians(270), Math.toRadians(180), 13.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(37.5, -58, Math.toRadians(90)))
                                // Drive to place first cone
                                .splineToSplineHeading(new Pose2d(34, -50, Math.toRadians(0)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(34, -20, Math.toRadians(0)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(28, -4, Math.toRadians(315)), Math.toRadians(135))
                                .waitSeconds(0.25)
                                // Drive to grab first cone on stack
                                .splineToSplineHeading(new Pose2d(47, -11.75, Math.toRadians(0)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(56.5, -11.75), Math.toRadians(0))
                                .waitSeconds(0.25)
                                // Go place first cone
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(47, -11.75, Math.toRadians(0)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(28, -4, Math.toRadians(315)), Math.toRadians(135))
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