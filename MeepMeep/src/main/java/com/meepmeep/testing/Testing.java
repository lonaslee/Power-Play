package com.meepmeep.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Testing {
    private static final MeepMeep meepmeep = new MeepMeep(800);
    private static final RoadRunnerBotEntity bot = new DefaultBotBuilder(meepmeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15).build();
    private static final DriveShim drive = bot.getDrive();

    public static void main(String[] args) {
        bot.followTrajectorySequence(
                drive.trajectorySequenceBuilder(new Pose2d())
                     .lineToSplineHeading(new Pose2d(30, 43, 43))
                     .build());

        launchWindow();
    }

    private static void launchWindow() {
        meepmeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
