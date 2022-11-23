package com.meepmeep.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.entity.TrajectorySequenceEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.sun.tools.javac.util.List;

public class Testing {
    private static final MeepMeep meepmeep = new MeepMeep(800);
    private static final RoadRunnerBotEntity bot = new DefaultBotBuilder(meepmeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15).build();
    private static final DriveShim drive = bot.getDrive();

    public static void main(String[] args) {
        TrajectorySequence toMid = drive.trajectorySequenceBuilder(new Pose2d(-36, -60)).lineToSplineHeading(new Pose2d(-40, -40, -150)).build();
        TrajectorySequence toCones = drive.trajectorySequenceBuilder(toMid.end()).lineToSplineHeading(new Pose2d(-36, -12, -180)).build();

        bot.followTrajectorySequence(toMid);

        bot.followTrajectorySequence(toCones);

        launchWindow();
    }

    private static void launchWindow() {
        meepmeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(bot).start();
    }
}
