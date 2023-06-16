package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(47.23340377866313, 30, 251.79378956888684, Math.toRadians(60), 15.25)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, 64, Math.toRadians(180)))
                                .splineTo(new Vector2d(-33, 0), Math.toRadians(360))
                                .waitSeconds(0.3)
                                .addDisplacementMarker(() -> {
                                    /* Everything in the marker callback should be commented out */

                                    // raiseLinearSlide()
                                    // place cone()
                                })
                                .waitSeconds(0.3)
                                .splineTo(new Vector2d(-36, 0), Math.toRadians(180))
                                .splineTo(new Vector2d(-58, 12), Math.toRadians(180))

                                .build()

        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

/*                               .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .build()
*/