package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "MainAuto", group = "Autonomous")
public class MainAuto2 extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Start Initialization

        telemetry.addData("Init Complete", false);
        telemetry.update();

        // Define Initial Pose and Subsystems
        Pose2d initialPose = new Pose2d(8.875, -61.71875, Math.toRadians(270));
        MecanumDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Subsystems.Claw claw = new Subsystems.Claw(hardwareMap);
        Subsystems.Arm arm = new Subsystems.Arm(hardwareMap);
        Subsystems.LinearSlide linearslide = new Subsystems.LinearSlide(hardwareMap);

        Action goToChamber = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(8.875, -37.71875)) // Go to Chamber
                .build();

        Action moveSamples = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(37.375, -37.71875)) // Move to the right
                .strafeTo(new Vector2d(37.375, -14.71875)) // Move Forward
                .strafeTo(new Vector2d(45.875, -14.71875)) // Move to Right
                .strafeTo(new Vector2d(45.875, -53.46875)) // Move to Wall
                .build();

        Action runTillEnd = drive.actionBuilder(initialPose)
                .waitSeconds(30)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.Rest());


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Init Complete", true);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                goToChamber,
                                new SequentialAction(
                                        arm.ToChamber(),
                                        claw.Open()
                                )
                        ),
                        linearslide.ToMaxOut(),
                        linearslide.ToMaxIn(),
                        new ParallelAction(
                                moveSamples,
                                arm.ToRest()
                        ),
                        runTillEnd
                )
        );
    }
}