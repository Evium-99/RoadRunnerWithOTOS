package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Config
@Autonomous(name = "LegacyAuto", group = "Autonomous")
public class MainAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Start Initialization

        telemetry.addData("Init Complete", false);
        telemetry.update();

        // Define Initial Pose and Subsystems
        Pose2d initialPose = new Pose2d(9, -61, Math.toRadians(270));
        MecanumDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Subsystems.Claw claw = new Subsystems.Claw(hardwareMap);
        Subsystems.Arm arm = new Subsystems.Arm(hardwareMap);
        Subsystems.LinearSlide linearslide = new Subsystems.LinearSlide(hardwareMap);

        Action goToChamber = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(9, -51)) // Go to Chamber
                .build();

        Action moveSamples = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(40, -40)) // Move Back and to the right
                .strafeTo(new Vector2d(35, -24)) // Move Forward
                .strafeTo(new Vector2d(49, -24)) // Move to Right
                .strafeTo(new Vector2d(49, -37)) // Move to Wall
                .strafeTo(new Vector2d(49, -24)) // Move Away from Wall
                .strafeTo(new Vector2d(59, -26)) // Move Right
                .strafeTo(new Vector2d(59, -37)) // Move To Wall
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