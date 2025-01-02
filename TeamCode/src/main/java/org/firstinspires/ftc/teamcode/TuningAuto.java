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
@Autonomous(name = "TuningAuto", group = "Autonomous")
public class TuningAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Init Complete", false);
        telemetry.update();
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(270));
        MecanumDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Subsystems.Claw claw = new Subsystems.Claw(hardwareMap);
        Subsystems.Arm arm = new Subsystems.Arm(hardwareMap);
        Subsystems.LinearSlide linearslide = new Subsystems.LinearSlide(hardwareMap);

        Action goToChamber = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(5, 0))
                .strafeTo(new Vector2d(5, 5))
                .strafeTo(new Vector2d(0, 5))
                .strafeTo(new Vector2d(0, 0))
                .strafeTo(new Vector2d(5, 0))
                .strafeTo(new Vector2d(5, 5))
                .strafeTo(new Vector2d(0, 5))
                .strafeTo(new Vector2d(0, 0))
                .strafeTo(new Vector2d(5, 0))
                .strafeTo(new Vector2d(5, 5))
                .strafeTo(new Vector2d(0, 5))
                .strafeTo(new Vector2d(0, 0))
                .strafeTo(new Vector2d(5, 0))
                .strafeTo(new Vector2d(5, 5))
                .strafeTo(new Vector2d(0, 5))
                .strafeTo(new Vector2d(0, 0))
                .strafeTo(new Vector2d(5, 0))
                .strafeTo(new Vector2d(5, 5))
                .strafeTo(new Vector2d(0, 5))
                .strafeTo(new Vector2d(0, 0))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.Close());


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Init Complete", true);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        goToChamber
                )
        );
    }
}