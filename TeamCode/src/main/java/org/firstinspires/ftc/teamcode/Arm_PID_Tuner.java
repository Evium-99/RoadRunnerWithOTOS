package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="Arm_PID_Tuner", group="TeleOp")
public class Arm_PID_Tuner extends OpMode {

    // Motors & Sensors
    private DcMotorEx cap;
    private DcMotorEx cap2;
    private PIDController capPID = new PIDController(0.02, 0, 0);
    public static double CapP = 0.02;
    public static double CapI = 0;
    public static double CapD = 0;
    public static double Capsetpoint = 100;

    private DcMotorEx slide;
    private PIDController slidePID = new PIDController(0.008, 0, 0);
    public static double SpinP = 0.008;
    public static double SpinI = 0;
    public static double SpinD = 0;
    public static double Spinsetpoint = 100;

    @Override
    public void init() {

        // Capstan Stuff

        cap = hardwareMap.get(DcMotorEx.class, "cap");
        cap2 = hardwareMap.get(DcMotorEx.class, "cap2");

        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap.setDirection(DcMotorSimple.Direction.FORWARD);
        cap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cap2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap2.setDirection(DcMotorSimple.Direction.REVERSE);
        capPID.setIntegrationBounds(0, 0.01);
        capPID.setTolerance(100, 100);

        // Slide Stuff

        slide = hardwareMap.get(DcMotorEx.class, "spindle");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidePID.setIntegrationBounds(0, 0.01);
        slidePID.setTolerance(100, 100);
    }

    @Override
    public void start() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    @Override
    public void loop() {
        capPID.setPID(CapP, CapI, CapD);
        double capPower = capPID.calculate(cap.getCurrentPosition(), Capsetpoint);
        cap.setPower(capPower);
        cap2.setPower(capPower);

        slidePID.setPID(SpinP, SpinI, SpinD);
        double spinPower = slidePID.calculate(slide.getCurrentPosition(), Spinsetpoint);
        slide.setPower(spinPower);
        telemetry.addData("SlidePow", spinPower);
        telemetry.addData("SlidePos", slide.getCurrentPosition());
        telemetry.addData("CapPow", capPower);
        telemetry.addData("CapPos", cap.getCurrentPosition());
        telemetry.addData("SlideAtSetpoint", slidePID.atSetPoint());
        telemetry.addData("CapAtSetpoint", capPID.atSetPoint());
        telemetry.update();
    }
}
