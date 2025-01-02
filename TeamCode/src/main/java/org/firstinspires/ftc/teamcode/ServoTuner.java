package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Config
@TeleOp(name="ServoTuner", group="TeleOp")
public class ServoTuner extends OpMode {

    // Motors & Sensors
    private ServoImplEx leftServo;
    private ServoImplEx rightServo;

    public static double leftServoValue = 0;
    public static double rightServoValue = 0;

    @Override
    public void init() {

        leftServo = hardwareMap.get(ServoImplEx.class, "LSLower");
        leftServo.setPwmEnable();
        leftServo.scaleRange(0, 1);
        rightServo = hardwareMap.get(ServoImplEx.class, "LSTop");
        rightServo.setPwmEnable();
        rightServo.scaleRange(0, 1);

    }

    @Override
    public void start() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    @Override
    public void loop() {
        leftServo.setPosition(leftServoValue);
        rightServo.setPosition(rightServoValue);
    }
}
