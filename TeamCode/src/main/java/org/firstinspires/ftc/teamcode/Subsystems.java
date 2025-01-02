package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.concurrent.TimeUnit;

public class Subsystems {
    public static class Arm {
        private DcMotorEx cap;
        private DcMotorEx cap2;
        private PIDController capPID = new PIDController(0.02, 0, 0);

        public Arm(HardwareMap hardwareMap) {
            cap = hardwareMap.get(DcMotorEx.class, "cap");
            cap2 = hardwareMap.get(DcMotorEx.class, "cap2");
            cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            cap.setDirection(DcMotorSimple.Direction.FORWARD);
            cap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            cap2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            cap2.setDirection(DcMotorSimple.Direction.REVERSE);
            capPID.setIntegrationBounds(0, 0.01);
            capPID.setTolerance(8);
        }

        public class ToChamber implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    capPID.reset();
                    initialized = true;
                }
                double pos = cap.getCurrentPosition();
                double capPower = capPID.calculate(pos, 385);
                cap.setPower(capPower);
                cap2.setPower(capPower);
                if (capPID.atSetPoint()) {
                    cap.setPower(0);
                    cap2.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action ToChamber() {
            return new ToChamber();
        }

        public class ToRest implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    capPID.reset();
                    initialized = true;
                }
                double pos = cap.getCurrentPosition();
                double capPower = capPID.calculate(pos, 30);
                cap.setPower(capPower);
                cap2.setPower(capPower);
                if (capPID.atSetPoint()) {
                    cap.setPower(0);
                    cap2.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action ToRest() {
            return new ToRest();
        }
    }
    public static class LinearSlide {
        private DcMotorEx slide;
        private PIDController slidePID = new PIDController(0.008, 0, 0);

        public LinearSlide(HardwareMap hardwareMap) {
            slide = hardwareMap.get(DcMotorEx.class, "spindle");
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setDirection(DcMotorSimple.Direction.FORWARD);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slidePID.setIntegrationBounds(0, 0.01);
            slidePID.setTolerance(100, 100);
        }

        public class ToMaxOut implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slidePID.reset();
                    initialized = true;
                }
                double pos = slide.getCurrentPosition();
                slide.setPower(slidePID.calculate(pos, 2300));
                packet.put("LinearSlidePos", slide.getCurrentPosition());
                if (slidePID.atSetPoint()) {
                    slide.setPower(0);
                    packet.put("RunningLinearSlide", false);
                    return false;
                } else {
                    packet.put("RunningLinearSlide", true);
                    return true;
                }
            }
        }
        public Action ToMaxOut() {
            return new ToMaxOut();
        }
        public class ToChamber implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slidePID.reset();
                    initialized = true;
                }
                double pos = slide.getCurrentPosition();
                slide.setPower(slidePID.calculate(pos, 2000));
                if (slidePID.atSetPoint()) {
                    slide.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action ToChamber() {
            return new ToChamber();
        }

        public class ToMaxIn implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slidePID.reset();
                    initialized = true;
                }
                double pos = slide.getCurrentPosition();
                slide.setPower(slidePID.calculate(pos, 0));
                if (slidePID.atSetPoint()) {
                    slide.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action ToMaxIn() {
            return new ToMaxIn();
        }
    }
    public static class Claw {
        private ServoImplEx leftServo;
        private ServoImplEx rightServo;
        public Claw(HardwareMap hardwareMap) {
            leftServo = hardwareMap.get(ServoImplEx.class, "LSLower");
            leftServo.setPwmEnable();
            leftServo.scaleRange(0, 1);
            rightServo = hardwareMap.get(ServoImplEx.class, "LSTop");
            rightServo.setPwmEnable();
            rightServo.scaleRange(0, 1);
        }

        public class Open implements Action {
            private Timing.Timer wait = new Timing.Timer(700, TimeUnit.MILLISECONDS);
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    wait.start();
                    initialized = true;
                }
                leftServo.setPosition(0.07);
                rightServo.setPosition(0.93);
                return !wait.done();
            }
        }
        public Action Open() {
            return new Open();
        }

        public class Rest implements Action {
            private Timing.Timer wait = new Timing.Timer(700, TimeUnit.MILLISECONDS);
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    wait.start();
                    initialized = true;
                }
                leftServo.setPosition(0.4);
                rightServo.setPosition(0.6);
                return !wait.done();
            }
        }
        public Action Rest() {
            return new Rest();
        }

        public class Close implements Action {
            private Timing.Timer wait = new Timing.Timer(700, TimeUnit.MILLISECONDS);
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    wait.start();
                    initialized = true;
                }
                leftServo.setPosition(0.73);
                rightServo.setPosition(0.27);
                return !wait.done();
            }
        }
        public Action Close() {
            return new Close();
        }
    }
}
