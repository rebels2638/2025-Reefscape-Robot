package frc.robot.subsystems.drivetrain.swerve.controllers;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConfigBase;
import frc.robot.lib.util.RebelUtil;

public class DriveFFController {
    private final ArrayList<double[]> points;

    private final SwerveConfigBase config;
    @SuppressWarnings("static-access")
    public DriveFFController(SwerveConfigBase config) {
        this.config = config;
        points = config.getSwerveDrivetrainControllerConfig().kDRIVE_FF_POINTS;
    }

    @SuppressWarnings("static-access")
    public double calculate(double imps, double irps) {

        double mps = RebelUtil.constrain(Math.abs(imps), 0, config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC);
        double rps = RebelUtil.constrain(Math.abs(irps), 0, config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC);

        double[] inputPoint = new double[] { mps, rps, 0 };

        double[][] p = new double[4][3];
        for (int i = 0; i < 4; i++) {
            p[i] = new double[] { Integer.MAX_VALUE, Integer.MAX_VALUE, -1 };
        }

        ArrayList<double[]> temp = new ArrayList<double[]>();
        temp.addAll(points);

        for (int n = 0; n < 4; n++) {
            boolean good = false;
            while (!good) {
                int index = 0;
                double minDist = Integer.MAX_VALUE;
                good = true;
                for (int i = 0; i < temp.size(); i++) {
                    if (distance(temp.get(i), inputPoint) < minDist) {
                        minDist = distance(temp.get(i), inputPoint);
                        index = i;
                    }
                }

                for (int m = 0; m < n; m++) {
                    if (distance(p[m], temp.get(index)) > Math.sqrt(2)) {
                        good = false;
                        break;
                    }
                }
                if (good) {
                    p[n] = temp.get(index);
                }

                temp.remove(index);
            }
        }

        double[] tr, tl, br, bl;
        tr = new double[] { -Integer.MAX_VALUE, -Integer.MAX_VALUE, 0 };
        tl = new double[] { Integer.MAX_VALUE, -Integer.MAX_VALUE, 0 };
        br = new double[] { -Integer.MAX_VALUE, Integer.MAX_VALUE, 0 };
        bl = new double[] { Integer.MAX_VALUE, Integer.MAX_VALUE, 0 };

        boolean trT = false;
        boolean tlT = false;
        boolean brT = false;
        boolean blT = false;

        for (int i = 0; i < 4; i++) {
            if (p[i][0] >= inputPoint[0] && p[i][1] >= inputPoint[1] && !trT) {
                tr = p[i];
                trT = true;
            } else if (p[i][0] <= inputPoint[0] && p[i][1] >= inputPoint[1] && !tlT) {
                tl = p[i];
                tlT = true;
            } else if (p[i][0] >= inputPoint[0] && p[i][1] <= inputPoint[1] && !brT) {
                br = p[i];
                brT = true;
            } else if (p[i][0] <= inputPoint[0] && p[i][1] <= inputPoint[1] && !blT) {
                bl = p[i];
                blT = true;
            }
        }

        double r1 = Math.abs(
                (tr[0] - inputPoint[0]) / (tr[0] - tl[0]) * tl[2] + (inputPoint[0] - tl[0]) / (tr[0] - tl[0]) * tr[2]);
        double r2 = Math.abs(
                (br[0] - inputPoint[0]) / (br[0] - bl[0]) * bl[2] + (inputPoint[0] - bl[0]) / (br[0] - bl[0]) * br[2]);
        double pf = Math
                .abs((inputPoint[1] - br[1]) / (tr[1] - br[1]) * r1 + (tr[1] - inputPoint[1]) / (tr[1] - bl[1]) * r2);

        Logger.recordOutput("SwerveDrive/driveFF/r1", r1);
        Logger.recordOutput("SwerveDrive/driveFF/r2", r2);
        Logger.recordOutput("SwerveDrive/driveFF/pf", pf);

        double multiplier = 1;
        if (imps >= 0 && irps <= 0) {
            multiplier = -1;
        } else if (imps <= 0 && irps >= 0) {
            multiplier = -1;
        }

        Logger.recordOutput("SwerveDrive/driveFF/multiplier", multiplier);

        if (!Double.isNaN(pf) && Double.isFinite(pf)) {
            return (pf * multiplier);
        }

        if (r1 == 0 && !Double.isNaN(r2) && Double.isFinite(r2)) {
            return (r2 * multiplier);
        }

        if (r2 == 0 && !Double.isNaN(r1) && Double.isFinite(r1)) {
            return (r1 * multiplier);
        }

        return 0;

    }

    @SuppressWarnings("static-access")
    private double distance(double[] a, double[] b) {
        double r;
        switch (Constants.currentMode) {
            case SIM:
                r = Math.sqrt(Math.pow(
                        (a[0] - b[0]) / config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC, 2)
                        + Math.pow(
                                (a[1] - b[1]) / config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC,
                                2));
                break;

            default:
                r = Math.sqrt(Math.pow(
                        (a[0] - b[0]) / config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC, 2)
                        + Math.pow(
                                (a[1] - b[1]) / config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC,
                                2));
                break;
        }

        return r;
    }
}