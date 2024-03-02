package frc.robot.utils.math;

import org.ejml.simple.SimpleMatrix;

public class PolynomialRegression {
    private SimpleMatrix coefficients;

    public PolynomialRegression(double[] x, double[] y, int degree) {
        // Add a column of ones to the x matrix for the y-intercept
        SimpleMatrix xMatrix = new SimpleMatrix(x.length, degree + 1);
        for (int i = 0; i < x.length; i++) {
            xMatrix.set(i, 0, 1);
            for (int j = 1; j <= degree; j++) {
                xMatrix.set(i, j, Math.pow(x[i], j));
            }
        }
        // Use the Moore-Penrose pseudoinverse to find the coefficients
        SimpleMatrix yMatrix = new SimpleMatrix(y.length, 1, true, y);
        SimpleMatrix transpose = xMatrix.transpose();
        SimpleMatrix xTx = transpose.mult(xMatrix);
        SimpleMatrix xTxInv = xTx.invert();
        SimpleMatrix xTy = transpose.mult(yMatrix);
        coefficients = xTxInv.mult(xTy);
    }

    public double predict(double x) {
        double y = 0;
        for (int i = 0; i < coefficients.getNumRows(); i++) {
            y += coefficients.get(i) * Math.pow(x, i);
        }
        return y;
    }

    public double R2(double[] x, double[] y) {

        double sst = 0, sse = 0, yMean = 0;
        for (int i = 0; i < y.length; i++) {
            yMean += y[i];
        }
        yMean /= y.length;
        for (int i = 0; i < y.length; i++) {
            double yPred = predict(x[i]);
            sst += (y[i] - yMean) * (y[i] - yMean);
            sse += (y[i] - yPred) * (y[i] - yPred);
        }
        return 1 - sse / sst;
    }

    public double beta(int degree) {
        return coefficients.get(degree);
    }
}