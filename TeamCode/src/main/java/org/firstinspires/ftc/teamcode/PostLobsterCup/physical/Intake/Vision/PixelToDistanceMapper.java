package org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Intake.Vision;

public class PixelToDistanceMapper {

    // Regression coefficients for each output
    private double[] directDistCoeffs;
    private double[] forwardDistCoeffs;
    private double[] horizOffsetCoeffs;


    public PixelToDistanceMapper(double[][] calibrationData) {
        directDistCoeffs = computeLinearRegression(calibrationData, 2);  // z = directDist
        horizOffsetCoeffs = computeLinearRegression(calibrationData, 3); // z = horizOffset
        forwardDistCoeffs = computeLinearRegression(calibrationData, 4); // z = forwardDist
    }

    private double[] computeLinearRegression(double[][] data, int outputIndex) {
        int n = data.length;
        double sumX = 0, sumY = 0, sumZ = 0;
        double sumXX = 0, sumYY = 0, sumXY = 0;
        double sumXZ = 0, sumYZ = 0;

        for (int i = 0; i < n; i++) {
            double x = data[i][0];
            double y = data[i][1];
            double z = data[i][outputIndex];

            sumX += x;
            sumY += y;
            sumZ += z;
            sumXX += x * x;
            sumYY += y * y;
            sumXY += x * y;
            sumXZ += x * z;
            sumYZ += y * z;
        }

        // Solve least squares for plane: z = a*x + b*y + c
        // Using normal equations
        double[][] A = {
                {sumXX, sumXY, sumX},
                {sumXY, sumYY, sumY},
                {sumX,  sumY,  n}
        };
        double[] B = {sumXZ, sumYZ, sumZ};

        return solveLinearSystem(A, B);  // returns [a, b, c]
    }

    private double[] solveLinearSystem(double[][] A, double[] B) {
        // Simple Gaussian elimination for 3x3 matrix
        int n = 3;
        double[][] mat = new double[n][n+1];
        for (int i = 0; i < n; i++) {
            System.arraycopy(A[i], 0, mat[i], 0, n);
            mat[i][n] = B[i];
        }

        for (int i = 0; i < n; i++) {
            // Pivot
            int maxRow = i;
            for (int k = i + 1; k < n; k++) {
                if (Math.abs(mat[k][i]) > Math.abs(mat[maxRow][i])) {
                    maxRow = k;
                }
            }
            double[] temp = mat[i];
            mat[i] = mat[maxRow];
            mat[maxRow] = temp;

            // Eliminate
            for (int k = i + 1; k < n; k++) {
                double factor = mat[k][i] / mat[i][i];
                for (int j = i; j <= n; j++) {
                    mat[k][j] -= factor * mat[i][j];
                }
            }
        }

        // Back substitution
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            x[i] = mat[i][n] / mat[i][i];
            for (int k = i - 1; k >= 0; k--) {
                mat[k][n] -= mat[k][i] * x[i];
            }
        }
        return x;
    }

    // Main mapping function
    public DistanceResult getDistanceFromPixel(double x, double y) {
        double directDist = evalModel(directDistCoeffs, x, y);
        double forwardDist = evalModel(forwardDistCoeffs, x, y);
        double horizOffset = evalModel(horizOffsetCoeffs, x, y);

        return new DistanceResult(directDist, forwardDist, horizOffset);
    }

    private double evalModel(double[] coeffs, double x, double y) {
        return coeffs[0] * x + coeffs[1] * y + coeffs[2];
    }

    // Result container class
    public static class DistanceResult {
        public double directDist;
        public double forwardDist;
        public double horizOffset;

        public DistanceResult(double d, double f, double h) {
            this.directDist = d;
            this.forwardDist = f;
            this.horizOffset = h;
        }
    }
}