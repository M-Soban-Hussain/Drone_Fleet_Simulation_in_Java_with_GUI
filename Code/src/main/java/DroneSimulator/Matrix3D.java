package DroneSimulator;

public class Matrix3D {


    private double[][] m;

    public Matrix3D(double[][] values) {
        m = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                m[i][j] = values[i][j];
            }
        }
    }
    
    public Matrix3D(Vector3 c1, Vector3 c2, Vector3 c3) {
        m = new double[3][3];
        m[0][0] = c1.getX();
        m[0][1] = c2.getX();
        m[0][2] = c3.getX();
        m[1][0] = c1.getY();
        m[1][1] = c2.getY();
        m[1][2] = c3.getY();
        m[2][0] = c1.getZ();
        m[2][1] = c2.getZ();
        m[2][2] = c3.getZ();
    }
    
    public Matrix3D(Matrix3D copy) {
    	m = new double[3][3];
        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) {
                m[i][j] = copy.get(i, j);
            }
        }
        
    }
    
    public Matrix3D Exponent(Vector3 w, double dt) {
        double theta = w.magnitude() * dt;

        if (theta == 0) {
            return new Matrix3D(this);  // No rotation
        }

        Vector3 wNorm = w.normalize();
        Matrix3D W = skew(wNorm);

        Matrix3D W2 = W.multiply(W);
        Matrix3D R_update = Matrix3D.identity().add(W.multiply(Math.sin(theta))).add(W2.multiply(1 - Math.cos(theta)));

        return this.multiply(R_update);
    }
    
    
    public static Matrix3D skew(Vector3 w) {
        return new Matrix3D(new double[][]{
            {0, -w.z, w.y},
            {w.z, 0, -w.x},
            {-w.y, w.x, 0}
        });
    }
    
    public Matrix3D multiply(double scalar) {
        double[][] result = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                result[i][j] = m[i][j] * scalar;
        return new Matrix3D(result);
    }
    
    
    public static Matrix3D identity() {
        return new Matrix3D(new double[][]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    }

    public Matrix3D add(Matrix3D other) {
        double[][] result = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result[i][j] = this.m[i][j] + other.m[i][j];
            }
        }
        return new Matrix3D(result);
    }
    public Matrix3D subtract(Matrix3D other) {
        double[][] result = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result[i][j] = this.m[i][j] - other.m[i][j];
            }
        }
        return new Matrix3D(result);
    }

    public Matrix3D multiply(Matrix3D other) {
        double[][] result = new double[3][3];

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    result[i][j] += this.m[i][k] * other.m[k][j];
                }
            }
        }
        return new Matrix3D(result);
    }

    public Vector3 multiply(Vector3 v) {
        return new Vector3(
                m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    public Matrix3D transpose() {
        double[][] result = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result[i][j] = m[j][i];
            }
        }
        return new Matrix3D(result);
    }
    

    public double get(int row, int col) {
        return m[row][col];
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            sb.append(String.format("[%.3f %.3f %.3f]\n",
                    m[i][0], m[i][1], m[i][2]));
        }
        return sb.toString();
    }
}
