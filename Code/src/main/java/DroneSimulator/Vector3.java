package DroneSimulator;

public class Vector3 {

	public double x;
    public double y;
    public double z;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public Vector3 add(Vector3 other) {
        return new Vector3(
                this.x + other.x,
                this.y + other.y,
                this.z + other.z
        );
    }

    public Vector3 subtract(Vector3 other) {
        return new Vector3(
                this.x - other.x,
                this.y - other.y,
                this.z - other.z
        );
    }

    public Vector3 multiply(double scalar) {
        return new Vector3(
                this.x * scalar,
                this.y * scalar,
                this.z * scalar
        );
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vector3 normalize() {
        double mag = magnitude();
        if (mag == 0) {
            return new Vector3(0, 0, 0);
        }
        return multiply(1.0 / mag);
    }

    public double dot(Vector3 other) {
        return this.x * other.x
             + this.y * other.y
             + this.z * other.z;
    }

    public Vector3 cross(Vector3 other) {
        return new Vector3(
                this.y * other.z - this.z * other.y,
                this.z * other.x - this.x * other.z,
                this.x * other.y - this.y * other.x
        );
    }

    public double distance(Vector3 other) {
        return this.subtract(other).magnitude();
    }

    public static Vector3 zero() {
        return new Vector3(0, 0, 0);
    }
    
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }
}

