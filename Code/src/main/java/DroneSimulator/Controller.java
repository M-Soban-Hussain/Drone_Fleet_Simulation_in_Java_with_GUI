package DroneSimulator;

public class Controller {

    private final double kp;      
    private final double kd;     
    private final double kR;      
    private final double kOmega; 

    private final double mass;    
    
    private static final Vector3 g = new Vector3(0, 0, -9.81);  
    
    private Vector3 a;
    
    private Matrix3D desiredRotation;     
    private Vector3 desiredAngularVelocity;

    
    public Controller(double kp, double kd, double kR, double kOmega, double mass) {
        this.kp = kp;
        this.kd = kd;
        this.kR = kR;
        this.kOmega = kOmega;
        this.mass = mass;

        this.desiredRotation = Matrix3D.identity();  
        this.desiredAngularVelocity = new Vector3(0, 0, 0);
    }
    
    public double getYaw(Vector3 velocity) {
        return Math.atan2(velocity.getY(), velocity.getX());
    }

   
    public Vector3 computeThrust(Drone drone) {
        
    	
        Vector3 ep = drone.getRtar().subtract(drone.getPosition());
        
        Vector3 ev = drone.getVtar().subtract(drone.getVelocity());
        
        Vector3 acmd = ep.multiply(kp).add(ev.multiply(kd)).add(g);
        
        setAlpha(acmd);
        
        Vector3 Tinitial = acmd.multiply(mass); 
        
        Matrix3D R = drone.getRotMat(); 
        Vector3 Tbody = R.transpose().multiply(Tinitial); 
        
        double Tz = Tbody.z;
        return new Vector3(0, 0, Tz); 
    }

    public Vector3 computeTorque(Drone drone) {
    	
    	double yaw = getYaw(drone.getVelocity());
    	
    	Vector3 Ti = getAlpha().multiply(mass);
        
        Matrix3D R = drone.getRotMat();
        Vector3 omega = drone.getAngularVelocity();
        
        setDesiredRotation(computeTargetRotMat(yaw,Ti));
        Matrix3D er = desiredRotation.subtract(R);
        
        Vector3 erv = new Vector3(er.get(2, 1) - er.get(1, 2),er.get(0, 2) - er.get(2, 0),er.get(1, 0) - er.get(0, 1))
        		.multiply(0.5);
        
        Vector3 ewv = desiredAngularVelocity.subtract(omega);
        
        Vector3 torque = erv.multiply(kR).add(ewv.multiply(kOmega));
        
        return torque;
    }

    

    public void setDesiredRotation(Matrix3D rotation) {
    	this.desiredRotation = rotation;
    }

    public void setDesiredAngularVelocity(Vector3 angularVelocity) {
        this.desiredAngularVelocity = angularVelocity;
    }
 
    public double getKp() { 
    	return kp; 
    }
    public double getKd() { 
    	return kd; 
    }
    public double getKR() { 
    	return kR; 
    }
    public double getKOmega() { 
    	return kOmega; 
    }
    public double getMass() { 
    	return mass; 
    }
    
    public Matrix3D computeTargetRotMat(double yaw,Vector3 Ti) {

        Vector3 z_b = Ti.normalize();
        Vector3 x_c;
        if (Math.abs(z_b.getZ()) > 0.99) {
            x_c = new Vector3(1, 0, 0);
        } 
        else {
            x_c = new Vector3(Math.cos(yaw), Math.sin(yaw), 0);
        }

        Vector3 y_b = z_b.cross(x_c).normalize();
        Vector3 x_b = y_b.cross(z_b).normalize();

        return new Matrix3D(x_b, y_b, z_b);
    }

	public Vector3 getAlpha() {
		return a;
	}

	public void setAlpha(Vector3 a) {
		this.a = a;
	}
}
    



