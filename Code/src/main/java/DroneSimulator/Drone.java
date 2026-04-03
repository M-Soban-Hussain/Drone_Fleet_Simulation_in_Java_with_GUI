package DroneSimulator;

public class Drone {
    private final int ID;
    
    private double mass;                
    private Vector3 I;          
    private double kd;                 
    
    private Vector3 position;           
    private Vector3 velocity;    
    
    private Matrix3D rotMat;  
    
    private Vector3 Fnet;
    private Vector3 Rtar;//FINAL POSITION
    private Vector3 Vtar;//FINAL VELOCITY
    
    private Vector3 Fg;   
    private Vector3 Ft;         
    private Vector3 Faero;   
    private Vector3 Frep;     
    private Vector3 Fform; 
    
    private Vector3 angVel;     
    
    private Vector3 torque;            
    private Vector3 angAcc;
    
    
    public Drone(int ID, double mass, Vector3 inertia, double kd,Vector3 initPos,Vector3 Rtar,Vector3 Vtar, Vector3 initVel,Matrix3D initRot) {
        this.ID = ID;
        this.mass = mass;
        this.I = inertia;
        this.kd = kd;
        	
        // We make new vectors so that the initials remain intact and donot get modified
        this.position = new Vector3(initPos.x, initPos.y, initPos.z);
        this.velocity = new Vector3(initVel.x, initVel.y, initVel.z);
        this.Rtar = Rtar;
        this.Vtar = Vtar;
        // Initialize rotation matrix with deep copy
        this.rotMat = new Matrix3D(initRot);
        
        this.angVel = Vector3.zero();
        this.angAcc = Vector3.zero();
        this.torque = Vector3.zero();
        
        this.Fg = Vector3.zero();
        this.Ft = Vector3.zero();
        this.Faero = Vector3.zero();
        this.Frep = Vector3.zero();
        this.Fform = Vector3.zero();
        this.Fnet = Vector3.zero();
    }
    
    public void resetForces() {
        Fg = Vector3.zero();
        Ft = Vector3.zero();
        Faero = Vector3.zero();
        Frep = Vector3.zero();
        Fform = Vector3.zero();
        Fnet = Vector3.zero();
        torque = Vector3.zero();
    }

    
    public void  computeFaero() {
    	this.Faero = velocity.multiply(-this.kd);
    }
    
    public void applyGravity(Vector3 gravity) {
        this.Fg = gravity;
    }

    public void applyThrust(Vector3 thrust) {
        this.Ft = rotMat.multiply(thrust);
    }

    public void applyFormationForce(Vector3 formationForce) {
        this.Fform = formationForce;
    }

    public void applyRepulsionForce(Vector3 repulsionForce) {
        this.Frep = repulsionForce;
    }

    public void applyTorque(Vector3 torque) {
        this.torque = torque;
    }
    	
    public void sumAllForces() {
        // Calculate total by using getters directly in the sum
    	computeFaero();
    	Fnet = new Vector3(
    			Fg.x+Ft.x+Faero.x+Frep.x+Fform.x,
    			Fg.y+Ft.y+Faero.y+Frep.y+Fform.y,
    			Fg.z+Ft.z+Faero.z+Frep.z+Fform.z
    			);

    }
    	public void calculateAngularAcceleration() 
    	{
       
        Vector3 temp = torque.subtract(angVel.cross(new Vector3(I.x*angVel.x,I.y*angVel.y,I.z*angVel.z)));
        angAcc = new Vector3(temp.x/I.x,temp.y/I.y,temp.z/I.z);
        }

    	    public void setMass(double mass) {
    	        this.mass = mass;
    	    }
    	    
    	    public void setInertia(Vector3 I) {
    	        this.I = I;
    	    }
    	    
    	    public void setKd(double kd) {
    	        this.kd = kd;
    	    }
    	    
    	    public void setPosition(Vector3 position) {
    	        this.position = position;
    	    }
    	    
    	    public void setVelocity(Vector3 velocity) {
    	        this.velocity = velocity;
    	    }
    	    
    	    public void setRotationMatrix(double[][] rotationMatrix) {
    	    	this.rotMat = new Matrix3D(rotationMatrix);
    	    }
    	    
    	    public void integrateAngular(double dt) {
    	        calculateAngularAcceleration();
    	        angVel = angVel.add(angAcc.multiply(dt)); 
    	       
    	        this.rotMat = rotMat.Exponent(angVel, dt); 
    	    }
    	    
    	    
    	    public void integrateLinear(double dt) {
    	        Vector3 acc = Fnet.multiply(1.0 / mass);
    	        velocity = velocity.add(acc.multiply(dt));
    	        position = position.add(velocity.multiply(dt));
    	    }
    	    
    	    public void setNetforce(Vector3 netforce) {
    	        this.Fnet = netforce;
    	    }

    	    public void setAngularVelocity(Vector3 angularVelocity) {
    	        this.angVel = angularVelocity;
    	    }
    	    
    	    public void setAngularAcceleration(Vector3 angularAcceleration) {
    	        this.angAcc = angularAcceleration;
    	    }

    	    public double getMass() {
    	        return mass;
    	    }
    	    
    	    public Vector3 getInertia() {
    	        return I;
    	    }
    	    
    	    public double getKd() {
    	        return kd;
    	    }
    	    
    	    public Vector3 getPosition() {
    	        return position;
    	    }
    	    
    	    public Vector3 getVelocity() {
    	        return velocity;
    	    }
    	    public Matrix3D getRotMat() {
    	    	return rotMat;
    	    }
    	    public double[][] getRotationMatrix() {
    	        double[][] copy = new double[3][3];
    	        for (int i = 0; i < 3; i++) {
    	            for (int j = 0; j < 3; j++) {
    	                copy[i][j] = rotMat.get(i, j);
    	            }
    	        }
    	        return copy;
    	    }
    	    
    	    public Vector3 getNetforce() {
    	        return Fnet;
    	    }
    	    
    	    public Vector3 getGravityForce() {
    	        return Fg;
    	    }
    	    
    	    public Vector3 getThrustForce() {
    	        return Ft;
    	    }
    	    
    	    public Vector3 getAerodynamicForce() {
    	        return Faero;
    	    }
    	    
    	    public Vector3 getRepulsionForce() {
    	        return Frep;
    	    }
    	    
    	    public Vector3 getFormationForce() {
    	        return Fform;
    	    }
    	    
    	    public Vector3 getAngularVelocity() {
    	        return angVel;
    	    }
    	    
    	    public Vector3 getTorque() {
    	        return torque;
    	    }
    	    
    	    public Vector3 getAngularAcceleration() {
    	        return angAcc;
    	    }
    	    
    	    public int getId() {
    	        return ID;
    	    }

			public Vector3 getRtar() {
				return Rtar;
			}

			public void setRtar(Vector3 rtar) {
				Rtar = rtar;
			}

			public Vector3 getVtar() {
				return Vtar;
			}

			public void setVtar(Vector3 vtar) {
				Vtar = vtar;
			}
    	
    	}