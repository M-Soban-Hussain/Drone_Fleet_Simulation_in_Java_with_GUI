package DroneSimulator;

import java.util.ArrayList;
import java.util.List;

public class ClassMain {

    private static final double dt = 0.01;         
    private static final double totalTime = 60.0;  
    private static final int numDrones = 4;         

    private static final double mass = 1.5;          
    private static final double kd = 0.2;           
    
    private static final double kp = 3.5;         
    private static final double kdCtrl = 2.5;        
    private static final double kR = 8.5;           
    private static final double kOmega = 0.15;       
    

    private static final double formKp = 0.0;      
    private static final double formKv = 0.0;       
    private static final double kRep = 1.0;        
    

    private static final double commRange = 50.0;   
    private static final double packetLoss = 0.01;   
    
    private static final double envWidth = 100.0;   
    private static final double envHeight = 100.0;   
    private static final double envDepth = 1000.0;    
    
  
    private static final double minSafeDistance = 1; 

    	
    	public static void main(String[] args) throws Exception {
    	    
    	    System.out.println("Drone Simulation Project\n");
    	    
    	    
    	    List<Drone> drones = new ArrayList<>();
    	    
    	    // Common initial position (all drones start from same location)
    	    Vector3 initialPosition = new Vector3(0, 0, 0);
    	    Vector3 initialVelocity = Vector3.zero();
    	    Matrix3D initialRotation = Matrix3D.identity();
    	    Vector3 inertia = new Vector3(0.02, 0.02, 0.04);
    	    
    	    System.out.println("Initializing " + numDrones + " drones...");
    	    System.out.println("  All drones starting at: (" + initialPosition.x + ", " + 
    	                      initialPosition.y + ", " + initialPosition.z + ")\n");
    	    
    	    System.out.print("Enter initial position: ");
    	    
    	    for (int i = 0; i < numDrones; i++) {
    	        
//    	         Random target positions (each drone has different destination)
//    	        double targetX = CommunicationModule.rand.nextDouble() * 80 + 10;  // 10-90 range
//    	        double targetY = CommunicationModule.rand.nextDouble() * 80 + 10;
//    	        double targetZ = CommunicationModule.rand.nextDouble() * 30 + 5;   // 5-35 range
    	    	
    	        double targetX = (i+1)*20;  // 10-90 range
    	        double targetY = 10;
    	        double targetZ = 2000;   // 5-35 range
//    	        
    	        // CREATE NEW VECTOR3 FOR EACH DRONE!
    	        Vector3 targetPosition = new Vector3(targetX, targetY, targetZ);
    	        Vector3 targetVelocity = Vector3.zero();
    	        
    	        Drone drone = new Drone(
    	            i,                    // ID
    	            mass,                 // mass
    	            inertia,              // inertia
    	            kd,                   // drag coefficient
    	            initialPosition,      // initPos - starting position
    	            targetPosition,       // Rtar - target position
    	            targetVelocity,       // Vtar - target velocity
    	            initialVelocity,      // initVel - starting velocity
    	            initialRotation       // initRot - starting rotation
    	        );
    	        
    	        drones.add(drone);
    	        
    	        System.out.printf("  Drone %d: Target=(%.2f, %.2f, %.2f)%n",
    	            i, targetX, targetY, targetZ);
    	    }
    	    
    	    System.out.println();

    	    Controller controller = new Controller(kp, kdCtrl, kR, kOmega, mass);
    	    controller.setDesiredRotation(Matrix3D.identity());
    	    controller.setDesiredAngularVelocity(Vector3.zero());
    	    
    	    FormationManager formationManager = new FormationManager(formKp, formKv);
    	    CollisionAvoidance collisionAvoidance = new CollisionAvoidance(kRep, minSafeDistance);
    	    CommunicationModule communicationModule = new CommunicationModule(commRange, packetLoss);
    	    Environment environment = new Environment(envWidth, envHeight, envDepth);
    	    
    	    Logger l = new Logger("Positions.csv","Metrics.txt",minSafeDistance);
    	    
    	    communicationModule.setLogger(l);
    	    
    	    Simulator simulator = new Simulator(drones,controller,formationManager,collisionAvoidance,
    	    		communicationModule,environment,l,dt);
    	    
    	    System.out.println("Starting simulation...");
    	    System.out.println("Duration: " + totalTime + "s");
    	    System.out.println("Time step: " + dt + "s");
    	    System.out.println("Total steps: " + (int)(totalTime / dt));
    	    System.out.println();
    	    
    	    try {
    	        simulator.run(totalTime);
    	        
    	          
    	    } catch (Exception e) {
    	        System.err.println("Simulation failed: " + e.getMessage());
    	        e.printStackTrace();
    	    }
    	}
    	
    
}
