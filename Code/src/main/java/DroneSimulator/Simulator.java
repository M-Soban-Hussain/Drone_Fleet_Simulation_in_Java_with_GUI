package DroneSimulator;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Simulator {

    private final List<Drone> drones;
    private final Controller controller;
    private final FormationManager formationManager;
    private final CollisionAvoidance collisionAvoidance;
    private final CommunicationModule communicationModule;
    private final Environment environment;
    private final Logger logger;

    private double currentTime;
    private final double dt; 

    public Simulator(List<Drone> drones, Controller controller,FormationManager formationManager,CollisionAvoidance collisionAvoidance,
                     CommunicationModule communicationModule, Environment environment,Logger logger,double dt) {

        this.drones = drones;
        this.controller = controller;
        this.formationManager = formationManager;
        this.collisionAvoidance = collisionAvoidance;
        this.communicationModule = communicationModule;
        this.environment = environment;
        this.logger = logger;
        this.dt = dt;
        this.currentTime = 0.0;
    }
    
    public void step() throws IOException {

        for (Drone drone : drones) {
        	
            drone.resetForces();
            
            Vector3 gravity = new Vector3(0, 0, -9.81-0.2).multiply(drone.getMass());
            drone.applyGravity(gravity);

            List<Drone> neighbors = communicationModule.getNeighbors(drone, drones);

//            Vector3 thrust = ;
//            Vector3 torque = ;
            
            drone.applyThrust(controller.computeThrust(drone));
            drone.applyTorque(controller.computeTorque(drone));

            
            Vector3 formationForce = formationManager.computeFormationForce(drone, neighbors);
            drone.applyFormationForce(formationForce);
            
            Vector3 repulsiveForce = collisionAvoidance.computeRepulsiveForce(drone, neighbors);
            logger.setCollisionC(collisionAvoidance.getCol());
            
            drone.applyRepulsionForce(repulsiveForce);
            
            drone.sumAllForces();

            drone.integrateLinear(dt);
            drone.integrateAngular(dt);
            
            

            environment.applyBounds(drone);
            System.out.println("\nId: " +drone.getId()+" Position: "+drone.getPosition());

            logger.log(currentTime, drone);
        }
        currentTime += dt;
    }
    
    public void run(double duration) throws IOException {
        int numSteps = (int) (duration / dt);
        
        System.out.println("Simulation Start");
        System.out.println("Duration: " + duration + "s, Time step: " + dt + "s");
        System.out.println("Total steps: " + numSteps);
        System.out.println("Number of drones: " + drones.size());
        
        for (int i = 0; i < numSteps; i++) {
            step();
        }
        
        logger.finalize(drones);
        
        System.out.println("Simulation complete!");
        System.out.println("Final time: " + currentTime + "s");
    }
    
    public void reset() {
        currentTime = 0.0;
        
        for (Drone drone : drones) {
            drone.resetForces();
        }
        logger.reset();
    }

    public void addDrone(Drone drone) {
        drones.add(drone);
    }

    
    public List<Drone> getDrones() {
        return new ArrayList<>(drones);  
    }

    public double getCurrentTime() {
        return currentTime;
    }

    public double getTimeStep() {
        return dt;
    }

    public int getDroneCount() {
        return drones.size();
    }

    public Controller getController() {
        return controller;
    }

    public FormationManager getFormationManager() {
        return formationManager;
    }

    public CollisionAvoidance getCollisionAvoidance() {
        return collisionAvoidance;
    }

    public CommunicationModule getCommunicationModule() {
        return communicationModule;
    }

    public Environment getEnvironment() {
        return environment;
    }
    

    public Logger getLogger() {
        return logger;
    }

   
    
}
