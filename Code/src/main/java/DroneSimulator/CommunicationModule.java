package DroneSimulator;

import java.util.*;

public class CommunicationModule {
    
    public static final Random rand = new Random();
    
    private double range;       
    private double ploss; 
    private Logger logger;
    
    public CommunicationModule() {
        this.range = 0.0;
        this.ploss = 0.0;
        this.logger = null;
    }
    
    public CommunicationModule(double range, double ploss) {
        this.range = range;
        this.ploss = ploss;
        this.logger = null;

    }
    public void setLogger(Logger logger) {
        this.logger = logger;
    }
    
    public double getRange() {
        return range;
    }
    
    public double getPacketLoss() {
        return ploss;
    }
    
    public void setRange(double range) {
        this.range = range;
    }
    
    public void setPacketLoss(double ploss) {
        this.ploss = ploss;
    }
    public boolean canExchange(Vector3 pi, Vector3 pj) {
    	if((pi.distance(pj)<range) && (rand.nextDouble()>ploss)) {
    		return true;
    	}
        return false;
    }
    
    public List<Drone> getNeighbors(Drone self, List<Drone> allDrones) {
        List<Drone> neighbors = new ArrayList<>();
        Vector3 selfPos = self.getPosition();
        
        for (Drone other : allDrones) {
            if ((other.getId() != self.getId())&&(canExchange(selfPos, other.getPosition()))) {
                neighbors.add(other);
                logger.logCommunication(true);
            }
            logger.logCommunication(false);


        }
        
        return neighbors;
    }
    
    
 
}