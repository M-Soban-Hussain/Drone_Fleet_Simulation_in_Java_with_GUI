package DroneSimulator;

import java.util.List;

public class FormationManager {

    private final double kp;  
    private final double kv;  

    
    public FormationManager(double kp, double kv) {
        this.kp = kp;
        this.kv = kv;
    }
    public Vector3 computeFormationForce(Drone self, List<Drone> neighbors) {
        
        Vector3 formationForce = Vector3.zero();
        
        Vector3 pi = self.getPosition();
        Vector3 vi = self.getVelocity();
        
        for (Drone neighbor : neighbors) {
            
            Vector3 pj = neighbor.getPosition();
            Vector3 vj = neighbor.getVelocity();
            
            Vector3 positionDiff = pi.subtract(pj);
            
            Vector3 velocityDiff = vi.subtract(vj);
            
            Vector3 force = positionDiff.multiply(kp).add(velocityDiff.multiply(kv));
            
            formationForce = formationForce.add(force);
        }
        
        return formationForce.multiply(-1);
    }

   
    
    public double getKp() {
        return kp;
    }
    
    public double getKv() {
        return kv;
    }
}