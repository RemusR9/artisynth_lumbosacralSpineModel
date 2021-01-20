/**
 * Copyright (c) 2021, by the Authors: Robin Remus (RUB)
 *
 * This software is freely available under a 3-clause BSD license. Please see
 * the LICENSE file in the GitHub distribution directory for details.
 */
package artisynth.models.passiveLSS_PlosOne.myMaterials;


import java.util.Set;
import java.util.TreeMap;

import artisynth.core.materials.AxialMaterial;
import maspack.properties.PropertyList;



/** 
 * Ligament material model - implemented as kind of look up table -  
 * for any non-linear force-strain curves.
 * 
 * The ligament force (tension only) is interpolated from the imported treeMap 
 * in form of TreeMap<Float, Float>. First float is the strain value and 
 * second float is the force value, wished at specific strain. The interpolation 
 * between the treeMap entries is linear.
 * 
 * The treeMap entry might be scaled with a gain value, e.g. to use the same 
 * stress-strain curve for different fibers but divide the stress by the fibers 
 * cross section. 
 * 
 */
public class LookUpTableMaterial extends AxialMaterial {

   protected TreeMap<Float, Float> matMap = new TreeMap<Float, Float>();
   //protected double entryGain;
   
    
   public LookUpTableMaterial(TreeMap<Float, Float> myMatMap, 
		   double myEntryGain, double myForceGain) {     
      matMap    = myMatMap;     // [Pa]
      entryGain = myEntryGain;  
      forceGain = myForceGain;  // [m^2]
      updateMap(matMap, entryGain);     
   }
   
  
   private void updateMap(TreeMap<Float, Float> myMatMap, double myEntryGain) {   
	   // scale the matMap stress values with the entryGain    
	   //this.matMap = myMatMap;
	   Set<Float> keys = matMap.keySet ();  // get all keys
	      
	   // update each entry via their keys
      for (float key : keys) {
         matMap.compute (key, (Key, Val)
                                 -> (Val == null)
                                       ? 1
                                       : Val*(float)myEntryGain);
      }   
   }
     
   
   protected static TreeMap<Float, Float> DEFAULT_MATERIAL_MAP = null; 
   protected static double DEFAULT_ENTRY_GAIN = 1;
   protected static double DEFAULT_FORCE_GAIN = 1;
   protected static double DEFAULT_REFERENCE_STRAIN = 0.5e-2; // slack until that point
   protected static double DEFAULT_STIFFNESS_DAMPING = 10; // typ constant
   protected static double DEFAULT_NORMALIZED_DAMPING = 0.01; // typ constant
   protected static double DEFAULT_MAX_FORCE = 0.0; // typ ligament specific
   protected static double DEFAULT_DAMPING = 0.0;
   
   protected TreeMap<Float, Float> referenceMaterialMap = DEFAULT_MATERIAL_MAP;
   protected double entryGain = DEFAULT_ENTRY_GAIN;
   protected double forceGain = DEFAULT_FORCE_GAIN;
   protected double referenceStrain = DEFAULT_REFERENCE_STRAIN;
   protected double stiffnessDamping = DEFAULT_STIFFNESS_DAMPING;
   protected double normalizedDamping = DEFAULT_NORMALIZED_DAMPING;
   protected double maxForce = DEFAULT_MAX_FORCE;
   protected double myDamping = DEFAULT_DAMPING; // damping
    
   
   public static PropertyList myProps =
      new PropertyList (LookUpTableMaterial.class, AxialMaterial.class);
   
   static {
      myProps.add ("referenceStrain", "zero force strain (not implemented)", 
         DEFAULT_REFERENCE_STRAIN, "%.4g");
      myProps.add ("entryGain", "gain for entry values (read only)", 
         DEFAULT_ENTRY_GAIN, "%.4g");
   }

   
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }
   

   public LookUpTableMaterial () {
   }

   
   @Override
   public double computeF (double l, double ldot, double l0, 
		   double excitation) {

      float strain = 0.0f;
      double forceTotal = 0.0;
      double forceSpring = 0.0;
      double forceDamping = 0.0;
      double strainRate = 0.0;
      
      
      if (l <= l0) 
         return 0;
      

      // Compute strain
      strain = (float)((l-l0)/l0);
      
      // Evaluate elastic force
      forceSpring = interpolateStiffness(matMap, strain);          
      // Compute strain rate
      strainRate = ldot/l0;     
      forceDamping = stiffnessDamping*strainRate;        
      // Total force
      forceTotal = forceSpring + forceDamping;            
      // Ensure the model is only acting in tension
      if (forceTotal < 0.0)
         forceTotal = 0.0;
  
      return forceTotal;
   }

   
   
   @Override
   public double computeDFdl (
      double l, double ldot, double l0, double excitation) {
      
      // Compute strain
      double strain = (l-l0)/l0;
      double strain_dl = (1 + referenceStrain) / l0;         
      // use the calculated force as additional damping value
      double stiffness = interpolateStiffness(matMap, strain);   
      return stiffness/forceGain * strain * strain_dl * stiffnessDamping;  
      // XXX zero if not in tension
   }

   @Override
   public double computeDFdldot (
      double l, double ldot, double l0, double excitation) {  
	   
	   double strainRate_dldot =  (1.0 + referenceStrain) / l0;
	   return stiffnessDamping * normalizedDamping * strainRate_dldot;      
	   // XXX zero if not in tension
   }

   @Override
   public boolean isDFdldotZero () {
      return false;
   }

   
   
   private double interpolateStiffness(TreeMap<Float, Float> myMatMap, 
		   double strain) 
   {  
      //System.out.println("interpolateStiffness() entered with strain = " + strain);
      float myStrain = (float)strain;
      // Search in table and interpolate new value for current strain
      Float[] hiVals = new Float[2];
      hiVals[0] = matMap.higherKey(myStrain);               // x1
      hiVals[1] = matMap.higherEntry(myStrain).getValue();  // y1
      Float[] loVals  = new Float[2];
      loVals[0] = matMap.lowerKey(myStrain);                // x0
      loVals[1] = matMap.lowerEntry(myStrain).getValue();   // y0
      // interpolate new value for stiffness linearly
      Float myInterp     = loVals[1] + (myStrain - loVals[0]) * (
    		  (hiVals[1] - loVals[1]) / (hiVals[0] - loVals[0]));      
      double myStiffness = (double)myInterp;  
      return myStiffness*forceGain;  // [N = Pa*m^2] its still called stiffness, but technically its a force from now 
   }
   

   /* property methods */
   
   public double getStiffnessDamping () {
	   return stiffnessDamping;
   }

   public void setStiffnessDamping (double stiffnessDamping) {
	   this.stiffnessDamping = stiffnessDamping;
   }
    
   public double getEntryGain() {
      return entryGain;
   }
   
   public void setEntryGain(double entryGain) {
      this.entryGain = entryGain;
      updateMap(matMap, entryGain);
   }
   
   public double getForceGain() {
	  return forceGain;
   }
	   
   public void setForceGain(double forceGain) {
	   this.forceGain = forceGain;
   }
   
   public double getReferenceStrain() {
      return referenceStrain;
   }
   
   public void setReferenceStrain(double referenceStrain){
      this.referenceStrain = referenceStrain;
   }
   
   public double getNormalizedDamping () {
      return normalizedDamping;
   }

   public void setNormalizedDamping (double normalizedDamping) {
      this.normalizedDamping = normalizedDamping;
   }

   public double getMaxForce () {
      return maxForce;
   }

   public void setMaxForce (double maxForce) {
      this.maxForce = maxForce;
   }
}
