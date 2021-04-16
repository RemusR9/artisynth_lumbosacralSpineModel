/**
 * Copyright (c) 2021, by the Authors: Robin Remus (RUB)
 *
 * This software is freely available under a 3-clause BSD license. Please see
 * the LICENSE file in the GitHub distribution directory for details.
 * 
 * If you use the model for your research cite the following reference:
 * 
 * Remus R, Lipphaus A, Neumann M, Bender B. "Calibration and validation of a 
 * novel hybrid model of the lumbosacral spine in ArtiSynth ñ The passive 
 * structures", PLOS ONE (accepted), 2021
 */
package artisynth.models.passiveComponents_PlosOne;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.TreeMap;

import javax.swing.JLabel;
import javax.swing.JSeparator;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.CubicHyperelastic;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.materials.SimpleAxialMuscle;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.Collidable.Collidability;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.PointFrameAttachment;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RigidMeshComp;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.passiveComponents_PlosOne.MyImportFunctions.CollFiberData_MP;
import artisynth.models.passiveComponents_PlosOne.MyImportFunctions.LigamentData;
import artisynth.models.passiveComponents_PlosOne.MyImportFunctions.LigamentMPData;
import artisynth.models.passiveComponents_PlosOne.myMaterials.LookUpTableMaterial;
import artisynth.models.passiveComponents_PlosOne.myMaterials.MyUWLigamentMaterial;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.render.Renderer;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.spatialmotion.Wrench;


public class LSSBase extends RootModel {
		
	MechModel mech;
	
	// Lumbar vertebrae and vertebral bodies
	RigidBody L1RB, L2RB, L3RB, L4RB, L5RB, S1RB, PelvisRB, ThoraxRB;
  		
	// Proc. articularis superior
	RigidBody L2FacRB, L3FacRB, L4FacRB, L5FacRB, S1FacRB;

	// Anulus (AN) = Anulus ground substances (AGS) + collagen fibers (CF)
	// Single rings of AGS
	FemModel3d L12AnR1, L12AnR2, L12AnR3, L12AnR4, L23AnR1, L23AnR2, L23AnR3, 
		L23AnR4, L34AnR1, L34AnR2, L34AnR3, L34AnR4, L45AnR1, L45AnR2, L45AnR3, 
		L45AnR4, L51AnR1, L51AnR2, L51AnR3, L51AnR4;	
	// Merged AGS rings
	FemModel3d L12An, L23An, L34An, L45An, L51An;	
	// Nucleus pulposus (NP)
	FemModel3d L12Np, L23Np, L34Np, L45Np, L51Np;
   
	// Facet cartilage of facies articularis inferior
	FemModel3d L12FacL1l, L12FacL1r, L23FacL2l, L23FacL2r, L34FacL3l, L34FacL3r, 
		L45FacL4l, L45FacL4r, L51FacL5l, L51FacL5r;
		
	// Facet joints
	PolygonalMesh L12_postTriPM, L12_latTriPM, L23_postTriPM, L23_latTriPM, 
		L34_postTriPM, L34_latTriPM, L45_postTriPM, L45_latTriPM, 
		L51_postTriPM, L51_latTriPM;

	// Frames for external forces via wrenches on vertebral centers
	Frame Fr_WrL1, Fr_WrL2, Fr_WrL3, Fr_WrL4, Fr_WrL5;
	
	// Global component lists
	ComponentList<FemModel3d> L12AnR1234, L23AnR1234, L34AnR1234, L45AnR1234, 
		L51AnR1234;
	
	// Ligament lists for FSU levels
	ComponentList<ModelComponent> L12Ligs, L23Ligs, L34Ligs, L45Ligs, L51Ligs; 
	// Collagen fibers list
	ComponentList<ModelComponent> CF_L12, CF_L23, CF_L34, CF_L45, CF_L51;

	// Global node lists
	LinkedList<FemNode3d> L12Npa_ou, L23Npa_ou, L34Npa_ou, L45Npa_ou, L51Npa_ou;
	
	CollisionResponse L12_CRl, L12_CRr, L23_CRl, L23_CRr, L34_CRl, L34_CRr, 
		L45_CRl, L45_CRr, L51_CRl, L51_CRr;
	
	// Relevant points from RB geometries
	Point3d L1Body_CoM, L2Body_CoM, L3Body_CoM, L4Body_CoM, L5Body_CoM, 
		S1Body_CoM;
	
	// Muscles between Particles for Follower Load
	MultiPointMuscle  FL_LSSl, FL_LSSr;
	MuscleExciter FL_lr;
	
    // Color for rigid bones
	Color RBColor = new Color (0.30f, 0.30f, 0.30f);  // light gray
//	Color RBColor = new Color(0.93f, 0.93f, 0.68f);  // bone color  
	
	// Gravity 
	Vector3d gavityVec = new Vector3d (0, 0, -9.8);  // z-up
	
	
	//--------------------------------------------------------------------------
	// Select different card angle (CA) variations via names for ISSLS 2021 
	// sensitivity study of L4/L5 facet joints.
	// However, CAX* is to be understood here for all CAX. (CAX = 90∞ - CAX*)
	// "CAX0" or "" is the default card set used for calibration and validation 
	// in Remus et al. (2021), PloS ONE, 10.1371/journal.pone.0250456.
	// For example "...p5" stands for "...+5∞" and "...m5" for "...-5∞"
	boolean cardAngleStudy = false;  // for ISSLS21 study set to true
	String cardIDISSLS = "_CAX0"; // e.g. "_CAXp5", "_CAYm2", etc.
	//--------------------------------------------------------------------------
	
	
	

	// Build passive hybrid LSS model
	public void build(String[] args) throws IOException {
		
		// Relative folder path to the geometry files used by this model
		String geometryRelPath = "geometry/";
		MyImportFunctions.setGeometrySubFolder (geometryRelPath);
		
		
		// Add the mech model to the root model
		mech = new MechModel ("mech"); 
		addModel (mech);
		
		mech.setIntegrator (Integrator.ConstrainedBackwardEuler); 
		mech.setMaxStepSize (0.01);
		mech.setProfiling (false);
		mech.setGravity (gavityVec);
		
		
		// Build LSS model
		addRigidBones();		
		addRBFacets();			
		setDamping();
		
		addAttachments_RBRB();
		addConstraints();		
				
		addAuxBodies();	
		addFEDiscs();
		addFEFacets();
				
		setFEDiscMaterials();
		setFEFacMaterials();
			
		addAttachments_FERB();
		addAttachments_FEFE();
		
		// Adding optional FrameMarkers e.g for measurement purposes
		addFrameMarkers(false);
			
		addCollagenFibers();						
		addLigaments();
		addContacts();
				
		// Deactivate FL in active model
		addFollowerLoad(700, false);
		
		addMyExternalLoads(0.000);  // hide frames with = 0  // 0.025
		
		    
		// Remove all aiding AN-rings stored in ComponentLists 
		mech.remove (L12AnR1234);
		mech.remove (L23AnR1234);
		mech.remove (L34AnR1234);
		mech.remove (L45AnR1234);
		mech.remove (L51AnR1234);		
	              
		addMyControlPanel();     		
	}	
	

	
	
	
	/**
	 * Create contacts & collisions for facet joints
	 */
	private void addContacts() {						
		// Set General collisions between objects
		L12Np.setCollidable(Collidability.OFF);
		L23Np.setCollidable(Collidability.OFF);
		L34Np.setCollidable(Collidability.OFF);
		L45Np.setCollidable(Collidability.OFF);
		L51Np.setCollidable(Collidability.OFF);

		L12An.setCollidable(Collidability.OFF);
		L23An.setCollidable(Collidability.OFF);
		L34An.setCollidable(Collidability.OFF);
		L45An.setCollidable(Collidability.OFF);
		L51An.setCollidable(Collidability.OFF);
		
		L1RB.setCollidable(Collidability.OFF);
		L2RB.setCollidable(Collidability.OFF);
		L3RB.setCollidable(Collidability.OFF);
		L4RB.setCollidable(Collidability.OFF);
		L5RB.setCollidable(Collidability.OFF);
		S1RB.setCollidable(Collidability.OFF);
		
		L12FacL1l.setCollidable(Collidability.OFF);
		L12FacL1r.setCollidable(Collidability.OFF);
		L23FacL2l.setCollidable(Collidability.OFF);
		L23FacL2r.setCollidable(Collidability.OFF);
		L34FacL3l.setCollidable(Collidability.OFF);
		L34FacL3r.setCollidable(Collidability.OFF);
		L45FacL4l.setCollidable(Collidability.OFF);
		L45FacL4r.setCollidable(Collidability.OFF);
		L51FacL5l.setCollidable(Collidability.OFF);
		L51FacL5r.setCollidable(Collidability.OFF);
		
				
		// -------------------------------------------
		// Facet collisions - w/o friction - contacts with friction are 
		// currently not stable 
		CollisionBehavior L12_CollFacL, L12_CollFacR, L23_CollFacL, L23_CollFacR, 
			L34_CollFacL, L34_CollFacR, L45_CollFacL, L45_CollFacR, 
			L51_CollFacL, L51_CollFacR;
		L12_CollFacL = new CollisionBehavior (true, /*mu*/0.0);	
		L12_CollFacR = new CollisionBehavior (true, /*mu*/0.0);
		L23_CollFacL = new CollisionBehavior (true, /*mu*/0.0);	
		L23_CollFacR = new CollisionBehavior (true, /*mu*/0.0);
		L34_CollFacL = new CollisionBehavior (true, /*mu*/0.0);	
		L34_CollFacR = new CollisionBehavior (true, /*mu*/0.0);
		L45_CollFacL = new CollisionBehavior (true, /*mu*/0.0);	
		L45_CollFacR = new CollisionBehavior (true, /*mu*/0.0);
		L51_CollFacL = new CollisionBehavior (true, /*mu*/0.0);	
		L51_CollFacR = new CollisionBehavior (true, /*mu*/0.0);
				
		mech.setCollisionBehavior(L12FacL1l, L2FacRB, L12_CollFacL);
		mech.setCollisionBehavior(L12FacL1r, L2FacRB, L12_CollFacR);
		mech.setCollisionBehavior(L23FacL2l, L3FacRB, L23_CollFacL);
		mech.setCollisionBehavior(L23FacL2r, L3FacRB, L23_CollFacR);
		mech.setCollisionBehavior(L34FacL3l, L4FacRB, L34_CollFacL);
		mech.setCollisionBehavior(L34FacL3r, L4FacRB, L34_CollFacR);
		mech.setCollisionBehavior(L45FacL4l, L5FacRB, L45_CollFacL);
		mech.setCollisionBehavior(L45FacL4r, L5FacRB, L45_CollFacR);
		mech.setCollisionBehavior(L51FacL5l, S1FacRB, L51_CollFacL);
		mech.setCollisionBehavior(L51FacL5r, S1FacRB, L51_CollFacR);
		
		// Collision damping stabilizes the contacts
		double collFacD = 0.001;  
		L12_CollFacL.setDamping(collFacD);
		L12_CollFacR.setDamping(collFacD);
		L23_CollFacL.setDamping(collFacD);
		L23_CollFacR.setDamping(collFacD);
		L34_CollFacL.setDamping(collFacD);
		L34_CollFacR.setDamping(collFacD);
		L45_CollFacL.setDamping(collFacD);
		L45_CollFacR.setDamping(collFacD);
		L51_CollFacL.setDamping(collFacD);
		L51_CollFacR.setDamping(collFacD);
		
		
		// RENDER RPOPS	
		CollisionManager L123451_CM = mech.getCollisionManager();		
		RenderProps.setVisible(L123451_CM, true);
		RenderProps.setLineWidth(L123451_CM, 5);
		RenderProps.setLineColor(L123451_CM, Color.RED);
		L123451_CM.setDrawContactNormals(true);		
		
		// FE-body at first position (indx = 0)
		L12_CRl = mech.setCollisionResponse (L12FacL1l, L2FacRB); 
		L12_CRr = mech.setCollisionResponse (L12FacL1r, L2FacRB); 
		L23_CRl = mech.setCollisionResponse (L23FacL2l, L3FacRB); 
		L23_CRr = mech.setCollisionResponse (L23FacL2r, L3FacRB); 
		L34_CRl = mech.setCollisionResponse (L34FacL3l, L4FacRB); 
		L34_CRr = mech.setCollisionResponse (L34FacL3r, L4FacRB); 
		L45_CRl = mech.setCollisionResponse (L45FacL4l, L5FacRB); 
		L45_CRr = mech.setCollisionResponse (L45FacL4r, L5FacRB); 
		L51_CRl = mech.setCollisionResponse (L51FacL5l, S1FacRB); 
		L51_CRr = mech.setCollisionResponse (L51FacL5r, S1FacRB); 
	
		L12_CRl.setName("L12_CRl");
		L12_CRr.setName("L12_CRr");	
		L23_CRl.setName("L23_CRl");
		L23_CRr.setName("L23_CRr");
		L34_CRl.setName("L34_CRl");
		L34_CRr.setName("L34_CRr");
		L45_CRl.setName("L45_CRl");
		L45_CRr.setName("L45_CRr");
		L51_CRl.setName("L51_CRl");
		L51_CRr.setName("L51_CRr");		
	}



	/**
	 * Create intervertebral ligaments, set material properties (using 
	 * customized UWLigamentMaterial), and set render props
	 */
	private void addLigaments() {
		ComponentList<ModelComponent> ligs = 
				new ComponentList<ModelComponent>(ModelComponent.class, "ligaments");
	
		Boolean isUW = true; 	// use UW-Material
		Color LigColor = new Color(1.00f, 0.92f, 0.70f);
		
		// (1) Ligaments as AxialSprings (AS) 				
		// Import ligament info (.lig) and material files
		LigamentData LigsDataImpL12, LigsDataImpL23, LigsDataImpL34, 
			LigsDataImpL45, LigsDataImpL51;
		LigsDataImpL12 = MyImportFunctions.ImportLigsData("L12_Ligs_AS", isUW, 
				  		  							    "L12_Ligs_AS_UWMat");
		LigsDataImpL23 = MyImportFunctions.ImportLigsData("L23_Ligs_AS", isUW, 
				  										"L23_Ligs_AS_UWMat");
		LigsDataImpL34 = MyImportFunctions.ImportLigsData("L34_Ligs_AS", isUW, 
	              										"L34_Ligs_AS_UWMat");
		LigsDataImpL45 = MyImportFunctions.ImportLigsData("L45_Ligs_AS", isUW, 
														"L45_Ligs_AS_UWMat");
		LigsDataImpL51 = MyImportFunctions.ImportLigsData("L51_Ligs_AS", isUW, 
	              										"L51_Ligs_AS_UWMat"); 
		
		// Optional decision which ligaments should be used can be performed 
		// here (e.g. for calibration)			
		// Get sum of all axial springs (AS) which describes the ligaments
		int ligASSumL12, ligASSumL23, ligASSumL34, ligASSumL45, ligASSumL51; 
		ligASSumL12 = LigsDataImpL12.Names.size();   
		ligASSumL23 = LigsDataImpL23.Names.size();   
		ligASSumL34 = LigsDataImpL34.Names.size();   
		ligASSumL45 = LigsDataImpL45.Names.size(); 
		ligASSumL51 = LigsDataImpL51.Names.size();
		// Use all ligaments that are provided via .lig-file		
		int ligsToUseL12[] = {0, ligASSumL12-1};  	// --> build all ligaments 
		int ligsToUseL23[] = {0, ligASSumL23-1};  			
		int ligsToUseL34[] = {0, ligASSumL34-1};  			
		int ligsToUseL45[] = {0, ligASSumL45-1};  
		int ligsToUseL51[] = {0, ligASSumL51-1};		
		// Use the ligaments from ... to; define by their indices
		// ALL+PLL+FC/CL: 0:19  -> w/o FL
		// ALL+PLL+FC/CL+FL: 0:30   -> w/o ISL
		// ALL+PLL+FC/CL+FL+ISL: 0:38   -> w/o SSL (& w/o ITL)
		//int LigsToUseL45[] = {0, 19};  // -> build by indices named ligaments only
		
		double[] ligIndxsL12_d, ligIndxsL23_d, ligIndxsL34_d, ligIndxsL45_d, ligIndxsL51_d;
		ligIndxsL12_d = linspace(ligsToUseL12[0], ligsToUseL12[1], ligsToUseL12[1]+1);
		ligIndxsL23_d = linspace(ligsToUseL23[0], ligsToUseL23[1], ligsToUseL23[1]+1);
		ligIndxsL34_d = linspace(ligsToUseL34[0], ligsToUseL34[1], ligsToUseL34[1]+1);
		ligIndxsL45_d = linspace(ligsToUseL45[0], ligsToUseL45[1], ligsToUseL45[1]+1);
		ligIndxsL51_d = linspace(ligsToUseL51[0], ligsToUseL51[1], ligsToUseL51[1]+1);
		
		int[] ligIndxsL12, ligIndxsL23, ligIndxsL34, ligIndxsL45, ligIndxsL51;
		ligIndxsL12 = new int[ligIndxsL12_d.length];
		ligIndxsL23 = new int[ligIndxsL23_d.length];
		ligIndxsL34 = new int[ligIndxsL34_d.length];
		ligIndxsL45 = new int[ligIndxsL45_d.length];
		ligIndxsL51 = new int[ligIndxsL51_d.length];
		// get indices of ligaments
		for(int i=0; i<ligIndxsL12_d.length; ++i)
			ligIndxsL12[i] = (int) ligIndxsL12_d[i];
		for(int i=0; i<ligIndxsL23_d.length; ++i)
			ligIndxsL23[i] = (int) ligIndxsL23_d[i];
		for(int i=0; i<ligIndxsL34_d.length; ++i)
			ligIndxsL34[i] = (int) ligIndxsL34_d[i];
		for(int i=0; i<ligIndxsL45_d.length; ++i)
			ligIndxsL45[i] = (int) ligIndxsL45_d[i];
		for(int i=0; i<ligIndxsL51_d.length; ++i)
			ligIndxsL51[i] = (int) ligIndxsL51_d[i];
		
		L12Ligs = new ComponentList<ModelComponent>(ModelComponent.class, "L12Ligs");
		L23Ligs = new ComponentList<ModelComponent>(ModelComponent.class, "L23Ligs");
		L34Ligs = new ComponentList<ModelComponent>(ModelComponent.class, "L34Ligs");
		L45Ligs = new ComponentList<ModelComponent>(ModelComponent.class, "L45Ligs");
		L51Ligs = new ComponentList<ModelComponent>(ModelComponent.class, "L51Ligs");
		
		// Call function in loop for setting up the ligaments by there indexes
		for (int Indx : ligIndxsL12) 
			CreateUWLigament(L12Ligs, LigsDataImpL12, Indx, L1RB, L2RB, mech, LigColor);
		for (int Indx : ligIndxsL23) 
			CreateUWLigament(L23Ligs, LigsDataImpL23, Indx, L2RB, L3RB, mech, LigColor);
		for (int Indx : ligIndxsL34) 
			CreateUWLigament(L34Ligs, LigsDataImpL34, Indx, L3RB, L4RB, mech, LigColor);   
		for (int Indx : ligIndxsL45) 
			CreateUWLigament(L45Ligs, LigsDataImpL45, Indx, L4RB, L5RB, mech, LigColor);
		for (int Indx : ligIndxsL51) 
			CreateUWLigament(L51Ligs, LigsDataImpL51, Indx, L5RB, S1RB, mech, LigColor); 
		
		
		
		// (2) Ligaments from MULTI POINT (MP) springs
		// Import ligament geometry and material files 
		LigamentMPData LigsMPDataImpL12, LigsMPDataImpL23, LigsMPDataImpL34, 
			LigsMPDataImpL45, LigsMPDataImpL51;
		LigsMPDataImpL12 =  MyImportFunctions.ImportLigsMPData("L12_Ligs_MP", 
				    										 "L12_Ligs_MP_UWMat",
				    										  L12An);
		LigsMPDataImpL23 =  MyImportFunctions.ImportLigsMPData("L23_Ligs_MP", 
				    										 "L23_Ligs_MP_UWMat",
				    										  L23An);
		LigsMPDataImpL34 =  MyImportFunctions.ImportLigsMPData("L34_Ligs_MP", 
		         											 "L34_Ligs_MP_UWMat",
		         											  L34An);
		LigsMPDataImpL45 =  MyImportFunctions.ImportLigsMPData("L45_Ligs_MP", 
															 "L45_Ligs_MP_UWMat",
														      L45An); 
		LigsMPDataImpL51 =  MyImportFunctions.ImportLigsMPData("L51_Ligs_MP", 
	            										     "L51_Ligs_MP_UWMat",
	            											  L51An);
		
		// Decide which MP Ligaments should be build
		// get sum of all axial springs (AS) which describes the ligaments
		int LigMPASSumL12, LigMPASSumL23, LigMPASSumL34, LigMPASSumL45, LigMPASSumL51;
		LigMPASSumL12 = LigsMPDataImpL12.Names.size(); 
		LigMPASSumL23 = LigsMPDataImpL23.Names.size();
		LigMPASSumL34 = LigsMPDataImpL34.Names.size(); 
		LigMPASSumL45 = LigsMPDataImpL45.Names.size();   
		LigMPASSumL51 = LigsMPDataImpL51.Names.size();
		int LigMPsToUseL12[] = {0, LigMPASSumL12-1};  // --> build all ligaments 
		int LigMPsToUseL23[] = {0, LigMPASSumL23-1};  
		int LigMPsToUseL34[] = {0, LigMPASSumL34-1};  
		int LigMPsToUseL45[] = {0, LigMPASSumL45-1};
		int LigMPsToUseL51[] = {0, LigMPASSumL51-1};
		
		double[] LigMPIndxsL12_d, LigMPIndxsL23_d, LigMPIndxsL34_d, LigMPIndxsL45_d, LigMPIndxsL51_d;
		LigMPIndxsL12_d = linspace(LigMPsToUseL12[0], LigMPsToUseL12[1], LigMPsToUseL12[1]+1);
		LigMPIndxsL23_d = linspace(LigMPsToUseL23[0], LigMPsToUseL23[1], LigMPsToUseL23[1]+1);
		LigMPIndxsL34_d = linspace(LigMPsToUseL34[0], LigMPsToUseL34[1], LigMPsToUseL34[1]+1);
		LigMPIndxsL45_d = linspace(LigMPsToUseL45[0], LigMPsToUseL45[1], LigMPsToUseL45[1]+1);
		LigMPIndxsL51_d = linspace(LigMPsToUseL51[0], LigMPsToUseL51[1], LigMPsToUseL51[1]+1);
		int[] LigMPIndxsL12, LigMPIndxsL23, LigMPIndxsL34, LigMPIndxsL45, LigMPIndxsL51;
		LigMPIndxsL12 = new int[LigMPIndxsL12_d.length];
		LigMPIndxsL23 = new int[LigMPIndxsL23_d.length];
		LigMPIndxsL34 = new int[LigMPIndxsL34_d.length];
		LigMPIndxsL45 = new int[LigMPIndxsL45_d.length];
		LigMPIndxsL51 = new int[LigMPIndxsL51_d.length];
		// get indices of ligaments
		for(int i=0; i<LigMPIndxsL12_d.length; ++i)
			LigMPIndxsL12[i] = (int) LigMPIndxsL12_d[i];
		for(int i=0; i<LigMPIndxsL23_d.length; ++i)
			LigMPIndxsL23[i] = (int) LigMPIndxsL23_d[i];
		for(int i=0; i<LigMPIndxsL34_d.length; ++i)
			LigMPIndxsL34[i] = (int) LigMPIndxsL34_d[i];
		for(int i=0; i<LigMPIndxsL45_d.length; ++i)
			LigMPIndxsL45[i] = (int) LigMPIndxsL45_d[i];
		for(int i=0; i<LigMPIndxsL51_d.length; ++i)
			LigMPIndxsL51[i] = (int) LigMPIndxsL51_d[i];
		
		// Call function in loop for setting up the ligaments by there indexes
		for (int IndxMP : LigMPIndxsL12) 
			CreateUWLigamentMP(L12Ligs, LigsMPDataImpL12, IndxMP, L1RB, L2RB, mech, LigColor);
		for (int IndxMP : LigMPIndxsL23) 
			CreateUWLigamentMP(L23Ligs, LigsMPDataImpL23, IndxMP, L2RB, L3RB, mech, LigColor);
		for (int IndxMP : LigMPIndxsL34) 
			CreateUWLigamentMP(L34Ligs, LigsMPDataImpL34, IndxMP, L3RB, L4RB, mech, LigColor);
		for (int IndxMP : LigMPIndxsL45) 
			CreateUWLigamentMP(L45Ligs, LigsMPDataImpL45, IndxMP, L4RB, L5RB, mech, LigColor);  
		for (int IndxMP : LigMPIndxsL51) 
			CreateUWLigamentMP(L51Ligs, LigsMPDataImpL51, IndxMP, L5RB, S1RB, mech, LigColor);  
		
		ligs.add(L12Ligs);
		ligs.add(L23Ligs);
		ligs.add(L34Ligs);
		ligs.add(L45Ligs);
		ligs.add(L51Ligs);	
		mech.add(ligs);
	}




	/**
	 *  Create Multi-Point collagen fibers (CF) from imported text-files
	 *	containing node info about CF paths.
	 */
	private void addCollagenFibers() throws IOException { 
		ComponentList<ModelComponent> CFs = 
				new ComponentList<ModelComponent>(ModelComponent.class, "collagenFibers");
		
		// Import data about collagen fiber positions from text file (.fiber)		
		CollFiberData_MP CF_L12AnR1i, CF_L12AnR1a, CF_L12AnR2a, CF_L12AnR3a, CF_L12AnR4a;
		CF_L12AnR1i = MyImportFunctions.ImportCollFibersMP ("L12_An_R1i", L12AnR1);
		CF_L12AnR1a = MyImportFunctions.ImportCollFibersMP ("L12_An_R1a", L12AnR1);
		CF_L12AnR2a = MyImportFunctions.ImportCollFibersMP ("L12_An_R2a", L12AnR2);
		CF_L12AnR3a = MyImportFunctions.ImportCollFibersMP ("L12_An_R3a", L12AnR3);
		CF_L12AnR4a = MyImportFunctions.ImportCollFibersMP ("L12_An_R4a", L12AnR4);
		
		CollFiberData_MP CF_L23AnR1i, CF_L23AnR1a, CF_L23AnR2a, CF_L23AnR3a, CF_L23AnR4a;
		CF_L23AnR1i = MyImportFunctions.ImportCollFibersMP ("L23_An_R1i", L23AnR1);
		CF_L23AnR1a = MyImportFunctions.ImportCollFibersMP ("L23_An_R1a", L23AnR1);
		CF_L23AnR2a = MyImportFunctions.ImportCollFibersMP ("L23_An_R2a", L23AnR2);
		CF_L23AnR3a = MyImportFunctions.ImportCollFibersMP ("L23_An_R3a", L23AnR3);
		CF_L23AnR4a = MyImportFunctions.ImportCollFibersMP ("L23_An_R4a", L23AnR4);
		
		CollFiberData_MP CF_L34AnR1i, CF_L34AnR1a, CF_L34AnR2a, CF_L34AnR3a, CF_L34AnR4a;
		CF_L34AnR1i = MyImportFunctions.ImportCollFibersMP ("L34_An_R1i", L34AnR1);
		CF_L34AnR1a = MyImportFunctions.ImportCollFibersMP ("L34_An_R1a", L34AnR1);
		CF_L34AnR2a = MyImportFunctions.ImportCollFibersMP ("L34_An_R2a", L34AnR2);
		CF_L34AnR3a = MyImportFunctions.ImportCollFibersMP ("L34_An_R3a", L34AnR3);
		CF_L34AnR4a = MyImportFunctions.ImportCollFibersMP ("L34_An_R4a", L34AnR4);
		
		CollFiberData_MP CF_L45AnR1i, CF_L45AnR1a, CF_L45AnR2a, CF_L45AnR3a, CF_L45AnR4a;
		CF_L45AnR1i = MyImportFunctions.ImportCollFibersMP ("L45_An_R1i", L45AnR1);
		CF_L45AnR1a = MyImportFunctions.ImportCollFibersMP ("L45_An_R1a", L45AnR1);
		CF_L45AnR2a = MyImportFunctions.ImportCollFibersMP ("L45_An_R2a", L45AnR2);
		CF_L45AnR3a = MyImportFunctions.ImportCollFibersMP ("L45_An_R3a", L45AnR3);
		CF_L45AnR4a = MyImportFunctions.ImportCollFibersMP ("L45_An_R4a", L45AnR4);
		
		CollFiberData_MP CF_L51AnR1i, CF_L51AnR1a, CF_L51AnR2a, CF_L51AnR3a, CF_L51AnR4a;
		CF_L51AnR1i = MyImportFunctions.ImportCollFibersMP ("L51_An_R1i", L51AnR1);
		CF_L51AnR1a = MyImportFunctions.ImportCollFibersMP ("L51_An_R1a", L51AnR1);
		CF_L51AnR2a = MyImportFunctions.ImportCollFibersMP ("L51_An_R2a", L51AnR2);
		CF_L51AnR3a = MyImportFunctions.ImportCollFibersMP ("L51_An_R3a", L51AnR3);
		CF_L51AnR4a = MyImportFunctions.ImportCollFibersMP ("L51_An_R4a", L51AnR4);
		
		// Import and mesh RB (triangle) to define different areas of disc
		// Create the meshs automatically from RB to be able to search for 
		// containing nodes
		double mQuali = 10;
		FemModel3d myL12_postTriFEM, myL12_latTriFEM, myL23_postTriFEM, 
			myL23_latTriFEM, myL34_postTriFEM, myL34_latTriFEM, myL45_postTriFEM,
			myL45_latTriFEM, myL51_postTriFEM, myL51_latTriFEM;
		myL12_postTriFEM = FemFactory.createFromMesh (null, L12_postTriPM, mQuali);
		myL12_latTriFEM  = FemFactory.createFromMesh (null, L12_latTriPM,  mQuali);
		myL23_postTriFEM = FemFactory.createFromMesh (null, L23_postTriPM, mQuali);
		myL23_latTriFEM  = FemFactory.createFromMesh (null, L23_latTriPM,  mQuali);
		myL34_postTriFEM = FemFactory.createFromMesh (null, L34_postTriPM, mQuali);
		myL34_latTriFEM  = FemFactory.createFromMesh (null, L34_latTriPM,  mQuali);
		myL45_postTriFEM = FemFactory.createFromMesh (null, L45_postTriPM, mQuali);
		myL45_latTriFEM  = FemFactory.createFromMesh (null, L45_latTriPM,  mQuali);
		myL51_postTriFEM = FemFactory.createFromMesh (null, L51_postTriPM, mQuali);
		myL51_latTriFEM  = FemFactory.createFromMesh (null, L51_latTriPM,  mQuali);
		
		// Define linear CF stiffness fraction in radial direction 
		// from inner to outer layers (Schmidt et al., 2006)
		double[] CFStiffFracs = {0.65, 0.738, 0.825, 0.913, 1.0};

		// Calc CF radius and cross sections from known constraints: CF-lengths, AN-volume etc.
		double[] L12_CFLengths = {CF_L12AnR1i.CFTotLength, CF_L12AnR1a.CFTotLength,
								  CF_L12AnR2a.CFTotLength, CF_L12AnR3a.CFTotLength,
								  CF_L12AnR4a.CFTotLength};
		double[] L23_CFLengths = {CF_L23AnR1i.CFTotLength, CF_L23AnR1a.CFTotLength,
								  CF_L23AnR2a.CFTotLength, CF_L23AnR3a.CFTotLength,
								  CF_L23AnR4a.CFTotLength};
		double[] L34_CFLengths = {CF_L34AnR1i.CFTotLength, CF_L34AnR1a.CFTotLength,
							  	  CF_L34AnR2a.CFTotLength, CF_L34AnR3a.CFTotLength,
							  	  CF_L34AnR4a.CFTotLength};	
		double[] L45_CFLengths = {CF_L45AnR1i.CFTotLength, CF_L45AnR1a.CFTotLength,
		  						  CF_L45AnR2a.CFTotLength, CF_L45AnR3a.CFTotLength,
		  						  CF_L45AnR4a.CFTotLength};	
		double[] L51_CFLengths = {CF_L51AnR1i.CFTotLength, CF_L51AnR1a.CFTotLength,
								  CF_L51AnR2a.CFTotLength, CF_L51AnR3a.CFTotLength,
								  CF_L51AnR4a.CFTotLength};
		
		// Calc the fundamental radius from known constraints 
		// Volume of all CFs from AN := 16%
		double L12_CF_r = calcCFRadius(CFStiffFracs, L12_CFLengths, L12An, 0.16);  
		double L23_CF_r = calcCFRadius(CFStiffFracs, L23_CFLengths, L23An, 0.16);  
		double L34_CF_r = calcCFRadius(CFStiffFracs, L34_CFLengths, L34An, 0.16);  
		double L45_CF_r = calcCFRadius(CFStiffFracs, L45_CFLengths, L45An, 0.16);  
		double L51_CF_r = calcCFRadius(CFStiffFracs, L51_CFLengths, L51An, 0.16);  
		
				
		// Calculate and set the material properties of the fibers
		String[] L45MapNames = {"L45_CFTreeMap_mainMat", 
				"L45_CFTreeMap_dorsalMat", "L45_CFTreeMap_lateralMat"};
		String[] L12MapNames = L45MapNames; // keep material values from L45 CFs
		String[] L23MapNames = L45MapNames; 
		String[] L34MapNames = L45MapNames; 
		String[] L51MapNames = L45MapNames; 
		
		RenderableComponentList<MultiPointSpring> CF_L12R1i, CF_L12R1a, CF_L12R2a, 
			CF_L12R3a, CF_L12R4a, CF_L23R1i, CF_L23R1a, CF_L23R2a, CF_L23R3a, 
			CF_L23R4a, CF_L34R1i, CF_L34R1a, CF_L34R2a, CF_L34R3a, CF_L34R4a, 
			CF_L45R1i, CF_L45R1a, CF_L45R2a, CF_L45R3a, CF_L45R4a, CF_L51R1i, 
			CF_L51R1a, CF_L51R2a, CF_L51R3a, CF_L51R4a;	
		CF_L12R1i = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L12R1i");
		CF_L12R1a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L12R1a");
		CF_L12R2a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L12R2a");
		CF_L12R3a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L12R3a");
		CF_L12R4a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L12R4a");
		CF_L23R1i = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L23R1i");
		CF_L23R1a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L23R1a");
		CF_L23R2a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L23R2a");
		CF_L23R3a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L23R3a");
		CF_L23R4a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L23R4a");
		CF_L34R1i = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L34R1i");
		CF_L34R1a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L34R1a");
		CF_L34R2a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L34R2a");
		CF_L34R3a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L34R3a");
		CF_L34R4a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L34R4a");
		CF_L45R1i = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L45R1i");
		CF_L45R1a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L45R1a");
		CF_L45R2a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L45R2a");
		CF_L45R3a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L45R3a");
		CF_L45R4a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L45R4a");
		CF_L51R1i = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L51R1i");
		CF_L51R1a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L51R1a");
		CF_L51R2a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L51R2a");
		CF_L51R3a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L51R3a");
		CF_L51R4a = new RenderableComponentList<MultiPointSpring>(MultiPointSpring.class, "CF_L51R4a");
		
		createCF_MP(CF_L12AnR1i, CF_L12R1i, L12An, myL12_postTriFEM, 
				myL12_latTriFEM, L12_CF_r, CFStiffFracs[0], L12MapNames);  
		createCF_MP(CF_L12AnR1a, CF_L12R1a, L12An, myL12_postTriFEM, 
				myL12_latTriFEM, L12_CF_r, CFStiffFracs[1], L12MapNames);  
		createCF_MP(CF_L12AnR2a, CF_L12R2a, L12An, myL12_postTriFEM, 
				myL12_latTriFEM, L12_CF_r, CFStiffFracs[2], L12MapNames);
		createCF_MP(CF_L12AnR3a, CF_L12R3a, L12An, myL12_postTriFEM, 
				myL12_latTriFEM, L12_CF_r, CFStiffFracs[3], L12MapNames);
		createCF_MP(CF_L12AnR4a, CF_L12R4a, L12An, myL12_postTriFEM, 
				myL12_latTriFEM, L12_CF_r, CFStiffFracs[4], L12MapNames);
		
		createCF_MP(CF_L23AnR1i, CF_L23R1i, L23An, myL23_postTriFEM, 
				myL23_latTriFEM, L23_CF_r, CFStiffFracs[0], L23MapNames);  
		createCF_MP(CF_L23AnR1a, CF_L23R1a, L23An, myL23_postTriFEM, 
				myL23_latTriFEM, L23_CF_r, CFStiffFracs[1], L23MapNames);  
		createCF_MP(CF_L23AnR2a, CF_L23R2a, L23An, myL23_postTriFEM, 
				myL23_latTriFEM, L23_CF_r, CFStiffFracs[2], L23MapNames);
		createCF_MP(CF_L23AnR3a, CF_L23R3a, L23An, myL23_postTriFEM, 
				myL23_latTriFEM, L23_CF_r, CFStiffFracs[3], L23MapNames);
		createCF_MP(CF_L23AnR4a, CF_L23R4a, L23An, myL23_postTriFEM, 
				myL23_latTriFEM, L23_CF_r, CFStiffFracs[4], L23MapNames);
		
		createCF_MP(CF_L34AnR1i, CF_L34R1i, L34An, myL34_postTriFEM, 
				myL34_latTriFEM, L34_CF_r, CFStiffFracs[0], L34MapNames);  
		createCF_MP(CF_L34AnR1a, CF_L34R1a, L34An, myL34_postTriFEM, 
				myL34_latTriFEM, L34_CF_r, CFStiffFracs[1], L34MapNames);  
		createCF_MP(CF_L34AnR2a, CF_L34R2a, L34An, myL34_postTriFEM, 
				myL34_latTriFEM, L34_CF_r, CFStiffFracs[2], L34MapNames);
		createCF_MP(CF_L34AnR3a, CF_L34R3a, L34An, myL34_postTriFEM, 
				myL34_latTriFEM, L34_CF_r, CFStiffFracs[3], L34MapNames);
		createCF_MP(CF_L34AnR4a, CF_L34R4a, L34An, myL34_postTriFEM, 
				myL34_latTriFEM, L34_CF_r, CFStiffFracs[4], L34MapNames);
		
		createCF_MP(CF_L45AnR1i, CF_L45R1i, L45An, myL45_postTriFEM, 
				myL45_latTriFEM, L45_CF_r, CFStiffFracs[0], L45MapNames);  
		createCF_MP(CF_L45AnR1a, CF_L45R1a, L45An, myL45_postTriFEM, 
				myL45_latTriFEM, L45_CF_r, CFStiffFracs[1], L45MapNames);  
		createCF_MP(CF_L45AnR2a, CF_L45R2a, L45An, myL45_postTriFEM, 
				myL45_latTriFEM, L45_CF_r, CFStiffFracs[2], L45MapNames);
		createCF_MP(CF_L45AnR3a, CF_L45R3a, L45An, myL45_postTriFEM, 
				myL45_latTriFEM, L45_CF_r, CFStiffFracs[3], L45MapNames);
		createCF_MP(CF_L45AnR4a, CF_L45R4a, L45An, myL45_postTriFEM, 
				myL45_latTriFEM, L45_CF_r, CFStiffFracs[4], L45MapNames);
		
		createCF_MP(CF_L51AnR1i, CF_L51R1i, L51An, myL51_postTriFEM, 
				myL51_latTriFEM, L51_CF_r, CFStiffFracs[0], L51MapNames);  
		createCF_MP(CF_L51AnR1a, CF_L51R1a, L51An, myL51_postTriFEM, 
				myL51_latTriFEM, L51_CF_r, CFStiffFracs[1], L51MapNames);  
		createCF_MP(CF_L51AnR2a, CF_L51R2a, L51An, myL51_postTriFEM, 
				myL51_latTriFEM, L51_CF_r, CFStiffFracs[2], L51MapNames);
		createCF_MP(CF_L51AnR3a, CF_L51R3a, L51An, myL51_postTriFEM, 
				myL51_latTriFEM, L51_CF_r, CFStiffFracs[3], L51MapNames);
		createCF_MP(CF_L51AnR4a, CF_L51R4a, L51An, myL51_postTriFEM, 
				myL51_latTriFEM, L51_CF_r, CFStiffFracs[4], L51MapNames);

		// Validation of collagen fiber volume fraction/content in Anulus
		// The CFs total fraction must be about 16% (Shirazi-Adl et al. 1986)
		double CFVolTotal = CF_L45AnR1i.CFCrossSec * CF_L45AnR1i.CFTotLength +
							CF_L45AnR1a.CFCrossSec * CF_L45AnR1a.CFTotLength +
							CF_L45AnR2a.CFCrossSec * CF_L45AnR2a.CFTotLength +
							CF_L45AnR3a.CFCrossSec * CF_L45AnR3a.CFTotLength +
							CF_L45AnR4a.CFCrossSec * CF_L45AnR4a.CFTotLength;
		double L45TotVol           = L45An.updateVolume();
		double Valid_L45CFsVolFrac = CFVolTotal/L45TotVol;  
		boolean validate = false;
		if (validate) {
			System.out.println("Validation: Total CFs volume fraction in anulus L45: " 
					+ String.format("%.3f %n", Valid_L45CFsVolFrac));
		}

	
		// add all (renderable) fiber components to one disc respectively
		CF_L12 = new ComponentList<ModelComponent> (ModelComponent.class, "CF_L12");  	
		CF_L23 = new ComponentList<ModelComponent> (ModelComponent.class, "CF_L23");  	
		CF_L34 = new ComponentList<ModelComponent> (ModelComponent.class, "CF_L34");  	
		CF_L45 = new ComponentList<ModelComponent> (ModelComponent.class, "CF_L45");  	
		CF_L51 = new ComponentList<ModelComponent> (ModelComponent.class, "CF_L51"); 
		
		CF_L12.add(CF_L12R1i);
		CF_L12.add(CF_L12R1a);
		CF_L12.add(CF_L12R2a);
		CF_L12.add(CF_L12R3a);
		CF_L12.add(CF_L12R4a);
		
		CF_L23.add(CF_L23R1i);
		CF_L23.add(CF_L23R1a);
		CF_L23.add(CF_L23R2a);
		CF_L23.add(CF_L23R3a);
		CF_L23.add(CF_L23R4a);
		
		CF_L34.add(CF_L34R1i);
		CF_L34.add(CF_L34R1a);
		CF_L34.add(CF_L34R2a);
		CF_L34.add(CF_L34R3a);
		CF_L34.add(CF_L34R4a);
		
		CF_L45.add(CF_L45R1i);
		CF_L45.add(CF_L45R1a);
		CF_L45.add(CF_L45R2a);
		CF_L45.add(CF_L45R3a);
		CF_L45.add(CF_L45R4a);
		
		CF_L51.add(CF_L51R1i);
		CF_L51.add(CF_L51R1a);
		CF_L51.add(CF_L51R2a);
		CF_L51.add(CF_L51R3a);
		CF_L51.add(CF_L51R4a);
		
		// set the render properties of the CF RenderableComponentLists  	// good value for radius 0.00011
		double CF_ScaleR = 1;  // Scales the "real" radius of the CFs by reduceR
		RenderProps.setCylindricalLines(CF_L12R1i, Math.sqrt (
				CF_L12AnR1i.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.95f, 0.95f, 0.2f));
		RenderProps.setCylindricalLines(CF_L12R1a, Math.sqrt (
				CF_L12AnR1a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.90f, 0.75f, 0.2f));
		RenderProps.setCylindricalLines(CF_L12R2a, Math.sqrt (
				CF_L12AnR2a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.85f, 0.60f, 0.2f));
		RenderProps.setCylindricalLines(CF_L12R3a, Math.sqrt (
				CF_L12AnR3a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.80f, 0.45f, 0.2f));
		RenderProps.setCylindricalLines(CF_L12R4a, Math.sqrt (
				CF_L12AnR4a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.80f, 0.45f, 0.2f));
		
		RenderProps.setCylindricalLines(CF_L23R1i, Math.sqrt (
				CF_L23AnR1i.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.95f, 0.95f, 0.2f));
		RenderProps.setCylindricalLines(CF_L23R1a, Math.sqrt (
				CF_L23AnR1a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.90f, 0.75f, 0.2f));
		RenderProps.setCylindricalLines(CF_L23R2a, Math.sqrt (
				CF_L23AnR2a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.85f, 0.60f, 0.2f));
		RenderProps.setCylindricalLines(CF_L23R3a, Math.sqrt (
				CF_L23AnR3a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.80f, 0.45f, 0.2f));
		RenderProps.setCylindricalLines(CF_L23R4a, Math.sqrt (
				CF_L23AnR4a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.80f, 0.45f, 0.2f));
		
		RenderProps.setCylindricalLines(CF_L34R1i, Math.sqrt (
				CF_L34AnR1i.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.95f, 0.95f, 0.2f));
		RenderProps.setCylindricalLines(CF_L34R1a, Math.sqrt (
				CF_L34AnR1a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.90f, 0.75f, 0.2f));
		RenderProps.setCylindricalLines(CF_L34R2a, Math.sqrt (
				CF_L34AnR2a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.85f, 0.60f, 0.2f));
		RenderProps.setCylindricalLines(CF_L34R3a, Math.sqrt (
				CF_L34AnR3a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.80f, 0.45f, 0.2f));
		RenderProps.setCylindricalLines(CF_L34R4a, Math.sqrt (
				CF_L34AnR4a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.80f, 0.45f, 0.2f));

		RenderProps.setCylindricalLines(CF_L34R1i, Math.sqrt (
				CF_L45AnR1i.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.95f, 0.95f, 0.2f));
		RenderProps.setCylindricalLines(CF_L34R1a, Math.sqrt (
				CF_L45AnR1a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.90f, 0.75f, 0.2f));
		RenderProps.setCylindricalLines(CF_L34R2a, Math.sqrt (
				CF_L45AnR2a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.85f, 0.60f, 0.2f));
		RenderProps.setCylindricalLines(CF_L34R3a, Math.sqrt (
				CF_L45AnR3a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.80f, 0.45f, 0.2f));
		RenderProps.setCylindricalLines(CF_L34R4a, Math.sqrt (
				CF_L45AnR4a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.80f, 0.45f, 0.2f));
		
		RenderProps.setCylindricalLines(CF_L51R1i, Math.sqrt (
				CF_L51AnR1i.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.95f, 0.95f, 0.2f));
		RenderProps.setCylindricalLines(CF_L51R1a, Math.sqrt (
				CF_L51AnR1a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.90f, 0.75f, 0.2f));
		RenderProps.setCylindricalLines(CF_L51R2a, Math.sqrt (
				CF_L51AnR2a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.85f, 0.60f, 0.2f));
		RenderProps.setCylindricalLines(CF_L51R3a, Math.sqrt (
				CF_L51AnR3a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.80f, 0.45f, 0.2f));
		RenderProps.setCylindricalLines(CF_L51R4a, Math.sqrt (
				CF_L51AnR4a.CFCrossSec/Math.PI)*CF_ScaleR, new Color(0.80f, 0.45f, 0.2f));
		
		CFs.add(CF_L12);
		CFs.add(CF_L23);
		CFs.add(CF_L34);
		CFs.add(CF_L45);
		CFs.add(CF_L51);
		mech.add(CFs);		
	}




	/**
	 * Create and attach optional FrameMarker (FM) (e.g. for measurements 
	 * via MATLAB)
	 */
	private void addFrameMarkers(boolean showFM) {		
		// FM attached to disc (e.g. disc bulge measurement)		
		createFM_FEM (new FrameMarker ("FM_L12_ante"), 
				new Point3d(21.41e-3, 0, 170.03e-3),         L12An, showFM); // central & most anterior point
		createFM_FEM (new FrameMarker ("FM_L12_latL"), 
				new Point3d( 2.20e-3,  24.00e-3, 164.97e-3), L12An, showFM); // (central &) most lateral point
		createFM_FEM (new FrameMarker ("FM_L12_latR"), 
				new Point3d( 2.20e-3, -24.00e-3, 164.97e-3), L12An, showFM); // (central &) most lateral point 
		createFM_FEM (new FrameMarker ("FM_L12_post"), 
				new Point3d(-15.72e-3, 0, 160.72e-3),        L12An, showFM); // central & most posterior point
		createFM_FEM (new FrameMarker ("FM_L23_ante"), 
				new Point3d(31.52e-3, 0, 128.69e-3),         L23An, showFM);
		createFM_FEM (new FrameMarker ("FM_L23_latL"), 
				new Point3d(11.01e-3,  23.47e-3, 125.80e-3), L23An, showFM); 
		createFM_FEM (new FrameMarker ("FM_L23_latR"), 
				new Point3d(11.01e-3, -23.47e-3, 125.80e-3), L23An, showFM);  
		createFM_FEM (new FrameMarker ("FM_L23_post"), 
				new Point3d( 6.67e-3, 0, 123.60e-3),         L23An, showFM);
		createFM_FEM (new FrameMarker ("FM_L34_ante"), 
				new Point3d(35.47e-3, 0, 84.99e-3),         L34An, showFM);
		createFM_FEM (new FrameMarker ("FM_L34_latL"), 
				new Point3d(16.05e-3,  24.13e-3, 84.73e-3), L34An, showFM); 
		createFM_FEM (new FrameMarker ("FM_L34_latR"), 
				new Point3d(16.05e-3, -24.13e-3, 84.73e-3), L34An, showFM);  
		createFM_FEM (new FrameMarker ("FM_L34_post"), 
				new Point3d(-3.03e-3, 0, 85.04e-3),         L34An, showFM); 
		createFM_FEM (new FrameMarker ("FM_L45_ante"), 
				new Point3d(34.36e-3, 0, 40.23e-3),         L45An, showFM);
		createFM_FEM (new FrameMarker ("FM_L45_latL"), 
				new Point3d(14.73e-3,  26.51e-3, 43.53e-3), L45An, showFM); 
		createFM_FEM (new FrameMarker ("FM_L45_latR"), 
				new Point3d(14.73e-3, -26.51e-3, 43.53e-3), L45An, showFM);  
		createFM_FEM (new FrameMarker ("FM_L45_post"), 
				new Point3d(-2.32e-3, 0, 46.57e-3),         L45An, showFM); 
		createFM_FEM (new FrameMarker ("FM_L51_ante"), 
				new Point3d(16.53e-3,  0, -2.68e-3),       L51An, showFM);
		createFM_FEM (new FrameMarker ("FM_L51_latL"), 
				new Point3d( 3.29e-3,  27.20e-3, 4.91e-3), L51An, showFM); 
		createFM_FEM (new FrameMarker ("FM_L51_latR"), 
				new Point3d( 3.29e-3, -27.20e-3, 4.91e-3), L51An, showFM);  
		createFM_FEM (new FrameMarker ("FM_L51_post"), 
				new Point3d(-11.27e-3,  0, 14.08e-3),      L51An, showFM);	
		
		// FM attached to disc at the center of the NP
		createFM_FEM(new FrameMarker("FM_L12_NpCoM"), 
				new Point3d( 1.11e-3,  0, 164.89e-3), L12Np, showFM);
		createFM_FEM(new FrameMarker("FM_L23_NpCoM"), 
				new Point3d(10.29e-3,  0, 125.87e-3), L23Np, showFM);
		createFM_FEM(new FrameMarker("FM_L34_NpCoM"), 
				new Point3d(14.39e-3,  0,  85.06e-3), L34Np, showFM);
		createFM_FEM(new FrameMarker("FM_L45_NpCoM"), 
				new Point3d(14.14e-3,  0,  43.75e-3), L45Np, showFM);
		createFM_FEM(new FrameMarker("FM_L51_NpCoM"), 
				new Point3d( 2.19e-3,  0,   6.00e-3), L51Np, showFM);
				
		
		// FM attached to rigid vertebrae (e.g. disc height measurement)
		createFM_RB(new FrameMarker("FM_L1_supAnte"),  
				new Point3d( 11.16e-3,   0, 203.02e-3),       L1RB, showFM);
		createFM_RB(new FrameMarker("FM_L1_supLatL"),  
				new Point3d( -9.36e-3,  23.32e-3, 197.83e-3), L1RB, showFM); 
		createFM_RB(new FrameMarker("FM_L1_supLatR"),  
				new Point3d( -9.36e-3, -23.32e-3, 197.83e-3), L1RB, showFM);
		createFM_RB(new FrameMarker("FM_L1_supPost"),  
				new Point3d(-24.34e-3,   0, 193.89e-3),       L1RB, showFM);  
		createFM_RB(new FrameMarker("FM_L1_infAnte"),  
				new Point3d( 19.50e-3,   0, 175.88e-3),       L1RB, showFM); 
		createFM_RB(new FrameMarker("FM_L1_infLatL"),  
				new Point3d( -1.13e-3,  24.22e-3, 169.90e-3), L1RB, showFM); 		
		createFM_RB(new FrameMarker("FM_L1_infLatR"),  
				new Point3d( -1.13e-3, -24.22e-3, 169.90e-3), L1RB, showFM); 
		createFM_RB(new FrameMarker("FM_L1_infPost"),  
				new Point3d( -16.88e-3,   0, 165.31e-3),      L1RB, showFM);  
		createFM_RB(new FrameMarker("FM_L1_CoM_body"), L1Body_CoM, L1RB, showFM);
		
		createFM_RB(new FrameMarker("FM_L2_supAnte"),  
				new Point3d( 23.33e-3,   0, 164.19e-3),       L2RB, showFM);
		createFM_RB(new FrameMarker("FM_L2_supLatL"),  
				new Point3d(  2.56e-3,  24.21e-3, 159.77e-3), L2RB, showFM); 
		createFM_RB(new FrameMarker("FM_L2_supLatR"),  
				new Point3d(  2.56e-3, -24.21e-3, 159.77e-3), L2RB, showFM);
		createFM_RB(new FrameMarker("FM_L2_supPost"),  
				new Point3d(-14.55e-3,   0, 156.13e-3),       L2RB, showFM);  
		createFM_RB(new FrameMarker("FM_L2_infAnte"),  
				new Point3d( 30.57e-3,   0, 135.02e-3),       L2RB, showFM); 
		createFM_RB(new FrameMarker("FM_L2_infLatL"),  
				new Point3d(  8.71e-3,  23.69e-3, 131.36e-3), L2RB, showFM); 		
		createFM_RB(new FrameMarker("FM_L2_infLatR"),  
				new Point3d(  8.71e-3, -23.69e-3, 131.36e-3), L2RB, showFM); 
		createFM_RB(new FrameMarker("FM_L2_infPost"),  
				new Point3d( -7.39e-3,   0, 128.67e-3),       L2RB, showFM);  
		createFM_RB(new FrameMarker("FM_L2_CoM_body"), L2Body_CoM, L2RB, showFM);
		
		createFM_RB(new FrameMarker("FM_L3_supAnte"),  
				new Point3d(32.46e-3,  0, 122.36e-3),     	L3RB, showFM);
		createFM_RB(new FrameMarker("FM_L3_supLatL"),  
				new Point3d(10.98e-3,  23.81e-3, 120.20e-3), L3RB, showFM); 
		createFM_RB(new FrameMarker("FM_L3_supLatR"),  
				new Point3d(10.98e-3, -23.81e-3, 120.20e-3), L3RB, showFM);
		createFM_RB(new FrameMarker("FM_L3_supPost"),  
				new Point3d(-5.95e-3,  0, 118.53e-3),        L3RB, showFM);  
		createFM_RB(new FrameMarker("FM_L3_infAnte"),  
				new Point3d(34.78e-3,  0, 92.08e-3),         L3RB, showFM); 
		createFM_RB(new FrameMarker("FM_L3_infLatL"),  
				new Point3d(13.77e-3,  24.55e-3, 90.90e-3),  L3RB, showFM); 		
		createFM_RB(new FrameMarker("FM_L3_infLatR"),  
				new Point3d(13.77e-3, -24.55e-3, 90.90e-3),  L3RB, showFM); 
		createFM_RB(new FrameMarker("FM_L3_infPost"),  
				new Point3d(-3.49e-3,  0, 89.94e-3),         L3RB, showFM);  
		createFM_RB(new FrameMarker("FM_L3_CoM_body"), L3Body_CoM, L3RB, showFM);
		
		createFM_RB(new FrameMarker("FM_L4_supAnte"),  
				new Point3d(36.15e-3, 0, 77.91e-3),     	L4RB, showFM);
		createFM_RB(new FrameMarker("FM_L4_supLatL"),  
				new Point3d(14.98e-3,  24.24e-3, 79.13e-3), L4RB, showFM); 
		createFM_RB(new FrameMarker("FM_L4_supLatR"),  
				new Point3d(14.98e-3, -24.24e-3, 79.13e-3), L4RB, showFM);
		createFM_RB(new FrameMarker("FM_L4_supPost"),  
				new Point3d(-2.58e-3, 0, 80.14e-3),         L4RB, showFM);  
		createFM_RB(new FrameMarker("FM_L4_infAnte"),  
				new Point3d(34.94e-3, 0, 48.24e-3),         L4RB, showFM); 
		createFM_RB(new FrameMarker("FM_L4_infLatL"),  
				new Point3d(15.50e-3,  26.09e-3, 50.05e-3), L4RB, showFM); 
		createFM_RB(new FrameMarker("FM_L4_infLatR"),  
				new Point3d(15.50e-3, -26.09e-3, 50.05e-3), L4RB, showFM); 
		createFM_RB(new FrameMarker("FM_L4_infPost"),  
				new Point3d(-2.45e-3, 0, 51.71e-3),         L4RB, showFM);  
		createFM_RB(new FrameMarker("FM_L4_CoM_body"), L4Body_CoM, L4RB, showFM); 	
				
		createFM_RB(new FrameMarker ("FM_L5_supAnte"),  
				new Point3d(33.77e-3, 0, 32.25e-3),     	L5RB, showFM);
		createFM_RB(new FrameMarker ("FM_L5_supLatL"),  
				new Point3d(14.69e-3,  26.77e-3, 37.12e-3), L5RB, showFM); 
		createFM_RB(new FrameMarker ("FM_L5_supLatR"),  
				new Point3d(14.69e-3, -26.77e-3, 37.12e-3), L5RB, showFM);
		createFM_RB(new FrameMarker ("FM_L5_supPost"),  
				new Point3d(-2.18e-3, 0, 41.43e-3),         L5RB, showFM); 
		createFM_RB(new FrameMarker ("FM_L5_infAnte"),  
				new Point3d(21.20e-3, 0,  3.35e-3),         L5RB, showFM); 
		createFM_RB(new FrameMarker ("FM_L5_infLatL"),  
				new Point3d( 5.07e-3,  26.83e-3, 11.33e-3), L5RB, showFM); 
		createFM_RB(new FrameMarker ("FM_L5_infLatR"),  
				new Point3d( 5.07e-3, -26.83e-3, 11.33e-3), L5RB, showFM); 
		createFM_RB(new FrameMarker ("FM_L5_infPost"),  
				new Point3d(-9.28e-3, 0, 18.42e-3),         L5RB, showFM); 
		createFM_RB(new FrameMarker ("FM_L5_CoM_body"), L5Body_CoM, L5RB, showFM);  
		
		createFM_RB (new FrameMarker ("FM_S1_supAnte"),  
				new Point3d(11.86e-3, 0, -8.71e-3),        S1RB, showFM);
		createFM_RB (new FrameMarker ("FM_S1_supLatL"),  
				new Point3d(-1.88e-3,  28.17e-3, 1.38e-3), S1RB, showFM); 
		createFM_RB (new FrameMarker ("FM_S1_supLatR"),  
				new Point3d(-1.88e-3, -28.17e-3, 1.38e-3), S1RB, showFM);
		createFM_RB (new FrameMarker ("FM_S1_supPost"),  
				new Point3d(-13.27e-3, 0, 9.75e-3),        S1RB, showFM); 
		createFM_RB (new FrameMarker ("FM_S1_CoM_body"), S1Body_CoM, S1RB, showFM);	
	}




	/**
	 * Create FE to FE attachments
	 */
	private void addAttachments_FEFE() {	
		// For Np to An: always get the nodes from the outer set (a = auﬂen)
		//  and connect to closest element/node		
		for (FemNode3d n : L12Npa_ou) 		
			mech.attachPoint(n, L12An);
		for (FemNode3d n : L23Npa_ou) 		
			mech.attachPoint(n, L23An);
		for (FemNode3d n : L34Npa_ou) 		
			mech.attachPoint(n, L34An);
		for (FemNode3d n : L45Npa_ou) 		
			mech.attachPoint(n, L45An);
		for (FemNode3d n : L51Npa_ou) 		
			mech.attachPoint(n, L51An);	
	}


	
	/**
	 * Add FL to the model and set max force and set its visibility 
	 */
	private void addFollowerLoad(double maxF, boolean showFL) {
		// Create the Follower Load (FL) for entire LSS (L1-S1)
		ComponentList<ModelComponent> FL = 
				new ComponentList<> (ModelComponent.class, "followerLoad");
		ComponentList<Particle> myFLPartListl = 
				new ComponentList<Particle> (Particle.class, "FLParticlesl");
		ComponentList<Particle> myFLPartListr = 
				new ComponentList<Particle> (Particle.class, "FLParticlesr");	
		FL_LSSl = new MultiPointMuscle ("FL_LSSl");
		FL_LSSr = new MultiPointMuscle ("FL_LSSr");
		
		// ---- Particles (attached to RB) -----
		FL.add (myFLPartListl);
		FL.add (myFLPartListr);
		mech.add(FL);
		
		addParticleRB (myFLPartListl, "FLL1l", L1RB, -7.00e-3,  23.80e-3, 183.70e-3);
		addParticleRB (myFLPartListr, "FLL1r", L1RB, -7.00e-3, -23.80e-3, 183.70e-3);
		addParticleRB (myFLPartListl, "FLL2l", L2RB,  6.70e-3,  23.70e-3, 146.30e-3);
		addParticleRB (myFLPartListr, "FLL2r", L2RB,  6.70e-3, -23.70e-3, 146.30e-3);
		addParticleRB (myFLPartListl, "FLL3l", L3RB, 11.40e-3,  24.20e-3, 105.20e-3);
		addParticleRB (myFLPartListr, "FLL3r", L3RB, 11.40e-3, -24.20e-3, 105.20e-3);
		addParticleRB (myFLPartListl, "FLL4l", L4RB, 15.00e-3,  25.20e-3,  64.10e-3);
		addParticleRB (myFLPartListr, "FLL4r", L4RB, 15.00e-3, -25.20e-3,  64.10e-3);
		addParticleRB (myFLPartListl, "FLL5l", L5RB, 10.80e-3,  26.60e-3,  23.60e-3);
		addParticleRB (myFLPartListr, "FLL5r", L5RB, 10.80e-3, -26.60e-3,  23.60e-3);
		addParticleRB (myFLPartListl, "FLS1l", S1RB, -6.70e-3,  27.90e-3,  -4.00e-3);
		addParticleRB (myFLPartListr, "FLS1r", S1RB, -6.70e-3, -27.90e-3,  -4.00e-3);						
	
		
		// Attach left and right Multi-Point FL-Muscles to Particles		
		for (Particle p : myFLPartListl) 
			FL_LSSl.addPoint(p);		
		for (Particle p : myFLPartListr) 
			FL_LSSr.addPoint(p);
		
		FL.add (FL_LSSl);
		FL.add (FL_LSSr);
		
		// Set MATERIAL PROPS 
		SimpleAxialMuscle Mat_FL = new SimpleAxialMuscle();
		Mat_FL.setStiffness (0);
		Mat_FL.setDamping (0);
		Mat_FL.setMaxForce (maxF/2);		
		// set for muscles 
		FL_LSSl.setMaterial (Mat_FL);
		FL_LSSr.setMaterial (Mat_FL);
		
		// Combine left and right muscle for simultaneous excitation
		FL_lr = new MuscleExciter("totalFL");
		FL_lr.addTarget(FL_LSSl, 1.0);
		FL_lr.addTarget(FL_LSSr, 1.0);
		mech.addMuscleExciter(FL_lr);
		
		
		// Set FL RENDER PROPS of particles and 'muscles'
		if (showFL) {
			for (Particle p : myFLPartListl) 
				RenderProps.setSphericalPoints (p, 0.001, Color.RED);		
			for (Particle p : myFLPartListr) 
				RenderProps.setSphericalPoints (p, 0.001, Color.RED);		
		    RenderProps.setCylindricalLines (FL_LSSl, 0.0005, Color.GRAY);
		    RenderProps.setCylindricalLines (FL_LSSr, 0.0005, Color.GRAY);	
		}
		else {
			RenderProps.setVisible(FL_LSSl, showFL);
			RenderProps.setVisible(FL_LSSr, showFL);
		}
	}



	/**
	 * Create all FE to RB attachments 
	 */
	private void addAttachments_FERB() throws IOException {	
		// Import node info from text-files which define node numbers to 
		// connect to RB
		// L12Np
		LinkedList<FemNode3d> L12Npo, L12Npu, L12Npa;
		L12Npo = MyImportFunctions.ImportNodeSet ("L12_Npo", L12Np);
		L12Npu = MyImportFunctions.ImportNodeSet ("L12_Npu", L12Np);
		L12Npa = MyImportFunctions.ImportNodeSet ("L12_Npa", L12Np);
		// L23Np
		LinkedList<FemNode3d> L23Npo, L23Npu, L23Npa;
		L23Npo = MyImportFunctions.ImportNodeSet ("L23_Npo", L23Np);
		L23Npu = MyImportFunctions.ImportNodeSet ("L23_Npu", L23Np);
		L23Npa = MyImportFunctions.ImportNodeSet ("L23_Npa", L23Np);
		// L34Np
		LinkedList<FemNode3d> L34Npo, L34Npu, L34Npa;
		L34Npo = MyImportFunctions.ImportNodeSet ("L34_Npo", L34Np);
		L34Npu = MyImportFunctions.ImportNodeSet ("L34_Npu", L34Np);
		L34Npa = MyImportFunctions.ImportNodeSet ("L34_Npa", L34Np);
		// L45Np
		LinkedList<FemNode3d> L45Npo, L45Npu, L45Npa;
		L45Npo = MyImportFunctions.ImportNodeSet ("L45_Npo", L45Np);
		L45Npu = MyImportFunctions.ImportNodeSet ("L45_Npu", L45Np);
		L45Npa = MyImportFunctions.ImportNodeSet ("L45_Npa", L45Np);
		// L51Np
		LinkedList<FemNode3d> L51Npo, L51Npu, L51Npa;
		L51Npo = MyImportFunctions.ImportNodeSet ("L51_Npo", L51Np);
		L51Npu = MyImportFunctions.ImportNodeSet ("L51_Npu", L51Np);
		L51Npa = MyImportFunctions.ImportNodeSet ("L51_Npa", L51Np);
		
		// L12An		
		LinkedList<LinkedList<FemNode3d>> L12AnR1234o, L12AnR1234u;
		L12AnR1234o = new LinkedList<LinkedList<FemNode3d>>();
		L12AnR1234u = new LinkedList<LinkedList<FemNode3d>>();
		L12AnR1234o.add(MyImportFunctions.ImportNodeSet ("L12_An_R1o", L12AnR1));
		L12AnR1234u.add(MyImportFunctions.ImportNodeSet ("L12_An_R1u", L12AnR1));
		L12AnR1234o.add(MyImportFunctions.ImportNodeSet ("L12_An_R2o", L12AnR2));
		L12AnR1234u.add(MyImportFunctions.ImportNodeSet ("L12_An_R2u", L12AnR2));
		L12AnR1234o.add(MyImportFunctions.ImportNodeSet ("L12_An_R3o", L12AnR3));
		L12AnR1234u.add(MyImportFunctions.ImportNodeSet ("L12_An_R3u", L12AnR3));
		L12AnR1234o.add(MyImportFunctions.ImportNodeSet ("L12_An_R4o", L12AnR4));
		L12AnR1234u.add(MyImportFunctions.ImportNodeSet ("L12_An_R4u", L12AnR4));
		// L23An
		LinkedList<LinkedList<FemNode3d>> L23AnR1234o, L23AnR1234u;
		L23AnR1234o = new LinkedList<LinkedList<FemNode3d>>();
		L23AnR1234u = new LinkedList<LinkedList<FemNode3d>>();
		L23AnR1234o.add(MyImportFunctions.ImportNodeSet ("L23_An_R1o", L23AnR1));
		L23AnR1234u.add(MyImportFunctions.ImportNodeSet ("L23_An_R1u", L23AnR1));
		L23AnR1234o.add(MyImportFunctions.ImportNodeSet ("L23_An_R2o", L23AnR2));
		L23AnR1234u.add(MyImportFunctions.ImportNodeSet ("L23_An_R2u", L23AnR2));
		L23AnR1234o.add(MyImportFunctions.ImportNodeSet ("L23_An_R3o", L23AnR3));
		L23AnR1234u.add(MyImportFunctions.ImportNodeSet ("L23_An_R3u", L23AnR3));
		L23AnR1234o.add(MyImportFunctions.ImportNodeSet ("L23_An_R4o", L23AnR4));
		L23AnR1234u.add(MyImportFunctions.ImportNodeSet ("L23_An_R4u", L23AnR4));
		// L34An
		LinkedList<LinkedList<FemNode3d>> L34AnR1234o, L34AnR1234u;
		L34AnR1234o = new LinkedList<LinkedList<FemNode3d>>();
		L34AnR1234u = new LinkedList<LinkedList<FemNode3d>>();
		L34AnR1234o.add(MyImportFunctions.ImportNodeSet ("L34_An_R1o", L34AnR1));
		L34AnR1234u.add(MyImportFunctions.ImportNodeSet ("L34_An_R1u", L34AnR1));
		L34AnR1234o.add(MyImportFunctions.ImportNodeSet ("L34_An_R2o", L34AnR2));
		L34AnR1234u.add(MyImportFunctions.ImportNodeSet ("L34_An_R2u", L34AnR2));
		L34AnR1234o.add(MyImportFunctions.ImportNodeSet ("L34_An_R3o", L34AnR3));
		L34AnR1234u.add(MyImportFunctions.ImportNodeSet ("L34_An_R3u", L34AnR3));
		L34AnR1234o.add(MyImportFunctions.ImportNodeSet ("L34_An_R4o", L34AnR4));
		L34AnR1234u.add(MyImportFunctions.ImportNodeSet ("L34_An_R4u", L34AnR4));
		// L45An
		LinkedList<LinkedList<FemNode3d>> L45AnR1234o, L45AnR1234u;
		L45AnR1234o = new LinkedList<LinkedList<FemNode3d>>();
		L45AnR1234u = new LinkedList<LinkedList<FemNode3d>>();
		L45AnR1234o.add(MyImportFunctions.ImportNodeSet ("L45_An_R1o", L45AnR1));
		L45AnR1234u.add(MyImportFunctions.ImportNodeSet ("L45_An_R1u", L45AnR1));
		L45AnR1234o.add(MyImportFunctions.ImportNodeSet ("L45_An_R2o", L45AnR2));
		L45AnR1234u.add(MyImportFunctions.ImportNodeSet ("L45_An_R2u", L45AnR2));
		L45AnR1234o.add(MyImportFunctions.ImportNodeSet ("L45_An_R3o", L45AnR3));
		L45AnR1234u.add(MyImportFunctions.ImportNodeSet ("L45_An_R3u", L45AnR3));
		L45AnR1234o.add(MyImportFunctions.ImportNodeSet ("L45_An_R4o", L45AnR4));
		L45AnR1234u.add(MyImportFunctions.ImportNodeSet ("L45_An_R4u", L45AnR4));
		// L51An
		LinkedList<LinkedList<FemNode3d>> L51AnR1234o, L51AnR1234u;
		L51AnR1234o = new LinkedList<LinkedList<FemNode3d>>();
		L51AnR1234u = new LinkedList<LinkedList<FemNode3d>>();
		L51AnR1234o.add(MyImportFunctions.ImportNodeSet ("L51_An_R1o", L51AnR1));
		L51AnR1234u.add(MyImportFunctions.ImportNodeSet ("L51_An_R1u", L51AnR1));
		L51AnR1234o.add(MyImportFunctions.ImportNodeSet ("L51_An_R2o", L51AnR2));
		L51AnR1234u.add(MyImportFunctions.ImportNodeSet ("L51_An_R2u", L51AnR2));
		L51AnR1234o.add(MyImportFunctions.ImportNodeSet ("L51_An_R3o", L51AnR3));
		L51AnR1234u.add(MyImportFunctions.ImportNodeSet ("L51_An_R3u", L51AnR3));
		L51AnR1234o.add(MyImportFunctions.ImportNodeSet ("L51_An_R4o", L51AnR4));
		L51AnR1234u.add(MyImportFunctions.ImportNodeSet ("L51_An_R4u", L51AnR4));

		
		// Remove nodes which might be listed in more than one set --> preventing errors
		L12Npa_ou = RemoveDoubleNodes_SetSet (RemoveDoubleNodes_SetSet (L12Npa, L12Npo), L12Npu);
		L23Npa_ou = RemoveDoubleNodes_SetSet (RemoveDoubleNodes_SetSet (L23Npa, L23Npo), L23Npu);
		L34Npa_ou = RemoveDoubleNodes_SetSet (RemoveDoubleNodes_SetSet (L34Npa, L34Npo), L34Npu);
		L45Npa_ou = RemoveDoubleNodes_SetSet (RemoveDoubleNodes_SetSet (L45Npa, L45Npo), L45Npu);
		L51Npa_ou = RemoveDoubleNodes_SetSet (RemoveDoubleNodes_SetSet (L51Npa, L51Npo), L51Npu);
				
		
				
		// ------------------- ATTACHMENTS -------------------------------			
		// --- FEM to RB ----	
		LinkedList<Point3d> L12Ano_P, L12Anu_P, L23Ano_P, L23Anu_P, L34Ano_P, 
			L34Anu_P, L45Ano_P, L45Anu_P, L51Ano_P, L51Anu_P;
		L12Ano_P = new LinkedList<Point3d>();
		L12Anu_P = new LinkedList<Point3d>();
		L23Ano_P = new LinkedList<Point3d>();
		L23Anu_P = new LinkedList<Point3d>();
		L34Ano_P = new LinkedList<Point3d>();
		L34Anu_P = new LinkedList<Point3d>();
		L45Ano_P = new LinkedList<Point3d>();
		L45Anu_P = new LinkedList<Point3d>();
		L51Ano_P = new LinkedList<Point3d>();
		L51Anu_P = new LinkedList<Point3d>();

		// Select all node locations from imported sets and FemBodies
		// o - oben (top)
		CollectNodesLocations (L12Ano_P, L12AnR1234o);			
		CollectNodesLocations (L23Ano_P, L23AnR1234o);			
		CollectNodesLocations (L34Ano_P, L34AnR1234o);			
		CollectNodesLocations (L45Ano_P, L45AnR1234o);		
		CollectNodesLocations (L51Ano_P, L51AnR1234o);
		// u - unten (below)
		CollectNodesLocations (L12Anu_P, L12AnR1234u);
		CollectNodesLocations (L23Anu_P, L23AnR1234u);
		CollectNodesLocations (L34Anu_P, L34AnR1234u);
		CollectNodesLocations (L45Anu_P, L45AnR1234u);
		CollectNodesLocations (L51Anu_P, L51AnR1234u);

					
		// use collected locations to find the (new) nodes of merged FE AN
		double nodeTol = 0.0001;
		LinkedList<FemNode3d> L12Ano, L12Anu, L23Ano, L23Anu, L34Ano, L34Anu, 
			L45Ano, L45Anu, L51Ano, L51Anu;
		L12Ano = getNodesFromLocations (L12Ano_P, L12An, nodeTol);
		L12Anu = getNodesFromLocations (L12Anu_P, L12An, nodeTol);
		L23Ano = getNodesFromLocations (L23Ano_P, L23An, nodeTol);
		L23Anu = getNodesFromLocations (L23Anu_P, L23An, nodeTol);
		L34Ano = getNodesFromLocations (L34Ano_P, L34An, nodeTol);
		L34Anu = getNodesFromLocations (L34Anu_P, L34An, nodeTol);
		L45Ano = getNodesFromLocations (L45Ano_P, L45An, nodeTol);
		L45Anu = getNodesFromLocations (L45Anu_P, L45An, nodeTol);
		L51Ano = getNodesFromLocations (L51Ano_P, L51An, nodeTol);
		L51Anu = getNodesFromLocations (L51Anu_P, L51An, nodeTol);
		
		//RenderNodeSet(L12Npa_ou, Color.GREEN);
		//RenderNodeSet(L12Ano, Color.RED);
		//RenderNodeSet(L12Anu, Color.BLUE);
		//RenderNodeSet(L12Npo, Color.MAGENTA);
		
					
		// Connect respective NP nodes of FE Disc to RB
		ConnectFemToRB(L12Npo, L1RB);
		ConnectFemToRB(L12Npu, L2RB);
		ConnectFemToRB(L23Npo, L2RB);
		ConnectFemToRB(L23Npu, L3RB);
		ConnectFemToRB(L34Npo, L3RB);
		ConnectFemToRB(L34Npu, L4RB);
		ConnectFemToRB(L45Npo, L4RB);
		ConnectFemToRB(L45Npu, L5RB);
		ConnectFemToRB(L51Npo, L5RB);
		ConnectFemToRB(L51Npu, S1RB);
		// Connect respective AN nodes of FEM Disc to RB
		ConnectFemToRB(L12Ano, L1RB);
		ConnectFemToRB(L12Anu, L2RB);
		ConnectFemToRB(L23Ano, L2RB);
		ConnectFemToRB(L23Anu, L3RB);
		ConnectFemToRB(L34Ano, L3RB);
		ConnectFemToRB(L34Anu, L4RB);
		ConnectFemToRB(L45Ano, L4RB); 
		ConnectFemToRB(L45Anu, L5RB);	
		ConnectFemToRB(L51Ano, L5RB);
		ConnectFemToRB(L51Anu, S1RB);
	}




	/**
	 * Create and add a control panel to change external loads
	 */
	private void addMyControlPanel() {
		ControlPanel myPanel = new ControlPanel("Simulation Controls");

		myPanel.addWidget(new JLabel("External Loads:"));
		myPanel.addWidget("Wrench L1", Fr_WrL1, "externalForce");
		myPanel.addWidget("Wrench L2", Fr_WrL2, "externalForce");
		myPanel.addWidget("Wrench L3", Fr_WrL3, "externalForce");
		myPanel.addWidget("Wrench L4", Fr_WrL4, "externalForce");
		myPanel.addWidget("Wrench L5", Fr_WrL5, "externalForce");		
		myPanel.addWidget(new JSeparator());
		
		SimpleAxialMuscle myFlMat = (SimpleAxialMuscle) FL_LSSl.getMaterial();
		double FlF = myFlMat.getMaxForce()*2;
			
		myPanel.addWidget(new JLabel("Follower Load Excitations: (F_max = " + 
				String.format("%.0f", FlF) + "N)"));
		myPanel.addWidget("Total", FL_lr, "excitation");
		myPanel.addWidget("Left side", FL_LSSl, "excitation");
		myPanel.addWidget("Right side", FL_LSSr, "excitation");
		
		addControlPanel(myPanel);		
	}




	/**
	 * 	Add boundary conditions resp. external loads via Wrenches
	 */
	private void addMyExternalLoads(double framesAxisLength) {
		// -------------------------------------------------
		// ------------------- EXTERNAL LOADS --------------
		// -------------------------------------------------

		// Add external force to the center of the vertebrae - the force
		// orientation depends on the vertebrae orientations	
		// Set up wrench with zero forces and moments	
		Wrench WrL1, WrL2, WrL3, WrL4, WrL5;
		WrL1 = new Wrench(); 
		WrL2 = new Wrench(); 
		WrL3 = new Wrench(); 
		WrL4 = new Wrench(); 
		WrL5 = new Wrench(); 
		// Reference frames for relative pose change measurements
		Frame Fr_WrL1_atL2, Fr_WrL2_atL3, Fr_WrL3_atL4, Fr_WrL4_atL5, Fr_WrL5_atS1;	
			
		Fr_WrL1 = newFrame ("FrameForL1Wrench");  
		Fr_WrL2 = newFrame ("FrameForL2Wrench");  
		Fr_WrL3 = newFrame ("FrameForL3Wrench");  
		Fr_WrL4 = newFrame ("FrameForL4Wrench");  
		Fr_WrL5 = newFrame ("FrameForL5Wrench");  
			
		Fr_WrL1_atL2 = newFrame ("FrameForDiffToL1Wr");
		Fr_WrL2_atL3 = newFrame ("FrameForDiffToL2Wr");
		Fr_WrL3_atL4 = newFrame ("FrameForDiffToL3Wr");
		Fr_WrL4_atL5 = newFrame ("FrameForDiffToL4Wr");
		Fr_WrL5_atS1 = newFrame ("FrameForDiffToL5Wr");
		

		Point3d L1_antC = new Point3d(15.33e-3, 0, 189.54e-3);   // anterior center of L1_body
		Point3d L2_antC = new Point3d(26.95e-3, 0, 149.60e-3);   // anterior center of L2_body  
		Point3d L3_antC = new Point3d(33.62e-3, 0, 107.22e-3);   // anterior center of L3_body
		Point3d L4_antC = new Point3d(35.55e-3, 0,  63.08e-3);   // anterior center of L4_body  
		Point3d L5_antC = new Point3d(27.49e-3, 0,  17.80e-3);   // anterior center of L5_body
		Point3d L1_antS = new Point3d(11.16e-3, 0, 203.20e-3);   // most anterior and superior point of L1_body
		Point3d L2_antS = new Point3d(23.33e-3, 0, 164.19e-3);   // most anterior and superior point of L2_body
		Point3d L3_antS = new Point3d(32.46e-3, 0, 122.36e-3);   // most anterior and superior point of L3_body
		Point3d L4_antS = new Point3d(36.15e-3, 0,  77.91e-3);   // most anterior and superior point of L4_body  
		Point3d L5_antS = new Point3d(33.77e-3, 0,  32.25e-3);   // most anterior and superior point of L5_body
		
		Vector3d L1_xAx  = L1_antC.sub (L1Body_CoM);
		Vector3d L1_yAx  = L1_antS.sub (L1_antC);
		Vector3d L2_xAx  = L2_antC.sub (L2Body_CoM);
		Vector3d L2_yAx  = L2_antS.sub (L2_antC);
		Vector3d L3_xAx  = L3_antC.sub (L3Body_CoM);
		Vector3d L3_yAx  = L3_antS.sub (L3_antC);
		Vector3d L4_xAx  = L4_antC.sub (L4Body_CoM);
		Vector3d L4_yAx  = L4_antS.sub (L4_antC);	
		Vector3d L5_xAx  = L5_antC.sub (L5Body_CoM);
		Vector3d L5_yAx  = L5_antS.sub (L5_antC);

		
		// Set Pose of Frames with Wrenches
		double rotXFrWr = 270*Math.PI/180;
		setWrenchsPoseWithFrame (WrL1, Fr_WrL1, L1Body_CoM, L1_xAx, L1_yAx, 
				rotXFrWr, framesAxisLength);
		setWrenchsPoseWithFrame (WrL2, Fr_WrL2, L2Body_CoM, L2_xAx, L2_yAx, 
				rotXFrWr, framesAxisLength);
		setWrenchsPoseWithFrame (WrL3, Fr_WrL3, L3Body_CoM, L3_xAx, L3_yAx, 
				rotXFrWr, framesAxisLength);
		setWrenchsPoseWithFrame (WrL4, Fr_WrL4, L4Body_CoM, L4_xAx, L4_yAx, 
				rotXFrWr, framesAxisLength);
		setWrenchsPoseWithFrame (WrL5, Fr_WrL5, L5Body_CoM, L5_xAx, L5_yAx, 
				rotXFrWr, framesAxisLength);
		
		mech.attachFrame (Fr_WrL1, L1RB);
		mech.attachFrame (Fr_WrL2, L2RB);
		mech.attachFrame (Fr_WrL3, L3RB);
		mech.attachFrame (Fr_WrL4, L4RB);
		mech.attachFrame (Fr_WrL5, L5RB);
		
		// Set Pose of Frames only
		setFramesPoseInSpace (Fr_WrL1_atL2, L1Body_CoM, L1_xAx, L1_yAx, 
				rotXFrWr, framesAxisLength);
		setFramesPoseInSpace (Fr_WrL2_atL3, L2Body_CoM, L2_xAx, L2_yAx, 
				rotXFrWr, framesAxisLength);
		setFramesPoseInSpace (Fr_WrL3_atL4, L3Body_CoM, L3_xAx, L3_yAx, 
				rotXFrWr, framesAxisLength);
		setFramesPoseInSpace (Fr_WrL4_atL5, L4Body_CoM, L4_xAx, L4_yAx, 
				rotXFrWr, framesAxisLength);
		setFramesPoseInSpace (Fr_WrL5_atS1, L5Body_CoM, L5_xAx, L5_yAx,
				rotXFrWr, framesAxisLength);
		mech.attachFrame (Fr_WrL1_atL2, L2RB);
		mech.attachFrame (Fr_WrL2_atL3, L3RB);
		mech.attachFrame (Fr_WrL3_atL4, L4RB);
		mech.attachFrame (Fr_WrL4_atL5, L5RB);
		mech.attachFrame (Fr_WrL5_atS1, S1RB);			
	}





	private void setFEFacMaterials() {
		// Facet joint cartilage material properties
		// Neo-Hookean, initial values by Finley et al. (2018): E=30MPa & v=0.4
		double[] NH_Fac = {3.5*1e6, 0.4}; 		
		NeoHookeanMaterial Mat_Fac = 
				new NeoHookeanMaterial(NH_Fac[0], NH_Fac[1]);
		
		L12FacL1l.setMaterial (Mat_Fac);
		L12FacL1r.setMaterial (Mat_Fac);
		L23FacL2l.setMaterial (Mat_Fac);
		L23FacL2r.setMaterial (Mat_Fac);
		L34FacL3l.setMaterial (Mat_Fac);
		L34FacL3r.setMaterial (Mat_Fac);
		L45FacL4l.setMaterial (Mat_Fac);
		L45FacL4r.setMaterial (Mat_Fac);
		L51FacL5l.setMaterial (Mat_Fac);
		L51FacL5r.setMaterial (Mat_Fac);
		
		// facets density is represented by the vetebrae, but must not be 0
		double Fac_dens = 1E-8;  
		L12FacL1l.setDensity(Fac_dens);  
		L12FacL1r.setDensity(Fac_dens);
		L23FacL2l.setDensity(Fac_dens);  
		L23FacL2r.setDensity(Fac_dens);
		L34FacL3l.setDensity(Fac_dens);  
		L34FacL3r.setDensity(Fac_dens);
		L45FacL4l.setDensity(Fac_dens);  
		L45FacL4r.setDensity(Fac_dens);
		L51FacL5l.setDensity(Fac_dens);  
		L51FacL5r.setDensity(Fac_dens);
		
		// To make the contact more stable (e.g. prevent mesh vibrations)
		double meshContStiffnD = 0.1;
		L12FacL1l.setStiffnessDamping (meshContStiffnD); 
		L12FacL1r.setStiffnessDamping (meshContStiffnD);
		L23FacL2l.setStiffnessDamping (meshContStiffnD); 
		L23FacL2r.setStiffnessDamping (meshContStiffnD);	
		L34FacL3l.setStiffnessDamping (meshContStiffnD); 
		L34FacL3r.setStiffnessDamping (meshContStiffnD);
		L45FacL4l.setStiffnessDamping (meshContStiffnD); 
		L45FacL4r.setStiffnessDamping (meshContStiffnD);
		L51FacL5l.setStiffnessDamping (meshContStiffnD); 
		L51FacL5r.setStiffnessDamping (meshContStiffnD);
		
		// Compressibility settings
		IncompMethod softIncMFac = IncompMethod.NODAL;
		L12FacL1l.setSoftIncompMethod (softIncMFac);
		L12FacL1r.setSoftIncompMethod (softIncMFac);
		L23FacL2l.setSoftIncompMethod (softIncMFac);
		L23FacL2r.setSoftIncompMethod (softIncMFac);
		L34FacL3l.setSoftIncompMethod (softIncMFac);
		L34FacL3r.setSoftIncompMethod (softIncMFac);
		L45FacL4l.setSoftIncompMethod (softIncMFac);
		L45FacL4r.setSoftIncompMethod (softIncMFac);
		L51FacL5l.setSoftIncompMethod (softIncMFac);
		L51FacL5r.setSoftIncompMethod (softIncMFac);
		
		IncompMethod incMFac = IncompMethod.OFF;
		L12FacL1l.setIncompressible (incMFac);
		L12FacL1r.setIncompressible (incMFac);
		L23FacL2l.setIncompressible (incMFac);
		L23FacL2r.setIncompressible (incMFac);
		L34FacL3l.setIncompressible (incMFac);
		L34FacL3r.setIncompressible (incMFac);
		L45FacL4l.setIncompressible (incMFac);
		L45FacL4r.setIncompressible (incMFac);
		L51FacL5l.setIncompressible (incMFac);
		L51FacL5r.setIncompressible (incMFac);		
	}




	/**
	 * Set intervertebral discs (AN & NP) material properties
	 */
	private void setFEDiscMaterials() {		
		// Nucleus pulposus (NP) - Yeoh: g10, g20, g30, K; 
		double[] Y_Np = {0.20*1E6, 0.20*1E6, 6.0E6, 1.80E8};   // [Pa]
		CubicHyperelastic Mat_Np = new CubicHyperelastic(); 
		Mat_Np.setG10 (Y_Np[0]);
		Mat_Np.setG20 (Y_Np[1]);
		Mat_Np.setG30 (Y_Np[2]);
		Mat_Np.setBulkModulus (Y_Np[3]);
		
		L12Np.setMaterial (Mat_Np);  
		L23Np.setMaterial (Mat_Np);
		L34Np.setMaterial (Mat_Np);
		L45Np.setMaterial (Mat_Np);
		L51Np.setMaterial (Mat_Np);
						
		// Anulus ground substance (AN resp. AGS) - Mooney-Rivlin
		double[] MR_An = {0.18*1E6, 0.045*1E6, 0}; 	// c01, c10 [Pa]		 	     	
		double v_An    =  0.40;						// [-] Poissonís ratio
		MooneyRivlinMaterial Mat_An = new MooneyRivlinMaterial();						
		MR_An[2] = (1-2*v_An)/(MR_An[0]+MR_An[1]);  // D				
		Mat_An.setC10 (MR_An[0]);
		Mat_An.setC01 (MR_An[1]);
		Mat_An.setBulkModulus (2/MR_An[2]);  // k = 2/D
		
		L12An.setMaterial (Mat_An);
		L23An.setMaterial (Mat_An);
		L34An.setMaterial (Mat_An);
		L45An.setMaterial (Mat_An);
		L51An.setMaterial (Mat_An);
				
		// Densities 	
		double NP_dens  = 1.0E3; 	// [kg/m^3] 
		L12Np.setDensity (NP_dens);
		L23Np.setDensity (NP_dens);
		L34Np.setDensity (NP_dens);
		L45Np.setDensity (NP_dens);
		L51Np.setDensity (NP_dens);
		
		double AN_dens  = 1.2E3; 	// [kg/m^3]  
		L12An.setDensity (AN_dens);
		L23An.setDensity (AN_dens);
		L34An.setDensity (AN_dens);
		L45An.setDensity (AN_dens);
		L51An.setDensity (AN_dens);
		
		// Compressibility settings
		IncompMethod softIncMNp = IncompMethod.ELEMENT;
		L12Np.setSoftIncompMethod (softIncMNp);
		L23Np.setSoftIncompMethod (softIncMNp);
		L34Np.setSoftIncompMethod (softIncMNp);
		L45Np.setSoftIncompMethod (softIncMNp);
		L51Np.setSoftIncompMethod (softIncMNp);
		
		IncompMethod softIncMAn = IncompMethod.ELEMENT;
		L12An.setSoftIncompMethod (softIncMAn);
		L23An.setSoftIncompMethod (softIncMAn);
		L34An.setSoftIncompMethod (softIncMAn);		
		L45An.setSoftIncompMethod (softIncMAn);
		L51An.setSoftIncompMethod (softIncMAn);
			
		IncompMethod incMNp = IncompMethod.OFF;
		L12Np.setIncompressible (incMNp);
		L23Np.setIncompressible (incMNp);
		L34Np.setIncompressible (incMNp);
		L45Np.setIncompressible (incMNp);
		L51Np.setIncompressible (incMNp);
		
		IncompMethod incMAn = IncompMethod.OFF;
		L12An.setIncompressible (incMAn);
		L23An.setIncompressible (incMAn);
		L34An.setIncompressible (incMAn);
		L45An.setIncompressible (incMAn);
		L51An.setIncompressible (incMAn);
	}




	/**
	 * Create FE discs (AN & NP) from imported mesh data 
	 */
	private void addFEDiscs() {	
		MyImportFunctions.setFemPathAddition ("/LSS_20200529/");
		
		ComponentList<FemModel3d> NPs = 
				new ComponentList<FemModel3d>(FemModel3d.class, "nucleusFE");
		ComponentList<FemModel3d> ANs = 
				new ComponentList<FemModel3d>(FemModel3d.class, "anulusFE");
				
		L12Np = new FemModel3d ("L12Np"); 
		L23Np = new FemModel3d ("L23Np"); 
		L34Np = new FemModel3d ("L34Np"); 
		L45Np = new FemModel3d ("L45Np"); 
		L51Np = new FemModel3d ("L51Np");  	
		
		L12An = new FemModel3d ("L12An");
		L23An = new FemModel3d ("L23An");
		L34An = new FemModel3d ("L34An");
		L45An = new FemModel3d ("L45An");
		L51An = new FemModel3d ("L51An");
		
		L12AnR1 = new FemModel3d ("L12AnR1");  
		L12AnR2 = new FemModel3d ("L12AnR2");  
		L12AnR3 = new FemModel3d ("L12AnR3"); 
		L12AnR4 = new FemModel3d ("L12AnR4"); 
		L23AnR1 = new FemModel3d ("L23AnR1");  
		L23AnR2 = new FemModel3d ("L23AnR2");  
		L23AnR3 = new FemModel3d ("L23AnR3"); 
		L23AnR4 = new FemModel3d ("L23AnR4"); 
		L34AnR1 = new FemModel3d ("L34AnR1");  
		L34AnR2 = new FemModel3d ("L34AnR2");  
		L34AnR3 = new FemModel3d ("L34AnR3"); 
		L34AnR4 = new FemModel3d ("L34AnR4"); 
		L45AnR1 = new FemModel3d ("L45AnR1");  
		L45AnR2 = new FemModel3d ("L45AnR2");  
		L45AnR3 = new FemModel3d ("L45AnR3"); 
		L45AnR4 = new FemModel3d ("L45AnR4"); 
		L51AnR1 = new FemModel3d ("L51AnR1");  
		L51AnR2 = new FemModel3d ("L51AnR2");  
		L51AnR3 = new FemModel3d ("L51AnR3"); 
		L51AnR4 = new FemModel3d ("L51AnR4"); 
		
		// Import mesh data in Ansys format - scaled in meter
		MyImportFunctions.readFromAnsysReader (L12Np,   "L12_Np");
		MyImportFunctions.readFromAnsysReader (L12An,   "L12_An_R1");
		MyImportFunctions.readFromAnsysReader (L12AnR1, "L12_An_R1");
		MyImportFunctions.readFromAnsysReader (L12AnR2, "L12_An_R2");
		MyImportFunctions.readFromAnsysReader (L12AnR3, "L12_An_R3");
		MyImportFunctions.readFromAnsysReader (L12AnR4, "L12_An_R4");
		
		MyImportFunctions.readFromAnsysReader (L23Np,   "L23_Np");
		MyImportFunctions.readFromAnsysReader (L23An,   "L23_An_R1");
		MyImportFunctions.readFromAnsysReader (L23AnR1, "L23_An_R1");
		MyImportFunctions.readFromAnsysReader (L23AnR2, "L23_An_R2");
		MyImportFunctions.readFromAnsysReader (L23AnR3, "L23_An_R3");
		MyImportFunctions.readFromAnsysReader (L23AnR4, "L23_An_R4");
		
		MyImportFunctions.readFromAnsysReader (L34Np,   "L34_Np");
		MyImportFunctions.readFromAnsysReader (L34An,   "L34_An_R1");
		MyImportFunctions.readFromAnsysReader (L34AnR1, "L34_An_R1");
		MyImportFunctions.readFromAnsysReader (L34AnR2, "L34_An_R2");
		MyImportFunctions.readFromAnsysReader (L34AnR3, "L34_An_R3");
		MyImportFunctions.readFromAnsysReader (L34AnR4, "L34_An_R4");
		
		MyImportFunctions.readFromAnsysReader (L45Np,   "L45_Np");
		MyImportFunctions.readFromAnsysReader (L45An,   "L45_An_R1");
		MyImportFunctions.readFromAnsysReader (L45AnR1, "L45_An_R1");
		MyImportFunctions.readFromAnsysReader (L45AnR2, "L45_An_R2");
		MyImportFunctions.readFromAnsysReader (L45AnR3, "L45_An_R3");
		MyImportFunctions.readFromAnsysReader (L45AnR4, "L45_An_R4");
		
		MyImportFunctions.readFromAnsysReader (L51Np,   "L51_Np");
		MyImportFunctions.readFromAnsysReader (L51An,   "L51_An_R1");
		MyImportFunctions.readFromAnsysReader (L51AnR1, "L51_An_R1");
		MyImportFunctions.readFromAnsysReader (L51AnR2, "L51_An_R2");
		MyImportFunctions.readFromAnsysReader (L51AnR3, "L51_An_R3");
		MyImportFunctions.readFromAnsysReader (L51AnR4, "L51_An_R4");
		
							
		// For more order store FE disc parts in ComponentLists
		L12AnR1234 = new ComponentList<FemModel3d>(FemModel3d.class);
		L23AnR1234 = new ComponentList<FemModel3d>(FemModel3d.class);
		L34AnR1234 = new ComponentList<FemModel3d>(FemModel3d.class);
		L45AnR1234 = new ComponentList<FemModel3d>(FemModel3d.class);
		L51AnR1234 = new ComponentList<FemModel3d>(FemModel3d.class);
		
		L12AnR1234.add(L12AnR1);
		L12AnR1234.add(L12AnR2);
		L12AnR1234.add(L12AnR3);
		L12AnR1234.add(L12AnR4);
		
		L23AnR1234.add(L23AnR1);
		L23AnR1234.add(L23AnR2);
		L23AnR1234.add(L23AnR3);
		L23AnR1234.add(L23AnR4);
		
		L34AnR1234.add(L34AnR1);
		L34AnR1234.add(L34AnR2);
		L34AnR1234.add(L34AnR3);
		L34AnR1234.add(L34AnR4);
		
		L45AnR1234.add(L45AnR1);
		L45AnR1234.add(L45AnR2);
		L45AnR1234.add(L45AnR3);
		L45AnR1234.add(L45AnR4);
		
		L51AnR1234.add(L51AnR1);
		L51AnR1234.add(L51AnR2);
		L51AnR1234.add(L51AnR3);
		L51AnR1234.add(L51AnR4);
		

		// Add FE ComponentLists to mechModel
		NPs.add(L12Np);
		NPs.add(L23Np);
		NPs.add(L34Np);
		NPs.add(L45Np);
		NPs.add(L51Np);
		mech.add(NPs);
		
		// will finally be removed 
		mech.add(L12AnR1234);
		mech.add(L23AnR1234);
		mech.add(L34AnR1234);
		mech.add(L45AnR1234);
		mech.add(L51AnR1234);	

			
		// Merge multiple FE AN bodies to single AN respectively
		FemFactory.addFem (L12An, L12AnR2);
		FemFactory.addFem (L12An, L12AnR3);
		FemFactory.addFem (L12An, L12AnR4);		
		FemFactory.addFem (L23An, L23AnR2);
		FemFactory.addFem (L23An, L23AnR3);
		FemFactory.addFem (L23An, L23AnR4);	
		FemFactory.addFem (L34An, L34AnR2);
		FemFactory.addFem (L34An, L34AnR3);
		FemFactory.addFem (L34An, L34AnR4);		
		FemFactory.addFem (L45An, L45AnR2);
		FemFactory.addFem (L45An, L45AnR3);
		FemFactory.addFem (L45An, L45AnR4);		
		FemFactory.addFem (L51An, L51AnR2);
		FemFactory.addFem (L51An, L51AnR3);
		FemFactory.addFem (L51An, L51AnR4);
		
		ANs.add(L12An);
		ANs.add(L23An);
		ANs.add(L34An);
		ANs.add(L45An);
		ANs.add(L51An);	
		mech.add(ANs);
		
		
		// Set RENDER PROPS
		// NP
	    Color NpFaClr = new Color (0.68f, 0.60f, 0.78f); //heather blossom	
	    Color NpLiClr = new Color (0.10f, 0.10f, 0.10f); // dark gray
	    Color AnFaClr = new Color (0.25f, 0.98f, 0.99f); // mint
	    Color AnLiClr = NpLiClr;
	
		// NP
	    for (FemModel3d n : NPs) {
	    	n.setSurfaceRendering (SurfaceRender.Shaded);	    	
	    	RenderProps.setFaceColor (n, NpFaClr);
	    	RenderProps.setLineColor (n, NpLiClr);
	    	RenderProps.setAlpha (n, 1);
	    }   
		// AN
		for (FemModel3d a : ANs) {
			a.setSurfaceRendering (SurfaceRender.Shaded);
			RenderProps.setFaceColor (a, AnFaClr);
			RenderProps.setLineColor (a, AnLiClr);
			RenderProps.setAlpha (a, 1);
			
		}	
	}

	
	
	/**
	 * Create inferior articular FE facets from triangulated surfaces via 
	 * extrusion. Initial facet gap, facet thickness, and number of wedge 
	 * element layers is parameterized. 
	 * To study the effect of varying card angles on the FSU L4/L5 different  
	 * underlying surfaces and RB may be imported. For more information see 
	 * e-poster for ISSLS virtual annual meeting 2021. 
	 */
	private void addFEFacets() throws IOException {		
		PolygonalMesh L12FacL1l_PM, L12FacL1r_PM, L23FacL2l_PM, L23FacL2r_PM, 
			L34FacL3l_PM, L34FacL3r_PM, L45FacL4l_PM, L45FacL4r_PM, 
			L51FacL5l_PM, L51FacL5r_PM;
		
		// Create file path
		String currGeomPath = MyImportFunctions.getGeometryDir();
		String facSubFolder    = "FE/LSS_Facets/";
		String facCaVarsFolder = "ISSLS21_caVars/";
		if (cardAngleStudy != true) { 
			// remove any card angle ID and sub folder when no variation is targeted	
			facCaVarsFolder = "";
			cardIDISSLS = "";
		}
		
		
		// Import surface meshes for the inferior articular facets
		L12FacL1l_PM = new PolygonalMesh (new File (currGeomPath +
				 facSubFolder + "L12FacL1l_i" + ".obj")); 	
		L12FacL1r_PM = new PolygonalMesh (new File (currGeomPath +
				facSubFolder + "L12FacL1r_i" + ".obj"));
		L23FacL2l_PM = new PolygonalMesh (new File ( currGeomPath +
				facSubFolder + "L23FacL2l_i" + ".obj")); 	
		L23FacL2r_PM = new PolygonalMesh (new File ( currGeomPath +
				facSubFolder + "L23FacL2r_i" + ".obj"));
		L34FacL3l_PM = new PolygonalMesh (new File ( currGeomPath +
				facSubFolder + "L34FacL3l_i" + ".obj")); 	
		L34FacL3r_PM = new PolygonalMesh (new File ( currGeomPath +
				facSubFolder + "L34FacL3r_i" + ".obj"));
		L45FacL4l_PM = new PolygonalMesh (new File ( currGeomPath +
				facSubFolder + facCaVarsFolder + "L45FacL4l_i" + cardIDISSLS + ".obj")); 	
		L45FacL4r_PM = new PolygonalMesh (new File ( currGeomPath +
				facSubFolder + facCaVarsFolder + "L45FacL4r_i" + cardIDISSLS + ".obj"));
		L51FacL5l_PM = new PolygonalMesh (new File ( currGeomPath +
				facSubFolder + "L51FacL5l_i" + ".obj")); 	
		L51FacL5r_PM = new PolygonalMesh (new File ( currGeomPath +
				facSubFolder + "L51FacL5r_i" + ".obj"));
		
		
		// Translate inferior facet surface mesh for initial facet gap
		// Assume a resulting vector for translation from all facets normals
		double initFacGap = 0.5e-3; // [m]
		translateInfFacetSurface (L12FacL1l_PM, initFacGap,  1);
		translateInfFacetSurface (L12FacL1r_PM, initFacGap, -1);
		translateInfFacetSurface (L23FacL2l_PM, initFacGap,  1);
		translateInfFacetSurface (L23FacL2r_PM, initFacGap, -1);
		translateInfFacetSurface (L34FacL3l_PM, initFacGap,  1);
		translateInfFacetSurface (L34FacL3r_PM, initFacGap, -1);
		translateInfFacetSurface (L45FacL4l_PM, initFacGap,  1);
		translateInfFacetSurface (L45FacL4r_PM, initFacGap, -1);
		translateInfFacetSurface (L51FacL5l_PM, initFacGap,  1);
		translateInfFacetSurface (L51FacL5r_PM, initFacGap, -1);


		// -------- Create FE bodies from surface via extrusion ---------------
		// Create FEM Model from surface via factory extrusion
		int numLayers = 1; 			// number of element layers
		double facThickn = 1.5e-3; 	// Thickness of inferior articular facets
			
		// Create the inferior articular facets from imported mesh data
		ComponentList<ModelComponent> facsFE = 
				new ComponentList<ModelComponent>(ModelComponent.class, "facetsFE");		
		L12FacL1l = new FemModel3d ("L12FacL1l");  
		L12FacL1r = new FemModel3d ("L12FacL1r");
		L23FacL2l = new FemModel3d ("L23FacL2l");  
		L23FacL2r = new FemModel3d ("L23FacL2r");
		L34FacL3l = new FemModel3d ("L34FacL3l");  
		L34FacL3r = new FemModel3d ("L34FacL3r");
		L45FacL4l = new FemModel3d ("L45FacL4l");  
		L45FacL4r = new FemModel3d ("L45FacL4r");
		L51FacL5l = new FemModel3d ("L51FacL5l");  
		L51FacL5r = new FemModel3d ("L51FacL5r");
		
		FemFactory.createExtrusion (L12FacL1l, numLayers,  facThickn, 0, L12FacL1l_PM);	
		FemFactory.createExtrusion (L12FacL1r, numLayers, -facThickn, 0, L12FacL1r_PM);
		FemFactory.createExtrusion (L23FacL2l, numLayers,  facThickn, 0, L23FacL2l_PM);	
		FemFactory.createExtrusion (L23FacL2r, numLayers, -facThickn, 0, L23FacL2r_PM);
		FemFactory.createExtrusion (L34FacL3l, numLayers,  facThickn, 0, L34FacL3l_PM);
		FemFactory.createExtrusion (L34FacL3r, numLayers, -facThickn, 0, L34FacL3r_PM);
		FemFactory.createExtrusion (L45FacL4l, numLayers,  facThickn, 0, L45FacL4l_PM);
		FemFactory.createExtrusion (L45FacL4r, numLayers, -facThickn, 0, L45FacL4r_PM);
		FemFactory.createExtrusion (L51FacL5l, numLayers,  facThickn, 0, L51FacL5l_PM);
		FemFactory.createExtrusion (L51FacL5r, numLayers, -facThickn, 0, L51FacL5r_PM);
		

		// List of nodes of the facies articularis inferior attache to RB
		LinkedList<FemNode3d> L12FacL1lrb, L12FacL1rrb, L23FacL2lrb, L23FacL2rrb, 
			L34FacL3lrb, L34FacL3rrb, L45FacL4lrb, L45FacL4rrb, 
			L51FacL5lrb, L51FacL5rrb;		
		// Search for nodes to attach to rigid vertebra - use the initial 
		// surface to find the nodes that are facing away from the contact side	
		double searchTol = 1.0e-3;
		L12FacL1lrb = facNodesToAttach (L12FacL1l, L12FacL1l_PM, searchTol, null);
		L12FacL1rrb = facNodesToAttach (L12FacL1r, L12FacL1r_PM, searchTol, null);
		L23FacL2lrb = facNodesToAttach (L23FacL2l, L23FacL2l_PM, searchTol, null);
		L23FacL2rrb = facNodesToAttach (L23FacL2r, L23FacL2r_PM, searchTol, null);
		L34FacL3lrb = facNodesToAttach (L34FacL3l, L34FacL3l_PM, searchTol, null);
		L34FacL3rrb = facNodesToAttach (L34FacL3r, L34FacL3r_PM, searchTol, null);
		L45FacL4lrb = facNodesToAttach (L45FacL4l, L45FacL4l_PM, searchTol, null);
		L45FacL4rrb = facNodesToAttach (L45FacL4r, L45FacL4r_PM, searchTol, null);
		L51FacL5lrb = facNodesToAttach (L51FacL5l, L51FacL5l_PM, searchTol, null);
		L51FacL5rrb = facNodesToAttach (L51FacL5r, L51FacL5r_PM, searchTol, null);
		

		// Store all facets in group and add to mech model
		ComponentList<FemModel3d> L12FacL1, L23FacL2, L34FacL3, L45FacL4, L51FacL5;		
		L12FacL1 = new ComponentList<FemModel3d> (FemModel3d.class, "L12FacL1");
		L23FacL2 = new ComponentList<FemModel3d> (FemModel3d.class, "L23FacL2");
		L34FacL3 = new ComponentList<FemModel3d> (FemModel3d.class, "L34FacL3");
		L45FacL4 = new ComponentList<FemModel3d> (FemModel3d.class, "L45FacL4");
		L51FacL5 = new ComponentList<FemModel3d> (FemModel3d.class, "L51FacL5");
		
		L12FacL1.add (L12FacL1l);
		L23FacL2.add (L23FacL2l);
		L34FacL3.add (L34FacL3l);
		L45FacL4.add (L45FacL4l);
		L51FacL5.add (L51FacL5l);		
		L12FacL1.add (L12FacL1r);
		L23FacL2.add (L23FacL2r);
		L34FacL3.add (L34FacL3r);
		L45FacL4.add (L45FacL4r);
		L51FacL5.add (L51FacL5r);
		
		facsFE.add (L12FacL1);
		facsFE.add (L23FacL2);
		facsFE.add (L34FacL3);
		facsFE.add (L45FacL4);
		facsFE.add (L51FacL5);
		mech.add (facsFE);
		
		
		// Connect FE nodes of facets to RB
		ConnectFemToRB (L12FacL1lrb, L1RB);
		ConnectFemToRB (L12FacL1rrb, L1RB);
		ConnectFemToRB (L23FacL2lrb, L2RB);
		ConnectFemToRB (L23FacL2rrb, L2RB);
		ConnectFemToRB (L34FacL3lrb, L3RB);
		ConnectFemToRB (L34FacL3rrb, L3RB);
		ConnectFemToRB (L45FacL4lrb, L4RB);
		ConnectFemToRB (L45FacL4rrb, L4RB);
		ConnectFemToRB (L51FacL5lrb, L5RB);
		ConnectFemToRB (L51FacL5rrb, L5RB);
				
		// Set render props
		setFacRenderProps (L12FacL1l);
		setFacRenderProps (L12FacL1r);
		setFacRenderProps (L23FacL2l);
		setFacRenderProps (L23FacL2r);
		setFacRenderProps (L34FacL3l);
		setFacRenderProps (L34FacL3r);
		setFacRenderProps (L45FacL4l);
		setFacRenderProps (L45FacL4r);
		setFacRenderProps (L51FacL5l);
		setFacRenderProps (L51FacL5r);		
	}



	/**
	 * Use damping to stabilize model (e.g. to prevent inverted finite elements)
	 */
	private void setDamping() {
		
		double DampFr = 0.2; // Frame damping for RB vertebrae
		double DampRt = 0.1; // Rotatory damping for RB vertebrae
		
		// Damping to stabilize model - quasi static 
		L1RB.setFrameDamping (DampFr*2);
		L1RB.setRotaryDamping (DampRt*2);
		L2RB.setFrameDamping (DampFr);
		L2RB.setRotaryDamping (DampRt);
		L3RB.setFrameDamping (DampFr);
		L3RB.setRotaryDamping (DampRt);
		L4RB.setFrameDamping (DampFr);
		L4RB.setRotaryDamping (DampRt);
		L5RB.setFrameDamping (DampFr);
		L5RB.setRotaryDamping (DampRt);	
	}


	
	/**
	 * Connect RB to RB (e.g. for parts/segments of one bone)
	 */
	private void addAttachments_RBRB() {		
		mech.attachFrame (L2FacRB, L2RB);
		mech.attachFrame (L3FacRB, L3RB);
		mech.attachFrame (L4FacRB, L4RB);
		mech.attachFrame (L5FacRB, L5RB);
		mech.attachFrame (S1FacRB, S1RB);	
	}



	/**
	 * Set model constraints
	 */
	private void addConstraints() {	
		L5RB.setDynamic(true);  // optional for lumbar spine evaluation
		S1RB.setDynamic(false); // fix sacrum in space					
	}



	/**
	 * Add superior articular processes of facet joints and set properties
	 */
	private void addRBFacets() throws IOException {	
		ComponentList<RigidBody> FacetsRB = 
				new ComponentList<RigidBody> (RigidBody.class, "factesRB");
		L2FacRB = new RigidBody ("L2FacLRB");  
		L3FacRB = new RigidBody ("L3FacLRB");  
		L4FacRB = new RigidBody ("L4FacLRB");  
		L5FacRB = new RigidBody ("L5FacLRB"); 
		S1FacRB = new RigidBody ("S1FacLRB"); 		
		
		// In case of card angle (CA) study set to true to reach sub-folder  
		// with superior articular processes varied in their CA (ISSLS 2021)
		String caFolder = "";
		if (cardAngleStudy == true) {
			caFolder = "ISSLS21_caVars/";
		}
				
		// Facets - import and add the parts to the main vertebrae RB components
		PolygonalMesh L2FacLPM, L3FacLPM, L4FacLPM, L5FacLPM, S1FacLPM, 
			L2FacRPM, L3FacRPM, L4FacRPM, L5FacRPM, S1FacRPM;
		
		L2FacLPM = MyImportFunctions.addObjMesh ("L2_PDl_a");
		L3FacLPM = MyImportFunctions.addObjMesh ("L3_PDl_a");
		L4FacLPM = MyImportFunctions.addObjMesh ("L4_PDl_a");
		L5FacLPM = MyImportFunctions.addObjMesh (caFolder + "L5_PDl_a" + cardIDISSLS);
		S1FacLPM = MyImportFunctions.addObjMesh ("S1_PDl_a");
		
		L2FacRPM = MyImportFunctions.addObjMesh ("L2_PDr_a");
		L3FacRPM = MyImportFunctions.addObjMesh ("L3_PDr_a");
		L4FacRPM = MyImportFunctions.addObjMesh ("L4_PDr_a");
		L5FacRPM = MyImportFunctions.addObjMesh (caFolder + "L5_PDr_a" + cardIDISSLS);
		S1FacRPM = MyImportFunctions.addObjMesh ("S1_PDr_a");
			
		double facMass = 1e-8;
		L2FacRB.setMass (facMass);
		L3FacRB.setMass (facMass);
		L4FacRB.setMass (facMass);
		L5FacRB.setMass (facMass);
		S1FacRB.setMass (facMass);
		
		L2FacRB.setCollidable (Collidability.EXTERNAL);
		L3FacRB.setCollidable (Collidability.EXTERNAL);
		L4FacRB.setCollidable (Collidability.EXTERNAL);
		L5FacRB.setCollidable (Collidability.EXTERNAL);
		S1FacRB.setCollidable (Collidability.EXTERNAL);
		
		
		L2FacRB.addMesh (L2FacLPM, /*hasMass*/ false, /*isCollidable*/true);
		L3FacRB.addMesh (L3FacLPM, /*hasMass*/ false, /*isCollidable*/true);
		L4FacRB.addMesh (L4FacLPM, /*hasMass*/ false, /*isCollidable*/true);
		L5FacRB.addMesh (L5FacLPM, /*hasMass*/ false, /*isCollidable*/true);
		S1FacRB.addMesh (S1FacLPM, /*hasMass*/ false, /*isCollidable*/true);
		
		L2FacRB.addMesh (L2FacRPM, /*hasMass*/ false, /*isCollidable*/true);
		L3FacRB.addMesh (L3FacRPM, /*hasMass*/ false, /*isCollidable*/true);
		L4FacRB.addMesh (L4FacRPM, /*hasMass*/ false, /*isCollidable*/true);
		L5FacRB.addMesh (L5FacRPM, /*hasMass*/ false, /*isCollidable*/true);
		S1FacRB.addMesh (S1FacRPM, /*hasMass*/ false, /*isCollidable*/true);	
				
		FacetsRB.add (L2FacRB);
		FacetsRB.add (L3FacRB);
		FacetsRB.add (L4FacRB);
		FacetsRB.add (L5FacRB);
		FacetsRB.add (S1FacRB);
		mech.add (FacetsRB);
		
		// Render Props
		RenderProps.setFaceColor (L2FacRB, RBColor.darker());
		RenderProps.setFaceColor (L3FacRB, RBColor.darker());
		RenderProps.setFaceColor (L4FacRB, RBColor.darker());
		RenderProps.setFaceColor (L5FacRB, RBColor.darker());
		RenderProps.setFaceColor (S1FacRB, RBColor.darker());
	}



	/**
	 * Add auxiliary bodies to automatically find the different located portions 
	 * of the CF in AN
	 */
	private void addAuxBodies() throws IOException {			
		String auxFolder = "auxGeometries/";		

		L12_postTriPM = MyImportFunctions.addObjMesh (auxFolder + "L12_postTriangle_100deg");
		L12_latTriPM  = MyImportFunctions.addObjMesh (auxFolder + "L12_latTriangle_30deg");
		L23_postTriPM = MyImportFunctions.addObjMesh (auxFolder + "L23_postTriangle_100deg");
		L23_latTriPM  = MyImportFunctions.addObjMesh (auxFolder + "L23_latTriangle_30deg");
		L34_postTriPM = MyImportFunctions.addObjMesh (auxFolder + "L34_postTriangle_100deg");
		L34_latTriPM  = MyImportFunctions.addObjMesh (auxFolder + "L34_latTriangle_30deg");
		L45_postTriPM = MyImportFunctions.addObjMesh (auxFolder + "L45_postTriangle_100deg");
		L45_latTriPM  = MyImportFunctions.addObjMesh (auxFolder + "L45_latTriangle_30deg");
		L51_postTriPM = MyImportFunctions.addObjMesh (auxFolder + "L51_postTriangle_100deg");
		L51_latTriPM  = MyImportFunctions.addObjMesh (auxFolder + "L51_latTriangle_30deg");	
	}



	/**
	 * Add all rigid bodies (vertebrae) to mech model and set basic properties
	 */
	private void addRigidBones() throws IOException {
		L1RB = new RigidBody ("L1RB"); 
		L2RB = new RigidBody ("L2RB"); 
		L3RB = new RigidBody ("L3RB"); 	
		L4RB = new RigidBody ("L4RB"); 	
		L5RB = new RigidBody ("L5RB");	
		S1RB = new RigidBody ("S1RB");	
		
		// Additional RBs only for calculation purpose - no import to mech model		
		RigidBody L1BodyRB, L2BodyRB, L3BodyRB, L4BodyRB, L5BodyRB, S1BodyRB;	
		L1BodyRB = new RigidBody();
		L2BodyRB = new RigidBody();
		L3BodyRB = new RigidBody();
		L4BodyRB = new RigidBody();
		L5BodyRB = new RigidBody();
		S1BodyRB = new RigidBody();		
		
		// Components of the whole vertebrae
		PolygonalMesh L1BodyPM, L2BodyPM, L3BodyPM, L4BodyPM, L5BodyPM, S1BodyPM;
		PolygonalMesh L1GesPM, L2GesPM, L3GesPM, L4GesPM, L5GesPM, S1GesPM;
		// Additional components for visualization
		RigidMeshComp L1GesRMC, L2GesRMC, L3GesRMC, L4GesRMC, L5GesRMC, S1GesRMC;
		

		// Import and assemble vertebrae
		// L1		
		L1BodyPM = MyImportFunctions.addObjMesh ("L1_body");	
		L1BodyPM.setName("L1body");
		L1GesPM  = MyImportFunctions.addObjMesh ("L1_Schrumpfverpacken");
		L1RB.addMesh (L1BodyPM, /*hasMass*/ false, /*isCollidable*/false);
		L1GesRMC = L1RB.addMesh (L1GesPM,  /*hasMass*/ true,  /*isCollidable*/false);
		L1BodyRB.addMesh (L1BodyPM, /*hasMass*/ true,  /*isCollidable*/false);
		// L2		
		L2BodyPM = MyImportFunctions.addObjMesh ("L2_body");
		L2BodyPM.setName("L2body");
		L2GesPM  = MyImportFunctions.addObjMesh ("L2_Schrumpfverpacken");
		L2RB.addMesh (L2BodyPM, false, false);
		L2GesRMC = L2RB.addMesh (L2GesPM, true, false);
		L2BodyRB.addMesh (L2BodyPM, true, false);		
		// L3
		L3BodyPM = MyImportFunctions.addObjMesh ("L3_body");
		L3BodyPM.setName("L3body");
		L3GesPM  = MyImportFunctions.addObjMesh ("L3_Schrumpfverpacken");
		L3RB.addMesh (L3BodyPM, false, false);
		L3GesRMC = L3RB.addMesh (L3GesPM, true,  false);
		L3BodyRB.addMesh (L3BodyPM, true, false);
		// L4
		L4BodyPM = MyImportFunctions.addObjMesh ("L4_body");
		L4BodyPM.setName("L4body");
		L4GesPM  = MyImportFunctions.addObjMesh ("L4_Schrumpfverpacken");
		L4RB.addMesh (L4BodyPM, false, false);
		L4GesRMC = L4RB.addMesh (L4GesPM, true, false);
		L4BodyRB.addMesh (L4BodyPM, true, false);
		// L5 
		L5BodyPM = MyImportFunctions.addObjMesh ("L5_body");
		L5BodyPM.setName("L5body");
		L5GesPM  = MyImportFunctions.addObjMesh ("L5_Schrumpfverpacken");
		L5RB.addMesh (L5BodyPM, false, false);
		L5GesRMC = L5RB.addMesh (L5GesPM, true, false);
		L5BodyRB.addMesh (L5BodyPM, true, false); 		
		// S1
		S1BodyPM = MyImportFunctions.addObjMesh ("S1_body");
		S1BodyPM.setName("S1body");
		S1GesPM  = MyImportFunctions.addObjMesh ("S1_Schrumpfverpacken");
		S1RB.addMesh(S1BodyPM, false, false);
		S1GesRMC  = S1RB.addMesh (S1GesPM,  true, false);
		S1BodyRB.addMesh (S1BodyPM, true, false);  
			
		
		// Set bone density properties
		double boneD = 1500; 	// [kg/m^3] 
		L1RB.setDensity (boneD);
		L2RB.setDensity (boneD);
		L3RB.setDensity (boneD);
		L4RB.setDensity (boneD);
		L5RB.setDensity (boneD);
		S1RB.setDensity (boneD);

				
		// Add bones to mechModel 
		mech.addRigidBody (L1RB);
		mech.addRigidBody (L2RB);
		mech.addRigidBody (L3RB);
		mech.addRigidBody (L4RB);
		mech.addRigidBody (L5RB);
		mech.addRigidBody (S1RB);

		
		// Get center of masses of vertebral bodies (e.g. for Wrenches)
		L1Body_CoM = L1BodyRB.getCenterOfMass();
		L2Body_CoM = L2BodyRB.getCenterOfMass();
		L3Body_CoM = L3BodyRB.getCenterOfMass();
		L4Body_CoM = L4BodyRB.getCenterOfMass();
		L5Body_CoM = L5BodyRB.getCenterOfMass();
		S1Body_CoM = S1BodyRB.getCenterOfMass();
		
		
		// ---------------------- SET RENDER PROPS ------------------------    
	    double RBAlpha = 0.8;
	    // Vertebrae
	    RenderProps.setFaceColor (L1RB, RBColor);
	    RenderProps.setFaceColor (L2RB, RBColor);
	    RenderProps.setFaceColor (L3RB, RBColor);
	    RenderProps.setFaceColor (L4RB, RBColor);
	    RenderProps.setFaceColor (L5RB, RBColor);
	    RenderProps.setFaceColor (S1RB, RBColor);
	    RenderProps.setAlpha (L1GesRMC, RBAlpha);
	    RenderProps.setAlpha (L2GesRMC, RBAlpha);
	    RenderProps.setAlpha (L3GesRMC, RBAlpha);
		RenderProps.setAlpha (L4GesRMC, RBAlpha);
		RenderProps.setAlpha (L5GesRMC, RBAlpha);
		RenderProps.setAlpha (S1GesRMC, RBAlpha);
		// set vertebral bodies visible/invisible
		boolean vertebralBodies = false;
		RenderProps.setVisible((Renderable) 
				L1RB.findComponent("meshes/L1body"), vertebralBodies);
		RenderProps.setVisible((Renderable) 
				L2RB.findComponent("meshes/L2body"), vertebralBodies);
		RenderProps.setVisible((Renderable) 
				L3RB.findComponent("meshes/L3body"), vertebralBodies);
		RenderProps.setVisible((Renderable) 
				L4RB.findComponent("meshes/L4body"), vertebralBodies);
		RenderProps.setVisible((Renderable) 
				L5RB.findComponent("meshes/L5body"), vertebralBodies);
		RenderProps.setVisible((Renderable) 
				S1RB.findComponent("meshes/S1body"), vertebralBodies);
	}


	
	
	
	
	


	// ------------ RENDER PROPERTIES -----------------------
	// sets the FEM's render properties
	protected void setNpRenderProps (FemModel3d fem) {
		RenderProps.setLineColor (fem, Color.BLUE);
		fem.setSurfaceRendering (SurfaceRender.Stress);
	}	
	
	protected void setAnRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering (SurfaceRender.Stress);
	    RenderProps.setLineColor (fem, Color.BLUE);
		RenderProps.setAlpha (fem, 0.1);
	}
	
	protected void setFacRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering (SurfaceRender.Shaded);
	    RenderProps.setLineColor (fem, Color.BLACK);
	    RenderProps.setFaceColor (fem, new Color (0.3f, 0.3f, 0.3f));
		RenderProps.setAlpha (fem, 0.8);
	}
	
	
	// make marked nodes in LinkedList visible
	protected void RenderNodeSet (LinkedList<FemNode3d> nodes, Color color) {
		for(int i = 0; i < nodes.size(); i++) {
			FemNode3d node = nodes.get(i);
			RenderProps.setAlpha (node, 1);
			RenderProps.setSphericalPoints (node, 0.0005, color);
		}
	}
	

	// Connect (slave) nodes to (master) rigid body (RB) 
	protected void ConnectFemToRB (LinkedList<FemNode3d> NodeList, RigidBody RB) {
		for(FemNode3d node : NodeList) {
			// check if node is already attached to sth.
			if(node.getAttachment() == null) {  
				mech.addAttachment(new PointFrameAttachment (RB, node));
				//System.out.println("RB: " + RB + ", node: "  + node.getNumber());
			}
		}
	}
	
	
	// Remove nodes in nList1 when also listed in nList2 and returns a 
	// new list nList3 without double nodes
	protected LinkedList<FemNode3d> RemoveDoubleNodes_SetSet (
			LinkedList<FemNode3d> nList1, LinkedList<FemNode3d> nList2) {
	   LinkedList<FemNode3d> nList3 = new LinkedList<FemNode3d>();
	   for(FemNode3d n : nList1) {
		   if(!nList2.contains(n)) {
			   nList3.add(n);   
		   }
	   }   
	   return nList3;
	}
	
	
	
	
	private void CreateUWLigament (ComponentList<ModelComponent> compList, 
			LigamentData ligsDataImp, int ligIndx, RigidBody RB1, RigidBody RB2,
			MechModel mech, Color LigColor) {	
		LinkedList<Point3d> ligsStart     = ligsDataImp.getStarts();
		LinkedList<Point3d> ligsEnd       = ligsDataImp.getEnds();
		LinkedList<String>  ligsName      = ligsDataImp.getNames();
		LinkedList<Double>  ligsLinStiff  = ligsDataImp.getUWLinStiff();
		LinkedList<Double>  ligsEpsr      = ligsDataImp.getUWEpsr();
		LinkedList<Double>  ligsEpst      = ligsDataImp.getUWEpst();
		
		// Create FrameMarkers and add to mech model
		FrameMarker mkr1 = new FrameMarker(ligsStart.get(ligIndx).x, 
				ligsStart.get(ligIndx).y, ligsStart.get(ligIndx).z);
		FrameMarker mkr2 = new FrameMarker(ligsEnd.get(ligIndx).x,   
				ligsEnd.get(ligIndx).y, ligsEnd.get(ligIndx).z);
		mkr1.setFrame(RB1);
		mkr2.setFrame(RB2);
		mech.addFrameMarker(mkr1);
		mech.addFrameMarker(mkr2);
		// Create AxialSpring as ligament and add to mech model
		AxialSpring Lig0 = new AxialSpring (ligsName.get(ligIndx));
		Lig0.setPoints(mkr1,mkr2);
		// Add to model component list
		compList.add(Lig0);
		
		MyUWLigamentMaterial ligMat = new MyUWLigamentMaterial();
		// define material parameters
		double epsr = ligsEpsr.get(ligIndx);
		double lGeo = Lig0.getLength();  // get (real rest) length of axial spring
		ligMat.setLinearStiffness (ligsLinStiff.get(ligIndx));  		// [N/m]
		ligMat.setLigamentTransitionStrain (ligsEpst.get(ligIndx));  // [m/m]
		ligMat.setReferenceStrain(epsr);      						// [m/m]
		
		// l0 is NOT the initial length here but the reference length 
		// and must therefore be calculated accordingly  
		Lig0.setRestLength (lGeo*(epsr+1));  // [m]
      
		// finally apply all material parameters to ligament
		Lig0.setMaterial (ligMat);
		
		RenderProps.setCylindricalLines (Lig0, 0.0004, LigColor);
	}
	
	
	/**
	 *  Create one Multi-Point (MP) ligament and return it to compList
	 */
	private void CreateUWLigamentMP (ComponentList<ModelComponent> compList, 
			LigamentMPData ligsMPDataImp, int ligIndx, RigidBody RB1, 
			RigidBody RB2, MechModel mech, Color LigColor) {	
		LinkedList<FemNode3d[]> ligMPNodes 	= ligsMPDataImp.getMPNodes();
		LinkedList<Point3d> ligsStart       = ligsMPDataImp.getStarts();
		LinkedList<Point3d> ligsEnd       	= ligsMPDataImp.getEnds();
		LinkedList<String>  ligsName      	= ligsMPDataImp.getNames();
		LinkedList<Double>  ligsLinStiff  	= ligsMPDataImp.getUWLinStiff();
		LinkedList<Double>  ligsEpsr      	= ligsMPDataImp.getUWEpsr();
		LinkedList<Double>  ligsEpst      	= ligsMPDataImp.getUWEpst();
		
		// Create FrameMarkers and add to mech model
		FrameMarker mkr1 = new FrameMarker(ligsStart.get(ligIndx).x, 
				ligsStart.get(ligIndx).y, ligsStart.get(ligIndx).z);
		FrameMarker mkr2 = new FrameMarker(ligsEnd.get(ligIndx).x,   
				ligsEnd.get(ligIndx).y, ligsEnd.get(ligIndx).z);
		mkr1.setFrame(RB1);
		mkr2.setFrame(RB2);
		mech.addFrameMarker(mkr1);
		mech.addFrameMarker(mkr2);
		// Create AxialSpring as ligament and add to mech model
		MultiPointSpring ligMP0 = new MultiPointSpring (ligsName.get(ligIndx));
		int v = ligMPNodes.get(ligIndx).length; // number of via points of MP
		for (int P = 0; P < v; P++) {
			// Differentiate between points added to MP 
			if(P == 0) {
				// Connect first point to upper RB
				ligMP0.addPoint (mkr1);
			}
			else if(P == v-1){
				// Connect last point to lower RB
				ligMP0.addPoint (mkr2);
			}
			else {
				// Connect all other MP Spring points to FE nodes (of AN)
				ligMP0.addPoint (ligMPNodes.get (ligIndx)[P]);
			}
		}
		// Add to component list
		compList.add(ligMP0);
					
		MyUWLigamentMaterial ligMPMat = new MyUWLigamentMaterial();
		// define material parameters
		double epsr = ligsEpsr.get(ligIndx);
		double lMPGeo = ligMP0.getLength();  // get (real rest) length of MP spring
		ligMPMat.setLinearStiffness (ligsLinStiff.get (ligIndx));  	   // [N/m]
		ligMPMat.setLigamentTransitionStrain (ligsEpst.get(ligIndx));  // [m/m]
		ligMPMat.setReferenceStrain (epsr);      					   // [m/m]
		
		// l0 is NOT the initial length here but the reference length 
		// and must therefore be calculated accordingly
		ligMP0.setRestLength (lMPGeo*(1*epsr+1));  	// [m]
      
		// finally apply all material parameters to MP ligament
		ligMP0.setMaterial (ligMPMat);
		
		RenderProps.setCylindricalLines (ligMP0, 0.0004, LigColor);	
	}
	
	
	protected Particle addParticleRB (ComponentList<Particle> partList, 
			String string, RigidBody RB, double x, double y, double z) {		
		Particle p = new Particle (string, /*mass*/0.0, x, y, z);
		partList.add (p);
		mech.attachPoint (p, RB);				
		return p;
	}
		
	
	protected FrameMarker createFM_RB (FrameMarker FM, Point3d loc, 
			RigidBody RB, Boolean visible) {
		FrameMarker myFM = FM;
		// Add FrameMarker to RB and mechModel
		mech.addFrameMarker(myFM, RB, loc);			
		
		// Render Props
		if (visible == true) {
			RenderProps.setPointStyle (myFM, Renderer.PointStyle.POINT);
		    RenderProps.setPointSize (myFM, 8);
		    RenderProps.setPointColor (myFM, new Color(51,255,104));
		}	
		return myFM;
	}
	
	
	protected FrameMarker createFM_FEM(FrameMarker FM, Point3d loc, 
			FemModel3d FEM, Boolean visible) {
		FrameMarker myFM = FM;
		// Find node at Location loc and create new Frame at this location
		FemNode3d Nd = FEM.findNearestNode(loc, 0.01);		
		Frame FR = new Frame (new RigidTransform3d (
				Nd.getPosition().x, Nd.getPosition().y, Nd.getPosition().z));		
		// Add Frame and to FEM at the found node
		mech.addFrame(FR);
		mech.attachFrame(FR, FEM);		
		// Add FrameMarker at Frames position to mechModel
		mech.addFrameMarker(myFM, FR, new Point3d(0,0,0));	
		
		// Render Props
		if (visible == true) {
			RenderProps.setPointStyle (myFM, Renderer.PointStyle.POINT);
		    RenderProps.setPointSize (myFM, 8);
		    RenderProps.setPointColor (myFM, new Color(51,255,204));    
		    //FR.setAxisLength(0.03);  // makes the frame visible
		}
		return myFM;
	}
	
	
	
	private void setWrenchsPoseWithFrame(Wrench Wr, Frame Fr_Wr, 
			Point3d Pnt_Loc, Vector3d Vec_xAx, Vector3d Vec_yAx, 
			double RotX, double AxLngth) {
		// use two vectors and a point to set a frames pose 
		// apply a wrench to the frame and use its pose
		Fr_Wr.setPosition(Pnt_Loc);  		// set frames position at point Pnt_loc		

		RotationMatrix3d rotXY = new RotationMatrix3d();
		rotXY.setXYDirections (Vec_xAx.normalize(), Vec_yAx.normalize()); 	
		// it might be easier to extract the most anterior and superior points, 
		// so the y- and z-axis must be 'switched'		
		rotXY.mulRotX(RotX); 
		// set frames orientation based on OffsetFrameSprings calculations 
		Fr_Wr.setOrientation (rotXY.getAxisAngle()); 	
			
		Vector3d myMoments = new Vector3d();  // moments about initial axis	
		Wr.m.set(myMoments);				 // apply rotated moments to frame (=0)
		Wr.transform (Fr_Wr.getPose().R, Wr);
			
		Fr_Wr.addExternalForce (Wr);	// add wrench as external force to frame
		Fr_Wr.setAxisLength (AxLngth);  // make frame visible		
		mech.addFrame (Fr_Wr);			// add frame to mech model
	}
	
	
	
	/** 
	 * Use to set a frame pose by two vectors and a point 	 
	 * @param Fr_Wr Wrench which pose is to be set
	 */
	private void setFramesPoseInSpace (Frame Fr_Wr, Point3d Pnt_Loc, 
			Vector3d Vec_xAx, Vector3d Vec_yAx, double RotX, double AxLngth) {
		// apply a wrench to the frame and use its pose
		Fr_Wr.setPosition (Pnt_Loc);  // set frames position at point Pnt_loc		

		RotationMatrix3d rotXY = new RotationMatrix3d();
		rotXY.setXYDirections (Vec_xAx.normalize(), Vec_yAx.normalize()); 	
		// it might be easier to extract the most anterior and superior points, 
		// so the y- and z-axis must be 'switched'		
		rotXY.mulRotX (RotX); 	
		// set frames orientation based on OffsetFrameSprings calculations 
		Fr_Wr.setOrientation (rotXY.getAxisAngle()); 	
						
		Fr_Wr.setAxisLength (AxLngth);   // make frame visible	
		mech.addFrame (Fr_Wr);	 		
	}
	
	
	
	/** 
	 * Simply select all the nodal locations from a LinkedList<FemNode3d> 
	 * and adds them to 'myPointList'.
	 * More than one LinkedList with FemNode3ds can be imported.*/ 
	protected void CollectNodesLocations (LinkedList<Point3d> myPointList, 
			LinkedList<LinkedList<FemNode3d>> LinkedFemNodeList) {		
		for (LinkedList<FemNode3d> myFemNodeList : LinkedFemNodeList) {
			for (FemNode3d myNode : myFemNodeList) {						
				myPointList.add (myNode.getPosition());
			}
		}
	}
		
	
	// Get the nodes from 'myFemModel' that lay at the locations 'myPointList' 
	// that is within the distance 'tol'
	private LinkedList<FemNode3d> getNodesFromLocations (
			LinkedList<Point3d> myPointList, FemModel3d myFemModel, double tol) {			
		LinkedList<FemNode3d> myNodeList = new LinkedList<FemNode3d>();
		for (Point3d myPoint : myPointList) {
			FemNode3d myNode = myFemModel.findNearestNode(myPoint, tol);
			// if is not null, a node is found
			if (myNode != null) { 
				myNodeList.add (myNode);
			}
		}
		return myNodeList;
	}	
	   
	
	// Calculate reference radius for the thickest and outermost CF
	private double calcCFRadius (double[] CFStiffFracs, double[] CFLengths, 
			FemModel3d AN_FEM, double CF_VFracGes) {			
		double w = 0;			
		for(int i = 0; i < CFStiffFracs.length; i++) 
			w += Math.pow (CFStiffFracs[i], 2) * CFLengths[i];				
		double CF_r = Math.sqrt ((CF_VFracGes*AN_FEM.updateVolume()) / (Math.PI*w));	
		
		//System.out.println ("CF reference radius = " + CF_r);		
		return CF_r;
	}
	

	private Frame newFrame(String name) {
		Frame myFrame = new Frame();
		myFrame.setName(name);
		return myFrame;
	}
	
	
	/**
	 * Translate inferior facet surface mesh for initial facet gap. 
	 * Assume a resulting vector for translation from all facets normals
	 * @param infSurfMesh Polygonal surface meshes for inferior articular facet
	 * @param initFacGap initial facet gap
	 * @param direction direction of extrusion 
	 */
	private void translateInfFacetSurface (
			PolygonalMesh infSurfMesh, double initFacGap, int direction) {
		ArrayList<Vector3d> surfMeshNormals = infSurfMesh.getNormals();
		Vector3d dirVector = new Vector3d();
		for (Vector3d v : surfMeshNormals) {
			dirVector.add(v);
		}
		dirVector.normalize();	
		infSurfMesh.translate(dirVector.scale(initFacGap));
	}

	
	/**
	 * Selecting the nodes of FE joint facets extruded in advance from a 
	 * triangulated surface mesh. The surface mesh is used to identify the 
	 * correct nodes, facing away from the contact side.
	 * 
	 * @param inferFacFE FE model of the inferior articular facet
	 * @param inferFacSurface Underlying surface mesh of the inferior articular facet
	 * @param searchTol Tolerance in relation to the surface within which no nodes are selected 
	 * @param color set to null if the selected nodes are not to be highlighted
	 * @return the nodes to be used for an attachment
	 */
	private LinkedList<FemNode3d> facNodesToAttach (FemModel3d inferFacFE, 
			PolygonalMesh inferFacSurface, double searchTol, Color color) {		
				
		LinkedList<FemNode3d> selectedNodes = new LinkedList<FemNode3d>();
		for (FemNode3d n : inferFacFE.getNodes()) {		
			if (inferFacSurface.distanceToPoint(n.getPosition()) > searchTol) {
				selectedNodes.add(n);			
			}
		}		
		
		if (color != null) {
			RenderNodeSet(selectedNodes, color);
		}			
		return selectedNodes;
	}

	
	
   
	// ------------- MATERIAL PROPERTIES ------------------	
	// Set the Collagen Fiber (CF) material properties	
	protected void createCF_MP (CollFiberData_MP CFData, 
			RenderableComponentList<MultiPointSpring> CF_RCL, 
			FemModel3d fem, FemModel3d MeshedRBIntersPost, FemModel3d MeshedRBIntersLat, 
			double CF_r, double CFVolFrac, String[] myMapNames) throws IOException 
	{	
		// Load maps for LookUp table for force calculations of CF
		// Map1 for anterior CF (or not lateral and not dorsal)
		// MapP for the posterior area of AN, specific stress-strain curve is used
		// MapL for the lateral area(s) of the AN, specific stress-strain curve is used
		TreeMap<Float, Float> myMap1, myMapP, myMapL;
		myMap1 = MyImportFunctions.ImportCFTreeMapData (myMapNames[0]); // main Mat
		myMapP = MyImportFunctions.ImportCFTreeMapData (myMapNames[1]); // posterior Mat
		myMapL = MyImportFunctions.ImportCFTreeMapData (myMapNames[2]); // lateral Mat
		
		LinkedList<FemNode3d[]> CFNodes = new LinkedList<FemNode3d[]>();
		
		// step through the MP-Springs of one CF-ring (the nodes are in an array)
		for(int c = 0; c < CFData.CFNodes.size(); c++) {
			Point3d[] myCFSpringLocs = CFData.CFPoints.get(c);  
			LinkedList<Point3d> myCFSpringLocsLL = new LinkedList<Point3d>();
			Collections.addAll(myCFSpringLocsLL, myCFSpringLocs);  // convert array to linkedList
					
			// search for nodes in merged AN-bodie via imported locations			
			LinkedList<FemNode3d> myCFSpringLL = getNodesFromLocations (
					myCFSpringLocsLL, fem, 0.0001);
			// transfer to data format (LinkedList<FemNode3d[]>), initially written for not merged AN						
			FemNode3d[] myCFSpringNodes = myCFSpringLL.toArray (
					new FemNode3d[myCFSpringLocsLL.size()]);
			CFNodes.add(myCFSpringNodes);	
		}
		
		// Calc the CF cross section from whole/merged AN (no single rings exist)
		double CF_Radius = CF_r*CFVolFrac;		
		//System.out.println ("CF_Radius = " + CF_Radius + ", CFVolFrac= " + CFVolFrac);
		double CFCrossSec = Math.PI * CF_Radius*CF_Radius;
		
		// update the volume fraction information in specific value-class 
		// (until this step == null)		
		CFData.CFCrossSec 	= CFCrossSec;   // [m^2]
		CFData.CFVolFrac 	= CFVolFrac;	// [-]	
		
		// For scaling the true radius of the plotted CFs (for debug only!)
		double CF_ScaleR    = 1;  
		// Damping for AN nodes which are connected to CFs
		double CFNode_damping = 10.0;   
		
		// Create Multi-Point-spring and set material props
		// --> step through each CF of layer 'c' with its nodes 'p'
		for (int c = 0; c < CFNodes.size(); c++)  {		
			// Set up MP spring and construct name
			MultiPointSpring CFbr = 
					new MultiPointSpring ("CF_" + CFData.getCFLayerName() + "_" + c);	
			
			// step through each node p of a Multi-Point (MP) spring CF
			for (int p = 0; p < CFNodes.get(c).length; ++p){				
				CFbr.addPoint(CFNodes.get(c)[p]);
				// add small damping value to nodes	(check if still required!)		
				CFNodes.get(c)[p].setPointDamping (CFNode_damping);	
			}	
			
			// Differentiate between different AN locations (dorsal and lateral)		
			// check for the first and last node of CF if the posterior meshed 
			// auxiliary RB contains it
			FemElement3d myTmpElemP1 = 
					MeshedRBIntersPost.findContainingElement (
							CFNodes.get (c)[0].getPosition());
			FemElement3d myTmpElemP2 = 
					MeshedRBIntersPost.findContainingElement (
							CFNodes.get (c)[CFNodes.get(c).length-1].getPosition());
			// if is not null, node is in any element of meshed intersection RB (posterior)
			if (myTmpElemP1 != null || myTmpElemP2 != null) {  
				//System.out.println("CF with name: '" + CFbr.getName() + "' lays dorsally.");
				CFbr.setMaterial(new LookUpTableMaterial (myMapP, 1, CFCrossSec));	
				RenderProps.setCylindricalLines (CFbr, CF_Radius*CF_ScaleR, 
						new Color(0.60f, 0.40f, 0.2f));
			}
			else {  // CF is not in posterior specified region. Check for lateral intersection second
				FemElement3d myTmpElemL1 = 
						MeshedRBIntersLat.findContainingElement (
								CFNodes.get (c)[0].getPosition()); // first node
				FemElement3d myTmpElemLm = 
						MeshedRBIntersLat.findContainingElement (
								CFNodes.get (c)[Math.round( 
									(CFNodes.get(c).length-1)/2)].getPosition()); // middle node
				FemElement3d myTmpElemL2 = 
						MeshedRBIntersLat.findContainingElement (
								CFNodes.get (c)[
								    CFNodes.get(c).length-1].getPosition());	// last node
			// if is not null, node is in any element of meshed intersection RB (lateral)
			if (myTmpElemL1 != null || myTmpElemL2 != null || myTmpElemLm != null) { 
				//System.out.println("CF with name: '" + CFbr.getName() + "' lays laterally.");
				CFbr.setMaterial (new LookUpTableMaterial(myMapL, 1, CFCrossSec));	
				RenderProps.setCylindricalLines(CFbr, CF_Radius*CF_ScaleR, 
						new Color(0.91f, 0.51f, 0.29f));					
			}
			else {	// CF is not in any of the two explicitly specified auxiliary regions	
				CFbr.setMaterial (new LookUpTableMaterial(myMap1, 1, CFCrossSec));	
				// set radius (is the same for all CF of a ring) and default color for 
				// not dorsal or lateral CF
				RenderProps.setCylindricalLines (CFbr, CF_Radius*CF_ScaleR, 
						new Color(0.98f, 0.98f, 0.2f));
				}
			}
			
			
			LookUpTableMaterial myCFbrMat = (LookUpTableMaterial)CFbr.getMaterial();
			// Set material properties, especially for damping reasons (keep minimal)
			myCFbrMat.setStiffnessDamping (0.001);  
			myCFbrMat.setNormalizedDamping (100);  
			// approx. slack until that point (assumed for all CF)
			myCFbrMat.setReferenceStrain (0.5e-2f); 
			
			// For stability or debugging CF can be slightly pretensioned from 
			// model startup
			CFbr.setRestLength (CFbr.getLength()*1);  						
			CF_RCL.add(CFbr);  			// add fibers to RenderableComponentList
			
			//System.out.println("CFCrossSec: " + CFCrossSec); 
		}		
	}
		
	


	// -------------------- MORE CALCULATIONS ----------------------------------
	// linspace() function: known from Matlab
	public static double[] linspace (double start, double stop, int n) {
		LinkedList<Double> temp = new LinkedList<Double>();
		double step = (stop-start)/(n-1);
		for(int i = 0; i <= n-2; ++i){
			temp.add (start + (i * step));
		}
		temp.add (stop);			
		Double[] resultD = temp.toArray (new Double[temp.size()]);
		double[] result = new double[temp.size()];
		for (int d=0; d<temp.size(); d++){
			result[d] = resultD[d].doubleValue();
		}		
		return result;
	}
	
		
	
	
	// ------------------------------------------------------------------------
	// Make background white by default and set viewers eye
	@Override
    public void attach (DriverInterface driver) {
       super.attach (driver);      
       GLViewer viewer = driver.getViewerManager().getViewer(0); 
       if (viewer != null) {
	       viewer.setBlendSourceFactor (BlendFactor.GL_ONE_MINUS_CONSTANT_ALPHA);
	       viewer.setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);     
	       viewer.setBackgroundColor (Color.WHITE);
	       viewer.setEye (new Point3d(0.246044, -0.434962, 0.280147));
	       viewer.setCenter (new Point3d(-0.00429798, 0.0110715, 0.0601638));
       }
    }
	

}
