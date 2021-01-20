/**
 * Copyright (c) 2021, by the Authors: Robin Remus (RUB)
 *
 * This software is freely available under a 3-clause BSD license. Please see
 * the LICENSE file in the GitHub distribution directory for details.
 */
package artisynth.models.passiveLSS_PlosOne;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
//import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.LinkedList;
import java.util.TreeMap;
import java.lang.String;

import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.LigamentAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
//import artisynth.models.MotionSegments.MyImprtFctnsV02.CollFiberData_MP;
//import artisynth.models.MotionSegments.Simple_IntervJoint_L4L5_V02.LigamentData;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import maspack.util.PathFinder;



/**
* To build the hybrid LSS all user defined import functions are stored in this 
* class, which are needed to import data (e.g. nodal, set etc.) from text
* files used as database and usually stored in a sub-directory. 
*/
public class MyImportFunctions {
	
	// ----------------  VARIABLES -----------------------
	// path from which meshes and geometries will be read - set elsewhere	
	//e.g. = "/geometry_L45_Validierung/"	
	static String GeometrySubFolder = null;
	static String FemPathAddition   = null;	

	
	// ------------------------------------------------------
	
	public static String getGeometrySubFolder() {	
		return GeometrySubFolder;
	}	
	public static void setGeometrySubFolder (String GSF) {
		GeometrySubFolder = GSF;		
		geometryDir = PathFinder.getSourceRelativePath (
				MyImportFunctions.class, GSF);
	}
	
	public static String getFemPathAddition() {	
		return FemPathAddition;
	}	
	public static void setFemPathAddition (String FPA) {
		FemPathAddition = FPA;		
		geometryDirFem  = geometryDir + "FE/" + FPA;
	}
	
	
	
	// complete path from which meshes and geometries will be read in this class
	private static String geometryDir = PathFinder.getSourceRelativePath (
			MyImportFunctions.class, GeometrySubFolder);

	// path to the mesh- and set-files
	private static String geometryDirFem = geometryDir + "FE/" + FemPathAddition;		
	
	
	
	// ----------------- IMPORT MKS data ---------------------------	
   /** Import and create PolygonalMesh from an OBJ file via its name. Files 
    * must be stores in <code>geometryDir</code> in subfolder "RB/"
    * 
    * @param name with optional subfolders and without extension (.obj)
    */
   public static PolygonalMesh addObjMesh (String name) throws IOException {
	   PolygonalMesh mesh = 
			   new PolygonalMesh (new File (geometryDir + "RB/" + name + ".obj"));
	   return mesh;
   	}
	
	




	// --------------- IMPORT FEM data ----------------------
	// Read Files (.node, .elem) with Ansys reader to build FE Model
	public static void readFromAnsysReader (FemModel3d fe, String filename) {
	    // options = 0 -> default
		int options = 0;
		// if the options are needed, an additional input could be added
		try {
			AnsysReader.read (fe, geometryDirFem + filename + ".node", 
						   		   geometryDirFem + filename + ".elem", 1, null, 
						   		   options);
		} 
		catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	
	
	
	// ---------------------------------------------------------------------------
	// ---------------- IMPORT DATA SETS -----------------------------------------
	
	// Get Node-Sets by their - in ANSYS defined - Set names from text file 
	public static LinkedList<FemNode3d> ImportNodeSet(String setName, FemModel3d fem) 
			throws IOException {
		// Import the name of the node sets text file and the FEM model the nodes 
		// are part of.
		// The geometryDir must be known

		File file = new File (geometryDirFem + setName + ".txt");			
		byte[] bytes = new byte[(int) file.length()];
		FileInputStream fis = new FileInputStream(file);
		fis.read(bytes);
		fis.close();	   
		String[] valueStr = new String(bytes).trim().split("\\s+");
	   
		// Convert the Strings line by line to int[] array
		int[] importNodeNum = new int[valueStr.length];	   
		for (int i = 0; i < valueStr.length; i++) 
			importNodeNum[i] = Integer.parseInt(valueStr[i]);
	   		   		   
		// get Nodes by their Number (the ID mustn't be the same!!)
		LinkedList<FemNode3d> ImportNodeSet = new LinkedList<FemNode3d>();
		for(int n = 0; n < importNodeNum.length; n++)
		{			   
			int NodeNumber = importNodeNum[n]-1;  
			FemNode3d node = fem.getNodes().getByNumber(NodeNumber);
			ImportNodeSet.add(node);
		}	   		   
		return ImportNodeSet;	   		   
	}
	
	
	// Get Collagen fiber (CF) material stress-strain curve for 
	// tree-Map input and force calculation in LookUpTableMaterial()
	public static TreeMap<Float, Float> ImportCFTreeMapData(String CFFileName) 
			throws IOException
	{
		TreeMap<Float, Float> myMap = new TreeMap<Float, Float>();
		
		// read text-file	
		File file = new File(geometryDir + CFFileName + ".txt");
		byte[] bytes = new byte[(int) file.length()];
		FileInputStream fis;
		
		fis = new FileInputStream(file);	
		fis.read(bytes);
		fis.close();
		String[] strLines   = new String(bytes).trim().split("\\s+");
		for(int l = 0; l<strLines.length; l++)
		{
			// Convert imported string data to float and save in treeMap
			String[] strLine    = strLines[l].split(",");
			Float floatKey   = Float.parseFloat(strLine[0]);
			Float floatValue = Float.parseFloat(strLine[1]);
			myMap.put(floatKey,  floatValue);
		}		 
		return myMap;	
	}
	
	
	   
	   
	
	// -------------------------------------------------------------------------
	// ------------- IMPORT LIGAMENT DATA AND CREATE LIGAMENTS -----------------
	
	// USUAL LIGAMENTS CREATED FROM TWO POINTS (all w/o ALL&PLL)
	public static LigamentData ImportLigsData (
			String FileNameData, Boolean isUW, String FileNameMaterial) 
	{ 	
	   /* Import the Ligament-Textfile and get start- and end-location of 
	    * each ligament with it's names
	    * 
	    * !! Import Unit was [mm] NEU [m], E-Modulus in [MPa] and Cross-Section 
	    * Area in [mm^2] !!
	    * 
	    * https://stackoverflow.com/questions/2832472/how-to-return-2-values-
	    * from-a-java-method/2832516 
	    */
	   
	   // Variable declaration
	   LinkedList<Point3d> LigsStart    = new LinkedList<Point3d>(); 
	   LinkedList<Point3d> LigsEnd      = new LinkedList<Point3d>(); 
	   LinkedList<String>  LigsName     = new LinkedList<String>();
	   LinkedList<Double>  LigsEModul   = new LinkedList<Double>();
	   LinkedList<Double>  LigsCrossSec = new LinkedList<Double>();
	   LinkedList<Double>  LigsLengthSum= new LinkedList<Double>();
	   LinkedList<Double>  UWLinStiff   = new LinkedList<Double>();
	   LinkedList<Double>  UWEpsr 		= new LinkedList<Double>();
	   LinkedList<Double>  UWEpst	   	= new LinkedList<Double>();
	   
	   // build import path
	   String fileD = new String(geometryDir+FileNameData+    ".lig"); 
	   String fileM = new String(geometryDir+FileNameMaterial+".lig"); 
	   BufferedReader readerD;
	   BufferedReader readerM;
	   	  	   
	   // Import the Ligaments data first
	   try {
		   //int index = 0;  		// index to seperate the lines
		   //int cntAS = 0;
		   String[] lineArrD; 	// Ligament Geometry line
		   String[] lineArrM;	// Ligament material line
		   Point3d pnt1; 		// point 1 of Ligament
		   Point3d pnt2;		// point 2 of Ligament
		   		   
		   readerD = new BufferedReader(new FileReader(fileD));  
		   readerM = new BufferedReader(new FileReader(fileM));
		   String lineD = readerD.readLine();  	// read first line
		   String lineM = readerM.readLine();  	// read first line
		   		   	   
		   // first: check if the sum of ligaments and materials are equal
		   if (Double.parseDouble(lineD) == Double.parseDouble(lineM)) {
			   // number of ligaments (NOT AXIAL SPRINGS!)
			   int ligSum = (int)Double.parseDouble(lineD);  
			   // for each ligament (may contain multiple 1D Springs)
			   for (int L = 0; L < ligSum; L++) {
				   lineD = readerD.readLine();  // read each first line of each ligamentData block
				   lineM = readerM.readLine();  // read material values of each ligament
				   lineArrD = lineD.split(","); // split imported line
				   lineArrM = lineM.split(",");
				   
				   String ligsName     = lineArrD[0];
				   // get number of springs which describe on ligament
				   int ligSpringsSum   = (int)Double.parseDouble(lineArrD[1]);  
				   // get sum of all AxialSpring-lengths = ligament length
				   double ligLengthSum = Double.parseDouble(lineArrD[2]); // [m]		
				   double ligEModul    = 0;
				   double ligCrossSec  = 0;
				   double ligLinStiff  = 0;
				   double ligEpsr	   = 0;
				   double ligEpst 	   = 0;
				   				   
				   if (isUW == true) { // UWLigamentFormulation
					   ligLinStiff = Double.parseDouble(lineArrM[3]);   // [N/(m/m)]
					   ligEpsr 	   = Double.parseDouble(lineArrM[1]);	// [m/m]
					   ligEpst 	   = Double.parseDouble(lineArrM[2]);	// [m/m]
					   ligCrossSec = Double.parseDouble(lineArrM[4])*1E-6;  // [mm^2] --> [m^2]
				   }
				   else { // Linear Ligament Formulation
					   ligEModul    = Double.parseDouble(lineArrM[1])*1E6;   // [MPa]  --> [Pa]
					   ligCrossSec  = Double.parseDouble(lineArrM[2])*1E-6;  // [mm^2] --> [m^2]
				   }
					   
					   				   			   
				   for (int N = 0; N < ligSpringsSum; N++) {
					   // read every line of AxialSprings start and end locations
					   lineD    = readerD.readLine();   
					   lineArrD = lineD.split(",");
					   pnt1 = new Point3d(Double.parseDouble(lineArrD[0]), 
					   			 	   	  Double.parseDouble(lineArrD[1]), 
					   			 	   	  Double.parseDouble(lineArrD[2])); 
					   pnt2 = new Point3d(Double.parseDouble(lineArrD[3]), 
					   			 		  Double.parseDouble(lineArrD[4]), 
					   			 		  Double.parseDouble(lineArrD[5])); 
					   // 
					   LigsStart.add(pnt1);
					   LigsEnd.add(pnt2);
					   LigsName.add(ligsName + "_" + Integer.toString(N));
					   
					   // Add material parameters to Ligament field
					   UWLinStiff.add(ligLinStiff); 
					   UWEpsr.add(ligEpsr);
					   UWEpst.add(ligEpst);
					   LigsEModul.add(ligEModul);
					   LigsCrossSec.add(ligCrossSec);
					   LigsLengthSum.add(ligLengthSum);					     
				   }
			   }
			   readerD.close();
			   readerM.close();
		   }
		   else {
			   System.out.println("Die Anzahlen der übergebenen Ligamente (" 
					   + Double.parseDouble(lineD) + ") stimmt nicht mit der "
					   + "Anzahl der eingelesenen Materialdaten für Ligamente (" 
					   + Double.parseDouble(lineM)+ ") überein!");
		   }		      
	   } 
	   catch(IOException e){
		   e.printStackTrace(); }		   		      
	   
	   // add to return data class (for declaration see above)
	   return new LigamentData(LigsStart, LigsEnd, LigsName, LigsEModul, 
			   LigsCrossSec, LigsLengthSum, UWLinStiff, UWEpsr, UWEpst);
	}
	
	
	
	
	
	// For MULTI-POINT LIGAMENTS (ALL & PLL)
	public static LigamentMPData ImportLigsMPData(String FileNameData, 
			String FileNameMaterial, FemModel3d fem) 
	{ 	
	   /* Import the Ligament-Textfile and get start- and end-location of 
	    * each Multi-Point (MP) ligament with it's names
	    * 
	    * !! Import Unit was [mm] but NOW [m], E-Modulus in [MPa] and 
	    * Cross-Section Area in [mm^2] !!
	    */
	   
	   // Variable declaration
		LinkedList<FemNode3d[]> LigMPNodes 	= new LinkedList<FemNode3d[]>(); 
		LinkedList<Point3d[]>   LigMPPoints = new LinkedList<Point3d[]>(); 
		LinkedList<Point3d> LigsStart      	= new LinkedList<Point3d>(); 
		LinkedList<Point3d> LigsEnd      	= new LinkedList<Point3d>(); 
		LinkedList<String>  LigsName     	= new LinkedList<String>();
		LinkedList<Double>  LigsEModul  	= new LinkedList<Double>();
		LinkedList<Double>  LigsCrossSec 	= new LinkedList<Double>();
		LinkedList<Double>  LigsLengthSum	= new LinkedList<Double>();
		LinkedList<Double>  UWLinStiff   	= new LinkedList<Double>();
		LinkedList<Double>  UWEpsr 			= new LinkedList<Double>();
		LinkedList<Double>  UWEpst	   		= new LinkedList<Double>();
	   
	   // build import path
	   String fileD = new String(geometryDir+FileNameData+    ".lig"); 
	   String fileM = new String(geometryDir+FileNameMaterial+".lig"); 
	   BufferedReader readerD;
	   BufferedReader readerM;
	   	  	   
	   // Import the Ligaments data first
	   try {
		   String[] lineArrD; 	// Ligament Geometry line
		   String[] lineArrM;		// Ligament material line
		   //Point3d[] LigPnts;   	// Point array of imported MP Ligament
		   Point3d pnt1 = null; 	// point 1 of Ligament
		   Point3d pnt2 = null;		// point 2 of Ligament
		   		   
		   readerD = new BufferedReader(new FileReader(fileD));  // Ligament geometry data
		   readerM = new BufferedReader(new FileReader(fileM));  // Material data
		   String lineD = readerD.readLine();  	// read the first line
		   String lineM = readerM.readLine();  	// read the first line
		   
		   // first: check if the declared sum of ligaments and materials are equal	   	   
		   if (Double.parseDouble(lineD) == Double.parseDouble(lineM)) { 
			   // number of ligaments (NOT AXIAL/MP SPRINGS!)
			   int ligSum = (int)Double.parseDouble(lineD);  
			   // for each ligament (may contain multiple MP Springs)  
			   for(int L = 0; L < ligSum; L++)  {		  	 
				   lineD = readerD.readLine();  // read first line of each ligamentData block
				   lineM = readerM.readLine();  // read material values of each ligament
				   lineArrD = lineD.split(","); // split imported line
				   lineArrM = lineM.split(",");
				   
				   String ligsName     = lineArrD[0];
				// get number of springs which describe on ligament
				   int ligSpringsSum   = (int)Double.parseDouble(lineArrD[1]);  
				   // sum of points a MP Spring is build of
				   int ligMPPoints     = (int)Double.parseDouble(lineArrD[2]); 
				   double ligLengthSum = 0;
				   double ligEModul    = 0;
				   double ligCrossSec  = 0;
				   double ligLinStiff  = 0;
				   double ligEpsr	   = 0;
				   double ligEpst 	   = 0;
				   				   
				   ligLinStiff = Double.parseDouble(lineArrM[3]);   // [N/(m/m)]
				   ligEpsr 	   = Double.parseDouble(lineArrM[1]);	// [m/m]
				   ligEpst 	   = Double.parseDouble(lineArrM[2]);	// [m/m]
				   ligCrossSec = Double.parseDouble(lineArrM[4])*1E-6;  // [mm^2] --> [m^2]

					   				   			   
				   for(int N = 0; N < ligSpringsSum; N++) {
					   // read every line of AxialSprings start and end locations
					   lineD    = readerD.readLine();   
					   lineArrD = lineD.split(",");
					   
					   // Initialize variable for each MP Spring (-> clear)
					   FemNode3d[] LigNodes = new FemNode3d[ligMPPoints];
					   Point3d[] LigPnts    = new Point3d[ligMPPoints];  
					   
					   int cnt = 0;					   
					   for(int P = 0; P < ligMPPoints; P++) { // separate points of each MP Spring
						   // Get the locations and create Point3d arrays
						   LigPnts[P] = new Point3d(Double.parseDouble(lineArrD[cnt+0]), 
				   			 	   	  			   	Double.parseDouble(lineArrD[cnt+1]), 
				   			 	   	  			   	Double.parseDouble(lineArrD[cnt+2])); 
						   cnt = cnt + 3;
						   
						   // Find corresponding FE node from imported location, 
						   // transferred to Point3d
						   LigNodes[P] = fem.findNearestNode(LigPnts[P], 0.001);						   
						   
						   // Save first and last point of MP Spring separately
						   if (P == 0) 
							   pnt1 = LigPnts[P]; 						   
						   else if (P == ligMPPoints-1) 
							   pnt2 = LigPnts[P];						   						   
					   }
					   
					   LigMPNodes.add(LigNodes);
					   LigMPPoints.add(LigPnts);
					   LigsStart.add(pnt1);
					   LigsEnd.add(pnt2);
					   LigsName.add(ligsName + "_" + Integer.toString(N));
					   
					   // Add material parameters to Ligament field
					   UWLinStiff.add(ligLinStiff); 
					   UWEpsr.add(ligEpsr);
					   UWEpst.add(ligEpst);
					   LigsEModul.add(ligEModul);
					   LigsCrossSec.add(ligCrossSec);
					   LigsLengthSum.add(ligLengthSum);					     
				   }
			   }
			   readerD.close();
			   readerM.close();
		   }
		   else {
			   System.out.println("Die Anzahlen der übergebenen MP Ligamente (" 
					   + Double.parseDouble(lineD) + 
					   ") stimmt nicht mit der Anzahl der eingelesenen "
					   + "Materialdaten für MP Ligamente (" + 
					   Double.parseDouble(lineM)+ ") überein!");
		   }		      
	   } 
	   catch(IOException e){
		   e.printStackTrace(); }		   		      
	   
	   // add to return data class (for declaration see above)
	   return new LigamentMPData (LigMPNodes, LigMPPoints, LigsStart, LigsEnd, 
			   LigsName, LigsEModul, LigsCrossSec, LigsLengthSum, UWLinStiff, 
			   UWEpsr, UWEpst);
   }
	   
	  
   
   
   // Create Ligaments between two rigid bodies with previously collected data
   public static void CreateLigaments (LigamentData ligsDataImp, int ligIndx, 
		   RigidBody RB1, RigidBody RB2, MechModel mech, Color LigColor) {
		/* Create ligaments with the Information about the ligaments in 
		 * LigamentData. Select the ligament info to create with the ligIndx
		 */
		LinkedList<Point3d> LigsStart = ligsDataImp.getStarts();
		LinkedList<Point3d> LigsEnd   = ligsDataImp.getEnds();
		LinkedList<String> LigsName   = ligsDataImp.getNames();
		LinkedList<Double> LigsEModul = ligsDataImp.getEModuls();
		LinkedList<Double> LigsCSecA  = ligsDataImp.getCSecAs();
		
		// Create FrameMarker and add to mech model
		FrameMarker mkr1 = new FrameMarker(LigsStart.get(ligIndx).x, 
				LigsStart.get(ligIndx).y, LigsStart.get(ligIndx).z);
		FrameMarker mkr2 = new FrameMarker(LigsEnd.get(ligIndx).x,   
				LigsEnd.get(ligIndx).y,   LigsEnd.get(ligIndx).z);
		mkr1.setFrame(RB1);
		mkr2.setFrame(RB2);
		mech.addFrameMarker(mkr1);
		mech.addFrameMarker(mkr2);
		
		// Create AxialSpring and add to MechModel
		AxialSpring Lig0 = new AxialSpring(LigsName.get(ligIndx));
		Lig0.setPoints(mkr1,mkr2);
		mech.addAxialSpring(Lig0);
		Lig0.setRestLength(Lig0.getLength());
		Lig0.setMaterial(new LigamentAxialMaterial(
				calcStiffn(LigsCSecA.get(ligIndx), LigsEModul.get(ligIndx), 
						Lig0.getLength()), /*kCompr*/0, /*d*/0.0001));
		RenderProps.setCylindricalLines(Lig0, 0.0005, LigColor);
	}

	
   
   // Calculation of axial spring stiffness with literature values. For instance 
   // for fibers
	public static double calcStiffn (double crossSec, double YoungsModul, 
			double length) {
		double springsStiffn = YoungsModul * crossSec / length;
		return springsStiffn;
	}
	
	
	
	
	
	// -------------------------------------------------------------------------------------------
	// ---------- IMPORT COLLAGEN FIBER information ----------------------------------------------
	// Get information about collagen fibers from text files (.fiber) -- 
	// Fibers to connect two nodes only    
	public static CollFiberData_MP ImportCollFibersMP(String FileName, 
			FemModel3d fem) throws IOException {
		/* Commit the name of the collagen fibers text file (.fiber) and the 
		 * related FemModel. The node numbers of in the chosen text file will 
		 * be imported and the specific nodes will be selected and stored in 
		 * CollFiberData. (Each fiber will be connected to two nodes of the 
		 * related FemModel) 
		 */
	
		// Variable declaration
		LinkedList<FemNode3d[]> CFNodes   = new LinkedList<FemNode3d[]>();
		LinkedList<Point3d[]> CFPoints    = new LinkedList<Point3d[]>();
		Double CFTotLength 				  = null;
		String CFLayerName 				  = FileName.replaceAll("_", "");  // use names without under score
		Double CFCrossSec 				  = null;  	// not defined in this function (see build function)
		Double CFVolFrac 			      = null;  	// not defined in this function (see build function)
				
		// build import path
		String file = new String(geometryDirFem + FileName + ".fiber"); 
		   BufferedReader reader;
		   
		   // try to import the data from text file
		   try {
			   reader = new BufferedReader(new FileReader(file));
			   String line = reader.readLine();  		// read first line
			   line = line.replaceAll(" ", "");
			   int fiberNum = Integer.parseInt(line);   // number of CFs
			   
			   // internal variables
			   String[] lineArr;
			   int lineInt = 0;
			   int i = 0;
			   			   
			   // skip through whole MP spring (NOT nodes/points)
			   while (i < fiberNum) {   // until all node pairs (=CFs) are read		   
				   line    = reader.readLine();  		// read line with node pair
				   lineArr = line.split("\\t");  		// split line by tabulator				   
				   FemNode3d[] nodeArr = new FemNode3d[lineArr.length];
				   Point3d[] pntArr    = new Point3d[lineArr.length]; 
				   
				   // go through each node/point of a MP spring
				   for (int f = 0; f < lineArr.length; ++f) {
					   	lineArr[f] = lineArr[f].replaceAll(" ", "");   		// remove spaces in string
					   	lineInt    = Integer.parseInt(lineArr[f])-1;   		// java starts counting from 0
					   	nodeArr[f] = fem.getNodes().getByNumber(lineInt);   // get node by number from FemModel
					   	pntArr[f]  = nodeArr[f].getPosition();				// get location of node
					   	}	
				   
				   	CFNodes.add(nodeArr);  // save node array in linkedList
				   	CFPoints.add(pntArr);  // save node location array in linkedList
				   	fem.getNodes().getByNumber(lineInt).setName(CFLayerName 
				   			+ "_" + lineInt);   // set name of affected nodes
				   i++;
			   }
			   
			   // read (and skip) line with "end"
			   line = reader.readLine();   			   
			   // read line with total fiber length
			   CFTotLength = Double.parseDouble(reader.readLine());			   
			   reader.close();		   
		   } 		   
		   catch(IOException e){
			   e.printStackTrace(); }
			   
		// store and return all data from text file in self defined format			   
		return new CollFiberData_MP(CFNodes, CFPoints, CFTotLength, CFLayerName,   
				CFCrossSec, CFVolFrac);	   		   
   }
	
	
	
	
	
	// --------------------------------------------------------------------
	// ----------------- SELF DEFINED CLASSES -----------------------------
	
	// POJO to be able to return three values from class
	public static class LigamentData {	
		public LinkedList<Point3d> Starts;  	// Start-Locations [m]
		public LinkedList<Point3d> Ends;    	// End-Locations [m]
		public LinkedList<String>  Names;   	// Name of Ligament
		public LinkedList<Double>  EModuls;     // E-Modulus of Ligament [Pa]
		public LinkedList<Double>  CrossSec;    // Cross-Sectional Area [m^2]
		public LinkedList<Double>  LengthSum;   // Length sum about all axial springs which define one ligament
		public LinkedList<Double>  UWLinStiff;  // Linear Stiffness k [kg/m] in UWLigament Formulation
		public LinkedList<Double>  UWEpsr;		// Reference Strain epsr [m/m] in UWLigament Formulation
		public LinkedList<Double>  UWEpst;		// Transition Strain epst [m/m] in UWLigament Formulation
	   
		public LigamentData (LinkedList<Point3d> LigsStart, 
							LinkedList<Point3d> LigsEnd, 
							LinkedList<String> LigsName, 
							LinkedList<Double> LigsEModul, 
							LinkedList<Double> LigsCSecA,
							LinkedList<Double> LigsLengthSum,
							LinkedList<Double> UWLinStiff,
							LinkedList<Double> UWEpsr,
							LinkedList<Double> UWEpst) {
			this.Starts     = LigsStart;
			this.Ends       = LigsEnd;
		   	this.Names      = LigsName;
		   	this.EModuls    = LigsEModul;
		   	this.CrossSec   = LigsCSecA;
		   	this.LengthSum  = LigsLengthSum;
		   	this.UWLinStiff = UWLinStiff;
		   	this.UWEpsr 	= UWEpsr;
		   	this.UWEpst 	= UWEpst;
		}
	   
		public LinkedList<Point3d> getStarts(){
			return Starts;
		}
		public LinkedList<Point3d> getEnds(){
			return Ends;
		}
		public LinkedList<String> getNames(){
			return Names;
		}
		public LinkedList<Double> getEModuls(){
			return EModuls;
		}	
		public LinkedList<Double> getCSecAs(){
			return CrossSec;
		}	
		public LinkedList<Double> getLengthSum(){
			return LengthSum;
		}
		public LinkedList<Double> getUWLinStiff(){
			return UWLinStiff;
		}
		public LinkedList<Double> getUWEpsr(){
			return UWEpsr;
		}
		public LinkedList<Double> getUWEpst(){
			return UWEpst;
		}
	}
	
	
	// POJO to be able to return three values from class
	public static class LigamentMPData {	
		public LinkedList<FemNode3d[]> Nodes;  	// Nodes of Multi Point (MP) Ligaments (n>=1)
		public LinkedList<Point3d[]>   Points;  // Locations of MP Ligaments (n>=1)
		public LinkedList<Point3d> Starts;  	// Start-Locations [m]
		public LinkedList<Point3d> Ends;    	// End-Locations [m]
		public LinkedList<String>  Names;   	// Name of Ligament
		public LinkedList<Double>  EModuls;     // E-Modulus of Ligament [Pa]
		public LinkedList<Double>  CrossSec;    // Cross-Sectional Area [m^2]
		public LinkedList<Double>  LengthSum;   // Length sum about all axial springs which define one ligament
		public LinkedList<Double>  UWLinStiff;  // Linear Stiffness k [kg/m] in UWLigament Formulation
		public LinkedList<Double>  UWEpsr;		// Reference Strain epsr [m/m] in UWLigament Formulation
		public LinkedList<Double>  UWEpst;		// Transition Strain epst [m/m] in UWLigament Formulation
	   
		public LigamentMPData (LinkedList<FemNode3d[]> MPNodes,
							  LinkedList<Point3d[]> MPPoints,
							  LinkedList<Point3d> LigsStart, 
							  LinkedList<Point3d> LigsEnd, 
							  LinkedList<String> LigsName, 
							  LinkedList<Double> LigsEModul, 
							  LinkedList<Double> LigsCSecA,
							  LinkedList<Double> LigsLengthSum,
							  LinkedList<Double> UWLinStiff,
							  LinkedList<Double> UWEpsr,
							  LinkedList<Double> UWEpst) {
			this.Nodes 		= MPNodes;
			this.Points 	= MPPoints;
			this.Starts     = LigsStart;
			this.Ends       = LigsEnd;
		   	this.Names      = LigsName;
		   	this.EModuls    = LigsEModul;
		   	this.CrossSec   = LigsCSecA;
		   	this.LengthSum  = LigsLengthSum;
		   	this.UWLinStiff = UWLinStiff;
		   	this.UWEpsr 	= UWEpsr;
		   	this.UWEpst 	= UWEpst;
		}
	   
		public LinkedList<FemNode3d[]> getMPNodes(){
			return Nodes;
		}
		public LinkedList<Point3d[]> getMPPoints(){
			return Points;
		}
		public LinkedList<Point3d> getStarts(){
			return Starts;
		}
		public LinkedList<Point3d> getEnds(){
			return Ends;
		}
		public LinkedList<String> getNames(){
			return Names;
		}
		public LinkedList<Double> getEModuls(){
			return EModuls;
		}	
		public LinkedList<Double> getCSecAs(){
			return CrossSec;
		}	
		public LinkedList<Double> getLengthSum(){
			return LengthSum;
		}
		public LinkedList<Double> getUWLinStiff(){
			return UWLinStiff;
		}
		public LinkedList<Double> getUWEpsr(){
			return UWEpsr;
		}
		public LinkedList<Double> getUWEpst(){
			return UWEpst;
		}
	}
	
	
	
	// Store all info about collagen fibers (CFs) of specific CF-Set (layer) in one variable
	public static class CollFiberData_MP {
		public LinkedList<FemNode3d[]> CFNodes;  		// Nodes of collagen fibers (CF) (n>=1)
		public LinkedList<Point3d[]>   CFPoints;  		// Locations of collagen fibers (CF) (n>=1)
		public Double   		       CFTotLength;   	// calculated total length of all fibers
		public String 				   CFLayerName;   	// Store the name of the specific CF layer (set)
		public Double 				   CFCrossSec; 		// Cross section of CFs in specific layer
		public Double 				   CFVolFrac;		// Volume fraction respectively fiber content in specific layer
		
		public CollFiberData_MP(LinkedList<FemNode3d[]> CFNodes, 
								LinkedList<Point3d[]> CFPoints,
								Double CFTotLength,
								String CFLayerName,
								Double CFCrossSec,
								Double CFVolFrac) {
			this.CFNodes 	  = CFNodes;
			this.CFPoints 	  = CFPoints;
			this.CFTotLength  = CFTotLength;
			this.CFLayerName  = CFLayerName;
			this.CFCrossSec   = CFCrossSec;
			this.CFVolFrac    = CFVolFrac;
		}
		
		public LinkedList<FemNode3d[]> getCFNodes(){
			return CFNodes;
		}
		public LinkedList<Point3d[]> getCFpoints(){
			return CFPoints;
		}
		public Double getCFTotLength(){
			return CFTotLength;
		}
		public String getCFLayerName() {
			return CFLayerName;
		}
		public Double getCFCrossSec(){
			return CFCrossSec;
		}
		public Double getCFVolFrac(){
			return CFVolFrac;
		}

	}

}
