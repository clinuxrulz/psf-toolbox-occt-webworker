#include <emscripten.h>
#include <emscripten/bind.h>
#include <string>
#include <map>
#include <BRep_Builder.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Quaternion.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp_Explorer.hxx>
#include <nlohmann/json.hpp>
#include "gzip.hpp"
#include "base64.hpp"

using json = nlohmann::json;

static std::map<std::string, TopoDS_Shape*> shapesHeap;

static unsigned long next_id = 0;

std::string gen_unique_id() {
    return std::to_string(next_id++);
}

std::string result_ok() {
    return ((json){
        { "type", "ok", },
        { "value", (json){}, }
    }).dump();
}

std::string result_ok(json result) {
    return ((json){
        { "type", "ok", },
        { "value", result, },
    }).dump();
}

std::string result_err(std::string message) {
    return ((json){
        { "type", "err", },
        { "message", message, },
    }).dump();
}

TopoDS_Shape MakeBottle(const Standard_Real myWidth, const Standard_Real myHeight,
                        const Standard_Real myThickness);
json save_shape_to_brep_base_64_string(json params);
json load_shape_from_brep_base_64_string(json params);
json apply_transform_to_shape(json params);
json clone_shape(json params);

std::string process_message(std::string message) {
    try {
        json data = json::parse(message);
        std::string type = data["type"].template get<std::string>();
        if (type == "setupOpenCascade") {
            // Do nothing here, it is a legacy setup.
            return result_ok((json){});
        } else if (type == "makeBottle") {
            TopoDS_Shape shape = MakeBottle(1.0, 1.0, 1.0);
            auto shapeId = gen_unique_id();
            shapesHeap[shapeId] = new TopoDS_Shape(shape);
            return result_ok((json){
                { "shapeId", shapeId, },
            });
        } else if (type == "saveShapeToBrepBase64String") {
            return save_shape_to_brep_base_64_string(data["params"]);
        } else if (type == "loadShapeFromBrepBase64String") {
            return load_shape_from_brep_base_64_string(data["params"]);
        } else if (type == "applyTransformToShape") {
            return apply_transform_to_shape(data["params"]);
        } else if (type == "cloneShape") {
            return clone_shape(data["params"]);
        }
    } catch (Standard_Failure err) {
        return result_err(err.GetMessageString());
    } catch (json::exception err) {
        return result_err(err.what());
    }
    return std::string("TODO");
}

EMSCRIPTEN_BINDINGS(my_module) {
    emscripten::function("process_message", &process_message);
}

TopoDS_Shape
MakeBottle(const Standard_Real myWidth, const Standard_Real myHeight,
           const Standard_Real myThickness)
{
  // Profile : Define Support Points
  gp_Pnt aPnt1(-myWidth / 2., 0, 0);        
  gp_Pnt aPnt2(-myWidth / 2., -myThickness / 4., 0);
  gp_Pnt aPnt3(0, -myThickness / 2., 0);
  gp_Pnt aPnt4(myWidth / 2., -myThickness / 4., 0);
  gp_Pnt aPnt5(myWidth / 2., 0, 0);

  // Profile : Define the Geometry
  Handle(Geom_TrimmedCurve) anArcOfCircle = GC_MakeArcOfCircle(aPnt2,aPnt3,aPnt4);
  Handle(Geom_TrimmedCurve) aSegment1 = GC_MakeSegment(aPnt1, aPnt2);
  Handle(Geom_TrimmedCurve) aSegment2 = GC_MakeSegment(aPnt4, aPnt5);

  // Profile : Define the Topology
  TopoDS_Edge anEdge1 = BRepBuilderAPI_MakeEdge(aSegment1);
  TopoDS_Edge anEdge2 = BRepBuilderAPI_MakeEdge(anArcOfCircle);
  TopoDS_Edge anEdge3 = BRepBuilderAPI_MakeEdge(aSegment2);
  TopoDS_Wire aWire  = BRepBuilderAPI_MakeWire(anEdge1, anEdge2, anEdge3);

  // Complete Profile
  gp_Ax1 xAxis = gp::OX();
  gp_Trsf aTrsf;

  aTrsf.SetMirror(xAxis);
  BRepBuilderAPI_Transform aBRepTrsf(aWire, aTrsf);
  TopoDS_Shape aMirroredShape = aBRepTrsf.Shape();
  TopoDS_Wire aMirroredWire = TopoDS::Wire(aMirroredShape);

  BRepBuilderAPI_MakeWire mkWire;
  mkWire.Add(aWire);
  mkWire.Add(aMirroredWire);
  TopoDS_Wire myWireProfile = mkWire.Wire();

  // Body : Prism the Profile
  TopoDS_Face myFaceProfile = BRepBuilderAPI_MakeFace(myWireProfile);
  gp_Vec aPrismVec(0, 0, myHeight);
  TopoDS_Shape myBody = BRepPrimAPI_MakePrism(myFaceProfile, aPrismVec);

  // Body : Apply Fillets
  BRepFilletAPI_MakeFillet mkFillet(myBody);
  TopExp_Explorer anEdgeExplorer(myBody, TopAbs_EDGE);
  while(anEdgeExplorer.More()){
    TopoDS_Edge anEdge = TopoDS::Edge(anEdgeExplorer.Current());
    //Add edge to fillet algorithm
    mkFillet.Add(myThickness / 12., anEdge);
    anEdgeExplorer.Next();
  }

  myBody = mkFillet.Shape();

  // Body : Add the Neck	
  gp_Pnt neckLocation(0, 0, myHeight);
  gp_Dir neckAxis = gp::DZ();
  gp_Ax2 neckAx2(neckLocation, neckAxis);

  Standard_Real myNeckRadius = myThickness / 4.;
  Standard_Real myNeckHeight = myHeight / 10.;

  BRepPrimAPI_MakeCylinder MKCylinder(neckAx2, myNeckRadius, myNeckHeight);
  TopoDS_Shape myNeck = MKCylinder.Shape();

  myBody = BRepAlgoAPI_Fuse(myBody, myNeck);

  // Body : Create a Hollowed Solid
  TopoDS_Face   faceToRemove;
  Standard_Real zMax = -1;

  for(TopExp_Explorer aFaceExplorer(myBody, TopAbs_FACE); aFaceExplorer.More(); aFaceExplorer.Next()){
    TopoDS_Face aFace = TopoDS::Face(aFaceExplorer.Current());
    // Check if <aFace> is the top face of the bottle�s neck 
    Handle(Geom_Surface) aSurface = BRep_Tool::Surface(aFace);
    if(aSurface->DynamicType() == STANDARD_TYPE(Geom_Plane)){
      Handle(Geom_Plane) aPlane = Handle(Geom_Plane)::DownCast(aSurface);
      gp_Pnt aPnt = aPlane->Location();
      Standard_Real aZ   = aPnt.Z();
      if(aZ > zMax){
        zMax = aZ;
        faceToRemove = aFace;
      }
    }
  }

  TopTools_ListOfShape facesToRemove;
  facesToRemove.Append(faceToRemove);
  BRepOffsetAPI_MakeThickSolid aSolidMaker;
  aSolidMaker.MakeThickSolidByJoin(myBody, facesToRemove, -myThickness / 50, 1.e-3);
  myBody = aSolidMaker.Shape();
  // Threading : Create Surfaces
  Handle(Geom_CylindricalSurface) aCyl1 = new Geom_CylindricalSurface(neckAx2, myNeckRadius * 0.99);
  Handle(Geom_CylindricalSurface) aCyl2 = new Geom_CylindricalSurface(neckAx2, myNeckRadius * 1.05);

  // Threading : Define 2D Curves
  gp_Pnt2d aPnt(2. * M_PI, myNeckHeight / 2.);
  gp_Dir2d aDir(2. * M_PI, myNeckHeight / 4.);
  gp_Ax2d anAx2d(aPnt, aDir);

  Standard_Real aMajor = 2. * M_PI;
  Standard_Real aMinor = myNeckHeight / 10;

  Handle(Geom2d_Ellipse) anEllipse1 = new Geom2d_Ellipse(anAx2d, aMajor, aMinor);
  Handle(Geom2d_Ellipse) anEllipse2 = new Geom2d_Ellipse(anAx2d, aMajor, aMinor / 4);
  Handle(Geom2d_TrimmedCurve) anArc1 = new Geom2d_TrimmedCurve(anEllipse1, 0, M_PI);
  Handle(Geom2d_TrimmedCurve) anArc2 = new Geom2d_TrimmedCurve(anEllipse2, 0, M_PI);
  gp_Pnt2d anEllipsePnt1 = anEllipse1->Value(0);
  gp_Pnt2d anEllipsePnt2 = anEllipse1->Value(M_PI);

  Handle(Geom2d_TrimmedCurve) aSegment = GCE2d_MakeSegment(anEllipsePnt1, anEllipsePnt2);
  // Threading : Build Edges and Wires
  TopoDS_Edge anEdge1OnSurf1 = BRepBuilderAPI_MakeEdge(anArc1, aCyl1);
  TopoDS_Edge anEdge2OnSurf1 = BRepBuilderAPI_MakeEdge(aSegment, aCyl1);
  TopoDS_Edge anEdge1OnSurf2 = BRepBuilderAPI_MakeEdge(anArc2, aCyl2);
  TopoDS_Edge anEdge2OnSurf2 = BRepBuilderAPI_MakeEdge(aSegment, aCyl2);
  TopoDS_Wire threadingWire1 = BRepBuilderAPI_MakeWire(anEdge1OnSurf1, anEdge2OnSurf1);
  TopoDS_Wire threadingWire2 = BRepBuilderAPI_MakeWire(anEdge1OnSurf2, anEdge2OnSurf2);
  BRepLib::BuildCurves3d(threadingWire1);
  BRepLib::BuildCurves3d(threadingWire2);

  // Create Threading 
  BRepOffsetAPI_ThruSections aTool(Standard_True);
  aTool.AddWire(threadingWire1);
  aTool.AddWire(threadingWire2);
  aTool.CheckCompatibility(Standard_False);

  TopoDS_Shape myThreading = aTool.Shape();

  // Building the Resulting Compound 
  TopoDS_Compound aRes;
  BRep_Builder aBuilder;
  aBuilder.MakeCompound (aRes);
  aBuilder.Add (aRes, myBody);
  aBuilder.Add (aRes, myThreading);

  return aRes;
}

json save_shape_to_brep_base_64_string(json params) {
    auto shapeId = params["shapeId"].template get<std::string>();
    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found");
    }
    auto shape = shapesHeap[shapeId];
    BRepTools::Write(*shape, "./output.brep");
    // Compress file
    gz_compress_file("./output.brep", "./output.brep.gz");
    // Remove uncompressed file
    unlink("./output.brep");
    // Copy file to string
    std::string result;
    std::ifstream f("./output.brep.gz", std::ios::binary);
    f.seekg(0, std::ios::end);
    result.reserve(f.tellg());
    f.seekg(0, std::ios::beg);
    result.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    f.close();
    // Base64 encode
    result = base64_encode(result);
    // Remove compressed file
    unlink("./output.brep.gz");
    // Return result
    return result_ok(result);
}

json load_shape_from_brep_base_64_string(json params) {
    auto base64String = params["base64String"].template get<std::string>();
    // Base64 decode
    auto data = base64_decode(base64String);
    std::ofstream f("./output.brep.gz", std::ios::binary);
    f << data;
    f.close();
    // Decompress file
    gz_decompress_file("./output.brep.gz", "./output.brep");
    // Remove compressed file
    unlink("./output.brep.gz");
    // Read file
    TopoDS_Shape shape;
    BRep_Builder brepBuilder;
    BRepTools::Read(shape, "./output.brep", brepBuilder);
    unlink("./output.brep");
    // Return result
    auto shapeId = gen_unique_id();
    shapesHeap[shapeId] = new TopoDS_Shape(shape);
    return result_ok(shapeId);
}

json apply_transform_to_shape(json params) {
    auto shapeId = params["shapeId"].template get<std::string>();
    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found");
    }
    auto shape = shapesHeap[shapeId];
    json transform = params["transform"];
    json o = transform["o"];
    json q = transform["q"];
    gp_Trsf transform2;
    transform2.SetTranslationPart(gp_Vec(o[0], o[1], o[2]));
    transform2.SetRotationPart(gp_Quaternion(q[1], q[2], q[3], q[0]));
    BRepBuilderAPI_Transform builder(transform2);
    builder.Perform(*shape);
    builder.Build();
    TopoDS_Shape result = builder.Shape();
    auto id = gen_unique_id();
    shapesHeap[id] = new TopoDS_Shape(result);
    return result_ok(id);
}

json clone_shape(json params) {
    auto shapeId = params["shapeId"].template get<std::string>();
    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found");
    }
    auto shape = shapesHeap[shapeId];
    auto id = gen_unique_id();
    shapesHeap[id] = new TopoDS_Shape(*shape);
    return result_ok(id);
}
