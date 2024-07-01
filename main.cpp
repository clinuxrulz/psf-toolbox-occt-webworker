#include <emscripten.h>
#include <emscripten/bind.h>
#include <string>
#include <map>
#include <set>
#include <BOPAlgo_Tools.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_FindPlane.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepLib.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Quaternion.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp_Explorer.hxx>
#include <TColgp_Array1OfDir.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <ShapeFix_ShapeTolerance.hxx>
#include <Poly_Connect.hxx>
#include <StdPrs_ToolTriangulatedShape.hxx>
#include <ShapeAnalysis_Surface.hxx>
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
json delete_shape(json params);
json make_cylinder(json params);
json shape_to_edges(json params);
json fillet_edge(json params);
json cut_v2(json params);
json shape_faces(json params);
json fuse_shapes_with_transforms(json params);
json extrude_face(json params);
json get_shape_type(json params);
json make_faces_for_lines(json params);
json flip_face_normal(json params);
json make_solid_from_faces(json params);
json intersect_line_with_face(json params);
json does_intersect_same_plane_faces(json params);
json intersect_same_plane_faces(json params);
json difference_same_plane_faces(json params);
json intersect_with_infinite_prism_from_face(json params);
json make_box(json params);
json shape_solids(json params);
json shape_points(json params);
json shape_to_mesh_with_uv_coords(json params);

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
        } else if (type == "deleteShape") {
            return delete_shape(data["params"]);
        } else if (type == "makeCylinder") {
            return make_cylinder(data["params"]);
        } else if (type == "shapeToEdges") {
            return shape_to_edges(data["params"]);
        } else if (type == "filletEdge") {
            return fillet_edge(data["params"]);
        } else if (type == "cutV2") {
            return cut_v2(data["params"]);
        } else if (type == "shapeFaces") {
            return shape_faces(data["params"]);
        } else if (type == "fuseShapesWithTransforms") {
            return fuse_shapes_with_transforms(data["params"]);
        } else if (type == "extrudeFace") {
            return extrude_face(data["params"]);
        } else if (type == "getShapeType") {
            return get_shape_type(data["params"]);
        } else if (type == "makeFacesForLines") {
            return make_faces_for_lines(data["params"]);
        } else if (type == "flipFaceNormal") {
            return flip_face_normal(data["params"]);
        } else if (type == "makeSolidFromFaces") {
            return make_solid_from_faces(data["params"]);
        } else if (type == "intersectLineWithFace") {
            return intersect_line_with_face(data["params"]);
        } else if (type == "doesIntersectSamePlaneFaces") {
            return does_intersect_same_plane_faces(data["params"]);
        } else if (type == "intersectSamePlaneFaces") {
            return intersect_same_plane_faces(data["params"]);
        } else if (type == "differenceSamePlaneFaces") {
            return difference_same_plane_faces(data["params"]);
        } else if (type == "intersectWithInfinitePrismFromFace") {
            return intersect_with_infinite_prism_from_face(data["params"]);
        } else if (type == "makeBox") {
            return make_box(data["params"]);
        } else if (type == "shapeSolids") {
            return shape_solids(data["params"]);
        } else if (type == "shapePoints") {
            return shape_points(data["params"]);
        } else if (type == "shapeToMeshWithUVCoords") {
            return shape_to_mesh_with_uv_coords(data["params"]);
        }
        return std::string("Unrecognized message type: ") + type;
    } catch (Standard_Failure err) {
        return result_err(err.GetMessageString());
    } catch (json::exception err) {
        return result_err(err.what());
    }
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

json delete_shape(json params) {
    auto shapeId = params["shapeId"].template get<std::string>();
    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found");
    }
    auto shape = shapesHeap[shapeId];
    shapesHeap.erase(shapeId);
    delete shape;
    return result_ok();
}

json make_cylinder(json params) {
    auto radius = params["radius"].template get<double>();
    auto height = params["height"].template get<double>();
    BRepPrimAPI_MakeCylinder cylinder(radius, height);
    cylinder.Build();
    TopoDS_Shape shape = cylinder.Shape();
    auto id = gen_unique_id();
    shapesHeap[id] = new TopoDS_Shape(shape);
    return result_ok(id);
}

json shape_to_edges(json params) {
    const double tolerance = 1.0;
    const double angularTolerance = 5.0;
    auto shapeId = params["shapeId"].template get<std::string>();
    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found");
    }
    auto shape = shapesHeap[shapeId];
    typedef struct EdgeGroup {
        int start;
        int count;
        size_t edgeId;
    } EdgeGroup;
    std::set<size_t> recordedEdges;
    std::vector<double> lines;
    std::vector<EdgeGroup> edgeGroups;
    TopTools_ShapeMapHasher hasher;
    TopLoc_Location aLocation;
    class AddEdge {
    private:
        std::vector<double>& lines;
        std::vector<EdgeGroup>& edgeGroups;
        std::set<size_t>& recordedEdges;
        int start;
        double previousPoint[3];
        bool hasPreviousPoint;
    public:
        AddEdge(
            std::vector<double>& lines,
            std::vector<EdgeGroup>& edgeGroups,
            std::set<size_t>& recordedEdges
        ):
            lines(lines),
            edgeGroups(edgeGroups),
            recordedEdges(recordedEdges),
            hasPreviousPoint(false)
        {
        }

        void recordPoint(gp_Pnt p) {
            if (hasPreviousPoint) {
                lines.push_back(previousPoint[0]);
                lines.push_back(previousPoint[1]);
                lines.push_back(previousPoint[2]);
                previousPoint[0] = p.X();
                previousPoint[1] = p.Y();
                previousPoint[2] = p.Z();
                lines.push_back(previousPoint[0]);
                lines.push_back(previousPoint[1]);
                lines.push_back(previousPoint[2]);
            } else {
                previousPoint[0] = p.X();
                previousPoint[1] = p.Y();
                previousPoint[2] = p.Z();
                hasPreviousPoint = true;
            }
        }

        void done(size_t edgeHash) {
            int start = this->start / 3;
            int count = (lines.size() - start) / 3;
            size_t edgeId = edgeHash;
            edgeGroups.push_back(EdgeGroup {
                start,
                count,
                edgeId
            });
            recordedEdges.insert(edgeHash);
        }
    };
    {
        TopExp_Explorer explorer(*shape, TopAbs_FACE);
        while (explorer.More()) {
            auto at = explorer.Current();
            if (at.ShapeType() == TopAbs_FACE) {
                auto atFace = TopoDS::Face(at);
                auto triangulation = BRep_Tool::Triangulation(atFace, aLocation);
                if (triangulation.IsNull()) {
                    explorer.Next();
                    continue;
                }
                auto tri = triangulation.get();
                TopExp_Explorer explorer_2(atFace, TopAbs_EDGE);
                while (explorer_2.More()) {
                    auto at_2 = explorer_2.Current();
                    if (at_2.ShapeType() == TopAbs_EDGE) {
                        auto atEdge = TopoDS::Edge(at_2);
                        if (recordedEdges.find(hasher(atEdge)) != recordedEdges.end()) {
                            explorer_2.Next();
                            continue;
                        }
                        TopLoc_Location edgeLoc;
                        auto polygon = BRep_Tool::PolygonOnTriangulation(
                            atEdge,
                            triangulation,
                            edgeLoc
                        );
                        if (polygon.IsNull()) {
                            explorer_2.Next();
                            continue;
                        }
                        auto polygon2 = polygon.get();
                        auto edgeNodes = polygon2->Nodes();
                        AddEdge adder(lines, edgeGroups, recordedEdges);
                        for (int i = edgeNodes.Lower(); i <= edgeNodes.Upper(); i++) {
                            auto p = tri->Node(edgeNodes.Value(i)).Transformed(edgeLoc.Transformation());
                            adder.recordPoint(p);
                        }
                        adder.done(hasher(atEdge));
                    }
                    explorer_2.Next();
                }
            }
            explorer.Next();
        }
    }
    {
        TopExp_Explorer explorer(*shape, TopAbs_EDGE);
        while (explorer.More()) {
            auto at = explorer.Current();
            if (at.ShapeType() == TopAbs_EDGE) {
                auto atEdge = TopoDS::Edge(at);
                if (recordedEdges.find(hasher(atEdge)) != recordedEdges.end()) {
                    explorer.Next();
                    continue;
                }
                auto adaptorCurve = BRepAdaptor_Curve(atEdge);
                auto tangDef = GCPnts_TangentialDeflection(
                    adaptorCurve,
                    tolerance,
                    angularTolerance  
                );
                AddEdge adder(lines, edgeGroups, recordedEdges);
                for (int j = 0; j < tangDef.NbPoints(); j++) {
                    auto p = tangDef.Value(j + 1).Transformed(aLocation.Transformation());
                    adder.recordPoint(p);
                }
                adder.done(hasher(atEdge));
            }
            explorer.Next();
        }
    }
    json result_lines = json::array();
    json result_edge_groups = json::array();
    for (auto line : lines) {
        result_lines.push_back(line);
    }
    for (auto edgeGroup : edgeGroups) {
        result_edge_groups.push_back((json){
            { "start", edgeGroup.start, },
            { "count", edgeGroup.count, },
            { "edgeId", edgeGroup.edgeId, },
        });
    }
    json result = {
        { "lines", result_lines, },
        { "edgeGroups", result_edge_groups, },
    };
    return result_ok(result);
}

json fillet_edge(json params) {
    auto shapeId = params["shapeId"].template get<std::string>();
    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found");
    }
    auto shape = shapesHeap[shapeId];
    auto edgeId = params["edgeId"].template get<std::string>();
    auto radius = params["radius"].template get<double>();
    TopExp_Explorer explorer(*shape, TopAbs_EDGE);
    bool foundEdge = false;
    TopoDS_Edge edge;
    TopTools_ShapeMapHasher hasher;
    while (explorer.More()) {
        auto at = explorer.Current();
        if (at.ShapeType() == TopAbs_EDGE) {
            auto atEdge = TopoDS::Edge(at);
            auto atEdgeId = std::to_string(hasher(atEdge));
            if (atEdgeId == edgeId) {
                edge = atEdge;
                foundEdge = true;
                break;
            }
        }
        explorer.Next();
    }
    if (!foundEdge) {
        return result_err("Edge not found");
    }
    auto makeFillet = BRepFilletAPI_MakeFillet(*shape);
    makeFillet.Add(radius, edge);
    makeFillet.Build();
    TopoDS_Shape result = makeFillet.Shape();
    auto id = gen_unique_id();
    shapesHeap[id] = new TopoDS_Shape(result);
    return result_ok(id);
}

json cut_v2(json params) {
    auto shape1Id = params["shape1Id"].template get<std::string>();
    auto shape2Id = params["shape2Id"].template get<std::string>();
    if (shapesHeap.find(shape1Id) == shapesHeap.end()) {
        return result_err("Shape 1 not found");
    }
    if (shapesHeap.find(shape2Id) == shapesHeap.end()) {
        return result_err("Shape 2 not found");
    }
    auto shape1 = shapesHeap[shape1Id];
    auto shape2 = shapesHeap[shape2Id];
    auto cutBuilder = BRepAlgoAPI_Cut(*shape1, *shape2);
    cutBuilder.Build();
    auto result = cutBuilder.Shape();
    auto id = gen_unique_id();
    shapesHeap[id] = new TopoDS_Shape(result);
    return result_ok(id);
}

json shape_faces(json params) {
    auto shapeId = params["shapeId"].template get<std::string>();
    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found");
    }
    auto shape = shapesHeap[shapeId];
    typedef struct ResultRow {
        std::string faceShapeId;
        std::string faceId;
    } ResultRow;
    std::vector<ResultRow> result;
    TopExp_Explorer explorer(*shape, TopAbs_FACE);
    int index = 0;
    while (explorer.More()) {
        auto at = explorer.Current();
        if (at.ShapeType() == TopAbs_FACE) {
            auto face = new TopoDS_Shape(at);
            auto id = gen_unique_id();
            shapesHeap[id] = face;
            std::ostringstream ss;
            ss << "Face[" << index << "]";
            auto faceId = ss.str();
            ResultRow result_row = {
                .faceShapeId = id,
                .faceId = faceId,
            };
            result.push_back(result_row);
            index += 1;
        }
        explorer.Next();
    }
    json result2 = json::array();
    for (auto row : result) {
        json result_row = {
            { "faceShapeId", row.faceShapeId, },
            { "faceId", row.faceId, },
        };
        result2.push_back(result_row);
    }
    return result_ok(result2);
}

json fuse_shapes_with_transforms(json params) {
    std::vector<TopoDS_Shape*> shapes;

    for (auto& shapeIdTransform : params["shapeIdTransformList"]) {
        auto shapeId = shapeIdTransform["shapeId"].template get<std::string>();
        auto transform = shapeIdTransform["transform"];

        // Retrieve the shape from the heap
        auto shapePtr = shapesHeap[shapeId];
        if (shapePtr == nullptr) {
            return result_err("Shape not found");
        }

        // Clone the shape
        TopoDS_Shape shape = TopoDS_Shape(*shapePtr);

        // Apply transformation
        gp_Trsf trsf;
        // Get the quaternion
        auto q = transform["q"];
        gp_Quaternion quat(q[1], q[2], q[3], q[0]);
        trsf.SetRotationPart(quat);
        // Get the translation
        auto o = transform["o"];
        gp_Vec translation(o[0], o[1], o[2]);
        trsf.SetTranslationPart(translation);

        // Apply the transformation
        BRepBuilderAPI_Transform transformBuilder(shape, trsf);
        transformBuilder.Build();
        TopoDS_Shape shape2 = transformBuilder.Shape();

        shapes.push_back(new TopoDS_Shape(shape2));
    }

    // Fuse the shapes
    TopoDS_Shape result;
    if (shapes.size() == 0) {
        return result_err("No shapes to fuse");
    }
    result = *shapes[0];
    for (size_t i = 1; i < shapes.size(); ++i) {
        BRepAlgoAPI_Fuse fuse(result, *shapes[i]);
        fuse.Build();
        result = fuse.Shape();
    }
    // Clean up the shapes
    for (auto shape : shapes) {
        delete shape;
    }

    // Store the result in the heap and return
    auto shapeId = gen_unique_id();
    shapesHeap[shapeId] = new TopoDS_Shape(result);
    return result_ok(shapeId);
}

json extrude_face(json params) {
    auto shapeId = params["shapeId"].get<std::string>();
    auto faceId = params["faceId"].get<std::string>();
    auto direction = params["direction"];

    // Retrieve the shape
    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found");
    }
    auto shape = shapesHeap[shapeId];

    TopTools_ShapeMapHasher hasher;

    // Find the face
    TopoDS_Face face;
    if (shapeId == faceId) {
        face = TopoDS::Face(*shape);
    } else {
        TopExp_Explorer explorer(*shape, TopAbs_FACE);
        while (explorer.More()) {
            auto at = explorer.Current();
            if (at.ShapeType() == TopAbs_FACE) {
                auto at_face = TopoDS::Face(at);
                if (std::to_string(hasher(at_face)) == faceId) {
                    face = TopoDS::Face(at_face);
                    break;
                }
            }
            explorer.Next();
        }
    }

    // Check if the face was found
    if (face.IsNull()) {
        return result_err("Face not found");
    }

    // Extrude the face
    try {
        gp_Vec directionVec(direction[0], direction[1], direction[2]);
        BRepPrimAPI_MakePrism prism(face, directionVec, true, true);
        prism.Build();
        TopoDS_Shape extrudedShape = prism.Shape();

        // Store the extruded shape in the heap
        auto shapeId = gen_unique_id();
        shapesHeap[shapeId] = new TopoDS_Shape(extrudedShape);
        return result_ok(shapeId);
    } catch (Standard_Failure const& ex) {
        return result_err(std::string("Failed to extrude: ") + ex.GetMessageString());
    }
}

json get_shape_type(json params) {
    auto shapeId = params["shapeId"].get<std::string>();

    // Retrieve the shape
    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found");
    }
    auto shape = shapesHeap[shapeId];

    // Determine the shape type
    TopAbs_ShapeEnum shapeType = shape->ShapeType();

    std::string shapeTypeName;
    switch (shapeType) {
        case TopAbs_COMPOUND:
            shapeTypeName = "Compound";
            break;
        case TopAbs_COMPSOLID:
            shapeTypeName = "CompSolid";
            break;
        case TopAbs_SOLID:
            shapeTypeName = "Solid";
            break;
        case TopAbs_SHELL:
            shapeTypeName = "Shell";
            break;
        case TopAbs_FACE:
            shapeTypeName = "Face";
            break;
        case TopAbs_WIRE:
            shapeTypeName = "Wire";
            break;
        case TopAbs_EDGE:
            shapeTypeName = "Edge";
            break;
        case TopAbs_VERTEX:
            shapeTypeName = "Vertex";
            break;
        case TopAbs_SHAPE:
            shapeTypeName = "Shape";
            break;
        default:
            return result_err("Unknown shape type.");
    }

    // Return the shape type
    return result_ok(shapeTypeName);
}

struct FaceTriangulation
{
    std::vector<double> vertices;
    std::vector<int> trianglesIndexes;
    std::vector<double> verticesNormals;
    std::vector<double> verticesUvs;
};

json make_faces_for_lines(json params)
{
    // Create edges for lines
    std::map<std::string, TopoDS_Edge> lineMap;
    TopoDS_Compound edges;
    BRep_Builder builder;
    builder.MakeCompound(edges);

    for (auto &line : params["lines"])
    {
        auto v1 = line["v1"];
        auto v2 = line["v2"];
        gp_Pnt p1(v1[0], v1[1], v1[2]);
        gp_Pnt p2(v2[0], v2[1], v2[2]);

        BRepBuilderAPI_MakeEdge edgeBuilder(p1, p2);
        edgeBuilder.Build();
        TopoDS_Edge edge = edgeBuilder.Edge();
        builder.Add(edges, edge);
        lineMap[line["id"].get<std::string>()] = edge;
    }

    // Create edges for arcs
    for (auto &arc : params["arcs"])
    {
        auto q = arc["transform"]["q"];
        gp_Quaternion quat(q[1], q[2], q[3], q[0]);
        gp_Vec n = quat.Multiply(gp_Vec(0.0, 0.0, 1.0));
        gp_Vec u = quat.Multiply(gp_Vec(1.0, 0.0, 0.0));
        auto o = arc["transform"]["o"];
        gp_Ax2 axis(gp_Pnt(o[0], o[1], o[2]), n, u);

        Geom_Circle circle(axis, arc["radius"].get<double>());
        Handle(Geom_Curve) circleHandle = new Geom_Circle(circle);

        double startAngle = arc["startAngle"].get<double>();
        double endAngle = arc["endAngle"].get<double>();
        if (endAngle < startAngle)
        {
            endAngle += 360.0;
        }

        Geom_TrimmedCurve arc2(circleHandle, startAngle * M_PI / 180.0, endAngle * M_PI / 180.0, true, true);
        Handle(Geom_Curve) arcHandle = new Geom_TrimmedCurve(arc2);

        BRepBuilderAPI_MakeEdge edgeBuilder(arcHandle);
        edgeBuilder.Build();
        TopoDS_Edge edge = edgeBuilder.Edge();
        builder.Add(edges, edge);
        lineMap[arc["id"].get<std::string>()] = edge;
    }

    // Convert edges to wires
    TopoDS_Shape wires;
    BOPAlgo_Tools::EdgesToWires(edges, wires, false, 1e-2);

    // Fix wires with tolerance
    ShapeFix_ShapeTolerance tol;
    tol.SetTolerance(wires, 0.001, TopAbs_SHAPE);

    // Convert wires to faces
    TopoDS_Shape faces;
    BOPAlgo_Tools::WiresToFaces(wires, faces, 1e-2);

    // Process faces
    std::vector<std::string> result;
    TopExp_Explorer explorer(faces, TopAbs_FACE);
    while (explorer.More())
    {
        TopoDS_Shape at = explorer.Current();
        if (at.ShapeType() == TopAbs_FACE)
        {
            TopoDS_Face face = TopoDS::Face(at);
            bool touchesAll = true;

            for (auto &mustTouchEdgeAnd : params["mustTouchAnds"])
            {
                auto mustTouchEdgeOrs = mustTouchEdgeAnd["mustTouchOrIds"];
                bool touches = false;
                for (auto &mustTouchEdgeOr : mustTouchEdgeOrs)
                {
                    TopoDS_Edge mustTouchEdge = lineMap[mustTouchEdgeOr];
                    if (mustTouchEdge.IsNull())
                    {
                        continue;
                    }

                    TopExp_Explorer explorer2(face, TopAbs_EDGE);
                    while (explorer2.More())
                    {
                        TopoDS_Shape at2 = explorer2.Current();
                        if (at2.ShapeType() == TopAbs_EDGE)
                        {
                            TopoDS_Edge edge = TopoDS::Edge(at2);
                            TopoDS_Vertex v1 = TopExp::FirstVertex(edge, false);
                            TopoDS_Vertex v2 = TopExp::LastVertex(edge, false);
                            TopoDS_Vertex v3 = TopExp::FirstVertex(mustTouchEdge, false);
                            TopoDS_Vertex v4 = TopExp::LastVertex(mustTouchEdge, false);
                            gp_Pnt p1 = BRep_Tool::Pnt(v1);
                            gp_Pnt p2 = BRep_Tool::Pnt(v2);
                            gp_Pnt p3 = BRep_Tool::Pnt(v3);
                            gp_Pnt p4 = BRep_Tool::Pnt(v4);
                            if (
                                p1.SquareDistance(p3) <= 1e-6 && p2.SquareDistance(p4) <= 1e-6 ||
                                p1.SquareDistance(p4) <= 1e-6 && p2.SquareDistance(p3) <= 1e-6)
                            {
                                touches = true;
                                break;
                            }
                        }
                        explorer2.Next();
                    }
                    if (touches)
                    {
                        break;
                    }
                }

                if (!touches)
                {
                    touchesAll = false;
                    break;
                }
            }

            if (!touchesAll)
            {
                explorer.Next();
                continue;
            }

            // Store the face in the heap and add its ID to the result
            auto shapeId = gen_unique_id();
            shapesHeap[shapeId] = new TopoDS_Shape(face);
            result.push_back(shapeId);
        }
        explorer.Next();
    }
    json result2 = json::array();
    for (auto &shapeId : result)
    {
        result2.push_back(shapeId);
    }
    return result_ok(result2);
}

json flip_face_normal(json params)
{
    auto faceId = params["faceId"].get<std::string>();

    if (shapesHeap.find(faceId) == shapesHeap.end())
    {
        return result_err("Face not found");
    }
    auto face = shapesHeap[faceId];

    // Flip the face normal
    auto flippedFace = face->Complemented();

    // Store the flipped face in the heap
    auto shapeId = gen_unique_id();
    shapesHeap[shapeId] = new TopoDS_Shape(flippedFace);
    return result_ok(shapeId);
}

json make_solid_from_faces(json params)
{
    json faceIds = params["faceIds"];

    // Create a sewing object
    BRepBuilderAPI_Sewing sewing(0.001, true, true, false, false);

    // Add faces to the sewing object
    for (auto &faceId : faceIds)
    {
        if (shapesHeap.find(faceId) == shapesHeap.end()) {
            continue;
        }
        auto face = shapesHeap[faceId];
        sewing.Add(*face);
    }

    // Perform the sewing operation
    sewing.Perform();

    // Check if the sewn shape is a shell
    TopoDS_Shape sewedShape = sewing.SewedShape();
    if (sewedShape.ShapeType() != TopAbs_SHELL)
    {
        return result_err("Not a shell");
    }

    // Create a solid from the shell
    TopoDS_Shell shell = TopoDS::Shell(sewedShape);
    BRepBuilderAPI_MakeSolid makeSolid;
    makeSolid.Add(shell);
    makeSolid.Build();

    // Store the solid in the heap
    auto shapeId = gen_unique_id();
    shapesHeap[shapeId] = new TopoDS_Shape(makeSolid.Solid());
    return result_ok(shapeId);
}

json intersect_line_with_face(json params)
{
    auto line = params["line"];
    auto faceId = params["faceId"].get<std::string>();

    // Retrieve the face
    if (shapesHeap.find(faceId) == shapesHeap.end()) {
        return result_err("Face not found");
    }
    auto face = shapesHeap[faceId];

    // Find the plane of the face
    BRepBuilderAPI_FindPlane planeFinder(*face, -1.0);
    if (!planeFinder.Found())
    {
        return result_err("Face not flat.");
    }

    // Get the plane and normal vector
    gp_Pln pln = planeFinder.Plane().get()->Pln();
    gp_Dir normal = pln.Axis().Direction();
    gp_Vec n(normal.X(), normal.Y(), normal.Z());
    n.Normalize();

    // Create an infinite prism from the face
    BRepPrimAPI_MakePrism infiniteExtrudeBuilder(*face, gp_Dir(n), true, false, true);
    infiniteExtrudeBuilder.Build();
    TopoDS_Shape infiniteExtrude = infiniteExtrudeBuilder.Shape();

    // Create edges from the line
    gp_Pnt p1(line["v1"][0], line["v1"][1], line["v1"][2]);
    gp_Pnt p2(line["v2"][0], line["v2"][1], line["v2"][2]);
    BRepBuilderAPI_MakeVertex vertexBuilder1(p1);
    vertexBuilder1.Build();
    TopoDS_Vertex v1 = vertexBuilder1.Vertex();
    BRepBuilderAPI_MakeVertex vertexBuilder2(p2);
    vertexBuilder2.Build();
    TopoDS_Vertex v2 = vertexBuilder2.Vertex();
    BRepBuilderAPI_MakeEdge edgeBuilder(v1, v2);
    edgeBuilder.Build();
    TopoDS_Edge edge = edgeBuilder.Edge();

    // Perform the intersection operation
    BRepAlgoAPI_Common cutter(edge, infiniteExtrude);
    cutter.Build();

    // Extract the intersection points
    std::vector<json> result;
    TopExp_Explorer explorer(cutter.Shape(), TopAbs_EDGE, TopAbs_SHAPE);
    while (explorer.More())
    {
        TopoDS_Shape at = explorer.Current();
        if (at.ShapeType() != TopAbs_EDGE)
        {
            explorer.Next();
            continue;
        }
        TopoDS_Edge edge = TopoDS::Edge(at);

        // Check if the edge is inside the infinite prism
        gp_Pnt midPt((BRep_Tool::Pnt(TopExp::FirstVertex(edge, false)).X() + BRep_Tool::Pnt(TopExp::LastVertex(edge, false)).X()) / 2.0,
                     (BRep_Tool::Pnt(TopExp::FirstVertex(edge, false)).Y() + BRep_Tool::Pnt(TopExp::LastVertex(edge, false)).Y()) / 2.0,
                     (BRep_Tool::Pnt(TopExp::FirstVertex(edge, false)).Z() + BRep_Tool::Pnt(TopExp::LastVertex(edge, false)).Z()) / 2.0);
        BRepClass3d_SolidClassifier classifier(infiniteExtrude);
        classifier.Perform(midPt, 0.001);
        if (classifier.State() != TopAbs_IN)
        {
            explorer.Next();
            continue;
        }

        // Extract the intersection points
        gp_Pnt p1 = BRep_Tool::Pnt(TopExp::FirstVertex(edge, false));
        gp_Pnt p2 = BRep_Tool::Pnt(TopExp::LastVertex(edge, false));
        result.push_back({{"v1", {p1.X(), p1.Y(), p1.Z()}},
                          {"v2", {p2.X(), p2.Y(), p2.Z()}}});

        explorer.Next();
    }

    return result_ok(result);
}

json does_intersect_same_plane_faces(json params)
{
    auto face1Id = params["face1Id"].get<std::string>();
    auto face2Id = params["face2Id"].get<std::string>();

    // Perform the intersection operation
    auto intersectionResult = intersect_same_plane_faces((json){ { "face1Id", face1Id },  { "face2Id", face2Id } });

    // Check if the intersection operation was successful
    if (intersectionResult.is_object() && intersectionResult["type"] == "Err")
    {
        return intersectionResult;
    }

    // Check if there are any intersection faces
    auto faces = shape_faces((json){ { "shapeId", intersectionResult["value"].get<std::string>() } });
    if (faces.is_object() && faces["type"] == "Err")
    {
        return faces;
    }

    // Remove the intersection faces from the heap
    for (auto &face : faces["value"])
    {
        auto faceShapeId = face["faceShapeId"].get<std::string>();
        auto facePtr = shapesHeap[faceShapeId];
        if (facePtr != nullptr)
        {
            delete facePtr;
            shapesHeap.erase(faceShapeId);
        }
    }

    return result_ok(faces["value"].size() > 0);
}

json intersect_same_plane_faces(json params)
{
    auto face1Id = params["face1Id"].get<std::string>();
    auto face2Id = params["face2Id"].get<std::string>();

    if (shapesHeap.find(face1Id) == shapesHeap.end()) {
        return result_err("Face not found: " + face1Id);
    }
    if (shapesHeap.find(face2Id) == shapesHeap.end()) {
        return result_err("Face not found: " + face2Id);
    }

    // Retrieve the faces
    auto face1 = shapesHeap[face1Id];
    auto face2 = shapesHeap[face2Id];

    // Perform the intersection operation
    BRepAlgoAPI_Common intersecter(*face1, *face2);
    intersecter.SetFuzzyValue(0.001);
    intersecter.SetNonDestructive(true);
    intersecter.SetGlue(BOPAlgo_GlueShift);
    intersecter.SetCheckInverted(true);
    intersecter.Build();

    // Store the result in the heap
    auto shapeId = gen_unique_id();
    shapesHeap[shapeId] = new TopoDS_Shape(intersecter.Shape());
    return result_ok(shapeId);
}

json difference_same_plane_faces(json params)
{
    auto face1Id = params["face1Id"].get<std::string>();
    auto face2Id = params["face2Id"].get<std::string>();

    if (shapesHeap.find(face1Id) == shapesHeap.end()) {
        return result_err("Face not found: " + face1Id);
    }
    if (shapesHeap.find(face2Id) == shapesHeap.end()) {
        return result_err("Face not found: " + face2Id);
    }

    // Retrieve the faces
    auto face1 = shapesHeap[face1Id];
    auto face2 = shapesHeap[face2Id];

    // Perform the difference operation
    BRepAlgoAPI_Cut cutter(*face1, *face2);
    cutter.SetFuzzyValue(0.001);
    cutter.SetNonDestructive(true);
    cutter.SetGlue(BOPAlgo_GlueShift);
    cutter.SetCheckInverted(true);
    cutter.Build();

    // Store the result in the heap
    auto shapeId = gen_unique_id();
    shapesHeap[shapeId] = new TopoDS_Shape(cutter.Shape());
    return result_ok(shapeId);
}

json intersect_with_infinite_prism_from_face(json params)
{
    auto shapeId = params["shapeId"].get<std::string>();
    auto faceId = params["faceId"].get<std::string>();

    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found: " + shapeId);
    }
    if (shapesHeap.find(faceId) == shapesHeap.end()) {
        return result_err("Face not found: " + faceId);
    }

    // Retrieve the shape and face
    auto shape = shapesHeap[shapeId];
    auto face = shapesHeap[faceId];

    // Find the plane of the face
    BRepBuilderAPI_FindPlane planeFinder(*shape, -1.0);
    if (!planeFinder.Found())
    {
        return result_err("Face not flat.");
    }

    // Get the plane and normal vector
    gp_Pln pln = planeFinder.Plane().get()->Pln();
    gp_Dir normal = pln.Axis().Direction();
    gp_Vec n(normal.X(), normal.Y(), normal.Z());
    n.Normalize();

    // Calculate the minimum and maximum distances from the shape points to the plane
    auto shapePoints = shape_points((json) { { "shapeId", shapeId } });
    if (shapePoints.is_object() && shapePoints["type"] == "Err")
    {
        return shapePoints;
    }
    double tMin = std::numeric_limits<double>::max();
    double tMax = std::numeric_limits<double>::min();
    gp_Pnt pt = pln.Axis().Location();
    for (auto &pt2 : shapePoints["value"])
    {
        gp_Vec v(pt2["x"].get<double>() - pt.X(), pt2["y"].get<double>() - pt.Y(), pt2["z"].get<double>() - pt.Z());
        double t = n.Dot(v);
        tMin = std::min(tMin, t);
        tMax = std::max(tMax, t);
    }

    // Create an infinite prism from the face
    double tLen = tMax - tMin;
    BRepPrimAPI_MakePrism infinitePrismBuilder(*face, gp_Vec(n.X() * (tLen + 1000.0), n.Y() * (tLen + 1000.0), n.Z() * (tLen + 1000.0)), false, true);
    infinitePrismBuilder.Build();
    TopoDS_Shape infinitePrism = infinitePrismBuilder.Shape();

    // Translate the infinite prism to encompass the shape
    gp_Trsf transform;
    transform.SetTranslationPart(gp_Vec(n.X() * (tMin - 500.0), n.Y() * (tMin - 500.0), n.Z() * (tMin - 500.0)));
    BRepBuilderAPI_Transform transformBuilder(transform);
    transformBuilder.Perform(infinitePrism, true);
    transformBuilder.Build();
    TopoDS_Shape prism = transformBuilder.Shape();

    // Perform the intersection operation
    BRepAlgoAPI_Common cutBuilder(*shape, prism);
    cutBuilder.Build();

    // Store the result in the heap
    auto id = gen_unique_id();
    shapesHeap[id] = new TopoDS_Shape(cutBuilder.Shape());
    return result_ok(id);
}

json make_box(json params)
{
    auto lenX = params["lenX"].get<double>();
    auto lenY = params["lenY"].get<double>();
    auto lenZ = params["lenZ"].get<double>();

    // Create a box
    BRepPrimAPI_MakeBox box(gp_Pnt(0.0, 0.0, 0.0), lenX, lenY, lenZ);
    box.Build();

    // Store the box in the heap
    auto shapeId = gen_unique_id();
    shapesHeap[shapeId] = new TopoDS_Shape(box.Shape());
    return result_ok(shapeId);
}

json shape_solids(json params)
{
    auto shapeId = params["shapeId"].get<std::string>();

    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found: " + shapeId);
    }

    // Retrieve the shape
    auto shape = shapesHeap[shapeId];

    // Extract the solids from the shape
    std::vector<std::string> result;
    TopExp_Explorer topoExplorer(*shape, TopAbs_SOLID, TopAbs_FACE);
    while (topoExplorer.More())
    {
        TopoDS_Shape at = topoExplorer.Current();
        if (at.ShapeType() == TopAbs_SOLID)
        {
            auto shapeId = gen_unique_id();
            shapesHeap[shapeId] = new TopoDS_Shape(at);
            result.push_back(shapeId);
        }
        topoExplorer.Next();
    }

    json result2 = json::array();
    for (auto shapeId : result) {
        result2.push_back(shapeId);
    }

    return result_ok(result2);
}

json shape_points(json params)
{
    auto shapeId = params["shapeId"].get<std::string>();

    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found: " + shapeId);
    }

    // Retrieve the shape
    auto shape = shapesHeap[shapeId];

    // Extract the vertices from the shape
    std::vector<json> result;
    TopExp_Explorer topoExplorer(*shape, TopAbs_VERTEX, TopAbs_VERTEX);
    while (topoExplorer.More())
    {
        TopoDS_Shape at = topoExplorer.Current();
        if (at.ShapeType() == TopAbs_VERTEX)
        {
            TopoDS_Vertex vertex = TopoDS::Vertex(at);
            gp_Pnt pt = BRep_Tool::Pnt(vertex);
            result.push_back({{"x", pt.X()},
                         {"y", pt.Y()},
                         {"z", pt.Z()}});
        }
        topoExplorer.Next();
    }
    json result2 = json::array();
    for (auto point : result) {
        result2.push_back(shapeId);
    }
    return result_ok(result2);
}

FaceTriangulation *triangulate_face(TopoDS_Face face, int index0);

json shape_to_mesh_with_uv_coords(json params)
{
    auto tolerance = params["tolerance"].get<double>();
    auto angularTolerance = params["angularTolerance"].get<double>();
    auto shapeId = params["shapeId"].get<std::string>();

    if (shapesHeap.find(shapeId) == shapesHeap.end()) {
        return result_err("Shape not found: " + shapeId);
    }

    // Retrieve the shape
    auto shape = shapesHeap[shapeId];

    // Create an incremental mesh for the shape
    BRepMesh_IncrementalMesh mesh(*shape, tolerance, false, angularTolerance, false);

    TopTools_ShapeMapHasher hasher;

    // Extract the mesh data
    std::vector<double> triangles;
    std::vector<double> vertices;
    std::vector<double> normals;
    std::vector<double> uvs;
    std::vector<json> faceGroups;
    int vertexIndex = 0;
    TopExp_Explorer explorer(*shape, TopAbs_FACE);
    while (explorer.More())
    {
        TopoDS_Shape at = explorer.Current();
        if (at.ShapeType() == TopAbs_FACE) {
            TopoDS_Face face = TopoDS::Face(at);
            auto faceTriangulation = triangulate_face(face, vertexIndex);
            // Check if the face triangulation was successful
            if (faceTriangulation == nullptr)
            {
                explorer.Next();
                continue;
            }
            // Add the triangulation data to the mesh
            faceGroups.push_back({{"start", triangles.size()},
                                {"count", faceTriangulation->trianglesIndexes.size()},
                                {"faceId", hasher(face)}});
            triangles.insert(triangles.end(), faceTriangulation->trianglesIndexes.begin(), faceTriangulation->trianglesIndexes.end());
            vertices.insert(vertices.end(), faceTriangulation->vertices.begin(), faceTriangulation->vertices.end());
            normals.insert(normals.end(), faceTriangulation->verticesNormals.begin(), faceTriangulation->verticesNormals.end());
            uvs.insert(uvs.end(), faceTriangulation->verticesUvs.begin(), faceTriangulation->verticesUvs.end());
            // Update the vertex index for the next face
            vertexIndex += faceTriangulation->vertices.size() / 3;
            // Delete the face triangulation
            delete faceTriangulation;
        }
        explorer.Next();
    }

    // Scale the vertices and UV coordinates
    double scale = 0.001;
    for (auto &vertex : vertices)
    {
        vertex *= scale;
    }
    for (auto &uv : uvs)
    {
        uv *= scale;
    }

    json triangles2 = json::array();
    for (auto x : triangles) {
        triangles2.push_back(x);
    }
    json vertices2 = json::array();
    for (auto x : vertices) {
        vertices2.push_back(x);
    }
    json normals2 = json::array();
    for (auto x : normals) {
        normals2.push_back(x);
    }
    json uvs2 = json::array();
    for (auto x : uvs) {
        uvs2.push_back(x);
    }
    json faceGroups2 = json::array();
    for (auto x : faceGroups) {
        faceGroups2.push_back(x);
    }

    // Return the mesh data
    return result_ok((json){{"triangles", triangles2},
                      {"vertices", vertices2},
                      {"normals", normals2},
                      {"uvs", uvs2},
                      {"faceGroups", faceGroups2}});
}

gp_Pnt2d get_face_uv_coordinate(TopoDS_Face face, gp_Pnt pt);

FaceTriangulation *triangulate_face(TopoDS_Face face, int index0)
{
    TopLoc_Location aLocation;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, aLocation, 0);

    BRepLib_ToolTriangulatedShape::ComputeNormals(face, triangulation);

    if (triangulation.IsNull())
    {
        return nullptr;
    }

    gp_Trsf transformation = aLocation.Transformation();

    FaceTriangulation *triangulatedFace = new FaceTriangulation();
    triangulatedFace->vertices.resize(triangulation->NbNodes() * 3);
    triangulatedFace->verticesNormals.resize(triangulation->NbNodes() * 3);
    triangulatedFace->verticesUvs.resize(triangulation->NbNodes() * 2);
    triangulatedFace->trianglesIndexes.resize(triangulation->NbTriangles() * 3);

    // Write vertex buffer
    for (int i = 1; i <= triangulation->NbNodes(); i++)
    {
        gp_Pnt p = triangulation->Node(i).Transformed(transformation);
        triangulatedFace->vertices[(i - 1) * 3 + 0] = p.X();
        triangulatedFace->vertices[(i - 1) * 3 + 1] = p.Y();
        triangulatedFace->vertices[(i - 1) * 3 + 2] = p.Z();

        // Get UV coordinates for the vertex
        gp_Pnt2d uv = get_face_uv_coordinate(face, p);
        triangulatedFace->verticesUvs[(i - 1) * 2 + 0] = uv.X();
        triangulatedFace->verticesUvs[(i - 1) * 2 + 1] = uv.Y();
    }

    // Write normal buffer
    for (int i = 1; i <= triangulation->NbNodes(); i++)
    {
        gp_Dir d = triangulation->Normal(i).Transformed(transformation);
        triangulatedFace->verticesNormals[(i - 1) * 3 + 0] = d.X();
        triangulatedFace->verticesNormals[(i - 1) * 3 + 1] = d.Y();
        triangulatedFace->verticesNormals[(i - 1) * 3 + 2] = d.Z();
    }

    // Write triangle buffer
    int validFaceTriCount = 0;
    for (int nt = 1; nt <= triangulation->NbTriangles(); nt++)
    {
        Poly_Triangle t = triangulation->Triangle(nt);
        int n1 = t.Value(1);
        int n2 = t.Value(2);
        int n3 = t.Value(3);
        // Orient the triangle according to face orientation
        if (true)
        {
            int tmp = n1;
            n1 = n2;
            n2 = tmp;
        }
        triangulatedFace->trianglesIndexes[validFaceTriCount * 3 + 0] = n1 - 1 + index0;
        triangulatedFace->trianglesIndexes[validFaceTriCount * 3 + 1] = n2 - 1 + index0;
        triangulatedFace->trianglesIndexes[validFaceTriCount * 3 + 2] = n3 - 1 + index0;
        validFaceTriCount++;
    }

    return triangulatedFace;
}

gp_Pnt2d get_face_uv_coordinate(TopoDS_Face face, gp_Pnt pt)
{
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    ShapeAnalysis_Surface sas(surface);
    return sas.ValueOfUV(pt, 0.01);
}
