#!/usr/bin/env python3
from pathlib import Path

path = Path(__file__).resolve().parents[1] / "src" / "main.cpp"
source = path.read_text()


def replace_once(old: str, new: str, label: str) -> None:
    global source
    if old not in source:
        raise SystemExit(f"ASTRA source patch anchor missing: {label}")
    source = source.replace(old, new, 1)

helper_anchor = '''static TopoDS_Shape transformShape(const TopoDS_Shape& shape, const gp_Trsf& trsf) {
    BRepBuilderAPI_Transform tx(shape, trsf, Standard_True);
    return tx.Shape();
}
'''
replace_once(
    helper_anchor,
    helper_anchor + '''
static TopoDS_Shape makeCompoundShape(const std::vector<TopoDS_Shape>& shapes) {
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    size_t added = 0;
    for (const TopoDS_Shape& shape : shapes) {
        if (shape.IsNull()) continue;
        builder.Add(compound, shape);
        ++added;
    }
    if (added == 0) throw std::runtime_error("Cannot create an empty compound");
    return compound;
}
''',
    "compound helper",
)

replace_once(
    '''    result.rim = makeRingCylinder(center, axis, rimRadius, 72.0, 0.76 * tireWidth);
    TopoDS_Shape hub = BRepPrimAPI_MakeCylinder(
        gp_Ax2(center.Translated(gp_Vec(axis) * (-0.40 * tireWidth)), axis),
        66.0, 0.80 * tireWidth).Shape();
    result.rim = safeFuse(result.rim, hub, "wheel hub");

    // Ten paired-Y spokes create a mechanically readable wheel rather than a solid disk.
    for (int i = 0; i < 10; ++i) {
        const double angle = 2.0 * kPi * double(i) / 10.0;
        TopoDS_Shape spoke = BRepPrimAPI_MakeBox(
            gp_Pnt(x - 16.0, y - 0.29 * tireWidth, z + 64.0),
            32.0, 0.58 * tireWidth, rimRadius - 92.0).Shape();
        gp_Trsf rotation;
        rotation.SetRotation(gp_Ax1(center, axis), angle);
        spoke = transformShape(spoke, rotation);
        result.rim = safeFuse(result.rim, spoke, "wheel spoke");
    }

    result.brakeDisc = BRepPrimAPI_MakeCylinder(
        gp_Ax2(center.Translated(gp_Vec(axis) * (-0.18 * tireWidth)), axis),
        172.0, 0.36 * tireWidth).Shape();
    // A center hat prevents the rotor from reading as a featureless plate.
    TopoDS_Shape rotorHat = BRepPrimAPI_MakeCylinder(
        gp_Ax2(center.Translated(gp_Vec(axis) * (-0.20 * tireWidth)), axis),
        82.0, 0.40 * tireWidth).Shape();
    result.brakeDisc = safeFuse(result.brakeDisc, rotorHat, "brake rotor hat");
''',
    '''    // Wheel subcomponents are retained as exact semantic compounds instead of
    // repeatedly boolean-fusing intersecting solids. Repeated spoke fusions are
    // both topologically destructive and needlessly memory-intensive.
    std::vector<TopoDS_Shape> rimPieces;
    rimPieces.reserve(12);
    rimPieces.push_back(makeRingCylinder(center, axis, rimRadius, 72.0, 0.76 * tireWidth));
    rimPieces.push_back(BRepPrimAPI_MakeCylinder(
        gp_Ax2(center.Translated(gp_Vec(axis) * (-0.40 * tireWidth)), axis),
        66.0, 0.80 * tireWidth).Shape());

    // Ten radial spokes create a mechanically readable wheel rather than a solid disk.
    for (int i = 0; i < 10; ++i) {
        const double angle = 2.0 * kPi * double(i) / 10.0;
        TopoDS_Shape spoke = BRepPrimAPI_MakeBox(
            gp_Pnt(x - 16.0, y - 0.29 * tireWidth, z + 64.0),
            32.0, 0.58 * tireWidth, rimRadius - 92.0).Shape();
        gp_Trsf rotation;
        rotation.SetRotation(gp_Ax1(center, axis), angle);
        rimPieces.push_back(transformShape(spoke, rotation));
    }
    result.rim = makeCompoundShape(rimPieces);

    TopoDS_Shape rotorPlate = BRepPrimAPI_MakeCylinder(
        gp_Ax2(center.Translated(gp_Vec(axis) * (-0.18 * tireWidth)), axis),
        172.0, 0.36 * tireWidth).Shape();
    // A center hat prevents the rotor from reading as a featureless plate.
    TopoDS_Shape rotorHat = BRepPrimAPI_MakeCylinder(
        gp_Ax2(center.Translated(gp_Vec(axis) * (-0.20 * tireWidth)), axis),
        82.0, 0.40 * tireWidth).Shape();
    result.brakeDisc = makeCompoundShape({rotorPlate, rotorHat});
''',
    "wheel compounds",
)

replace_once(
    '''    TopoDS_Shape seat = safeFuse(base, back, "seat back");
    seat = safeFuse(seat, head, "seat head restraint");
''',
    '''    TopoDS_Shape seat = makeCompoundShape({base, back, head});
''',
    "seat compound",
)

replace_once(
    '''static std::vector<Part> buildVehicle(const Package& pkg) {
    std::vector<Part> parts;
''',
    '''static std::vector<Part> buildVehicle(const Package& pkg) {
    std::vector<Part> parts;
    std::cerr << "[astra] building exterior body surfaces\\n";
''',
    "build stage log",
)
replace_once(
    '''    // Rolling hardware stays semantically separated so tire, rim, rotor, and caliper
''',
    '''    std::cerr << "[astra] building rolling hardware\\n";
    // Rolling hardware stays semantically separated so tire, rim, rotor, and caliper
''',
    "wheel stage log",
)
replace_once(
    '''    return parts;
}

static void makeCompound''',
    '''    std::cerr << "[astra] vehicle construction complete: " << parts.size() << " semantic parts\\n";
    return parts;
}

static void makeCompound''',
    "construction completion log",
)
replace_once(
    '''    const Package package;
    std::vector<Part> parts = buildVehicle(package);
''',
    '''    const Package package;
    std::cerr << "[astra] starting exact CAD generation\\n";
    std::vector<Part> parts = buildVehicle(package);
    std::cerr << "[astra] assembling semantic product structure\\n";
''',
    "run stage log",
)
replace_once(
    '''    BRepTools::Write(renderAssembly,''',
    '''    std::cerr << "[astra] exporting BREP and STEP assemblies\\n";
    BRepTools::Write(renderAssembly,''',
    "export stage log",
)
replace_once(
    '''    size_t totalVertices = 0, totalTriangles = 0;
''',
    '''    std::cerr << "[astra] tessellating exact parts for PBRT\\n";
    size_t totalVertices = 0, totalTriangles = 0;
''',
    "tessellation stage log",
)

path.write_text(source)
print(f"Patched {path} ({len(source)} bytes)")
