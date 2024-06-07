#include "lanelet2_io/io_handlers/CpmHandler.h"

#include <lanelet2_core/geometry/Point.h>

#include <boost/geometry/algorithms/is_valid.hpp>
#include <fstream>
#include <iostream>
#include <pugixml.hpp>
#include <sstream>
#include <string>

#include "lanelet2_io/Exceptions.h"
#include "lanelet2_io/io_handlers/Factory.h"
#include "lanelet2_core/utility/Optional.h"

#define POINT_MERGE_DISTANCE 0.001

using namespace std::string_literals;

namespace lanelet {
namespace io_handlers {

using Errors = std::vector<std::string>;

namespace {
// register with factories
RegisterParser<CpmParser> regParser;
}  // namespace

Id pointID_incrementor = 0;
Id lineStringID_incrementor = 0;
LaneletLayer::Map lanelets_;
AreaLayer::Map areas_;
RegulatoryElementLayer::Map regulatoryElements_;
PolygonLayer::Map polygons_;
LineStringLayer::Map lineStrings_;
PointLayer::Map points_;

Point3d CpmParser::parsePoint(pugi::xml_node xml_point, std::unique_ptr<LaneletMap>& map, Optional<Point3d> previousPoint) const {
  auto x = std::stod(xml_point.child("x").child_value());
  auto y = std::stod(xml_point.child("y").child_value());
  Point3d point = Point3d(InvalId, x, y, 0);
  bool merged = false;

  auto nearests = map->pointLayer.nearest(point, 1);
  if (nearests.size() >= 1) {
    auto nearest = nearests[0];
    auto distance = geometry::distance(point, nearest);
    // printf("Point: (%d) %f %f  Nearest: (%d) %f %f (distance %f)\n", pointID_incrementor, x, y, nearest.id(),
    //  nearest.x(), nearest.y(), distance);

    // Avoid linestrings having duplicate consecutive points
    bool nearest_is_previous = previousPoint && nearest.id() == previousPoint->id();
    merged = distance < POINT_MERGE_DISTANCE && !nearest_is_previous;
    if (merged) point = nearest;
  }

  if (!merged) {
    point.setId(++pointID_incrementor);
    map->add(point);
  }

  return point;
}

LineString3d CpmParser::parseBound(pugi::xml_node xml_boundary, std::unique_ptr<LaneletMap>& map) const {
  std::vector<Point3d> boundary_points;
  bool no_new_points = true;
  LineString3d linestring;

  Optional<Point3d> previousPoint = {};
  for (auto xml_point = xml_boundary.child("point"); xml_point; xml_point = xml_point.next_sibling("point")) {
    Id old_pointID_incrementor = pointID_incrementor;
    auto point = parsePoint(xml_point, map, previousPoint);
    assert(!previousPoint || point != *previousPoint);
    previousPoint = point;
    boundary_points.push_back(point);
    if (pointID_incrementor != old_pointID_incrementor) no_new_points = false;
  }
  auto lineMarking = xml_boundary.child("lineMarking").child_value();
  if (no_new_points) {
    // find the lanelet that uses the same start and end points
    struct {
      bool operator()(LineString3d a, LineString3d b) const { return a.id() < b.id(); }
    } compareFunction;
    LineStrings3d usages1 = map->lineStringLayer.findUsages(boundary_points.front());
    LineStrings3d usages2 = map->lineStringLayer.findUsages(boundary_points.back());
    sort(usages1.begin(), usages1.end(), compareFunction);
    sort(usages2.begin(), usages2.end(), compareFunction);

    std::cout << "Vector 1: ";
    for (int i = 0; i < usages1.size(); i++) std::cout << usages1[i].id() << " ";
    std::cout << std::endl;
    std::cout << "Vector 2: ";
    for (int i = 0; i < usages2.size(); i++) std::cout << usages2[i].id() << " ";
    std::cout << std::endl;

    LineStrings3d usages;
    std::set_intersection(usages1.begin(), usages1.end(), usages2.begin(), usages2.end(), std::back_inserter(usages),
                          compareFunction);
    std::cout << "Combined Usages: ";
    for (int i = 0; i < usages.size(); i++) std::cout << usages[i].id() << " ";
    std::cout << std::endl;

    if (usages.size() == 1) {
      linestring = usages.front();
      if (linestring.attribute("subtype") != lineMarking)
        printf("WARNING: subtype does not match with duplicate linestring %d\n", linestring.id());
    } else {
      printf("WARNING: no_new_points is true but usage intersection has %d members\n", usages.size());
    }
  }
  if (linestring.id() == InvalId) {
    AttributeMap attributes;
    attributes.insert(std::make_pair("type", "line_thin"));
    // assumes only "solid" or "dashed" attributes are used
    attributes.insert(std::make_pair("subtype", lineMarking));
    linestring = LineString3d(++lineStringID_incrementor, boundary_points, attributes);
    map->add(linestring);
  }
  printf("lineString %d: no_new_points = %d\n", linestring.id(), no_new_points);

  return linestring;
}

std::unique_ptr<LaneletMap> CpmParser::parse(const std::string& filename, ErrorMessages& errors) const {
  // read xml
  std::cout << "parsing file " << filename << std::endl;
  pugi::xml_document doc;
  auto result = doc.load_file(filename.c_str());
  if (!result) {
    throw lanelet::ParseError("Errors occured while parsing xml file: "s + result.description());
  }

  std::unique_ptr<LaneletMap> map = std::make_unique<LaneletMap>();
  auto xml_commonRoad = doc.child("commonRoad");
  try {
    for (auto xml_lanelet = xml_commonRoad.child("lanelet"); xml_lanelet;
         xml_lanelet = xml_lanelet.next_sibling("lanelet")) {
      auto leftBound = parseBound(xml_lanelet.child("leftBound"), map);
      auto rightBound = parseBound(xml_lanelet.child("rightBound"), map);

      static const std::unordered_map<std::string, std::string> lanelet_type_map{{"urban", "road"},
                                                                                 {"highway", "highway"}};

      AttributeMap attributes;
      attributes.insert(std::make_pair("subtype", lanelet_type_map.at(xml_lanelet.child("laneletType").child_value())));

      auto lanelet = Lanelet(xml_lanelet.attribute("id").as_llong(InvalId), leftBound, rightBound, attributes);
      map->add(lanelet);
    }
  } catch (std::exception e) {
    throw ParseError(e.what());
  }

  return map;
}
}  // namespace io_handlers
}  // namespace lanelet
