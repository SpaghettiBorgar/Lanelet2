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

Point3d CpmParser::parsePoint(pugi::xml_node xml_point, std::unique_ptr<LaneletMap>& map) const {
  auto x = std::stod(xml_point.child("x").child_value());
  auto y = std::stod(xml_point.child("y").child_value());
  Point3d point = Point3d(InvalId, x, y, 0);
  bool merged = false;

  auto nearests = map->pointLayer.nearest(point, 1);
  if (nearests.size() >= 1) {
    auto nearest = nearests[0];
    auto distance = geometry::distance(point, nearest);
    printf("Point: (%d) %f %f  Nearest: (%d) %f %f (distance %f)\n", pointID_incrementor, x, y, nearest.id(),
           nearest.x(), nearest.y(), distance);

    merged = distance < POINT_MERGE_DISTANCE;
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
  for (auto xml_point = xml_boundary.child("point"); xml_point; xml_point = xml_point.next_sibling("point")) {
    auto point = parsePoint(xml_point, map);
    boundary_points.push_back(point);
  }
  Id lineStringID = ++lineStringID_incrementor;
  AttributeMap attributes;
  attributes.insert(std::make_pair("type", "line_thin"));
  // assumes only "solid" or "dashed" attributes are used
  attributes.insert(std::make_pair("subtype", xml_boundary.child("lineMarking").child_value()));
  LineString3d linestring = LineString3d(lineStringID, boundary_points, attributes);

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

  // TODO align linestrings
  // TODO investigate inverted lanelets
  // TODO predecessor/successor validation
  std::unique_ptr<LaneletMap> map = std::make_unique<LaneletMap>();
  auto xml_commonRoad = doc.child("commonRoad");
  try {
    for (auto xml_lanelet = xml_commonRoad.child("lanelet"); xml_lanelet;
         xml_lanelet = xml_lanelet.next_sibling("lanelet")) {
      auto leftBound = parseBound(xml_lanelet.child("leftBound"), map);
      auto rightBound = parseBound(xml_lanelet.child("rightBound"), map);
      map->add(leftBound);
      map->add(rightBound);

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
