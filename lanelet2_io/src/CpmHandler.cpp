#include "lanelet2_io/io_handlers/CpmHandler.h"

#include <boost/geometry/algorithms/is_valid.hpp>
#include <fstream>
#include <iostream>
#include <pugixml.hpp>
#include <sstream>
#include <string>

#include "lanelet2_io/Exceptions.h"
#include "lanelet2_io/io_handlers/Factory.h"

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

LineString3d CpmParser::parseBound(pugi::xml_node xml_boundary) const {
  std::vector<Point3d> boundary_points;
  for (auto xml_point = xml_boundary.child("point"); xml_point; xml_point = xml_point.next_sibling("point")) {
    auto x = std::stod(xml_point.child("x").child_value());
    auto y = std::stod(xml_point.child("y").child_value());
    Id pointID = ++pointID_incrementor;
    auto point = Point3d(pointID, BasicPoint3d(x, y, 0));

    boundary_points.push_back(point);
    points_.emplace(pointID, point);
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
  // TODO collate nodes
  // TODO align linestrings
  // TODO investigate inverted lanelets
  // TODO predecessor/successor validation

  auto xml_commonRoad = doc.child("commonRoad");
  try {
    for (auto xml_lanelet = xml_commonRoad.child("lanelet"); xml_lanelet;
         xml_lanelet = xml_lanelet.next_sibling("lanelet")) {
      auto leftBound = parseBound(xml_lanelet.child("leftBound"));
      auto rightBound = parseBound(xml_lanelet.child("rightBound"));
      lineStrings_.emplace(leftBound.id(), leftBound);
      lineStrings_.emplace(rightBound.id(), rightBound);

      static const std::unordered_map<std::string, std::string> lanelet_type_map{{"urban", "road"},
                                                                                 {"highway", "highway"}};

      AttributeMap attributes;
      attributes.insert(std::make_pair("subtype", lanelet_type_map.at(xml_lanelet.child("laneletType").child_value())));

      auto lanelet = Lanelet(xml_lanelet.attribute("id").as_llong(InvalId), leftBound, rightBound, attributes);
      lanelets_.emplace(lanelet.id(), lanelet);
    }
  } catch (std::exception e) {
    throw ParseError(e.what());
  }

  auto map = std::make_unique<LaneletMap>(lanelets_, areas_, regulatoryElements_, polygons_, lineStrings_, points_);

  return map;
}
}  // namespace io_handlers
}  // namespace lanelet
