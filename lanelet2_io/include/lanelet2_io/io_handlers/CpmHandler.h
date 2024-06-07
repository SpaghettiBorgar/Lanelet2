#pragma once
#include <pugixml.hpp>

#include "lanelet2_io/io_handlers/Parser.h"

namespace lanelet {
namespace io_handlers {

/**
 * @brief Parser class for CPM XML files
 */
class CpmParser : public Parser {
 public:
  using Parser::Parser;

  std::unique_ptr<LaneletMap> parse(const std::string& filename, ErrorMessages& errors) const override;

  static constexpr const char* extension() { return ".xml"; }

  static constexpr const char* name() { return "cpm_handler"; }

 private:
  LineString3d parseBound(pugi::xml_node xml_boundary, std::unique_ptr<LaneletMap>& map) const;
  Point3d parsePoint(pugi::xml_node xml_point, std::unique_ptr<LaneletMap>& map, Optional<Point3d> previousPoint = {}) const;
};

}  // namespace io_handlers
}  // namespace lanelet
