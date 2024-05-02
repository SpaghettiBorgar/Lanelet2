#pragma once
#include "lanelet2_io/io_handlers/Parser.h"
#include <pugixml.hpp>

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

  LineString3d parseBound(pugi::xml_node xml_boundary) const ;
};

}  // namespace io_handlers
}  // namespace lanelet
