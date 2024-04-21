#include "lanelet2_io/io_handlers/CpmHandler.h"

#include <boost/geometry/algorithms/is_valid.hpp>
#include <fstream>
#include <iostream>
#include <pugixml.hpp>
#include <sstream>

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

std::unique_ptr<LaneletMap> CpmParser::parse(const std::string& filename, ErrorMessages& errors) const {
  // read xml
  pugi::xml_document doc;
  auto result = doc.load_file(filename.c_str());
  if (!result) {
    throw lanelet::ParseError("Errors occured while parsing xml file: "s + result.description());
  }

  auto map = std::make_unique<LaneletMap>();

  return map;
}
}  // namespace io_handlers
}  // namespace lanelet
