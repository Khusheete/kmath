#include "testing.hpp"

#include <format>


#define CSI_BOLD "\e[1m"
#define CSI_GREEN "\e[38;2;50;255;50m"
#define CSI_RED "\e[38;2;255;50;50m"
#define CSI_CLEAR "\e[0m"


Testing *Testing::singleton = nullptr;


std::string Testing::change_section(const std::string &p_section_name) {
  section = p_section_name;
  section_success_count = 0;
  section_assertion_count = 0;
  test_success_count = 0;
  test_assertion_count = 0;
  return std::format("{}===== {} ====={}", CSI_BOLD, section, CSI_CLEAR);
}


std::string Testing::change_test(const std::string &p_test_name) {
  test = p_test_name;
  test_success_count = 0;
  test_assertion_count = 0;
  return std::format("{}- {}{}", CSI_BOLD, test, CSI_CLEAR);
}


std::string Testing::get_test_report() {
  bool success = test_success_count == test_assertion_count;
  const char *color = (success)? CSI_GREEN : CSI_RED;
  const char *status = (success)? "success" : "failure";
  return std::format(
    "-> {}{}{}: {} {}/{}{}", color, CSI_BOLD, test, status, test_success_count, test_assertion_count, CSI_CLEAR
  );
}


std::string Testing::get_section_report() {
  bool success = section_success_count == section_assertion_count;
  const char *color = (success)? CSI_GREEN : CSI_RED;
  const char *status = (success)? "success" : "failure";
  return std::format(
    "=> {}{}{}: {} {}/{}{} <=", color, CSI_BOLD, section, status, section_success_count, section_assertion_count, CSI_CLEAR
  );
}


std::string Testing::get_final_report() {
  bool success = total_success_count == total_assertion_count;
  const char *color = (success)? CSI_GREEN : CSI_RED;
  const char *status = (success)? "success" : "failure";
  return std::format(
    "{}===== {}Test suit: {} {}/{}{} {}====={}", CSI_BOLD, color, status, total_success_count, total_assertion_count, CSI_CLEAR, CSI_BOLD, CSI_CLEAR
  );
}


std::string Testing::assert(const std::string &p_title, const bool p_success) {
  total_assertion_count += 1;
  section_assertion_count += 1;
  test_assertion_count += 1;
  if (p_success) {
    total_success_count += 1;
    section_success_count += 1;
    test_success_count += 1;
    return std::format("{}: {}success{}", p_title, CSI_GREEN, CSI_CLEAR);
  }
  return std::format("{}: {}failure{}", p_title, CSI_RED, CSI_CLEAR);
}


void Testing::init_singleton() {
  singleton = new Testing();
}


Testing::Testing() {}


void Testing::deinit_singleton() {
  delete singleton;
  singleton = nullptr;
}
