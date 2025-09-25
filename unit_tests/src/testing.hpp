#pragma once


#include "kmath/utils.hpp"

#include <string>
#include <iostream>


class Testing {
public:
  std::string change_section(const std::string &p_section_name);
  std::string change_test(const std::string &p_test_name);

  std::string get_test_report();
  std::string get_section_report();
  std::string get_final_report();

  std::string assert(const std::string &p_title, const bool p_success);

  bool has_succeeded() const;
  
  
public:
  static void init_singleton();
  static inline Testing *get_singleton() {
    return singleton;
  }
  static void deinit_singleton();

  
private:
  Testing();


private:
  std::string section;
  std::string test;
  int total_success_count = 0;
  int total_assertion_count = 0;
  int section_success_count = 0;
  int section_assertion_count = 0;
  int test_success_count = 0;
  int test_assertion_count = 0;


private:
  static Testing *singleton;
};



#define UNIT_TEST_SECTION(p_section_name, p_section) {                                          \
  std::cout << "\n\n" << Testing::get_singleton()->change_section(p_section_name) << std::endl; \
  p_section                                                                                     \
  std::cout << "\n" << Testing::get_singleton()->get_section_report() << std::endl;             \
}


#define UNIT_TEST(p_test_name, p_test) {                                                \
  std::cout << Testing::get_singleton()->change_test(p_test_name) << std::endl; \
  p_test                                                                                \
  std::cout << Testing::get_singleton()->get_test_report() << std::endl;                \
}

#define TEST(p_title, p_success) {                                        \
  bool success = p_success;                                               \
  std::string res = Testing::get_singleton()->assert(p_title, success); \
  if (!success) { std::cout << res << std::endl; }                    \
}
#define TEST_EQ_APPROX(p_title, p_expr1, p_expr2) TEST(p_title, kmath::is_approx(p_expr1, p_expr2))
#define TEST_EQ(p_title, p_expr1, p_expr2) TEST(p_title, p_expr1 == p_expr2)
