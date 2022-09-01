// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef READ_CSV_H
#define READ_CSV_H

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include "mars/data_utils/filesystem.h"

namespace mars
{
using CsvDataType = std::map<std::string, std::vector<double>>;
using HeaderMapType = std::map<int, std::string>;

class ReadCsv
{
  char delim{ ',' };

public:
  ReadCsv(CsvDataType* csv_data, const std::string& file_path)
  {
    if (!mars::filesystem::IsFile(file_path))
    {
      std::cout << "[Warning] File " << file_path << " does not exist." << std::endl;
      exit(EXIT_FAILURE);
    }

    file_.open(file_path);

    // Check for header
    const int first_value_row = check_for_header();

    if (first_value_row < 1)
    {
      std::cout << "Error: No header in CSV file" << std::endl;
      exit(EXIT_FAILURE);
    }

    HeaderMapType header_map = get_header(first_value_row - 1);

    // Set line counter to first valued row
    if (first_value_row > 0)
    {
      set_line_couter_of_file(first_value_row);
    }

    // Initialize CSV data type
    //const int rows = get_rows();

    // TODO(CHB) Possibly initialize/reserve map vectors
    CsvDataType csv_data_int;

    // Read columns associated to header tokens
    std::string line;
    int line_counter = 0;

    while (std::getline(file_, line))
    {
      std::stringstream row_stream(line);
      std::string token;
      int column_counter = 0;

      double item;
      while (std::getline(row_stream, token, delim))
      {
        // TODO(CHB) check collumn number and reject if it differs
        std::istringstream is(token);
        is >> item;
        csv_data_int[header_map[column_counter]].push_back(item);

        ++column_counter;
      }

      line_counter++;
    }

    file_.close();

    *csv_data = csv_data_int;
  }

private:
  std::ifstream file_;

  HeaderMapType get_header(const int& row = 0)
  {
    set_line_couter_of_file(row);

    HeaderMapType header_data;

    int count = 0;
    std::string line;
    std::getline(file_, line);
    std::stringstream row_stream(line);
    std::string token;

    while (std::getline(row_stream, token, delim))
    {
      token.erase(remove_if(token.begin(), token.end(), isspace), token.end());

      header_data[count] = token;
      count++;
    }

    file_.clear();  // reset line counter
    file_.seekg(0, std::ios::beg);
    return header_data;
  }

  int check_for_header()
  {
    int count = 0;
    std::string line;
    while (std::getline(file_, line))
    {
      if (std::isdigit(line[line.find_first_not_of(" \t")]))
      {
        break;
      }

      ++count;
    }

    file_.clear();  // reset line counter
    file_.seekg(0, std::ios::beg);
    return count;
  }

  void set_line_couter_of_file(const int& line_number)
  {
    file_.clear();  // reset line counter
    file_.seekg(0, std::ios::beg);

    std::string line;
    for (int k = 0; k < line_number; k++)
    {
      std::getline(file_, line);
    }
  }

  int get_rows()
  {
    int count = 0;
    std::string line;
    while (std::getline(file_, line))
    {
      ++count;
    }
    file_.clear();  // reset line counter
    file_.seekg(0, std::ios::beg);
    return count;
  }
};
}  // namespace mars

#endif  // READ_CSV_H
