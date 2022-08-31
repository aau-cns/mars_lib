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
#include <sstream>
#include <string>
#include <vector>
#include "mars/data_utils/filesystem.h"

namespace mars
{
using CsvDataType = std::vector<std::vector<double>>;

class ReadCsv
{
public:
  ReadCsv(CsvDataType* csv_data, const std::string& file_path, const int& expected_columns)
  {
    if (!mars::filesystem::IsFile(file_path))
    {
      std::cout << "[Warning] File " << file_path << " does not exist." << std::endl;
      exit(EXIT_FAILURE);
    }

    constexpr char delim(',');
    file_.open(file_path);

    const int columns = get_columns(delim);
    if (expected_columns != columns)
    {
      std::cout << "Error: CSV File does not have the expected number of columns" << std::endl;
      std::cout << "Expected " << expected_columns << "got " << columns << std::endl;
      exit(EXIT_FAILURE);
    }

    const int rows = get_rows();

    // Check for header and set line counter to first valued row
    int first_value_row = check_for_header();

    if (first_value_row > 0)
    {
      std::cout << "Info: Detected CSV header, skipping first line" << std::endl;
      // Set line counter to first valued row
      set_line_couter_of_file(first_value_row);
    }

    // Initialize CSV data type
    CsvDataType row_data(size_t(rows - first_value_row));

    std::string line;
    int line_counter = 0;

    while (std::getline(file_, line))
    {
      row_data[line_counter].resize(columns);

      std::stringstream row_stream(line);
      std::string token;
      int column_counter = 0;

      while (std::getline(row_stream, token, delim))
      {
        if (column_counter >= columns)
        {
          std::cout << "Error: CSV File has more columns than expected" << std::endl;
          exit(EXIT_FAILURE);
        }

        std::istringstream is(token);
        is >> row_data[line_counter][column_counter];

        ++column_counter;
      }

      line_counter++;
    }

    file_.close();

    *csv_data = row_data;
  }

private:
  std::ifstream file_;

  int get_columns(const char& delim)
  {
    int count = 0;
    std::string line;
    std::getline(file_, line);
    std::stringstream row_stream(line);
    std::string token;

    while (std::getline(row_stream, token, delim))
    {
      count++;
    }

    file_.clear();  // reset line counter
    file_.seekg(0, std::ios::beg);
    return count;
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
