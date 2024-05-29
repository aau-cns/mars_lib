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

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "mars/data_utils/filesystem.h"

namespace mars
{
using CsvDataType = std::map<std::string, std::vector<double>>;
using HeaderMapType = std::map<int, std::string>;

class ReadCsv
{
  char delim{ ',' };
  HeaderMapType header_map;

public:
  ReadCsv(CsvDataType* csv_data, const std::string& file_path, char delim_ = ',') : delim(delim_)
  {
    if (!mars::filesystem::IsFile(file_path))
    {
      std::cout << "ReadCsv(): [Warning] File " << file_path << " does not exist." << std::endl;
      exit(EXIT_FAILURE);
    }

    file_.open(file_path);

    // Check for header
    const int first_value_row = check_for_header();

    // Initialize CSV data type
    const int rows = get_rows();

    if (first_value_row < 1)
    {
      std::cout << "ReadCsv():Error: No header in CSV file" << std::endl;
      exit(EXIT_FAILURE);
    }

    header_map = get_header(first_value_row - 1);

    // Set line counter to first valued row
    if (first_value_row > 0)
    {
      set_line_couter_of_file(first_value_row);
    }

    CsvDataType csv_data_int;
    for (auto it = header_map.begin(); it != header_map.end(); it++)
    {
      csv_data_int[it->second].resize(rows - 1, 0.0);
    }

    // Read columns associated to header tokens
    std::string line;
    int line_counter = 0;
    int parsed_row_counter = first_value_row;  // header already parsed.

    while (std::getline(file_, line))
    {
      std::stringstream row_stream(line);
      std::string token;
      int column_counter = 0;

      double item;
      while (std::getline(row_stream, token, delim))
      {
        if (column_counter >= (int)header_map.size())
        {
          std::cout << "ReadCsv(): Warning: too many entries in row!" << std::endl;
          ++column_counter;  // to indicate a corrupted row
          break;
        }

        std::istringstream is(token);
        is >> item;
        csv_data_int[header_map[column_counter]][line_counter] = (item);
        ++column_counter;
      }

      // check if row was corrupted, if so, overwrite current line with the next one
      if (column_counter != (int)header_map.size())
      {
        std::cout << "ReadCsv(): Warning: corrupted row=" << parsed_row_counter << " will be skipped!" << std::endl;
      }
      else
      {
        line_counter++;
      }

      // increment parsed row counter
      parsed_row_counter++;
    }

    // shrink to the actual size
    for (auto it = header_map.begin(); it != header_map.end(); it++)
    {
      csv_data_int[it->second].resize(line_counter);
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
