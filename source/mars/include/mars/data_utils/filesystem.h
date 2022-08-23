// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include <string>

namespace mars
{
///
/// \brief The filesystem class implements a wrapper for file or directory interaction
///
/// If needed, adaptations to a specific OS can be made here.
///
class filesystem
{
public:
  filesystem();

  ///
  /// \brief filesystem::IsFile Check if the given path results in a file
  /// \param name string containing the absolute file path
  /// \return true if file, false if not a file
  ///
  static bool IsFile(const std::string& name);

  ///
  /// \brief filesystem::IsDir Check if the given path results in an directory
  /// \param name string containing the absolute directory path
  /// \return true if directory, false if not a directory
  ///
  static bool IsDir(const std::string& name);

  ///
  /// \brief filesystem::MakeDir Create a directory if it does not exist
  /// \param name string containing the absolute directory path
  /// \return true if folder was created or already existed, false if it could not be created
  ///
  static bool MakeDir(const std::string& name);
};
}  // namespace mars

#endif  // FILESYSTEM_H
